/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 / BCM2710 / BCM2837 BSC (I2C) master controller.
 *
 * Interrupt-driven Zephyr I2C master driver for the "Broadcom Serial
 * Controller" peripheral on the Raspberry Pi family. The BSC has a
 * 16-entry FIFO; the FIFO-threshold interrupts (TXW for writes, RXR
 * for reads) wake the ISR to top up / drain, and the DONE interrupt
 * completes the transfer. The calling thread blocks on a semaphore
 * that the ISR signals.
 *
 * Multi-message transfers chain via the TXW interrupt -- the next
 * message is armed before the current one's STOP, producing a
 * repeated start without a STOP between messages.
 *
 * Hardware restrictions surfaced as -ENOTSUP from i2c_transfer():
 *
 *   - 7-bit addressing only.
 *   - In a multi-message transfer only the last message may be a
 *     read. The state machine cannot reverse direction mid-sequence
 *     except on the transition into the final message.
 *
 * The BCM2835 has a known clock-stretching erratum; this driver
 * disables the hardware CLKT timeout (matches what Linux does) and
 * relies on CONFIG_I2C_TRANSFER_TIMEOUT_MS to bound transfer length.
 *
 * Reference: Linux drivers/i2c/busses/i2c-bcm2835.c; BCM2835 ARM
 * Peripherals datasheet ch. 3 (BSC).
 */

#define DT_DRV_COMPAT brcm_bcm2835_i2c

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_bcm2835, CONFIG_I2C_LOG_LEVEL);

/* Register offsets (BCM2835 ARM Peripherals datasheet ch. 3). */
#define BSC_C    0x00U /* control */
#define BSC_S    0x04U /* status */
#define BSC_DLEN 0x08U /* data length (1..0xFFFF) */
#define BSC_A    0x0cU /* slave address (7 bits, right-aligned) */
#define BSC_FIFO 0x10U /* TX / RX FIFO port */
#define BSC_DIV  0x14U /* clock divider (CDIV, must be even) */
#define BSC_DEL  0x18U /* falling-/rising-edge data delay */
#define BSC_CLKT 0x1cU /* clock-stretch timeout (0 disables) */

/* BSC_C bits. */
#define C_READ  BIT(0)  /* 1 = read, 0 = write */
#define C_CLEAR BIT(4)  /* bits 4 and 5 both clear the FIFO */
#define C_ST    BIT(7)  /* start transfer */
#define C_INTD  BIT(8)  /* interrupt on DONE */
#define C_INTT  BIT(9)  /* interrupt on TXW (TX FIFO needs writing) */
#define C_INTR  BIT(10) /* interrupt on RXR (RX FIFO needs reading) */
#define C_I2CEN BIT(15) /* controller enable */

/* BSC_S bits. */
#define S_TA   BIT(0) /* transfer active */
#define S_DONE BIT(1) /* transfer DONE (W1C) */
#define S_TXW  BIT(2) /* TX FIFO needs writing */
#define S_RXR  BIT(3) /* RX FIFO needs reading */
#define S_TXD  BIT(4) /* TX FIFO has space */
#define S_RXD  BIT(5) /* RX FIFO has data */
#define S_ERR  BIT(8) /* slave NACK (W1C) */
#define S_CLKT BIT(9) /* clock-stretch timeout (W1C) */

#define DEL_FEDL_SHIFT 16U /* falling-edge delay */
#define DEL_REDL_SHIFT 0U  /* rising-edge sample point */

#define CDIV_MIN 2U
#define CDIV_MAX 0xFFFEU

/* Internal sentinel: msg_buf_remaining != 0 at DONE means the slave
 * delivered fewer bytes than DLEN (or the master tried to send more).
 */
#define ERR_LEN BIT(31)

struct i2c_bcm2835_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pcfg;
	uint32_t core_clock_hz;
	uint32_t bitrate;
	void (*irq_config_func)(const struct device *dev);
};

struct i2c_bcm2835_data {
	DEVICE_MMIO_RAM;
	struct k_sem completion;
	struct k_mutex lock;
	struct i2c_msg *curr_msg;
	uint8_t *buf;
	size_t remaining;
	int num_msgs;
	uint32_t msg_err;
};

static inline uint32_t reg_read(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + off);
}

static inline void reg_write(const struct device *dev, uint32_t off,
			     uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + off);
}

/* CDIV such that core_clock / CDIV <= bitrate. The register is
 * interpreted as an even number (LSB ignored), so round up to the next
 * even value; this guarantees the bus rate never exceeds what the
 * caller asked for. FEDL / REDL match Linux's choices (CDIV/16, CDIV/4
 * with a floor of 1).
 */
static int set_bitrate(const struct device *dev, uint32_t bitrate)
{
	const struct i2c_bcm2835_config *cfg = dev->config;
	uint32_t divider, fedl, redl;

	if (bitrate == 0U) {
		return -EINVAL;
	}

	divider = DIV_ROUND_UP(cfg->core_clock_hz, bitrate);
	if (divider & 1U) {
		divider++;
	}
	if (divider < CDIV_MIN || divider > CDIV_MAX) {
		LOG_ERR("bitrate %u Hz unreachable from core %u Hz",
			bitrate, cfg->core_clock_hz);
		return -EINVAL;
	}

	fedl = MAX(divider / 16U, 1U);
	redl = MAX(divider / 4U, 1U);

	reg_write(dev, BSC_DIV, divider);
	reg_write(dev, BSC_DEL,
		  (fedl << DEL_FEDL_SHIFT) | (redl << DEL_REDL_SHIFT));

	return 0;
}

static int i2c_bcm2835_configure(const struct device *dev, uint32_t dev_config)
{
	uint32_t bitrate;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}
	if (dev_config & I2C_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = 100000U;
		break;
	case I2C_SPEED_FAST:
		bitrate = 400000U;
		break;
	case I2C_SPEED_FAST_PLUS:
		bitrate = 1000000U;
		break;
	default:
		return -ENOTSUP;
	}

	return set_bitrate(dev, bitrate);
}

static void fill_tx(const struct device *dev)
{
	struct i2c_bcm2835_data *data = dev->data;

	while (data->remaining > 0U) {
		if (!(reg_read(dev, BSC_S) & S_TXD)) {
			break;
		}
		reg_write(dev, BSC_FIFO, *data->buf);
		data->buf++;
		data->remaining--;
	}
}

static void drain_rx(const struct device *dev)
{
	struct i2c_bcm2835_data *data = dev->data;

	while (data->remaining > 0U) {
		if (!(reg_read(dev, BSC_S) & S_RXD)) {
			break;
		}
		*data->buf = (uint8_t)reg_read(dev, BSC_FIFO);
		data->buf++;
		data->remaining--;
	}
}

/* Arm the controller for the current message. Does NOT touch BSC_A --
 * the slave address is fixed for the whole transfer and is written
 * once in i2c_bcm2835_transfer() before the first start_msg().
 */
static void start_msg(const struct device *dev)
{
	struct i2c_bcm2835_data *data = dev->data;
	struct i2c_msg *msg = data->curr_msg;
	bool last = (data->num_msgs == 1);
	uint32_t c = C_ST | C_I2CEN;

	data->num_msgs--;
	data->buf = msg->buf;
	data->remaining = msg->len;

	if (msg->flags & I2C_MSG_READ) {
		c |= C_READ | C_INTR;
	} else {
		c |= C_INTT;
	}
	if (last) {
		c |= C_INTD;
	}

	reg_write(dev, BSC_DLEN, msg->len);
	reg_write(dev, BSC_C, c);
}

/* On error, C_CLEAR aborts the in-flight transaction. The datasheet
 * note: if we were mid-read, the state machine will queue a NACK +
 * STOP that lands the next time we enable the controller. The status
 * W1C below clears any latched error bits before the next transfer
 * starts.
 */
static void i2c_bcm2835_isr(const struct device *dev)
{
	struct i2c_bcm2835_data *data = dev->data;
	uint32_t s = reg_read(dev, BSC_S);
	uint32_t err;

	err = s & (S_ERR | S_CLKT);
	if (err && !(s & S_TA)) {
		data->msg_err = err;
	}

	if (s & S_DONE) {
		if (data->curr_msg &&
		    (data->curr_msg->flags & I2C_MSG_READ)) {
			drain_rx(dev);
			s = reg_read(dev, BSC_S);
		}
		if ((s & S_RXD) || data->remaining > 0U) {
			data->msg_err |= ERR_LEN;
		}
		goto complete;
	}

	if (s & S_TXW) {
		if (data->remaining == 0U) {
			data->msg_err |= ERR_LEN;
			goto complete;
		}

		fill_tx(dev);

		/* Current write message exhausted: chain into the next
		 * message (repeated-start, no STOP between them).
		 */
		if (data->num_msgs > 0 && data->remaining == 0U) {
			data->curr_msg++;
			start_msg(dev);
		}
		return;
	}

	if (s & S_RXR) {
		if (data->remaining == 0U) {
			data->msg_err |= ERR_LEN;
			goto complete;
		}
		drain_rx(dev);
		return;
	}

	return; /* spurious */

complete:
	reg_write(dev, BSC_C, C_CLEAR);
	reg_write(dev, BSC_S, S_DONE | S_ERR | S_CLKT);
	k_sem_give(&data->completion);
}

static int i2c_bcm2835_transfer(const struct device *dev,
				struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr)
{
	struct i2c_bcm2835_data *data = dev->data;
	int ret;

	if (num_msgs == 0U) {
		return 0;
	}
	if (addr & ~0x7fU) {
		return -ENOTSUP;
	}

	/* HW restriction: only the last message may be a read. Mid-
	 * sequence reads can't be chained -- the controller can't turn
	 * the bus around without a STOP, and a STOP ends the transfer.
	 */
	for (uint8_t i = 0; i + 1U < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_READ) {
			LOG_ERR("only the last message may be a read");
			return -ENOTSUP;
		}
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->curr_msg = msgs;
	data->num_msgs = num_msgs;
	data->msg_err = 0U;
	k_sem_reset(&data->completion);

	/* Clear any stale FIFO + W1C any latched status from a prior
	 * aborted transfer.
	 */
	reg_write(dev, BSC_S, S_DONE | S_ERR | S_CLKT);
	reg_write(dev, BSC_C, C_CLEAR);
	reg_write(dev, BSC_A, addr);

	start_msg(dev);

	if (k_sem_take(&data->completion,
		       K_MSEC(CONFIG_I2C_TRANSFER_TIMEOUT_MS)) != 0) {
		reg_write(dev, BSC_C, C_CLEAR);
		LOG_ERR("transfer to 0x%02x timed out", addr);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (data->msg_err & S_ERR) {
		ret = -ENXIO; /* NACK -- slave didn't respond */
	} else if (data->msg_err) {
		ret = -EIO;
	} else {
		ret = 0;
	}

done:
	data->curr_msg = NULL;
	data->num_msgs = 0;
	k_mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_API(i2c, i2c_bcm2835_api) = {
	.configure = i2c_bcm2835_configure,
	.transfer = i2c_bcm2835_transfer,
};

static int i2c_bcm2835_init(const struct device *dev)
{
	const struct i2c_bcm2835_config *cfg = dev->config;
	struct i2c_bcm2835_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("pinctrl apply failed: %d", ret);
			return ret;
		}
	}

	k_sem_init(&data->completion, 0, 1);
	k_mutex_init(&data->lock);

	/* Disable HW clock-stretch timeout (BCM2835 erratum); park the
	 * controller idle with any latched status cleared.
	 */
	reg_write(dev, BSC_CLKT, 0);
	reg_write(dev, BSC_C, 0);
	reg_write(dev, BSC_S, S_DONE | S_ERR | S_CLKT);

	ret = set_bitrate(dev, cfg->bitrate);
	if (ret < 0) {
		return ret;
	}

	cfg->irq_config_func(dev);

	return 0;
}

#define I2C_BCM2835_INIT(n)                                                    \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),                       \
		    (PINCTRL_DT_INST_DEFINE(n);), ())                          \
                                                                               \
	static void i2c_bcm2835_irq_config_##n(const struct device *dev)       \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),         \
			    i2c_bcm2835_isr, DEVICE_DT_INST_GET(n), 0);        \
		irq_enable(DT_INST_IRQN(n));                                   \
	}                                                                      \
                                                                               \
	static struct i2c_bcm2835_data i2c_bcm2835_data_##n;                   \
                                                                               \
	static const struct i2c_bcm2835_config i2c_bcm2835_config_##n = {      \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                          \
		.pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),       \
				    (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),       \
				    (NULL)),                                   \
		.core_clock_hz = DT_INST_PROP(n, core_clock_frequency),        \
		.bitrate = DT_INST_PROP_OR(n, clock_frequency,                 \
					   I2C_BITRATE_STANDARD),              \
		.irq_config_func = i2c_bcm2835_irq_config_##n,                 \
	};                                                                     \
                                                                               \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_bcm2835_init, NULL,                   \
				  &i2c_bcm2835_data_##n,                       \
				  &i2c_bcm2835_config_##n, POST_KERNEL,        \
				  CONFIG_I2C_INIT_PRIORITY,                    \
				  &i2c_bcm2835_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_BCM2835_INIT)
