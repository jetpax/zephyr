/*
 * Copyright (c) 2026 SS/pyDirect contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MSPI controller driver for the Raspberry Pi RP2350 QSPI Memory Interface
 * (QMI). Drives CS1 only — CS0/m[0] is owned by the existing flash-controller
 * driver (raspberrypi,pico-flash-controller) on the same hardware peripheral.
 *
 * Direct mode is used at PSRAM bring-up only (chip detect, reset,
 * quad-enable, m[1] format/timing programming). Steady state is pure
 * memory-mapped XIP via m[1]; subsequent mspi_transceive calls are not
 * expected and async/IRQ/DMA paths return -ENOTSUP.
 *
 * Register sequences are translated from
 *   github.com/sparkfun/sparkfun-pico/sparkfun_pico/sfe_psram.c
 * (CircuitPython-derived; MIT license, (c) 2024 SparkFun Electronics).
 *
 * QMI direct mode pauses flash XIP, so an instruction fetch missing the
 * 16KB XIP cache during the polling loop would deadlock. direct_xfer_one_packet
 * is therefore marked __ramfunc so its body executes from SRAM.
 */

#define DT_DRV_COMPAT raspberrypi_pico_qmi

#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hardware/clocks.h>
#include <hardware/regs/qmi.h>
#include <hardware/structs/qmi.h>
#include <hardware/structs/xip_ctrl.h>

LOG_MODULE_REGISTER(mspi_rpi_pico, CONFIG_MSPI_LOG_LEVEL);

/* fs-domain math constants (per sfe_psram.c). */
#define FS_PER_SEC                     1000000000000000ll
/* APS6404L max select pulse = 8us, in units of 64 system clocks. */
#define PSRAM_MAX_SELECT_FS64          125000000UL
/* APS6404L min deselect pulse = 50ns. */
#define PSRAM_MIN_DESELECT_FS          50000000UL
/* Direct-mode CLKDIV used for one-off bring-up commands (per sfe_psram). */
#define DIRECT_CLKDIV                  30U

struct mspi_rpi_pico_config {
	const struct pinctrl_dev_config *pcfg;
	struct mspi_cfg                  mspicfg;
};

struct mspi_rpi_pico_data {
	const struct mspi_dev_id *dev_id;
	struct mspi_dev_cfg       dev_cfg;
	struct mspi_xip_cfg       xip_cfg;
	struct k_mutex            lock;
};

static inline uint32_t iwidth_for_io_mode(enum mspi_io_mode mode)
{
	switch (mode) {
	case MSPI_IO_MODE_QUAD:
		return QMI_DIRECT_TX_IWIDTH_VALUE_Q;
	case MSPI_IO_MODE_SINGLE:
	default:
		return QMI_DIRECT_TX_IWIDTH_VALUE_S;
	}
}

static inline uint32_t cs_assert_bit(uint8_t ce_num)
{
	return (ce_num == 1) ? QMI_DIRECT_CSR_ASSERT_CS1N_BITS
			     : QMI_DIRECT_CSR_ASSERT_CS0N_BITS;
}

__ramfunc static int direct_xfer_one_packet(struct mspi_rpi_pico_data *data,
					    const struct mspi_xfer *xfer,
					    const struct mspi_xfer_packet *packet)
{
	const uint32_t iwidth = iwidth_for_io_mode(data->dev_cfg.io_mode);
	const uint32_t cs_bit = cs_assert_bit(data->dev_cfg.ce_num);
	const bool is_rx = (packet->dir == MSPI_RX);

	uint32_t key = irq_lock();

	/* Enter direct mode at the bring-up clock divisor. */
	qmi_hw->direct_csr = (DIRECT_CLKDIV << QMI_DIRECT_CSR_CLKDIV_LSB) |
			     QMI_DIRECT_CSR_EN_BITS;

	/* Drain any cooldown from a prior XIP cycle before asserting CS. */
	while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
	}

	qmi_hw->direct_csr |= cs_bit;

	/* Phase 1: command byte(s), MSB first. */
	for (uint8_t i = 0; i < xfer->cmd_length; i++) {
		uint8_t b = (packet->cmd >> (8 * (xfer->cmd_length - 1 - i))) & 0xff;

		qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
				    (iwidth << QMI_DIRECT_TX_IWIDTH_LSB) |
				    b;
		while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
		}
		(void)qmi_hw->direct_rx;
	}

	/* Phase 2: address bytes, MSB first. */
	for (uint8_t i = 0; i < xfer->addr_length; i++) {
		uint8_t b = (packet->address >> (8 * (xfer->addr_length - 1 - i))) & 0xff;

		qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
				    (iwidth << QMI_DIRECT_TX_IWIDTH_LSB) |
				    b;
		while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
		}
		(void)qmi_hw->direct_rx;
	}

	/*
	 * Phase 3: dummy cycles. We only emit dummy bytes in single-line mode
	 * (8 cycles per byte). Quad-mode dummies for XIP reads are programmed
	 * via m[1].rfmt.DUMMY_LEN, not via direct-mode bytes — and the memc
	 * driver's bring-up sequence does not require quad-mode dummies in
	 * direct path.
	 */
	if (data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE) {
		const uint16_t dummy_cycles = is_rx ? xfer->rx_dummy : xfer->tx_dummy;
		const uint8_t dummy_bytes = (uint8_t)(dummy_cycles / 8U);

		for (uint8_t i = 0; i < dummy_bytes; i++) {
			qmi_hw->direct_tx = (iwidth << QMI_DIRECT_TX_IWIDTH_LSB) | 0xff;
			while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
			}
			(void)qmi_hw->direct_rx;
		}
	}

	/*
	 * Phase 4: data. TX writes with OE; RX writes a NOOP byte without OE
	 * to clock the line and reads the response from direct_rx.
	 */
	for (uint32_t i = 0; i < packet->num_bytes; i++) {
		if (is_rx) {
			qmi_hw->direct_tx = (iwidth << QMI_DIRECT_TX_IWIDTH_LSB) | 0xff;
		} else {
			qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
					    (iwidth << QMI_DIRECT_TX_IWIDTH_LSB) |
					    packet->data_buf[i];
		}
		while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
		}
		uint32_t rx = qmi_hw->direct_rx;

		if (is_rx) {
			packet->data_buf[i] = (uint8_t)rx;
		}
	}

	qmi_hw->direct_csr &= ~(cs_bit | QMI_DIRECT_CSR_EN_BITS);

	irq_unlock(key);
	return 0;
}

static int mspi_rpi_pico_config(const struct mspi_dt_spec *spec)
{
	const struct device *controller = spec->bus;
	const struct mspi_rpi_pico_config *cfg = controller->config;
	int ret;

	if (spec->config.op_mode != MSPI_OP_MODE_CONTROLLER) {
		return -ENOTSUP;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state: %d", ret);
		return ret;
	}

	/* Make sure direct mode and CS lines are de-asserted at start. */
	qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS0N_BITS |
				QMI_DIRECT_CSR_ASSERT_CS1N_BITS |
				QMI_DIRECT_CSR_EN_BITS);
	return 0;
}

static int mspi_rpi_pico_dev_config(const struct device *controller,
				    const struct mspi_dev_id *dev_id,
				    const enum mspi_dev_cfg_mask param_mask,
				    const struct mspi_dev_cfg *dev_cfg)
{
	struct mspi_rpi_pico_data *data = controller->data;

	if (k_mutex_lock(&data->lock,
			 K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE)) != 0) {
		return -EBUSY;
	}

	data->dev_id = dev_id;

	if (param_mask == MSPI_DEVICE_CONFIG_NONE) {
		k_mutex_unlock(&data->lock);
		return 0;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_IO_MODE) {
		if (dev_cfg->io_mode != MSPI_IO_MODE_SINGLE &&
		    dev_cfg->io_mode != MSPI_IO_MODE_QUAD) {
			k_mutex_unlock(&data->lock);
			return -ENOTSUP;
		}
		data->dev_cfg.io_mode = dev_cfg->io_mode;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) {
		if (dev_cfg->data_rate != MSPI_DATA_RATE_SINGLE) {
			k_mutex_unlock(&data->lock);
			return -ENOTSUP;
		}
	}
	if (param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
		data->dev_cfg.freq = dev_cfg->freq;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_CE_NUM) {
		data->dev_cfg.ce_num = dev_cfg->ce_num;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_RX_DUMMY) {
		data->dev_cfg.rx_dummy = dev_cfg->rx_dummy;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_TX_DUMMY) {
		data->dev_cfg.tx_dummy = dev_cfg->tx_dummy;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_READ_CMD) {
		data->dev_cfg.read_cmd = dev_cfg->read_cmd;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_WRITE_CMD) {
		data->dev_cfg.write_cmd = dev_cfg->write_cmd;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_CMD_LEN) {
		data->dev_cfg.cmd_length = dev_cfg->cmd_length;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) {
		data->dev_cfg.addr_length = dev_cfg->addr_length;
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

static int mspi_rpi_pico_transceive(const struct device *controller,
				    const struct mspi_dev_id *dev_id,
				    const struct mspi_xfer *xfer)
{
	struct mspi_rpi_pico_data *data = controller->data;
	int ret = 0;

	if (xfer->async || xfer->xfer_mode != MSPI_PIO) {
		return -ENOTSUP;
	}
	if (dev_id != data->dev_id) {
		return -ESTALE;
	}

	if (k_mutex_lock(&data->lock, K_MSEC(xfer->timeout)) != 0) {
		return -EBUSY;
	}

	for (uint32_t i = 0; i < xfer->num_packet; i++) {
		ret = direct_xfer_one_packet(data, xfer, &xfer->packets[i]);
		if (ret) {
			break;
		}
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int mspi_rpi_pico_xip_config(const struct device *controller,
				    const struct mspi_dev_id *dev_id,
				    const struct mspi_xip_cfg *xip_cfg)
{
	struct mspi_rpi_pico_data *data = controller->data;

	if (dev_id != data->dev_id) {
		return -ESTALE;
	}

	if (k_mutex_lock(&data->lock,
			 K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE)) != 0) {
		return -EBUSY;
	}

	if (!xip_cfg->enable) {
		uint32_t key = irq_lock();

		xip_ctrl_hw->ctrl &= ~XIP_CTRL_WRITABLE_M1_BITS;
		qmi_hw->m[1].rfmt = 0;
		qmi_hw->m[1].rcmd = 0;
		qmi_hw->m[1].wfmt = 0;
		qmi_hw->m[1].wcmd = 0;
		qmi_hw->m[1].timing = 0;

		irq_unlock(key);
		data->xip_cfg = *xip_cfg;
		k_mutex_unlock(&data->lock);
		return 0;
	}

	if (data->dev_cfg.io_mode != MSPI_IO_MODE_QUAD) {
		LOG_ERR("XIP requires QUAD io_mode (got %d)", data->dev_cfg.io_mode);
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	const uint32_t sys_hz = (uint32_t)clock_get_hz(clk_sys);
	const uint32_t fs_per_cycle = (uint32_t)(FS_PER_SEC / sys_hz);
	const uint32_t target_hz = data->dev_cfg.freq ? data->dev_cfg.freq : 109000000U;
	const uint8_t clkdiv = (uint8_t)((sys_hz + target_hz - 1) / target_hz);
	const uint8_t max_select = (uint8_t)(PSRAM_MAX_SELECT_FS64 / fs_per_cycle);
	const uint8_t min_deselect =
		(uint8_t)((PSRAM_MIN_DESELECT_FS + fs_per_cycle - 1) / fs_per_cycle);

	uint32_t key = irq_lock();

	qmi_hw->m[1].timing =
		(QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB) |
		(3U << QMI_M1_TIMING_SELECT_HOLD_LSB) |
		(1U << QMI_M1_TIMING_COOLDOWN_LSB) |
		(1U << QMI_M1_TIMING_RXDELAY_LSB) |
		(max_select << QMI_M1_TIMING_MAX_SELECT_LSB) |
		(min_deselect << QMI_M1_TIMING_MIN_DESELECT_LSB) |
		(clkdiv << QMI_M1_TIMING_CLKDIV_LSB);

	qmi_hw->m[1].rfmt =
		(QMI_M1_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_PREFIX_WIDTH_LSB) |
		(QMI_M1_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M1_RFMT_ADDR_WIDTH_LSB)   |
		(QMI_M1_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_SUFFIX_WIDTH_LSB) |
		(QMI_M1_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M1_RFMT_DUMMY_WIDTH_LSB)  |
		(QMI_M1_RFMT_DUMMY_LEN_VALUE_24   << QMI_M1_RFMT_DUMMY_LEN_LSB)    |
		(QMI_M1_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M1_RFMT_DATA_WIDTH_LSB)   |
		(QMI_M1_RFMT_PREFIX_LEN_VALUE_8   << QMI_M1_RFMT_PREFIX_LEN_LSB)   |
		(QMI_M1_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_RFMT_SUFFIX_LEN_LSB);

	qmi_hw->m[1].rcmd = (data->dev_cfg.read_cmd << QMI_M1_RCMD_PREFIX_LSB);

	qmi_hw->m[1].wfmt =
		(QMI_M1_WFMT_PREFIX_WIDTH_VALUE_Q  << QMI_M1_WFMT_PREFIX_WIDTH_LSB) |
		(QMI_M1_WFMT_ADDR_WIDTH_VALUE_Q    << QMI_M1_WFMT_ADDR_WIDTH_LSB)   |
		(QMI_M1_WFMT_SUFFIX_WIDTH_VALUE_Q  << QMI_M1_WFMT_SUFFIX_WIDTH_LSB) |
		(QMI_M1_WFMT_DUMMY_WIDTH_VALUE_Q   << QMI_M1_WFMT_DUMMY_WIDTH_LSB)  |
		(QMI_M1_WFMT_DUMMY_LEN_VALUE_NONE  << QMI_M1_WFMT_DUMMY_LEN_LSB)    |
		(QMI_M1_WFMT_DATA_WIDTH_VALUE_Q    << QMI_M1_WFMT_DATA_WIDTH_LSB)   |
		(QMI_M1_WFMT_PREFIX_LEN_VALUE_8    << QMI_M1_WFMT_PREFIX_LEN_LSB)   |
		(QMI_M1_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_WFMT_SUFFIX_LEN_LSB);

	qmi_hw->m[1].wcmd = (data->dev_cfg.write_cmd << QMI_M1_WCMD_PREFIX_LSB);

	xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;

	irq_unlock(key);

	data->xip_cfg = *xip_cfg;
	k_mutex_unlock(&data->lock);

	LOG_INF("PSRAM XIP enabled at 0x11000000, %u MHz, clkdiv=%u",
		target_hz / 1000000U, clkdiv);
	return 0;
}

static int mspi_rpi_pico_get_channel_status(const struct device *controller, uint8_t ch)
{
	ARG_UNUSED(controller);
	ARG_UNUSED(ch);

	return (qmi_hw->direct_csr & QMI_DIRECT_CSR_EN_BITS) ? -EBUSY : 0;
}

static int mspi_rpi_pico_init(const struct device *controller)
{
	struct mspi_rpi_pico_data *data = controller->data;
	const struct mspi_rpi_pico_config *cfg = controller->config;
	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = cfg->mspicfg,
	};

	k_mutex_init(&data->lock);
	return mspi_rpi_pico_config(&spec);
}

static DEVICE_API(mspi, mspi_rpi_pico_driver_api) = {
	.config             = mspi_rpi_pico_config,
	.dev_config         = mspi_rpi_pico_dev_config,
	.xip_config         = mspi_rpi_pico_xip_config,
	.transceive         = mspi_rpi_pico_transceive,
	.get_channel_status = mspi_rpi_pico_get_channel_status,
};

#define MSPI_RPI_PICO_INIT(idx)                                                          \
	PINCTRL_DT_INST_DEFINE(idx);                                                     \
	static const struct mspi_rpi_pico_config mspi_rpi_pico_cfg_##idx = {             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                             \
		.mspicfg = {                                                             \
			.channel_num     = 0,                                            \
			.op_mode         = MSPI_OP_MODE_CONTROLLER,                      \
			.duplex          = MSPI_HALF_DUPLEX,                             \
			.max_freq        = DT_INST_PROP_OR(idx, clock_frequency,         \
							   109000000),                   \
			.dqs_support     = false,                                        \
			.num_periph      = DT_INST_CHILD_NUM(idx),                       \
			.sw_multi_periph = DT_INST_PROP(idx, software_multiperipheral),  \
			.num_ce_gpios    = 0,                                            \
		},                                                                       \
	};                                                                               \
	static struct mspi_rpi_pico_data mspi_rpi_pico_data_##idx;                       \
	DEVICE_DT_INST_DEFINE(idx, mspi_rpi_pico_init, NULL,                             \
			      &mspi_rpi_pico_data_##idx,                                 \
			      &mspi_rpi_pico_cfg_##idx,                                  \
			      POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY,                    \
			      &mspi_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_RPI_PICO_INIT)
