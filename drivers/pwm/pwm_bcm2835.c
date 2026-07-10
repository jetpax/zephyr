/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 / BCM2710 / BCM2837 PWM controller.
 *
 * One PWM block with two independent output channels sharing a
 * common reference clock. The driver runs the block in mark:space
 * mode (MSEN=1): RNGx holds the period and DATx the mark, both in
 * reference-clock ticks, so duty = DATx / RNGx. The serialiser /
 * FIFO (audio) mode of the block is not used.
 *
 * The reference clock comes from the CM_PWM slice of the clock
 * manager, which the driver programs directly at init: 19.2 MHz
 * crystal oscillator source divided by an integer divisor derived
 * from the clock-frequency devicetree property. The oscillator is
 * used instead of PLLD because PLLD scales with core_freq in
 * config.txt while the crystal never moves. Integer division only
 * (no MASH / DIVF) keeps the output jitter-free. Moving the clock
 * setup behind a clock-control provider is a documented follow-up
 * in the binding.
 *
 * Reference: Linux drivers/pwm/pwm-bcm2835.c + clk-bcm2835.c;
 * BCM2835 ARM Peripherals datasheet ch. 9 (PWM) and 6.3 (clock
 * manager).
 */

#define DT_DRV_COMPAT brcm_bcm2835_pwm

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pwm_bcm2835, CONFIG_PWM_LOG_LEVEL);

/* PWM block registers (datasheet ch. 9.6). */
#define PWM_CTL  0x00U /* control, both channels */
#define PWM_STA  0x04U /* status */
#define PWM_DMAC 0x08U /* DMA control (serialiser mode only) */
#define PWM_RNG1 0x10U /* channel 1 period, in reference-clock ticks */
#define PWM_DAT1 0x14U /* channel 1 mark */
#define PWM_FIF1 0x18U /* shared FIFO port (serialiser mode only) */
#define PWM_RNG2 0x20U /* channel 2 period */
#define PWM_DAT2 0x24U /* channel 2 mark */

/*
 * PWM_CTL carries one 8-bit field block per channel (channel 0 at
 * bit 0, channel 1 at bit 8). CLRF only exists in the channel-0
 * block (bit 6 is a write-1 FIFO-clear strobe; bit 14 is reserved)
 * -- the per-channel mask below always writes it back as 0.
 */
#define CTL_PWEN BIT(0) /* channel enable */
#define CTL_MODE BIT(1) /* 0 = PWM, 1 = serialiser */
#define CTL_RPTL BIT(2) /* repeat last FIFO word */
#define CTL_SBIT BIT(3) /* line state when not transmitting */
#define CTL_POLA BIT(4) /* invert output polarity */
#define CTL_USEF BIT(5) /* use FIFO instead of DAT */
#define CTL_MSEN BIT(7) /* 1 = mark:space, 0 = PWM-algorithm */

#define CTL_CHAN_SHIFT(ch) ((ch) * 8U)
#define CTL_CHAN_MASK      0xFFU

/*
 * CM_PWM clock-manager slice, mapped as its own two-register window
 * (CM_PWMCTL / CM_PWMDIV). Every write needs the password nibble.
 */
#define CM_CTL 0x00U
#define CM_DIV 0x04U

#define CM_PASSWD      (0x5AU << 24)
#define CM_CTL_SRC_OSC 0x1U /* 19.2 MHz crystal oscillator */
#define CM_CTL_ENAB    BIT(4)
#define CM_CTL_KILL    BIT(5)
#define CM_CTL_BUSY    BIT(7)
#define CM_DIV_DIVI(x) ((x) << 12)

#define CM_BUSY_TIMEOUT_US 1000U

#define PWM_NUM_CHANNELS 2U

struct pwm_bcm2835_config {
	DEVICE_MMIO_NAMED_ROM(pwm);
	DEVICE_MMIO_NAMED_ROM(cm);
	const struct pinctrl_dev_config *pcfg;
	uint32_t tick_hz; /* actual reference-clock rate, osc / divi */
	uint16_t divi;
};

struct pwm_bcm2835_data {
	DEVICE_MMIO_NAMED_RAM(pwm);
	DEVICE_MMIO_NAMED_RAM(cm);
	struct k_spinlock lock; /* PWM_CTL is shared by both channels */
};

#define DEV_CFG(dev)  ((const struct pwm_bcm2835_config *)(dev)->config)
#define DEV_DATA(dev) ((struct pwm_bcm2835_data *)(dev)->data)

static uint32_t pwm_reg_read(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, pwm) + off);
}

static void pwm_reg_write(const struct device *dev, uint32_t off, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_NAMED_GET(dev, pwm) + off);
}

static uint32_t cm_reg_read(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, cm) + off);
}

static void cm_reg_write(const struct device *dev, uint32_t off, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_NAMED_GET(dev, cm) + off);
}

static int cm_wait_busy(const struct device *dev, bool set)
{
	uint32_t remaining = CM_BUSY_TIMEOUT_US;

	while (!!(cm_reg_read(dev, CM_CTL) & CM_CTL_BUSY) != set) {
		if (remaining-- == 0U) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}

	return 0;
}

static int pwm_bcm2835_clock_init(const struct device *dev)
{
	const struct pwm_bcm2835_config *cfg = DEV_CFG(dev);
	int ret;

	/*
	 * Stop the generator (preserve the current SRC while ENAB
	 * drops, per the datasheet's "do not change SRC and ENAB in
	 * the same write" rule) and let the running cycle drain. A
	 * generator wedged by earlier firmware may never drop BUSY;
	 * KILL force-stops it.
	 */
	cm_reg_write(dev, CM_CTL,
		     CM_PASSWD | (cm_reg_read(dev, CM_CTL) & ~(CM_CTL_ENAB | CM_PASSWD)));
	ret = cm_wait_busy(dev, false);
	if (ret < 0) {
		cm_reg_write(dev, CM_CTL, CM_PASSWD | CM_CTL_KILL);
		ret = cm_wait_busy(dev, false);
		if (ret < 0) {
			return ret;
		}
	}

	cm_reg_write(dev, CM_DIV, CM_PASSWD | CM_DIV_DIVI((uint32_t)cfg->divi));
	cm_reg_write(dev, CM_CTL, CM_PASSWD | CM_CTL_SRC_OSC);
	cm_reg_write(dev, CM_CTL, CM_PASSWD | CM_CTL_SRC_OSC | CM_CTL_ENAB);

	return cm_wait_busy(dev, true);
}

static int pwm_bcm2835_set_cycles(const struct device *dev, uint32_t channel,
				  uint32_t period_cycles, uint32_t pulse_cycles,
				  pwm_flags_t flags)
{
	struct pwm_bcm2835_data *data = DEV_DATA(dev);
	uint32_t ctl_chan;
	k_spinlock_key_t key;
	uint32_t ctl;

	if (channel >= PWM_NUM_CHANNELS) {
		return -EINVAL;
	}

	if ((flags & ~PWM_POLARITY_MASK) != 0U) {
		return -ENOTSUP;
	}

	pulse_cycles = MIN(pulse_cycles, period_cycles);

	key = k_spin_lock(&data->lock);

	ctl = pwm_reg_read(dev, PWM_CTL);
	ctl &= ~(CTL_CHAN_MASK << CTL_CHAN_SHIFT(channel));

	if (period_cycles != 0U) {
		if (channel == 0U) {
			pwm_reg_write(dev, PWM_RNG1, period_cycles);
			pwm_reg_write(dev, PWM_DAT1, pulse_cycles);
		} else {
			pwm_reg_write(dev, PWM_RNG2, period_cycles);
			pwm_reg_write(dev, PWM_DAT2, pulse_cycles);
		}

		ctl_chan = CTL_MSEN | CTL_PWEN;
		if ((flags & PWM_POLARITY_INVERTED) != 0U) {
			ctl_chan |= CTL_POLA;
		}
		ctl |= ctl_chan << CTL_CHAN_SHIFT(channel);
	}

	pwm_reg_write(dev, PWM_CTL, ctl);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int pwm_bcm2835_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					  uint64_t *cycles)
{
	if (channel >= PWM_NUM_CHANNELS) {
		return -EINVAL;
	}

	*cycles = DEV_CFG(dev)->tick_hz;

	return 0;
}

static int pwm_bcm2835_init(const struct device *dev)
{
	const struct pwm_bcm2835_config *cfg = DEV_CFG(dev);
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, pwm, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, cm, K_MEM_CACHE_NONE);

	/*
	 * A missing "default" state (-ENOENT) is fine -- a consumer
	 * may carry only a custom state (e.g. ArduinoCore's
	 * per-channel "arduino" state) or mux imperatively at runtime.
	 */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("pinctrl apply failed: %d", ret);
			return ret;
		}
	}

	ret = pwm_bcm2835_clock_init(dev);
	if (ret < 0) {
		LOG_ERR("CM_PWM clock start failed: %d", ret);
		return ret;
	}

	/* Both channels off until the first pwm_set_cycles(). */
	pwm_reg_write(dev, PWM_CTL, 0);

	LOG_DBG("PWM clock: osc / %u = %u Hz", cfg->divi, cfg->tick_hz);

	return 0;
}

static DEVICE_API(pwm, pwm_bcm2835_api) = {
	.set_cycles = pwm_bcm2835_set_cycles,
	.get_cycles_per_sec = pwm_bcm2835_get_cycles_per_sec,
};

#define PWM_BCM2835_DIVI(n)                                                    \
	DIV_ROUND_CLOSEST(DT_INST_PROP(n, oscillator_frequency),               \
			  DT_INST_PROP(n, clock_frequency))

#define PWM_BCM2835_INIT(n)                                                    \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),                       \
		    (PINCTRL_DT_INST_DEFINE(n);), ())                          \
                                                                               \
	BUILD_ASSERT(PWM_BCM2835_DIVI(n) >= 2 && PWM_BCM2835_DIVI(n) <= 4095,  \
		     "clock-frequency needs an integer oscillator divisor "    \
		     "in 2..4095");                                            \
                                                                               \
	static const struct pwm_bcm2835_config pwm_bcm2835_cfg_##n = {         \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(pwm, DT_DRV_INST(n)),       \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(cm, DT_DRV_INST(n)),        \
		.pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),       \
				    (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),       \
				    (NULL)),                                   \
		.tick_hz = DT_INST_PROP(n, oscillator_frequency) /             \
			   PWM_BCM2835_DIVI(n),                                \
		.divi = PWM_BCM2835_DIVI(n),                                   \
	};                                                                     \
                                                                               \
	static struct pwm_bcm2835_data pwm_bcm2835_data_##n;                   \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, pwm_bcm2835_init, NULL,                       \
			      &pwm_bcm2835_data_##n, &pwm_bcm2835_cfg_##n,     \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,           \
			      &pwm_bcm2835_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_BCM2835_INIT)
