/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner H616/H618 CCU clock control, scoped to the clocks the
 * PiZZa bring-up consumes: the SMHC module/bus clocks and the display
 * path (video PLLs, DE, TCON-TV, HDMI). Register truth: Linux
 * drivers/clk/sunxi-ng/ccu-sun50i-h616.c and the H616 user manual.
 *
 * Rate model: OSC24M is the crystal, PLL_PERIPH0 is programmed by the
 * boot chain (SPL) and treated as fixed; its 2x tap is measured at
 * init. The video PLLs are owned by this driver: setting a leaf clock
 * that muxes from pll-video0 (tcon-tv0, hdmi) reprograms the PLL when
 * no exact divider exists at the current rate, mirroring Linux's
 * CLK_SET_RATE_PARENT arrangement. Concurrent leaves of one video PLL
 * must want the same pixel clock, which is what the hardware wires
 * them to anyway (TMDS = dot clock).
 */

#define DT_DRV_COMPAT allwinner_sun50i_h616_ccu

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/sun50i-h616-ccu.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ccu_h616, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define OSC24M_HZ		24000000U

#define PLL_PERIPH0_REG		0x020
#define PLL_VIDEO0_REG		0x040
#define PLL_VIDEO1_REG		0x048

#define PLL_ENABLE		BIT(31)
#define PLL_LOCK_ENABLE		BIT(29)
#define PLL_LOCK		BIT(28)
#define PLL_OUTPUT_ENABLE	BIT(27)
#define PLL_N(reg)		((((reg) >> 8) & 0xff) + 1)
#define PLL_N_FIELD(n)		((((n) - 1) & 0xff) << 8)
#define PLL_INPUT_DIV2		BIT(1)
#define PLL_OUTPUT_DIV2		BIT(0)

#define PLL_N_MIN		12U
#define PLL_N_MAX		255U
#define PLL_VCO_MAX		2400000000ULL
#define PLL_LOCK_TIMEOUT_US	10000

#define MOD_GATE		BIT(31)

/* The video PLL VCO steps in 24 MHz; the fixed /4 post-divider puts
 * the output granularity at 6 MHz.
 */
#define PLL_VIDEO_STEP_HZ	(OSC24M_HZ / 4U)

enum ccu_type {
	T_GATE,
	T_MOD,
	T_PLL,
};

enum ccu_parent {
	P_NONE,
	P_OSC24M,
	P_PERIPH0_2X,
	P_VIDEO0,
	P_VIDEO0_4X,
	P_VIDEO1,
	P_VIDEO1_4X,
};

struct ccu_clk {
	uint8_t id;
	uint8_t type;
	uint16_t reg;
	uint8_t bit;		/* T_GATE: gate bit */
	uint8_t m_width;	/* T_MOD: linear divider, factor = field + 1 */
	uint8_t p_shift;	/* T_MOD: power-of-two divider field */
	uint8_t p_width;	/* 0 = no such field */
	uint8_t mux_shift;
	uint8_t mux_width;
	uint8_t parent[4];	/* enum ccu_parent per mux value */
};

static const struct ccu_clk ccu_clks[] = {
	/* bus gates (reset lives in the same register, bit 16 + n,
	 * handled by the sibling reset controller)
	 */
	{ .id = CLK_BUS_MMC0,     .type = T_GATE, .reg = 0x84c, .bit = 0 },
	{ .id = CLK_BUS_DE,       .type = T_GATE, .reg = 0x60c, .bit = 0 },
	{ .id = CLK_BUS_TCON_TOP, .type = T_GATE, .reg = 0xb5c, .bit = 0 },
	{ .id = CLK_BUS_TCON_TV0, .type = T_GATE, .reg = 0xb9c, .bit = 0 },
	{ .id = CLK_BUS_HDMI,     .type = T_GATE, .reg = 0xb1c, .bit = 0 },
	{ .id = CLK_HDMI_SLOW,    .type = T_GATE, .reg = 0xb04, .bit = 31 },

	/*
	 * mmc0 lists only OSC24M although the hardware muxes the
	 * peripheral PLLs too: the SMHC driver runs the IP's new
	 * timing mode with card clock 1:1 to the module clock, and
	 * PLL sourcing needs sample-delay calibration it doesn't do.
	 */
	{ .id = CLK_MMC0, .type = T_MOD, .reg = 0x830, .m_width = 4,
	  .p_shift = 8, .p_width = 2, .mux_shift = 24, .mux_width = 2,
	  .parent = { P_OSC24M, P_NONE, P_NONE, P_NONE } },

	/* mux 0 is pll-de, not modelled: pll-periph0-2x / 4 = 300 MHz
	 * covers the DE33 without waking another PLL
	 */
	{ .id = CLK_DE, .type = T_MOD, .reg = 0x600, .m_width = 4,
	  .mux_shift = 24, .mux_width = 1,
	  .parent = { P_NONE, P_PERIPH0_2X, P_NONE, P_NONE } },

	{ .id = CLK_TCON_TV0, .type = T_MOD, .reg = 0xb80, .m_width = 4,
	  .p_shift = 8, .p_width = 2, .mux_shift = 24, .mux_width = 3,
	  .parent = { P_VIDEO0, P_VIDEO0_4X, P_VIDEO1, P_VIDEO1_4X } },

	/* mux 2 is pll-video2, absent on the parts we run */
	{ .id = CLK_HDMI, .type = T_MOD, .reg = 0xb00, .m_width = 4,
	  .mux_shift = 24, .mux_width = 2,
	  .parent = { P_VIDEO0, P_VIDEO0_4X, P_NONE, P_NONE } },

	{ .id = CLK_PLL_VIDEO0, .type = T_PLL, .reg = PLL_VIDEO0_REG },
	{ .id = CLK_PLL_VIDEO1, .type = T_PLL, .reg = PLL_VIDEO1_REG },
};

struct ccu_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
	uint32_t periph0_2x_hz;
	uint32_t pll_video_hz[2];
};

static const struct ccu_clk *ccu_find(uint32_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(ccu_clks); i++) {
		if (ccu_clks[i].id == id) {
			return &ccu_clks[i];
		}
	}
	return NULL;
}

static int pll_video_index(const struct ccu_clk *clk)
{
	return (clk->reg == PLL_VIDEO0_REG) ? 0 : 1;
}

static uint32_t parent_rate(struct ccu_data *data, enum ccu_parent p)
{
	switch (p) {
	case P_OSC24M:
		return OSC24M_HZ;
	case P_PERIPH0_2X:
		return data->periph0_2x_hz;
	case P_VIDEO0:
		return data->pll_video_hz[0];
	case P_VIDEO0_4X:
		return data->pll_video_hz[0] * 4U;
	case P_VIDEO1:
		return data->pll_video_hz[1];
	case P_VIDEO1_4X:
		return data->pll_video_hz[1] * 4U;
	default:
		return 0;
	}
}

static int pll_video_wait_lock(mm_reg_t base, uint16_t reg)
{
	for (int us = 0; us < PLL_LOCK_TIMEOUT_US; us += 2) {
		if (sys_read32(base + reg) & PLL_LOCK) {
			return 0;
		}
		k_busy_wait(2);
	}
	return -ETIMEDOUT;
}

static int pll_video_program(const struct device *dev, const struct ccu_clk *clk,
			     uint32_t rate)
{
	struct ccu_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint64_t vco = (uint64_t)rate * 4U;
	uint32_t n = vco / OSC24M_HZ;
	k_spinlock_key_t key;
	uint32_t val;
	int ret;

	if ((rate % PLL_VIDEO_STEP_HZ) != 0 || n < PLL_N_MIN ||
	    n > PLL_N_MAX || vco > PLL_VCO_MAX) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	val = sys_read32(base + clk->reg);
	val &= ~(0xffU << 8);
	val &= ~(PLL_INPUT_DIV2 | PLL_OUTPUT_DIV2);
	val |= PLL_N_FIELD(n);
	val |= PLL_ENABLE | PLL_LOCK_ENABLE | PLL_OUTPUT_ENABLE;
	sys_write32(val, base + clk->reg);
	k_spin_unlock(&data->lock, key);

	ret = pll_video_wait_lock(base, clk->reg);
	if (ret) {
		LOG_ERR("pll-video%d lock timeout (N=%u)",
			pll_video_index(clk), n);
		return ret;
	}

	data->pll_video_hz[pll_video_index(clk)] = rate;
	LOG_DBG("pll-video%d = %u Hz (N=%u)", pll_video_index(clk), rate, n);
	return 0;
}

/*
 * Best divider of parent_hz not above target: linear m in 1..2^mw,
 * power-of-two 2^p with p in 0..2^pw-1 (pw 0 = no p field). Iteration
 * order (p outer, m inner) makes ties land on the smallest p, which
 * reproduces the divider choices the SMHC driver shipped with.
 */
static uint32_t mod_best_div(uint32_t parent_hz, uint32_t target,
			     uint8_t mw, uint8_t pw,
			     uint32_t *best_m, uint32_t *best_p)
{
	uint32_t best = 0;

	for (uint32_t p = 0; p < BIT(pw ? pw : 1); p++) {
		if (pw == 0 && p > 0) {
			break;
		}
		for (uint32_t m = 1; m <= BIT(mw); m++) {
			uint32_t rate = (parent_hz >> p) / m;

			if (rate <= target && rate > best) {
				best = rate;
				*best_m = m;
				*best_p = p;
			}
		}
	}
	return best;
}

static int mod_set_rate(const struct device *dev, const struct ccu_clk *clk,
			uint32_t rate)
{
	struct ccu_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint32_t best = 0, best_m = 1, best_p = 0, best_mux = 0;
	k_spinlock_key_t key;
	uint32_t val;

	for (uint32_t mux = 0; mux < BIT(clk->mux_width); mux++) {
		uint32_t phz = parent_rate(data, clk->parent[mux]);
		uint32_t m, p, got;

		if (phz == 0) {
			continue;
		}
		got = mod_best_div(phz, rate, clk->m_width, clk->p_width,
				   &m, &p);
		if (got > best) {
			best = got;
			best_m = m;
			best_p = p;
			best_mux = mux;
		}
		if (best == rate) {
			break;
		}
	}

	/*
	 * No exact hit from the parents as they run now: if a video
	 * PLL feeds this clock, plan the PLL rate itself. Prefer the
	 * 1x tap; the 4x tap covers targets whose x1 PLL rate would
	 * fall below the PLL floor.
	 */
	if (best != rate) {
		for (uint32_t mux = 0; mux < BIT(clk->mux_width); mux++) {
			enum ccu_parent par = clk->parent[mux];
			uint32_t mult;

			if (par == P_VIDEO0 || par == P_VIDEO1) {
				mult = 1;
			} else if (par == P_VIDEO0_4X || par == P_VIDEO1_4X) {
				mult = 4;
			} else {
				continue;
			}

			for (uint32_t p = 0; p < BIT(clk->p_width ? clk->p_width : 1); p++) {
				for (uint32_t m = 1; m <= BIT(clk->m_width); m++) {
					uint64_t leaf = (uint64_t)rate * m << p;
					uint64_t pll = leaf / mult;
					const struct ccu_clk *pc;

					if (leaf % mult ||
					    pll % PLL_VIDEO_STEP_HZ ||
					    pll * 4 / OSC24M_HZ < PLL_N_MIN ||
					    pll * 4 > PLL_VCO_MAX) {
						continue;
					}
					pc = ccu_find((par == P_VIDEO0 ||
						       par == P_VIDEO0_4X) ?
						      CLK_PLL_VIDEO0 :
						      CLK_PLL_VIDEO1);
					if (pll_video_program(dev, pc, pll)) {
						continue;
					}
					best = rate;
					best_m = m;
					best_p = p;
					best_mux = mux;
					goto programmed;
				}
				if (clk->p_width == 0) {
					break;
				}
			}
		}
	}
programmed:

	if (best == 0) {
		return -ENOTSUP;
	}

	/* Full-register write, gate on: matches the sequence the SMHC
	 * driver shipped with (configure and enable in one store).
	 */
	val = MOD_GATE | (best_mux << clk->mux_shift) | (best_m - 1);
	if (clk->p_width) {
		val |= best_p << clk->p_shift;
	}
	key = k_spin_lock(&data->lock);
	sys_write32(val, base + clk->reg);
	k_spin_unlock(&data->lock, key);

	LOG_DBG("clk %u: %u Hz (mux=%u m=%u p=%u)", clk->id, best,
		best_mux, best_m, best_p);
	return 0;
}

static int ccu_h616_on_off(const struct device *dev, clock_control_subsys_t sys,
			   bool on)
{
	struct ccu_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	const struct ccu_clk *clk = ccu_find((uint32_t)(uintptr_t)sys);
	k_spinlock_key_t key;
	uint32_t bit, val;

	if (clk == NULL) {
		return -ENOTSUP;
	}

	switch (clk->type) {
	case T_GATE:
		bit = BIT(clk->bit);
		break;
	case T_MOD:
		bit = MOD_GATE;
		break;
	case T_PLL:
		bit = PLL_ENABLE;
		break;
	default:
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	val = sys_read32(base + clk->reg);
	if (clk->type == T_PLL && on) {
		val |= PLL_LOCK_ENABLE | PLL_OUTPUT_ENABLE;
		val &= ~PLL_OUTPUT_DIV2;
	}
	val = on ? (val | bit) : (val & ~bit);
	sys_write32(val, base + clk->reg);
	k_spin_unlock(&data->lock, key);

	if (clk->type == T_PLL && on) {
		return pll_video_wait_lock(base, clk->reg);
	}
	return 0;
}

static int ccu_h616_on(const struct device *dev, clock_control_subsys_t sys)
{
	return ccu_h616_on_off(dev, sys, true);
}

static int ccu_h616_off(const struct device *dev, clock_control_subsys_t sys)
{
	return ccu_h616_on_off(dev, sys, false);
}

static int ccu_h616_get_rate(const struct device *dev,
			     clock_control_subsys_t sys, uint32_t *rate)
{
	struct ccu_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	const struct ccu_clk *clk = ccu_find((uint32_t)(uintptr_t)sys);
	uint32_t val, phz;

	if (clk == NULL) {
		return -ENOTSUP;
	}

	switch (clk->type) {
	case T_PLL:
		val = sys_read32(base + clk->reg);
		if (!(val & PLL_ENABLE)) {
			*rate = 0;
			return 0;
		}
		phz = OSC24M_HZ / ((val & PLL_INPUT_DIV2) ? 2 : 1);
		*rate = (uint32_t)(((uint64_t)phz * PLL_N(val)) / 4U);
		return 0;
	case T_MOD: {
		uint32_t mux, m, p;

		val = sys_read32(base + clk->reg);
		mux = (val >> clk->mux_shift) & (BIT(clk->mux_width) - 1);
		m = (val & (BIT(clk->m_width) - 1)) + 1;
		p = clk->p_width ?
			((val >> clk->p_shift) & (BIT(clk->p_width) - 1)) : 0;
		phz = parent_rate(data, clk->parent[mux]);
		if (phz == 0) {
			return -ENOTSUP;
		}
		*rate = (phz >> p) / m;
		return 0;
	}
	default:
		return -ENOTSUP;
	}
}

static int ccu_h616_set_rate(const struct device *dev,
			     clock_control_subsys_t sys,
			     clock_control_subsys_rate_t sys_rate)
{
	const struct ccu_clk *clk = ccu_find((uint32_t)(uintptr_t)sys);
	uint32_t rate = (uint32_t)(uintptr_t)sys_rate;

	if (clk == NULL) {
		return -ENOTSUP;
	}

	switch (clk->type) {
	case T_PLL:
		return pll_video_program(dev, clk, rate);
	case T_MOD:
		return mod_set_rate(dev, clk, rate);
	default:
		return -ENOTSUP;
	}
}

static enum clock_control_status ccu_h616_get_status(const struct device *dev,
						     clock_control_subsys_t sys)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	const struct ccu_clk *clk = ccu_find((uint32_t)(uintptr_t)sys);
	uint32_t bit;

	if (clk == NULL) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
	bit = (clk->type == T_GATE) ? BIT(clk->bit) :
	      (clk->type == T_MOD) ? MOD_GATE : PLL_ENABLE;

	return (sys_read32(base + clk->reg) & bit) ?
		CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static int ccu_h616_init(const struct device *dev)
{
	struct ccu_data *data = dev->data;
	mm_reg_t base;
	uint32_t val, n, in_div, out_div;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	base = DEVICE_MMIO_GET(dev);

	/* pll-periph0 belongs to the boot chain; measure its 2x tap
	 * (= VCO: the fixed /2 post-divider and the 2x tap cancel)
	 */
	val = sys_read32(base + PLL_PERIPH0_REG);
	n = PLL_N(val);
	in_div = (val & PLL_INPUT_DIV2) ? 2 : 1;
	out_div = (val & PLL_OUTPUT_DIV2) ? 2 : 1;
	if (val & PLL_ENABLE) {
		data->periph0_2x_hz = OSC24M_HZ / in_div / out_div * n;
	}
	if (data->periph0_2x_hz != 1200000000U) {
		LOG_WRN("pll-periph0-2x at %u Hz, expected 1.2 GHz",
			data->periph0_2x_hz);
	}

	/* Cache video PLL rates a previous boot stage may have left
	 * running; a FEL/watchdog entry leaves them at reset (off).
	 */
	for (int i = 0; i < 2; i++) {
		uint16_t reg = i ? PLL_VIDEO1_REG : PLL_VIDEO0_REG;

		val = sys_read32(base + reg);
		if ((val & PLL_ENABLE) && (val & PLL_LOCK)) {
			uint32_t phz = OSC24M_HZ /
				((val & PLL_INPUT_DIV2) ? 2 : 1);

			data->pll_video_hz[i] =
				(uint32_t)(((uint64_t)phz * PLL_N(val)) / 4U);
		}
	}

	return 0;
}

static DEVICE_API(clock_control, ccu_h616_api) = {
	.on = ccu_h616_on,
	.off = ccu_h616_off,
	.get_rate = ccu_h616_get_rate,
	.set_rate = ccu_h616_set_rate,
	.get_status = ccu_h616_get_status,
};

#define CCU_H616_INIT(inst)						\
	static struct ccu_data ccu_data_##inst;				\
	static const struct {						\
		DEVICE_MMIO_ROM;					\
	} ccu_config_##inst = {						\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, ccu_h616_init, NULL,		\
			      &ccu_data_##inst, &ccu_config_##inst,	\
			      PRE_KERNEL_1,				\
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,	\
			      &ccu_h616_api);

DT_INST_FOREACH_STATUS_OKAY(CCU_H616_INIT)
