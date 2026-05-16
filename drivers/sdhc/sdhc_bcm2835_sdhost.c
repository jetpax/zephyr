/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 / BCM2710 / BCM2837 legacy SDHost host controller.
 *
 * Second SD controller on the BCM283x SoCs, distinct from the Arasan
 * SDHCI block at 0x3f300000 (sdhc_bcm2835.c). The SDHost peripheral
 * lives at 0x3f202000 and drives the external microSD slot via GPIO
 * 48..53 at ALT0 on Pi 3 / Pi Zero 2 W. Register layout is proprietary
 * Broadcom (SDCMD / SDARG / SDHSTS / SDEDM / SDDATA / SDCDIV at
 * offsets 0x00..0x50), NOT SDHCI-spec.
 *
 * Sub-task B scaffold: register/bit macros for the BCM SDHost set and
 * a soft-reset + safe-boot-state init() that mirrors Linux's
 * bcm2835_reset_internal() -- SDVDD off, SDTOUT/SDHSTS/SDHCFG/SDHBCT/
 * SDHBLC cleared, SDEDM FIFO thresholds set to 4/4 (silicon-bug
 * workaround), 20 ms settle, SDVDD on, slow card-ID clock divider
 * (SDCDIV = MAX -> ~120 kHz from a 250 MHz core_freq). All six
 * sdhc.h callbacks still return -ENOSYS; the command / set_io / data
 * paths arrive in sub-tasks C and D.
 *
 * Linux reference (read for understanding only, NOT lifted):
 *   drivers/mmc/host/bcm2835.c -- in particular bcm2835_reset_internal()
 *   at ~line 241 and the register/bit-field macros at lines 51..139.
 */

#define DT_DRV_COMPAT brcm_bcm2835_sdhost

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sdhc_bcm2835_sdhost, CONFIG_SDHC_LOG_LEVEL);

/* ===== Register offsets ===========================================
 * Names mirror Linux's drivers/mmc/host/bcm2835.c so cross-references
 * read 1:1. All accesses are 32-bit reads/writes.
 */
#define SDCMD	0x00	/* Command to SD card                - 16 R/W */
#define SDARG	0x04	/* Argument to SD card               - 32 R/W */
#define SDTOUT	0x08	/* Start value for timeout counter   - 32 R/W */
#define SDCDIV	0x0c	/* Start value for clock divider     - 11 R/W */
#define SDRSP0	0x10	/* SD card response  (31:0)          - 32 R   */
#define SDRSP1	0x14	/* SD card response  (63:32)         - 32 R   */
#define SDRSP2	0x18	/* SD card response  (95:64)         - 32 R   */
#define SDRSP3	0x1c	/* SD card response (127:96)         - 32 R   */
#define SDHSTS	0x20	/* SD host status                    - 11 R/W */
#define SDVDD	0x30	/* SD card power control             -  1 R/W */
#define SDEDM	0x34	/* Emergency debug mode              - 13 R/W */
#define SDHCFG	0x38	/* Host configuration                -  2 R/W */
#define SDHBCT	0x3c	/* Host byte count (debug)           - 32 R/W */
#define SDDATA	0x40	/* Data to/from SD card FIFO         - 32 R/W */
#define SDHBLC	0x50	/* Host block count (SDIO/SDHC)      -  9 R/W */

/* ===== SDCMD bits (low 16) ======================================== */
#define SDCMD_NEW_FLAG		0x8000	/* arm command */
#define SDCMD_FAIL_FLAG		0x4000	/* error during command */
#define SDCMD_BUSYWAIT		0x800	/* wait for DAT0 deassert after rsp */
#define SDCMD_NO_RESPONSE	0x400
#define SDCMD_LONG_RESPONSE	0x200	/* R2 (136-bit) */
#define SDCMD_WRITE_CMD		0x80	/* data phase, host -> card */
#define SDCMD_READ_CMD		0x40	/* data phase, card -> host */
#define SDCMD_CMD_MASK		0x3f

/* ===== SDCDIV ===================================================== */
#define SDCDIV_MAX_CDIV		0x7ff	/* slowest bus clock = clk_in/(0x7ff+2) */

/* ===== SDHSTS bits ================================================ */
#define SDHSTS_BUSY_IRPT	0x400	/* DAT0 deassert (post-busy R1b/R5b) */
#define SDHSTS_BLOCK_IRPT	0x200	/* block transfer done */
#define SDHSTS_SDIO_IRPT	0x100	/* SDIO async interrupt */
#define SDHSTS_REW_TIME_OUT	0x80	/* read/write timeout */
#define SDHSTS_CMD_TIME_OUT	0x40	/* command response timeout */
#define SDHSTS_CRC16_ERROR	0x20	/* data CRC error */
#define SDHSTS_CRC7_ERROR	0x10	/* response CRC error */
#define SDHSTS_FIFO_ERROR	0x08	/* FIFO over/under-run */
#define SDHSTS_DATA_FLAG	0x01	/* FIFO has data ready (read) or
					 * needs data (write) */
#define SDHSTS_W1C_ALL		0x7f8	/* all IRPT + error bits, W1C */
#define SDHSTS_ERROR_MASK	(SDHSTS_CMD_TIME_OUT | SDHSTS_REW_TIME_OUT | \
				 SDHSTS_CRC7_ERROR | SDHSTS_CRC16_ERROR | \
				 SDHSTS_FIFO_ERROR)

/* ===== SDVDD ====================================================== */
#define SDVDD_POWER_OFF		0
#define SDVDD_POWER_ON		1

/* ===== SDEDM bits ================================================= */
#define SDEDM_FSM_MASK			0xf
#define SDEDM_FSM_IDENTMODE		0x0	/* identification clock active */
#define SDEDM_FSM_DATAMODE		0x1	/* data clock active */
#define SDEDM_FSM_READDATA		0x2
#define SDEDM_FSM_WRITEDATA		0x3
#define SDEDM_FSM_READWAIT		0x4
#define SDEDM_FSM_READCRC		0x5
#define SDEDM_FSM_WRITECRC		0x6
#define SDEDM_FSM_WRITEWAIT1		0x7
#define SDEDM_FSM_POWERDOWN		0x8
#define SDEDM_FSM_POWERUP		0x9
#define SDEDM_FSM_WRITESTART1		0xa
#define SDEDM_FSM_WRITESTART2		0xb
#define SDEDM_FSM_GENPULSES		0xc
#define SDEDM_FSM_WRITEWAIT2		0xd
#define SDEDM_FSM_STARTPOWDOWN		0xf
#define SDEDM_FORCE_DATA_MODE		BIT(19)
#define SDEDM_CLOCK_PULSE		BIT(20)
#define SDEDM_BYPASS			BIT(21)
#define SDEDM_WRITE_THRESHOLD_SHIFT	9
#define SDEDM_READ_THRESHOLD_SHIFT	14
#define SDEDM_THRESHOLD_MASK		0x1f

/* ===== SDHCFG bits ================================================ */
#define SDHCFG_BUSY_IRPT_EN	BIT(10)	/* enable busy-end IRQ to ARM */
#define SDHCFG_BLOCK_IRPT_EN	BIT(8)	/* enable block-done IRQ */
#define SDHCFG_SDIO_IRPT_EN	BIT(5)	/* enable SDIO async IRQ */
#define SDHCFG_DATA_IRPT_EN	BIT(4)	/* enable FIFO-data IRQ */
#define SDHCFG_SLOW_CARD	BIT(3)	/* force ident-clock divisor at all
					 * times (data clock divisor is too
					 * coarse above core_freq=250) */
#define SDHCFG_WIDE_EXT_BUS	BIT(2)	/* 4-bit external SD slot */
#define SDHCFG_WIDE_INT_BUS	BIT(1)	/* 4-bit internal eMMC bus */
#define SDHCFG_REL_CMD_LINE	BIT(0)	/* release CMD line between commands */

/* ===== FIFO / timing constants ==================================== *
 * Linux comment for the FIFO threshold: "Limit fifo usage due to
 * silicon bug" -- read/write trigger at 4 words (out of 16) instead
 * of the spec maximum. Carries through verbatim from bcm2835.c.
 */
#define SDDATA_FIFO_WORDS	16
#define FIFO_READ_THRESHOLD	4
#define FIFO_WRITE_THRESHOLD	4

/* SDTOUT initial value: large enough that no command times out during
 * the slow-clock ID phase. Linux uses the same constant; set_io will
 * later reprogram this to (actual_bus_clock / 2) for a 500 ms timeout.
 */
#define SDTOUT_RESET_VALUE	0x00f00000

/* Power settle: SD spec says wait at least 1 ms after VDD ramp; Linux
 * uses 20 ms. We mirror.
 */
#define SDHOST_POWER_SETTLE_MS	20

struct sdhc_bcm2835_sdhost_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pincfg;
	uint32_t clock_freq;
	uint8_t bus_width;
};

struct sdhc_bcm2835_sdhost_data {
	DEVICE_MMIO_RAM;
	/* Cached hcfg / cdiv so we can recompose SDHCFG / SDCDIV on
	 * partial updates without losing other bits. Filled by init();
	 * later set_io will edit and re-write.
	 */
	uint32_t hcfg;
	uint32_t cdiv;
};

/* ===== Soft reset ==================================================
 * Drives the controller to a known idle state: power off, all counters
 * cleared, FIFO thresholds at the silicon-safe value, then power back
 * on with the slow card-ID divider. Mirrors Linux's
 * bcm2835_reset_internal() at drivers/mmc/host/bcm2835.c:241.
 */
static void sdhost_soft_reset(const struct device *dev)
{
	struct sdhc_bcm2835_sdhost_data *data = dev->data;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t edm;

	sys_write32(SDVDD_POWER_OFF, base + SDVDD);
	sys_write32(0, base + SDCMD);
	sys_write32(0, base + SDARG);
	sys_write32(SDTOUT_RESET_VALUE, base + SDTOUT);
	sys_write32(0, base + SDCDIV);
	sys_write32(SDHSTS_W1C_ALL, base + SDHSTS);	/* W1C clear */
	sys_write32(0, base + SDHCFG);
	sys_write32(0, base + SDHBCT);
	sys_write32(0, base + SDHBLC);

	/* RMW SDEDM: clear current FIFO thresholds, set both to 4 words */
	edm = sys_read32(base + SDEDM);
	edm &= ~((SDEDM_THRESHOLD_MASK << SDEDM_READ_THRESHOLD_SHIFT) |
		 (SDEDM_THRESHOLD_MASK << SDEDM_WRITE_THRESHOLD_SHIFT));
	edm |= (FIFO_READ_THRESHOLD << SDEDM_READ_THRESHOLD_SHIFT) |
	       (FIFO_WRITE_THRESHOLD << SDEDM_WRITE_THRESHOLD_SHIFT);
	sys_write32(edm, base + SDEDM);

	k_msleep(SDHOST_POWER_SETTLE_MS);
	sys_write32(SDVDD_POWER_ON, base + SDVDD);
	k_msleep(SDHOST_POWER_SETTLE_MS);

	/* Initial host state, written through to the controller.
	 *
	 * hcfg = 0: no IRQs enabled (we're polled in v1 -- the IRQ line
	 * isn't even connect()ed yet). REL_CMD_LINE / SLOW_CARD / bus-
	 * width bits are added later by set_io. Linux defaults hcfg to
	 * SDHCFG_BUSY_IRPT_EN because Linux is IRQ-driven; we are not.
	 *
	 * cdiv = MAX (0x7ff): slowest bus clock the divider can produce.
	 * With clock-frequency=250 MHz this gives ~122 kHz, comfortably
	 * below the 400 kHz card-identification spec. set_io will retune
	 * to 400 kHz once the SD subsystem starts probing the card.
	 */
	data->hcfg = 0;
	data->cdiv = SDCDIV_MAX_CDIV;
	sys_write32(data->hcfg, base + SDHCFG);
	sys_write32(data->cdiv, base + SDCDIV);
}

static int sdhc_bcm2835_sdhost_reset(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOSYS;
}

static int sdhc_bcm2835_sdhost_request(const struct device *dev,
				       struct sdhc_command *cmd,
				       struct sdhc_data *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cmd);
	ARG_UNUSED(data);
	return -ENOSYS;
}

static int sdhc_bcm2835_sdhost_set_io(const struct device *dev,
				      struct sdhc_io *ios)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ios);
	return -ENOSYS;
}

static int sdhc_bcm2835_sdhost_get_card_present(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOSYS;
}

static int sdhc_bcm2835_sdhost_card_busy(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOSYS;
}

static int sdhc_bcm2835_sdhost_get_host_props(const struct device *dev,
					      struct sdhc_host_props *props)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(props);
	return -ENOSYS;
}

static int sdhc_bcm2835_sdhost_init(const struct device *dev)
{
	const struct sdhc_bcm2835_sdhost_config *cfg = dev->config;
	uintptr_t base;
	uint32_t edm;
	int ret;

	/* K_MEM_CACHE_NONE (Device-nGnRnE), not K_MEM_ARM_DEVICE_nGnRE.
	 * With 64KB page granule, the SDHost MMIO region at 0x3F202000
	 * falls in the same 64KB page as the pin controller at 0x3F200000
	 * and PL011 at 0x3F201000. Both of those drivers map the page
	 * with K_MEM_CACHE_NONE; Zephyr's arm64 MMU is idempotent on a
	 * remap only when attributes match (arch/arm64/core/mmu.c::
	 * is_desc_superset). Using a different attribute (the Arasan
	 * driver's K_MEM_ARM_DEVICE_nGnRE, which works there because that
	 * controller has its own 64KB page at 0x3F300000) would trip
	 * -EBUSY from __arch_mem_map here.
	 */
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("%s pinctrl apply failed: %d", dev->name, ret);
		return ret;
	}

	sdhost_soft_reset(dev);

	base = DEVICE_MMIO_GET(dev);
	edm = sys_read32(base + SDEDM);
	LOG_INF("%s init ok (clk_in=%u Hz, bus=%u-bit, "
		"SDEDM=0x%08x [FSM=0x%x rd_thr=%u wr_thr=%u])",
		dev->name, cfg->clock_freq, cfg->bus_width, edm,
		edm & SDEDM_FSM_MASK,
		(edm >> SDEDM_READ_THRESHOLD_SHIFT) & SDEDM_THRESHOLD_MASK,
		(edm >> SDEDM_WRITE_THRESHOLD_SHIFT) & SDEDM_THRESHOLD_MASK);
	return 0;
}

static DEVICE_API(sdhc, sdhc_bcm2835_sdhost_api) = {
	.reset = sdhc_bcm2835_sdhost_reset,
	.request = sdhc_bcm2835_sdhost_request,
	.set_io = sdhc_bcm2835_sdhost_set_io,
	.get_card_present = sdhc_bcm2835_sdhost_get_card_present,
	.card_busy = sdhc_bcm2835_sdhost_card_busy,
	.get_host_props = sdhc_bcm2835_sdhost_get_host_props,
};

#define SDHC_BCM2835_SDHOST_INIT(inst)						\
	PINCTRL_DT_INST_DEFINE(inst);						\
	static const struct sdhc_bcm2835_sdhost_config				\
		sdhc_bcm2835_sdhost_cfg_##inst = {				\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.clock_freq = DT_INST_PROP(inst, clock_frequency),		\
		.bus_width = DT_INST_PROP(inst, bus_width),			\
	};									\
	static struct sdhc_bcm2835_sdhost_data sdhc_bcm2835_sdhost_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst,						\
			      sdhc_bcm2835_sdhost_init,				\
			      NULL,						\
			      &sdhc_bcm2835_sdhost_data_##inst,			\
			      &sdhc_bcm2835_sdhost_cfg_##inst,			\
			      POST_KERNEL,					\
			      CONFIG_SDHC_INIT_PRIORITY,			\
			      &sdhc_bcm2835_sdhost_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_BCM2835_SDHOST_INIT)
