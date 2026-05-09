/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 / BCM2710 / BCM2837 Arasan SDHCI host controller.
 *
 * Skeleton -- only the device-instance plumbing and a one-shot probe at
 * init are wired up. All six sdhc.h API methods stub to -ENOTSUP. The
 * register sequencer (command/data PIO) lands in subsequent commits as
 * the SDIO bring-up progresses.
 *
 * Two silicon facts shape the future driver:
 *
 *   - 32-bit-only register access. Every read/write must be aligned to
 *     a 32-bit boundary at 32-bit width; the controller silently corrupts
 *     or hangs on sub-word accesses. SDHCI's 8/16-bit registers are
 *     synthesised via RMW on the containing 32-bit word.
 *
 *   - Capabilities register reads as zero. The driver hardcodes the
 *     real values per the Linux sdhci-iproc bcm2835 variant data:
 *     max-block 1024, 3.3V VDD, high-speed support, driver type A/C.
 *
 * References (open as you read this file):
 *   - Linux: drivers/mmc/host/sdhci-iproc.c    -- our quirk source
 *   - Linux: drivers/mmc/host/sdhci.c          -- protocol core
 *   - BCM2835 ARM Peripherals datasheet, ch. 5 (External Mass Media
 *     Controller). The datasheet hand-waves to the Arasan
 *     SD3.0_Host_AHB_eMMC4.4 Users Guide for individual bit fields;
 *     for those, use the SD Host Controller Standard Specification 3.0
 *     (sdcard.org).
 */

#define DT_DRV_COMPAT brcm_bcm2835_sdhci

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(sdhc_bcm2835, CONFIG_SDHC_LOG_LEVEL);

/* Standard SDHCI register offsets we use. Names mirror Linux's
 * include/linux/mmc/sdhci.h so cross-references read 1:1. The BCM
 * datasheet packages some of these as 32-bit aggregates (e.g. CONTROL1
 * = CLOCK_CONTROL + TIMEOUT + SOFTWARE_RESET); we always touch the
 * containing 32-bit word.
 */
#define SDHCI_CLOCK_CONTROL		0x2C	/* low 32 of CONTROL1 word */
#define SDHCI_SLOT_INT_STATUS_VERSION	0xFC

/* CONTROL1 (32-bit at 0x2C) -- standard SDHCI clock + timeout + reset */
#define SDHCI_CTRL1_RESET_ALL		BIT(24)	/* SRST_HC: reset whole HC */
#define SDHCI_CTRL1_RESET_CMD		BIT(25)	/* SRST_CMD: cmd line reset */
#define SDHCI_CTRL1_RESET_DATA		BIT(26)	/* SRST_DATA: data line reset */

/* Software reset timeout. Linux's sdhci.c defaults to 100 ms here and
 * that's plenty for any sane controller -- the bit self-clears in
 * microseconds on healthy silicon.
 */
#define BCM2835_SDHCI_RESET_TIMEOUT_MS	100

/* Hardcoded host capabilities -- see sdhci-iproc.c::bcm2835_data.
 * The silicon's CAPABILITIES register at 0x40 reads as zero on this
 * controller, so we expose these constants from get_host_props().
 */
#define BCM2835_MAX_BLOCK_BYTES		1024	/* internal FIFO size */
#define BCM2835_F_MIN_HZ		400000	/* card identification */
#define BCM2835_F_MAX_HZ		50000000 /* SDR25 / high speed */

struct sdhc_bcm2835_config {
	DEVICE_MMIO_ROM;
	uint32_t clock_freq;
	uint8_t bus_width;
};

struct sdhc_bcm2835_data {
	DEVICE_MMIO_RAM;
	struct sdhc_io host_io;
};

static int sdhc_bcm2835_request(const struct device *dev,
				struct sdhc_command *cmd,
				struct sdhc_data *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cmd);
	ARG_UNUSED(data);
	return -ENOTSUP;
}

static int sdhc_bcm2835_set_io(const struct device *dev, struct sdhc_io *ios)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ios);
	return -ENOTSUP;
}

static int sdhc_bcm2835_get_card_present(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* On Pi Zero 2 W the EMMC port is hardwired to the CYW43439; the
	 * silicon has no card-detect line (SDHCI_QUIRK_BROKEN_CARD_DETECTION).
	 * From the SDIO core's view the chip is always present.
	 */
	return 1;
}

static int sdhc_bcm2835_card_busy(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int sdhc_bcm2835_get_host_props(const struct device *dev,
				       struct sdhc_host_props *props)
{
	const struct sdhc_bcm2835_config *cfg = dev->config;

	/* These mirror the hardcoded bcm2835 caps in Linux's
	 * sdhci-iproc.c::bcm2835_data (caps + caps1). The silicon's
	 * CAPABILITIES register reads as zero on this controller.
	 */
	memset(props, 0, sizeof(*props));
	props->f_min = BCM2835_F_MIN_HZ;
	props->f_max = BCM2835_F_MAX_HZ;
	props->host_caps.max_blk_len = 1;	/* 1 = 1024 bytes (FIFO size) */
	props->host_caps.high_spd_support = true;
	props->host_caps.vol_330_support = true;
	props->host_caps.drv_type_a_support = true;
	props->host_caps.drv_type_c_support = true;
	props->bus_4_bit_support = (cfg->bus_width == 4);
	props->is_spi = false;
	return 0;
}

/* Software reset of one or more controller domains. mask is any
 * combination of SDHCI_CTRL1_RESET_{ALL,CMD,DATA}. Writes the bit(s)
 * into CONTROL1 and polls until the silicon clears them. The reset
 * bit is self-clearing per spec; we time out in case the silicon
 * disagrees rather than spinning forever.
 *
 * We always RMW the full 32-bit CONTROL1 word -- per the BCM datasheet
 * the EMMC accepts only 32-bit aligned 32-bit accesses, and CONTROL1
 * has clock-control fields below the reset bits that we mustn't
 * disturb.
 */
static int sdhc_bcm2835_soft_reset(const struct device *dev, uint32_t mask)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t ctrl1;
	int64_t deadline = k_uptime_get() + BCM2835_SDHCI_RESET_TIMEOUT_MS;

	ctrl1 = sys_read32(base + SDHCI_CLOCK_CONTROL);
	sys_write32(ctrl1 | mask, base + SDHCI_CLOCK_CONTROL);

	while (sys_read32(base + SDHCI_CLOCK_CONTROL) & mask) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}

	return 0;
}

static int sdhc_bcm2835_reset(const struct device *dev)
{
	return sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_ALL);
}

static int sdhc_bcm2835_init(const struct device *dev)
{
	const struct sdhc_bcm2835_config *cfg = dev->config;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t slot_isr_ver = sys_read32(base + SDHCI_SLOT_INT_STATUS_VERSION);
	uint16_t version = (uint16_t)(slot_isr_ver >> 16);

	ret = sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_ALL);
	if (ret != 0) {
		printk("sdhc_bcm2835: %s reset timeout\n", dev->name);
		LOG_ERR("%s reset timeout", dev->name);
		return ret;
	}

	/* Temporary printk so the bring-up's "I'm here" line is visible in
	 * MP-style images that build without CONFIG_LOG. Drop once the real
	 * driver has its own LOG_* output worth seeing. */
	printk("sdhc_bcm2835: %s @ 0x%lx, host version 0x%04x, clk_emmc %u Hz, %u-bit, reset OK\n",
	       dev->name, (unsigned long)base, version, cfg->clock_freq, cfg->bus_width);

	LOG_INF("%s @ 0x%lx, host version 0x%04x, clk_emmc %u Hz, %u-bit, reset OK",
		dev->name, (unsigned long)base, version, cfg->clock_freq, cfg->bus_width);

	return 0;
}

static DEVICE_API(sdhc, sdhc_bcm2835_api) = {
	.request = sdhc_bcm2835_request,
	.set_io = sdhc_bcm2835_set_io,
	.get_host_props = sdhc_bcm2835_get_host_props,
	.get_card_present = sdhc_bcm2835_get_card_present,
	.reset = sdhc_bcm2835_reset,
	.card_busy = sdhc_bcm2835_card_busy,
};

#define SDHC_BCM2835_INIT(inst)							\
	static const struct sdhc_bcm2835_config sdhc_bcm2835_cfg_##inst = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),			\
		.clock_freq = DT_INST_PROP(inst, clock_frequency),		\
		.bus_width  = DT_INST_PROP(inst, bus_width),			\
	};									\
	static struct sdhc_bcm2835_data sdhc_bcm2835_data_##inst;		\
	DEVICE_DT_INST_DEFINE(inst,						\
			      sdhc_bcm2835_init,				\
			      NULL,						\
			      &sdhc_bcm2835_data_##inst,			\
			      &sdhc_bcm2835_cfg_##inst,				\
			      POST_KERNEL,					\
			      CONFIG_SDHC_INIT_PRIORITY,			\
			      &sdhc_bcm2835_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_BCM2835_INIT)
