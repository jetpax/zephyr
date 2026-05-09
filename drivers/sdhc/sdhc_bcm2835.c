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
#define SDHCI_ARG1			0x08	/* command argument (32-bit) */
#define SDHCI_CMDTM			0x0C	/* TRANSFER_MODE | COMMAND<<16 */
#define SDHCI_RESPONSE			0x10	/* RESP0..RESP3 (4x 32-bit) */
#define SDHCI_PRESENT_STATE		0x24	/* "STATUS" in BCM doc */
#define SDHCI_HOST_CONTROL		0x28	/* CONTROL0 word: HCTL+POWER+... */
#define SDHCI_CLOCK_CONTROL		0x2C	/* CONTROL1 word: CLK+TOUT+RESET */
#define SDHCI_INT_STATUS		0x30	/* INTERRUPT (W1C) */
#define SDHCI_INT_ENABLE		0x34	/* IRPT_MASK */
#define SDHCI_SIGNAL_ENABLE		0x38	/* IRPT_EN */
#define SDHCI_SLOT_INT_STATUS_VERSION	0xFC

/* PRESENT_STATE bits (we only need the inhibit flags) */
#define SDHCI_PSTATE_CMD_INHIBIT	BIT(0)	/* CMD line busy */
#define SDHCI_PSTATE_DATA_INHIBIT	BIT(1)	/* DAT lines busy */

/* CMDTM (32-bit at 0x0C). The low 16 bits are TRANSFER_MODE, high 16 bits
 * are COMMAND. Per spec, writing the COMMAND half (= writing the 32-bit
 * word) is what fires the command on the CMD line.
 */
#define SDHCI_CMDTM_TM_BLKCNT_EN	BIT(1)	/* enable block count for data */
#define SDHCI_CMDTM_TM_AUTO_CMD12	BIT(2)
#define SDHCI_CMDTM_TM_AUTO_CMD23	BIT(3)
#define SDHCI_CMDTM_TM_DAT_DIR_READ	BIT(4)	/* 1 = card->host */
#define SDHCI_CMDTM_TM_MULTI_BLOCK	BIT(5)
#define SDHCI_CMDTM_RSP_NONE		(0 << 16)
#define SDHCI_CMDTM_RSP_136		(1 << 16)	/* long, R2 */
#define SDHCI_CMDTM_RSP_48		(2 << 16)	/* short, R1/R3/R5/R6/R7 */
#define SDHCI_CMDTM_RSP_48_BUSY		(3 << 16)	/* short with busy, R1b */
#define SDHCI_CMDTM_CRC_CHECK		BIT(16 + 3)	/* check response CRC */
#define SDHCI_CMDTM_INDEX_CHECK		BIT(16 + 4)	/* check response index */
#define SDHCI_CMDTM_DATA_PRESENT	BIT(16 + 5)	/* command has data phase */
#define SDHCI_CMDTM_TYPE_ABORT		(3 << (16 + 6))
#define SDHCI_CMDTM_INDEX_SHIFT		(16 + 8)

/* INTERRUPT (32-bit at 0x30): low 16 = NORMAL, high 16 = ERROR. W1C. */
#define SDHCI_INT_CMD_COMPLETE		BIT(0)
#define SDHCI_INT_DATA_END		BIT(1)
#define SDHCI_INT_BUF_WRITE_READY	BIT(4)
#define SDHCI_INT_BUF_READ_READY	BIT(5)
#define SDHCI_INT_CARD_INSERT		BIT(6)
#define SDHCI_INT_CARD_REMOVE		BIT(7)
#define SDHCI_INT_CARD_INT		BIT(8)	/* SDIO async event */
#define SDHCI_INT_ERROR			BIT(15)
#define SDHCI_INT_CMD_TIMEOUT		BIT(16)
#define SDHCI_INT_CMD_CRC		BIT(17)
#define SDHCI_INT_CMD_END_BIT		BIT(18)
#define SDHCI_INT_CMD_INDEX		BIT(19)
#define SDHCI_INT_DATA_TIMEOUT		BIT(20)
#define SDHCI_INT_DATA_CRC		BIT(21)
#define SDHCI_INT_DATA_END_BIT		BIT(22)

#define SDHCI_INT_CMD_ERROR_MASK	(SDHCI_INT_CMD_TIMEOUT | \
					 SDHCI_INT_CMD_CRC     | \
					 SDHCI_INT_CMD_END_BIT | \
					 SDHCI_INT_CMD_INDEX)
#define SDHCI_INT_DATA_ERROR_MASK	(SDHCI_INT_DATA_TIMEOUT | \
					 SDHCI_INT_DATA_CRC     | \
					 SDHCI_INT_DATA_END_BIT)
#define SDHCI_INT_ALL_NORMAL		0x0000FFFFU
#define SDHCI_INT_ALL_ERROR		0xFFFF0000U
#define SDHCI_INT_ALL_W1C		(SDHCI_INT_ALL_NORMAL | SDHCI_INT_ALL_ERROR)

/* CONTROL0 (32-bit at 0x28) -- HCTL[7:0] + POWER[15:8] + BLOCK_GAP[23:16]
 * + WAKEUP[31:24]. We touch HCTL bits and the POWER subregister.
 */
#define SDHCI_CTRL0_HCTL_DWIDTH		BIT(1)	/* 4-bit data bus */
#define SDHCI_CTRL0_HCTL_HS		BIT(2)	/* high-speed mode (NO_HISPD_BIT
						 * quirk: silicon ignores this --
						 * speed is set via clock divider) */
#define SDHCI_CTRL0_HCTL_8BIT		BIT(5)	/* not supported on this silicon */
#define SDHCI_CTRL0_POWER_ON		BIT(8)	/* SD bus power enable */
#define SDHCI_CTRL0_VOLT_SHIFT		9
#define SDHCI_CTRL0_VOLT_MASK		(0x7 << SDHCI_CTRL0_VOLT_SHIFT)
#define SDHCI_CTRL0_VOLT_330		(0x7 << SDHCI_CTRL0_VOLT_SHIFT)

/* CONTROL1 (32-bit at 0x2C) -- standard SDHCI clock + timeout + reset */
#define SDHCI_CTRL1_CLK_INTLEN		BIT(0)	/* internal clock enable */
#define SDHCI_CTRL1_CLK_STABLE		BIT(1)	/* internal clock stable (RO) */
#define SDHCI_CTRL1_CLK_EN		BIT(2)	/* SD bus clock enable */
#define SDHCI_CTRL1_CLK_GENSEL		BIT(5)	/* 0 = divided clock mode */
#define SDHCI_CTRL1_CLK_FREQ_MS_SHIFT	6	/* upper 2 bits of 10-bit div */
#define SDHCI_CTRL1_CLK_FREQ_MS_MASK	(0x3 << SDHCI_CTRL1_CLK_FREQ_MS_SHIFT)
#define SDHCI_CTRL1_CLK_FREQ_LO_SHIFT	8	/* lower 8 bits of 10-bit div */
#define SDHCI_CTRL1_CLK_FREQ_LO_MASK	(0xFF << SDHCI_CTRL1_CLK_FREQ_LO_SHIFT)
#define SDHCI_CTRL1_CLK_FREQ_MASK	(SDHCI_CTRL1_CLK_FREQ_MS_MASK | \
					 SDHCI_CTRL1_CLK_FREQ_LO_MASK)
#define SDHCI_CTRL1_DATA_TOUT_SHIFT	16	/* timeout exponent */
#define SDHCI_CTRL1_DATA_TOUT_MASK	(0xF << SDHCI_CTRL1_DATA_TOUT_SHIFT)
#define SDHCI_CTRL1_RESET_ALL		BIT(24)	/* SRST_HC: reset whole HC */
#define SDHCI_CTRL1_RESET_CMD		BIT(25)	/* SRST_CMD: cmd line reset */
#define SDHCI_CTRL1_RESET_DATA		BIT(26)	/* SRST_DATA: data line reset */

/* Software reset timeout. Linux's sdhci.c defaults to 100 ms here and
 * that's plenty for any sane controller -- the bit self-clears in
 * microseconds on healthy silicon. Same ceiling for clock-stable.
 */
#define BCM2835_SDHCI_RESET_TIMEOUT_MS	100
#define BCM2835_SDHCI_CLK_STABLE_MS	100

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

/* Forward decl: error recovery in request() needs to reset the cmd line. */
static int sdhc_bcm2835_soft_reset(const struct device *dev, uint32_t mask);

/* Default command timeout when the caller leaves cmd->timeout_ms zero.
 * Linux uses 10s for non-data commands, but for SDIO control we expect
 * sub-millisecond turnarounds; 1s is plenty and short enough to surface
 * a hung controller during bring-up.
 */
#define BCM2835_SDHCI_CMD_TIMEOUT_MS	1000

/* Spin until the requested PRESENT_STATE.*_INHIBIT bits clear, with a
 * timeout. The card / controller has to actually finish whatever it was
 * doing before we can issue a new command.
 */
static int sdhc_bcm2835_wait_inhibit(const struct device *dev, uint32_t mask,
				     int timeout_ms)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (sys_read32(base + SDHCI_PRESENT_STATE) & mask) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}
	return 0;
}

/* Spin until INT_STATUS has either a success bit or an error bit set,
 * or we time out. Returns the raw int-status value (caller decides).
 * Does not ack -- the caller W1Cs the bits it actually consumed so a
 * pending unrelated interrupt isn't dropped on the floor.
 */
static uint32_t sdhc_bcm2835_wait_int(const struct device *dev,
				      uint32_t success_mask, uint32_t error_mask,
				      int timeout_ms)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	int64_t deadline = k_uptime_get() + timeout_ms;
	uint32_t status;

	while (true) {
		status = sys_read32(base + SDHCI_INT_STATUS);
		if (status & (success_mask | error_mask)) {
			return status;
		}
		if (k_uptime_get() > deadline) {
			return 0;	/* caller treats 0 as timeout */
		}
		k_busy_wait(10);
	}
}

/* Build the upper 16 bits of CMDTM (the COMMAND half) from a Zephyr
 * sdhc_command. Lower 16 (TRANSFER_MODE) is set elsewhere when data is
 * involved; for cmd-only requests it stays zero.
 *
 * We deliberately don't support R1b yet: the controller waits for DAT0
 * to clear after a busy response, which can hang indefinitely if the
 * card never deasserts busy. Once data-phase + interrupt-driven
 * completion are in, R1b becomes a small extension.
 */
static int sdhc_bcm2835_build_cmd(const struct sdhc_command *cmd, uint32_t *cmdtm)
{
	uint32_t flags = (uint32_t)cmd->opcode << SDHCI_CMDTM_INDEX_SHIFT;
	uint32_t rsp = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	switch (rsp) {
	case SD_RSP_TYPE_NONE:
		flags |= SDHCI_CMDTM_RSP_NONE;
		break;
	case SD_RSP_TYPE_R2:
		flags |= SDHCI_CMDTM_RSP_136 | SDHCI_CMDTM_CRC_CHECK;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		/* OCR / IO_SEND_OP_COND -- no CRC check, no index check */
		flags |= SDHCI_CMDTM_RSP_48;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		flags |= SDHCI_CMDTM_RSP_48 | SDHCI_CMDTM_CRC_CHECK |
			 SDHCI_CMDTM_INDEX_CHECK;
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		/* TODO: R1b/R5b need busy-wait on DAT0; not handled yet. */
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	*cmdtm = flags;
	return 0;
}

/* Copy the response register(s) into cmd->response[]. R2 (long form,
 * 136-bit CID/CSD) populates RESP0..RESP3 with the CRC/start bits
 * stripped; everyone else uses RESP0 alone.
 */
static void sdhc_bcm2835_read_response(const struct device *dev,
				       struct sdhc_command *cmd)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t rsp = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	cmd->response[0] = sys_read32(base + SDHCI_RESPONSE + 0);
	if (rsp == SD_RSP_TYPE_R2) {
		cmd->response[1] = sys_read32(base + SDHCI_RESPONSE + 4);
		cmd->response[2] = sys_read32(base + SDHCI_RESPONSE + 8);
		cmd->response[3] = sys_read32(base + SDHCI_RESPONSE + 12);
	}
}

static int sdhc_bcm2835_request(const struct device *dev,
				struct sdhc_command *cmd,
				struct sdhc_data *data)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t cmdtm;
	uint32_t int_status;
	int timeout_ms;
	int ret;

	if (cmd == NULL) {
		return -EINVAL;
	}
	if (data != NULL) {
		/* Data phase lands in a follow-up commit. */
		return -ENOTSUP;
	}

	ret = sdhc_bcm2835_build_cmd(cmd, &cmdtm);
	if (ret != 0) {
		return ret;
	}

	timeout_ms = cmd->timeout_ms ? cmd->timeout_ms : BCM2835_SDHCI_CMD_TIMEOUT_MS;

	ret = sdhc_bcm2835_wait_inhibit(dev, SDHCI_PSTATE_CMD_INHIBIT, timeout_ms);
	if (ret != 0) {
		return ret;
	}

	/* Clear any stale CMD-side interrupt bits so wait_int doesn't latch
	 * onto a previous command's completion.
	 */
	sys_write32(SDHCI_INT_CMD_COMPLETE | SDHCI_INT_CMD_ERROR_MASK,
		    base + SDHCI_INT_STATUS);

	sys_write32(cmd->arg, base + SDHCI_ARG1);
	sys_write32(cmdtm, base + SDHCI_CMDTM);	/* fires command */

	int_status = sdhc_bcm2835_wait_int(dev, SDHCI_INT_CMD_COMPLETE,
					   SDHCI_INT_CMD_ERROR_MASK, timeout_ms);

	if (int_status == 0) {
		return -ETIMEDOUT;
	}

	if (int_status & SDHCI_INT_CMD_ERROR_MASK) {
		/* Ack the error bits we observed; soft-reset the cmd line so
		 * the controller is ready for the next attempt.
		 */
		sys_write32(int_status & SDHCI_INT_CMD_ERROR_MASK,
			    base + SDHCI_INT_STATUS);
		(void)sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_CMD);
		if (int_status & SDHCI_INT_CMD_TIMEOUT) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	sdhc_bcm2835_read_response(dev, cmd);
	sys_write32(SDHCI_INT_CMD_COMPLETE, base + SDHCI_INT_STATUS);

	return 0;
}

/* Compute the 10-bit "divided clock mode" divider that yields the
 * largest SD bus clock <= target_hz, given clk_emmc as the input. The
 * SDHCI v3 spec defines the divisor as 2 * V where V is the 10-bit
 * value programmed into CLK_FREQ; V=0 means 1:1 (max base clock).
 *
 * V = ceil(clk_emmc / (2 * target_hz))
 *
 * Linux's sdhci.c does the same calculation (see __sdhci_calc_clock).
 * We ceil-divide so we never run the bus *above* target_hz.
 */
static uint32_t sdhc_bcm2835_calc_clk_div(uint32_t clk_in, uint32_t target_hz)
{
	uint32_t div;

	if (target_hz == 0 || target_hz >= clk_in) {
		return 0;	/* V=0 => bus = clk_in */
	}

	div = (clk_in + (target_hz * 2) - 1) / (target_hz * 2);
	if (div > 0x3FF) {
		div = 0x3FF;	/* clamp to 10-bit */
	}
	return div;
}

static int sdhc_bcm2835_set_clock(const struct device *dev, uint32_t target_hz)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	const struct sdhc_bcm2835_config *cfg = dev->config;
	uint32_t ctrl1;
	uint32_t div;
	int64_t deadline;

	/* Tear down the current bus clock + internal clock so we can
	 * reprogram the divider. SDHCI spec requires SDCE=0 before changing
	 * the divider; we also drop ICE so the controller observes the new
	 * value cleanly.
	 */
	ctrl1 = sys_read32(base + SDHCI_CLOCK_CONTROL);
	ctrl1 &= ~(SDHCI_CTRL1_CLK_EN | SDHCI_CTRL1_CLK_INTLEN |
		   SDHCI_CTRL1_CLK_FREQ_MASK | SDHCI_CTRL1_CLK_GENSEL |
		   SDHCI_CTRL1_DATA_TOUT_MASK);
	sys_write32(ctrl1, base + SDHCI_CLOCK_CONTROL);

	if (target_hz == 0) {
		return 0;	/* caller wants the clock gated */
	}

	/* Program the 10-bit divider in divided-clock mode (CLK_GENSEL=0).
	 * Set DATA_TOUT to the maximum exponent so card-side data timeouts
	 * don't trip during card identification. NO_HISPD_BIT quirk: we
	 * don't touch HCTL_HS in CONTROL0 -- the silicon ignores it; speed
	 * comes from the divider alone.
	 */
	div = sdhc_bcm2835_calc_clk_div(cfg->clock_freq, target_hz);
	ctrl1 |= ((div >> 8) & 0x3) << SDHCI_CTRL1_CLK_FREQ_MS_SHIFT;
	ctrl1 |= (div & 0xFF) << SDHCI_CTRL1_CLK_FREQ_LO_SHIFT;
	ctrl1 |= 0xE << SDHCI_CTRL1_DATA_TOUT_SHIFT;	/* TMCLK * 2^(14+13) */
	ctrl1 |= SDHCI_CTRL1_CLK_INTLEN;
	sys_write32(ctrl1, base + SDHCI_CLOCK_CONTROL);

	/* Wait for the internal clock to stabilise. */
	deadline = k_uptime_get() + BCM2835_SDHCI_CLK_STABLE_MS;
	while (!(sys_read32(base + SDHCI_CLOCK_CONTROL) & SDHCI_CTRL1_CLK_STABLE)) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}

	/* Internal clock is stable; now enable the SD bus clock. */
	ctrl1 = sys_read32(base + SDHCI_CLOCK_CONTROL);
	ctrl1 |= SDHCI_CTRL1_CLK_EN;
	sys_write32(ctrl1, base + SDHCI_CLOCK_CONTROL);

	return 0;
}

static void sdhc_bcm2835_set_bus_width(const struct device *dev,
				       enum sdhc_bus_width width)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t ctrl0 = sys_read32(base + SDHCI_HOST_CONTROL);

	/* 8-bit bus is unreachable on this silicon (caps say so + we don't
	 * advertise it); a future caller asking for it is a Zephyr SD core
	 * bug, not something we'd silently expand to. Treat 1-bit as the
	 * fallback for any non-4-bit value.
	 */
	ctrl0 &= ~(SDHCI_CTRL0_HCTL_DWIDTH | SDHCI_CTRL0_HCTL_8BIT);
	if (width == SDHC_BUS_WIDTH4BIT) {
		ctrl0 |= SDHCI_CTRL0_HCTL_DWIDTH;
	}

	sys_write32(ctrl0, base + SDHCI_HOST_CONTROL);
}

static void sdhc_bcm2835_set_power(const struct device *dev,
				   enum sdhc_power power_mode)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t ctrl0 = sys_read32(base + SDHCI_HOST_CONTROL);

	ctrl0 &= ~(SDHCI_CTRL0_POWER_ON | SDHCI_CTRL0_VOLT_MASK);

	if (power_mode == SDHC_POWER_ON) {
		/* This silicon is 3.3V-only; there's no actual voltage
		 * switching to do, but the controller's state machine still
		 * wants the voltage select bits programmed alongside POWER.
		 */
		ctrl0 |= SDHCI_CTRL0_VOLT_330 | SDHCI_CTRL0_POWER_ON;
	}

	sys_write32(ctrl0, base + SDHCI_HOST_CONTROL);
}

static int sdhc_bcm2835_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhc_bcm2835_data *data = dev->data;
	int ret;

	/* The Zephyr SD core calls set_io for every state change (clock,
	 * width, power, voltage, timing). We always reprogram unconditionally
	 * rather than diffing against the cached host_io -- it's a few extra
	 * register writes during init and saves any state-tracking bug
	 * masking real hardware issues. The cached host_io stays for future
	 * read-only consumers (e.g. timeout calculation in request()).
	 */
	sdhc_bcm2835_set_power(dev, ios->power_mode);
	sdhc_bcm2835_set_bus_width(dev, ios->bus_width);

	ret = sdhc_bcm2835_set_clock(dev, ios->clock);
	if (ret != 0) {
		return ret;
	}

	data->host_io = *ios;
	return 0;
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

	/* Reset clears INT_ENABLE to all-zero, which gates every status
	 * bit -- without this, INT_STATUS stays 0 forever even when the
	 * controller fires CMD_COMPLETE / DATA_END / errors internally.
	 * Enable everything we poll for. CARD_INSERT / CARD_REMOVE stay
	 * masked because BCM2835 has the BROKEN_CARD_DETECTION quirk
	 * (no card-detect line on this Arasan integration; the wireless
	 * chip is hardwired-on). SIGNAL_ENABLE stays 0 (polled mode);
	 * we'll flip CARD_INT on later when we add ISR support for SDIO
	 * async-event delivery from the wireless chip.
	 */
	sys_write32(SDHCI_INT_ALL_W1C &
		    ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE),
		    base + SDHCI_INT_ENABLE);
	sys_write32(0, base + SDHCI_SIGNAL_ENABLE);

	/* Scaffold self-test: exercise set_io with the canonical SD card
	 * identification config (400 kHz, 1-bit, 3.3V, power on). Validates
	 * the divider math + clock-stable handshake without needing
	 * subsys/sd to drive us. Drop along with the rest of the bring-up
	 * scaffolding when real traffic exercises set_io for free.
	 */
	struct sdhc_io ios = {
		.clock = SDMMC_CLOCK_400KHZ,
		.bus_width = SDHC_BUS_WIDTH1BIT,
		.power_mode = SDHC_POWER_ON,
		.signal_voltage = SD_VOL_3_3_V,
	};
	ret = sdhc_bcm2835_set_io(dev, &ios);
	if (ret != 0) {
		printk("sdhc_bcm2835: %s set_io self-test failed: %d\n",
		       dev->name, ret);
		LOG_ERR("%s set_io self-test failed: %d", dev->name, ret);
		return ret;
	}

	uint32_t ctrl0 = sys_read32(base + SDHCI_HOST_CONTROL);
	uint32_t ctrl1 = sys_read32(base + SDHCI_CLOCK_CONTROL);

	/* Scaffold self-test #2: issue CMD0 (GO_IDLE_STATE, no response,
	 * no data) via the request() path. CMD_COMPLETE depends only on
	 * the controller's internal state machine for no-response cmds,
	 * so this works without any card actually responding -- proves
	 * that ARG1 + CMDTM writes light up the CMD line and the
	 * INT_STATUS poll sees the completion. Drop with the rest of the
	 * scaffolding when subsys/sd traffic exercises request() for free.
	 */
	struct sdhc_command cmd0 = {
		.opcode = 0,
		.arg = 0,
		.response_type = SD_RSP_TYPE_NONE,
	};
	ret = sdhc_bcm2835_request(dev, &cmd0, NULL);
	uint32_t int_after = sys_read32(base + SDHCI_INT_STATUS);

	printk("sdhc_bcm2835: %s @ 0x%lx, ver 0x%04x, clk_emmc %u Hz, %u-bit dts, "
	       "CONTROL0=0x%08x CONTROL1=0x%08x, CMD0 ret=%d INT_STATUS=0x%08x\n",
	       dev->name, (unsigned long)base, version,
	       cfg->clock_freq, cfg->bus_width, ctrl0, ctrl1, ret, int_after);

	LOG_INF("%s @ 0x%lx, ver 0x%04x, clk_emmc %u Hz, %u-bit dts, "
		"CONTROL0=0x%08x CONTROL1=0x%08x, CMD0 ret=%d INT_STATUS=0x%08x",
		dev->name, (unsigned long)base, version,
		cfg->clock_freq, cfg->bus_width, ctrl0, ctrl1, ret, int_after);

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
