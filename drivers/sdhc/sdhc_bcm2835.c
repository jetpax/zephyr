/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 / BCM2710 / BCM2837 Arasan SDHCI host controller.
 *
 * Polled (no IRQ) implementation of the six sdhc.h device-API methods:
 * reset, set_io (clock + width + power), request (cmd + PIO data),
 * get_card_present, get_host_props, card_busy. Targets the EMMC port
 * that's wired to the on-module CYW43439 wireless chip on Pi 3 / Pi
 * Zero 2 W.
 *
 * Three silicon facts shape this driver:
 *
 *   - 32-bit-only register access. Every read/write must be aligned to
 *     a 32-bit boundary at 32-bit width; the controller silently
 *     corrupts or hangs on sub-word accesses. We compose the SDHCI
 *     32-bit aggregates (BLKSIZECNT, CMDTM, CONTROL0, CONTROL1) in our
 *     own 32-bit values rather than going through an iproc-style
 *     16-bit-write shadow layer -- the natural form here.
 *
 *   - Capabilities register reads as zero. The silicon's CAPABILITIES
 *     at offset 0x40 doesn't reflect actual capabilities -- we expose
 *     hardcoded values from get_host_props(): max-block 1024, 3.3V
 *     VDD, high-speed support, driver type A/C. Mirrors Linux's
 *     sdhci-iproc.c::bcm2835_data (caps + caps1).
 *
 *   - Several SDHCI-spec quirks hold here:
 *       * BROKEN_CARD_DETECTION  -- no CD line; chip is hardwired-on.
 *           CARD_INSERT/REMOVE masked out of INT_ENABLE.
 *       * NO_HISPD_BIT           -- HCTL_HS in CONTROL0 ignored.
 *           High speed comes from the clock divider only.
 *       * DATA_TIMEOUT_USES_SDCLK -- DATA_TOUT field counts SD clocks,
 *           not TMCLK. We program max (0xE).
 *       * PRESET_VALUE_BROKEN    -- preset registers don't work; we
 *           configure clock/voltage explicitly.
 *
 * IRQ-driven completion (and SDIO async events via CARD_INT) is left
 * for a future commit; polling is plenty for SDIO control traffic and
 * keeps bring-up debuggable. R1b / R5b (busy-after-response) are
 * deferred for the same reason.
 *
 * References (open as you read this file):
 *   - Linux: drivers/mmc/host/sdhci-iproc.c    -- the quirk source
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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/arm64/arm_mem.h>
#include <zephyr/sd/sd_spec.h>

LOG_MODULE_REGISTER(sdhc_bcm2835, CONFIG_SDHC_LOG_LEVEL);

/* Standard SDHCI register offsets we use. Names mirror Linux's
 * include/linux/mmc/sdhci.h so cross-references read 1:1. The BCM
 * datasheet packages some of these as 32-bit aggregates (e.g. CONTROL1
 * = CLOCK_CONTROL + TIMEOUT + SOFTWARE_RESET); we always touch the
 * containing 32-bit word.
 */
#define SDHCI_BLKSIZECNT		0x04	/* BLOCK_SIZE | BLOCK_COUNT<<16 */
#define SDHCI_ARG1			0x08	/* command argument (32-bit) */
#define SDHCI_BUFFER			0x20	/* PIO buffer data port (32-bit) */
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
/* TEMP: cap at 25 MHz while debugging CMD53 DATA_CRC at 50 MHz on the
 * WLAN DAT lines. CMD52 (CMD line only) works at 50 MHz; CMD53 fails
 * with DATA_CRC. Raise back to 50 MHz once the DAT-line timing /
 * signal-integrity issue is understood.
 */
#define BCM2835_F_MAX_HZ		25000000 /* SDR12 / default speed */

struct sdhc_bcm2835_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pincfg;
	struct gpio_dt_spec wifi_reg_on;	/* CYW43439 WL_REG_ON */
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
 * Linux uses 10s for non-data commands. On BCM2835 the hardware
 * CMD_TIMEOUT is governed by CONTROL1.DATA_TOUNIT (extension to
 * SDHCI spec on this Arasan integration), which we set to 0x6
 * (~1.3 s at 400 kHz SDCLK). 5 s here gives the hardware enough
 * margin to fire its bit before we give up.
 */
#define BCM2835_SDHCI_CMD_TIMEOUT_MS	5000

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

/* Build the full 32-bit CMDTM word -- TRANSFER_MODE in the low 16 bits,
 * COMMAND in the high 16. Writing this 32-bit word fires the command;
 * the BCM 32-bit-only access requirement makes the natural composition
 * here (write the whole word in one go) the right thing.
 *
 * We deliberately don't support R1b yet: the controller waits for DAT0
 * to clear after a busy response, which can hang indefinitely if the
 * card never deasserts busy. R1b becomes a small extension once we
 * have IRQ-driven completion; for now polled R1 is enough for SDIO
 * control traffic.
 */
static int sdhc_bcm2835_build_cmd(const struct sdhc_command *cmd,
				  const struct sdhc_data *data, uint32_t *cmdtm)
{
	uint32_t flags = (uint32_t)cmd->opcode << SDHCI_CMDTM_INDEX_SHIFT;
	uint32_t rsp = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	if (data != NULL) {
		flags |= SDHCI_CMDTM_DATA_PRESENT;
		/* TRANSFER_MODE bits (low 16): block-count enable, direction,
		 * multi-block if blocks > 1. AUTO_CMD12 / AUTO_CMD23 are not
		 * used -- SDIO CMD53 carries its own byte/block count and the
		 * card terminates without needing CMD12.
		 *
		 * Direction inference: Zephyr's sdhc_data has no explicit
		 * direction field. For canonical SD opcodes the direction is
		 * baked into the opcode; for SDIO CMD53 it's in arg bit 31
		 * per the SDIO spec (1=write, 0=read).
		 */
		bool is_read;

		switch (cmd->opcode) {
		case SDIO_RW_EXTENDED:
			is_read = !(cmd->arg & BIT(SDIO_CMD_ARG_RW_SHIFT));
			break;
		case SD_READ_SINGLE_BLOCK:
		case SD_READ_MULTIPLE_BLOCK:
		case SD_APP_SEND_SCR:
			is_read = true;
			break;
		case SD_WRITE_SINGLE_BLOCK:
		case SD_WRITE_MULTIPLE_BLOCK:
			is_read = false;
			break;
		default:
			/* Other opcodes shouldn't carry a data phase. Refuse
			 * rather than silently picking a direction. */
			return -EINVAL;
		}

		flags |= SDHCI_CMDTM_TM_BLKCNT_EN;
		if (is_read) {
			flags |= SDHCI_CMDTM_TM_DAT_DIR_READ;
		}
		if (data->blocks > 1) {
			flags |= SDHCI_CMDTM_TM_MULTI_BLOCK;
		}
	}

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

/* Per-block PIO transfer. Linux's sdhci_{read,write}_block_pio
 * collapses to this once you strip the sg_miter scatter-gather:
 * read 1 word from BUFFER (or write 1 word to BUFFER), assembling
 * bytes into / from the caller's buffer. Tail bytes < 4 share the
 * same scratch word -- a partial-word read still costs one full
 * BUFFER read, the controller delivers the FIFO contents word-aligned.
 *
 * Loop invariant: when this returns, exactly one block has moved.
 * The caller polls INT_STATUS for BUFFER_*_READY between blocks
 * (and DATA_END after the last one).
 */
static void sdhc_bcm2835_pio_xfer_one_block(uintptr_t base, uint8_t **bufp,
					    size_t blksize, bool is_read)
{
	uint8_t *buf = *bufp;
	uint32_t scratch;

	while (blksize >= 4) {
		if (is_read) {
			scratch = sys_read32(base + SDHCI_BUFFER);
			buf[0] = scratch & 0xFF;
			buf[1] = (scratch >> 8) & 0xFF;
			buf[2] = (scratch >> 16) & 0xFF;
			buf[3] = (scratch >> 24) & 0xFF;
		} else {
			scratch = (uint32_t)buf[0] |
				  ((uint32_t)buf[1] << 8) |
				  ((uint32_t)buf[2] << 16) |
				  ((uint32_t)buf[3] << 24);
			sys_write32(scratch, base + SDHCI_BUFFER);
		}
		buf += 4;
		blksize -= 4;
	}

	if (blksize > 0) {
		if (is_read) {
			scratch = sys_read32(base + SDHCI_BUFFER);
			while (blksize > 0) {
				*buf++ = scratch & 0xFF;
				scratch >>= 8;
				blksize--;
			}
		} else {
			scratch = 0;
			unsigned int shift = 0;

			while (blksize > 0) {
				scratch |= (uint32_t)*buf << shift;
				shift += 8;
				buf++;
				blksize--;
			}
			sys_write32(scratch, base + SDHCI_BUFFER);
		}
	}

	*bufp = buf;
}

/* Drive the data phase to completion: poll for BUFFER_*_READY, transfer
 * one block, ack, repeat. After the last block the controller asserts
 * DATA_END; we ack and return.
 */
static int sdhc_bcm2835_transfer_data(const struct device *dev,
				      struct sdhc_data *data, bool is_read,
				      int timeout_ms)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint8_t *buf = (uint8_t *)data->data;
	uint32_t blocks_left = data->blocks;
	uint32_t ready_bit = is_read ? SDHCI_INT_BUF_READ_READY
				     : SDHCI_INT_BUF_WRITE_READY;
	uint32_t int_status;

	while (blocks_left > 0) {
		int_status = sdhc_bcm2835_wait_int(dev, ready_bit,
						   SDHCI_INT_DATA_ERROR_MASK,
						   timeout_ms);
		if (int_status == 0) {
			LOG_ERR("data %s: BUF_%s_READY timeout (blocks_left=%u blk_size=%u pstate=0x%08x)",
				is_read ? "read" : "write",
				is_read ? "READ" : "WRITE",
				blocks_left, data->block_size,
				sys_read32(base + SDHCI_PRESENT_STATE));
			return -ETIMEDOUT;
		}
		if (int_status & SDHCI_INT_DATA_ERROR_MASK) {
			LOG_ERR("data %s: int_status=0x%08x (blocks_left=%u blk_size=%u pstate=0x%08x)",
				is_read ? "read" : "write", int_status,
				blocks_left, data->block_size,
				sys_read32(base + SDHCI_PRESENT_STATE));
			sys_write32(int_status & SDHCI_INT_DATA_ERROR_MASK,
				    base + SDHCI_INT_STATUS);
			(void)sdhc_bcm2835_soft_reset(dev,
						      SDHCI_CTRL1_RESET_DATA);
			if (int_status & SDHCI_INT_DATA_TIMEOUT) {
				return -ETIMEDOUT;
			}
			return -EIO;
		}

		sdhc_bcm2835_pio_xfer_one_block(base, &buf, data->block_size,
						is_read);
		sys_write32(ready_bit, base + SDHCI_INT_STATUS);
		blocks_left--;
	}

	int_status = sdhc_bcm2835_wait_int(dev, SDHCI_INT_DATA_END,
					   SDHCI_INT_DATA_ERROR_MASK,
					   timeout_ms);
	if (int_status == 0) {
		LOG_ERR("data %s: DATA_END timeout (pstate=0x%08x)",
			is_read ? "read" : "write",
			sys_read32(base + SDHCI_PRESENT_STATE));
		return -ETIMEDOUT;
	}
	if (int_status & SDHCI_INT_DATA_ERROR_MASK) {
		LOG_ERR("data %s end: int_status=0x%08x (pstate=0x%08x)",
			is_read ? "read" : "write", int_status,
			sys_read32(base + SDHCI_PRESENT_STATE));
		sys_write32(int_status & SDHCI_INT_DATA_ERROR_MASK,
			    base + SDHCI_INT_STATUS);
		(void)sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_DATA);
		return -EIO;
	}
	sys_write32(SDHCI_INT_DATA_END, base + SDHCI_INT_STATUS);
	data->bytes_xfered = data->blocks * data->block_size;
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
	uint32_t inhibit_mask;
	int cmd_timeout_ms;
	int data_timeout_ms;
	bool is_read;
	int ret;

	if (cmd == NULL) {
		return -EINVAL;
	}
	if (data != NULL) {
		if (data->data == NULL || data->blocks == 0 ||
		    data->block_size == 0 ||
		    data->block_size > BCM2835_MAX_BLOCK_BYTES) {
			return -EINVAL;
		}
	}

	ret = sdhc_bcm2835_build_cmd(cmd, data, &cmdtm);
	if (ret != 0) {
		return ret;
	}

	is_read = (cmdtm & SDHCI_CMDTM_TM_DAT_DIR_READ) != 0;
	cmd_timeout_ms = cmd->timeout_ms ? cmd->timeout_ms
					 : BCM2835_SDHCI_CMD_TIMEOUT_MS;
	data_timeout_ms = (data && data->timeout_ms) ? data->timeout_ms
						     : cmd_timeout_ms;

	inhibit_mask = SDHCI_PSTATE_CMD_INHIBIT;
	if (data != NULL) {
		inhibit_mask |= SDHCI_PSTATE_DATA_INHIBIT;
	}
	ret = sdhc_bcm2835_wait_inhibit(dev, inhibit_mask, cmd_timeout_ms);
	if (ret != 0) {
		return ret;
	}

	/* Clear any stale CMD/DATA-side interrupt bits so wait_int doesn't
	 * latch onto a previous request's completion.
	 *
	 * The BCM Arasan has a documented clock-domain-crossing bug
	 * (sdhci-iproc.c: "may lose the content of successive writes to
	 * the same register within two SD-card clock cycles of each
	 * other"). Linux's bcm2835_mmc_writel wraps every register write
	 * with `udelay((2*1000000)/max(host->clock, 400000) + 1)` = 6 us
	 * at 400 kHz. Our k_busy_wait(10) is comfortably > 4 SDCLK cycles
	 * at the 400 kHz initialisation clock and harmless at higher
	 * post-init rates.
	 */
	sys_write32(0xFFFFFFFF, base + SDHCI_INT_STATUS);
	k_busy_wait(10);

	if (data != NULL) {
		uint32_t blksizecnt = (data->block_size & 0x3FF) |
				      ((uint32_t)data->blocks << 16);
		sys_write32(blksizecnt, base + SDHCI_BLKSIZECNT);
		k_busy_wait(10);
	}

	sys_write32(cmd->arg, base + SDHCI_ARG1);
	k_busy_wait(10);
	sys_write32(cmdtm, base + SDHCI_CMDTM);	/* fires command */

	int_status = sdhc_bcm2835_wait_int(dev, SDHCI_INT_CMD_COMPLETE,
					   SDHCI_INT_CMD_ERROR_MASK,
					   cmd_timeout_ms);

	if (int_status == 0) {
		return -ETIMEDOUT;
	}

	if (int_status & SDHCI_INT_CMD_ERROR_MASK) {
		uint32_t cmd_err = int_status & SDHCI_INT_CMD_ERROR_MASK;

		/* Plain CMD_TIMEOUT (no CRC / end-bit / index error alongside)
		 * is expected for some probe commands -- notably CMD8 sent to
		 * legacy SDIO cards that don't implement it (BCM43430A1). The
		 * SD subsystem retries 11 times before falling back; logging
		 * all 11 at LOG_ERR was masking real failures during bring-up.
		 * Downgrade timeout-only to LOG_DBG; keep CRC / end-bit / index
		 * at LOG_ERR.
		 */
		if (cmd_err == SDHCI_INT_CMD_TIMEOUT) {
			LOG_DBG("CMD%u arg=0x%08x: timeout (pstate=0x%08x)",
				cmd->opcode, cmd->arg,
				sys_read32(base + SDHCI_PRESENT_STATE));
		} else {
			LOG_ERR("CMD%u arg=0x%08x: int_status=0x%08x pstate=0x%08x",
				cmd->opcode, cmd->arg, int_status,
				sys_read32(base + SDHCI_PRESENT_STATE));
		}
		sys_write32(int_status & SDHCI_INT_CMD_ERROR_MASK,
			    base + SDHCI_INT_STATUS);
		/* Reset the CMD state machine to recover from the error.
		 * Only reset the DATA state machine if this command had a
		 * data phase; CMD-only commands (CMD0/3/5/7/8/52) don't
		 * touch the DAT lines and pulsing RESET_DATA for them is
		 * unnecessary -- and on this silicon, the repeated
		 * RESET_DATA pulses during e.g. the 11 CMD8 retries of
		 * sd_init's SD-2.0 probe destabilise the DAT state machine
		 * for subsequent CMD53s.
		 */
		(void)sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_CMD);
		if (data != NULL) {
			(void)sdhc_bcm2835_soft_reset(dev,
						      SDHCI_CTRL1_RESET_DATA);
		}
		if (int_status & SDHCI_INT_CMD_TIMEOUT) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	sdhc_bcm2835_read_response(dev, cmd);
	sys_write32(SDHCI_INT_CMD_COMPLETE, base + SDHCI_INT_STATUS);

	if (data != NULL) {
		ret = sdhc_bcm2835_transfer_data(dev, data, is_read,
						 data_timeout_ms);
		if (ret != 0) {
			return ret;
		}
	}

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

	if (target_hz == 0) {
		/* Gate the bus clock by clearing CLK_EN only. */
		ctrl1 = sys_read32(base + SDHCI_CLOCK_CONTROL);
		ctrl1 &= ~SDHCI_CTRL1_CLK_EN;
		sys_write32(ctrl1, base + SDHCI_CLOCK_CONTROL);
		return 0;
	}

	/* Single CTL1 write with divider + DATA_TOUT + CLK_INTLEN + CLK_EN
	 * all at once, matching circle's emmcclk (addon/wlan/emmc.c:221-240).
	 * The spec-suggested disable / configure / enable dance disturbs
	 * the DAT-side state machine on this silicon, causing the first
	 * CMD53 after sd_init's bus-config ramps to fire spurious
	 * DATA_CRC. A single atomic CTL1 write avoids the issue.
	 *
	 * DATA_TOUNIT = 0xE -- 2^27 SDCLK cycles (~5.4 s at 25 MHz, the
	 * spec-max). NO_HISPD_BIT quirk: we don't touch HCTL_HS in CONTROL0
	 * -- the silicon ignores it; speed comes from the divider alone.
	 */
	div = sdhc_bcm2835_calc_clk_div(cfg->clock_freq, target_hz);
	ctrl1 = (((div >> 8) & 0x3) << SDHCI_CTRL1_CLK_FREQ_MS_SHIFT) |
		((div & 0xFF) << SDHCI_CTRL1_CLK_FREQ_LO_SHIFT) |
		(0xE << SDHCI_CTRL1_DATA_TOUT_SHIFT) |
		SDHCI_CTRL1_CLK_INTLEN |
		SDHCI_CTRL1_CLK_EN;
	sys_write32(ctrl1, base + SDHCI_CLOCK_CONTROL);

	/* Wait for the internal clock to stabilise. */
	deadline = k_uptime_get() + BCM2835_SDHCI_CLK_STABLE_MS;
	while (!(sys_read32(base + SDHCI_CLOCK_CONTROL) & SDHCI_CTRL1_CLK_STABLE)) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}

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

	/* RESET_DATA before the CTL0 write so the controller's internal
	 * DAT-side bus-width state machine picks up the new width on a
	 * freshly-reset state. Without this pair, on BCM43430A1 the first
	 * 4-bit data transfer after sd_init's width ramp fires spurious
	 * DATA_CRC even though every visible register reads correct.
	 * Empirically isolated at REPL: chip-side CMD52 to CCCR_BUS_IF
	 * doesn't help; host-side RESET_DATA + CTL0 write does.
	 */
	(void)sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_DATA);
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

#ifdef CONFIG_SDHC_BCM2835_SELFTEST
/* Bring-up self-test: fires CMD0 + CMD5 (inquiry & op_cond) + CMD3 + CMD7
 * + a CMD52 CCCR read through the driver's own request() path. Logs each
 * step; failures log-only so init still returns 0 (the device stays
 * available for later layers). Drop the Kconfig once Zephyr's SDIO
 * subsystem is wired up and exercises the same path.
 *
 * CMD7 is sent as R1 (not R1b): both Linux's _mmc_select_card and
 * Zephyr's own subsys/sd/sdio.c choose R1, because the SDIO chip doesn't
 * drive a meaningful busy on SELECT_CARD. The driver's request() rejects
 * R1b today and that's fine for this path.
 */
static void sdhc_bcm2835_selftest(const struct device *dev)
{
	struct sdhc_command cmd;
	uint32_t ocr;
	uint16_t rca;
	int ret;
	int i;

	LOG_INF("--- SDIO bring-up self-test ---");

	cmd = (struct sdhc_command){
		.opcode = SD_GO_IDLE_STATE,
		.response_type = SD_RSP_TYPE_NONE,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD0 GO_IDLE failed: %d", ret);
		return;
	}
	LOG_INF("CMD0 GO_IDLE: ok");

	cmd = (struct sdhc_command){
		.opcode = SDIO_SEND_OP_COND,
		.response_type = SD_RSP_TYPE_R4,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD5 inquiry failed: %d", ret);
		return;
	}
	ocr = cmd.response[0];
	LOG_INF("CMD5 inquiry: OCR=0x%08x", ocr);

	for (i = 0; i < 100; i++) {
		cmd = (struct sdhc_command){
			.opcode = SDIO_SEND_OP_COND,
			.arg = ocr & 0x00FFFFFF,
			.response_type = SD_RSP_TYPE_R4,
		};
		ret = sdhc_bcm2835_request(dev, &cmd, NULL);
		if (ret != 0) {
			LOG_ERR("CMD5 op_cond failed @ iter %d: %d", i, ret);
			return;
		}
		if (cmd.response[0] & SDIO_OCR_IO_READY_FLAG) {
			break;
		}
		k_msleep(10);
	}
	if (!(cmd.response[0] & SDIO_OCR_IO_READY_FLAG)) {
		LOG_ERR("CMD5 op_cond: chip never reported IO_READY "
			"(last OCR=0x%08x)", cmd.response[0]);
		return;
	}
	LOG_INF("CMD5 op_cond: OCR=0x%08x ready (%u io funcs%s)",
		cmd.response[0],
		(uint32_t)((cmd.response[0] & SDIO_OCR_IO_NUMBER) >>
			   SDIO_OCR_IO_NUMBER_SHIFT),
		(cmd.response[0] & SDIO_OCR_MEM_PRESENT_FLAG) ?
			", mem present" : "");

	cmd = (struct sdhc_command){
		.opcode = SD_SEND_RELATIVE_ADDR,
		.response_type = SD_RSP_TYPE_R6,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD3 SEND_RELATIVE_ADDR failed: %d", ret);
		return;
	}
	rca = cmd.response[0] >> 16;
	LOG_INF("CMD3 SEND_RELATIVE_ADDR: RCA=0x%04x", rca);

	cmd = (struct sdhc_command){
		.opcode = SD_SELECT_CARD,
		.arg = ((uint32_t)rca) << 16,
		.response_type = SD_RSP_TYPE_R1,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD7 SELECT_CARD failed: %d", ret);
		return;
	}
	LOG_INF("CMD7 SELECT_CARD: ok");

	cmd = (struct sdhc_command){
		.opcode = SDIO_RW_DIRECT,
		.arg = ((uint32_t)SDIO_CCCR_CCCR <<
			SDIO_CMD_ARG_REG_ADDR_SHIFT),
		.response_type = SD_RSP_TYPE_R5,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD52 CCCR read failed: %d", ret);
		return;
	}
	LOG_INF("CMD52 CCCR read @ 400kHz/1-bit: resp=0x%08x cccr_rev=%u sdio_rev=%u",
		cmd.response[0],
		(uint32_t)(cmd.response[0] & SDIO_CCCR_CCCR_REV_MASK),
		(uint32_t)((cmd.response[0] >> 4) & 0xF));

	/* CMD52 write to CCCR_BUS_IF (0x07) = 0b10: tell the chip to switch
	 * to 4-bit. The chip ACKs in R5 and switches on the response edge,
	 * so the next register touch must be the host-side width change.
	 */
	cmd = (struct sdhc_command){
		.opcode = SDIO_RW_DIRECT,
		.arg = (1U << SDIO_CMD_ARG_RW_SHIFT) |
		       ((uint32_t)SDIO_CCCR_BUS_IF <<
			SDIO_CMD_ARG_REG_ADDR_SHIFT) |
		       SDIO_CCCR_BUS_IF_WIDTH_4_BIT,
		.response_type = SD_RSP_TYPE_R5,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD52 write CCCR_BUS_IF=4bit failed: %d", ret);
		return;
	}
	LOG_INF("CMD52 write CCCR_BUS_IF=4bit: resp=0x%08x", cmd.response[0]);

	struct sdhc_io ios_fast = {
		.clock = SD_CLOCK_25MHZ,
		.bus_width = SDHC_BUS_WIDTH4BIT,
		.power_mode = SDHC_POWER_ON,
		.signal_voltage = SD_VOL_3_3_V,
	};
	ret = sdhc_bcm2835_set_io(dev, &ios_fast);
	if (ret != 0) {
		LOG_ERR("set_io(25MHz/4-bit) failed: %d", ret);
		return;
	}
	LOG_INF("set_io: 25 MHz, 4-bit, 3.3V");

	/* Re-read CCCR rev to confirm both sides survived the switch. */
	cmd = (struct sdhc_command){
		.opcode = SDIO_RW_DIRECT,
		.arg = ((uint32_t)SDIO_CCCR_CCCR <<
			SDIO_CMD_ARG_REG_ADDR_SHIFT),
		.response_type = SD_RSP_TYPE_R5,
	};
	ret = sdhc_bcm2835_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD52 CCCR read @ 25MHz/4-bit failed: %d", ret);
		return;
	}
	LOG_INF("CMD52 CCCR read @ 25MHz/4-bit: resp=0x%08x cccr_rev=%u sdio_rev=%u",
		cmd.response[0],
		(uint32_t)(cmd.response[0] & SDIO_CCCR_CCCR_REV_MASK),
		(uint32_t)((cmd.response[0] >> 4) & 0xF));

	LOG_INF("--- self-test complete ---");
}
#endif /* CONFIG_SDHC_BCM2835_SELFTEST */

static int sdhc_bcm2835_init(const struct device *dev)
{
	const struct sdhc_bcm2835_config *cfg = dev->config;
	int ret;

	/* MMIO mapping uses Device-nGnRE (early-write-ack permitted).
	 * Zephyr's default K_MEM_CACHE_NONE maps as Device-nGnRnE; the
	 * BCM283x peripheral region is documented as Device-nGnRE.
	 */
	DEVICE_MMIO_MAP(dev, K_MEM_ARM_DEVICE_nGnRE);

	/* Disconnect Arasan SDHCI from the SD card slot pads FIRST, before
	 * any other pinctrl runs. Mirrors circle's ether4330.c::sdioinit
	 * order: 48..53 -> ALT0 first, THEN 34..39 -> ALT3.
	 *
	 * On Pi 3 / Pi Zero 2 W the Arasan controller's CMD/DAT/CLK signals
	 * are routable to two pad sets: GPIO 34..39 (ALT3, WLAN module) and
	 * GPIO 48..53 (ALT3, microSD card slot). Pi firmware boots with the
	 * slot pins at ALT3, and the controller's RX input mux latches to
	 * whichever pad set is at ALT3 first. If we apply pinctrl on
	 * 34..39 while 48..53 is still at ALT3, both sets are simultaneously
	 * ALT3 and the RX mux stays tied to 48..53 -- only the CMD line
	 * happens to work (chip drives hard), while DAT1..3 read as the
	 * empty SD slot's pull-up state, manifesting as DATA_CRC on CMD53.
	 * Doing 48..53 -> ALT0 first ensures that when pinctrl puts
	 * 34..39 at ALT3, that's the only ALT3 set and RX latches cleanly.
	 *
	 * 3 bits per pin in GPFSEL4/5, ALT0 = 0b100 = 4.
	 */
	{
		volatile uint32_t *gpfsel4 = (volatile uint32_t *)0x3F200010;
		volatile uint32_t *gpfsel5 = (volatile uint32_t *)0x3F200014;
		uint32_t f4 = *gpfsel4;
		uint32_t f5 = *gpfsel5;
		uint32_t mask5 = 0, set5 = 0;

		f4 = (f4 & ~((7u << 24) | (7u << 27))) |
		     ((4u << 24) | (4u << 27));
		for (int n = 50; n <= 53; n++) {
			int s = (n - 50) * 3;
			mask5 |= 7u << s;
			set5 |= 4u << s;
		}
		f5 = (f5 & ~mask5) | set5;

		*gpfsel4 = f4;
		*gpfsel5 = f5;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("%s pinctrl apply failed: %d", dev->name, ret);
		return ret;
	}

	/* WL_REG_ON is driven HIGH by the Pi firmware before kernel handoff.
	 * The wireless chip is past its bootrom settle window by the time
	 * POST_KERNEL runs; toggling LOW->HIGH from here disturbs the
	 * chip's PMU and is not required.
	 */

	ret = sdhc_bcm2835_soft_reset(dev, SDHCI_CTRL1_RESET_ALL);
	if (ret != 0) {
		LOG_ERR("%s reset timeout", dev->name);
		return ret;
	}

	/* INT_ENABLE / SIGNAL_ENABLE: CMD_COMPLETE, DATA_END, the
	 * PIO buffer-ready signals, and all CMD/DATA error bits. Both
	 * registers carry the same value because this controller doesn't
	 * latch INT_STATUS bits unless the matching SIGNAL_ENABLE bit is
	 * set, even when the request path is polled. BUF_READ_READY and
	 * BUF_WRITE_READY are mandatory for PIO data phases -- mirrors
	 * Linux's bcm2835_mmc_set_transfer_irqs: pio_irqs =
	 * SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL.
	 */
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t int_en = SDHCI_INT_CMD_COMPLETE | SDHCI_INT_DATA_END |
			  SDHCI_INT_BUF_READ_READY | SDHCI_INT_BUF_WRITE_READY |
			  SDHCI_INT_CMD_TIMEOUT | SDHCI_INT_CMD_CRC |
			  SDHCI_INT_CMD_END_BIT | SDHCI_INT_CMD_INDEX |
			  SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC |
			  SDHCI_INT_DATA_END_BIT |
			  BIT(23);	/* BUS_POWER_ERR */
	sys_write32(int_en, base + SDHCI_INT_ENABLE);
	sys_write32(int_en, base + SDHCI_SIGNAL_ENABLE);

	struct sdhc_io ios = {
		.clock = SDMMC_CLOCK_400KHZ,
		.bus_width = SDHC_BUS_WIDTH1BIT,
		.power_mode = SDHC_POWER_ON,
		.signal_voltage = SD_VOL_3_3_V,
	};
	ret = sdhc_bcm2835_set_io(dev, &ios);
	if (ret != 0) {
		LOG_ERR("%s set_io failed: %d", dev->name, ret);
		return ret;
	}

#ifdef CONFIG_SDHC_BCM2835_SELFTEST
	sdhc_bcm2835_selftest(dev);
#endif

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
	PINCTRL_DT_INST_DEFINE(inst);						\
	static const struct sdhc_bcm2835_config sdhc_bcm2835_cfg_##inst = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.wifi_reg_on = GPIO_DT_SPEC_INST_GET_OR(inst,			\
				wifi_reg_on_gpios, {0}),			\
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
