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
 * Sub-task C scaffold: polled command path with no data phase. Five
 * of six sdhc.h callbacks are real: reset, set_io (clock + width +
 * power), request (command-only -- R1/R2/R3/R6/R7, no R1b),
 * get_card_present, get_host_props. card_busy still returns 0 (full
 * version needs DAT0 sampling via SDEDM FSM). request() rejects any
 * call with a non-NULL data argument with -ENOSYS; PIO data path
 * lands in sub-task D.
 *
 * Linux reference (read for understanding only, NOT lifted):
 *   drivers/mmc/host/bcm2835.c -- bcm2835_reset_internal() ~L241,
 *   bcm2835_send_command() ~L617, bcm2835_finish_command() ~L736,
 *   bcm2835_set_clock() ~L1091, bcm2835_set_ios() ~L1224.
 */

#define DT_DRV_COMPAT brcm_bcm2835_sdhost

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>

LOG_MODULE_REGISTER(sdhc_bcm2835_sdhost, CONFIG_SDHC_LOG_LEVEL);

/* ===== Register offsets =========================================== */
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
#define SDCMD_NEW_FLAG		0x8000
#define SDCMD_FAIL_FLAG		0x4000
#define SDCMD_BUSYWAIT		0x800
#define SDCMD_NO_RESPONSE	0x400
#define SDCMD_LONG_RESPONSE	0x200
#define SDCMD_WRITE_CMD		0x80
#define SDCMD_READ_CMD		0x40
#define SDCMD_CMD_MASK		0x3f

/* ===== SDCDIV ===================================================== */
#define SDCDIV_MAX_CDIV		0x7ff

/* ===== SDHSTS bits ================================================ */
#define SDHSTS_BUSY_IRPT	0x400
#define SDHSTS_BLOCK_IRPT	0x200
#define SDHSTS_SDIO_IRPT	0x100
#define SDHSTS_REW_TIME_OUT	0x80
#define SDHSTS_CMD_TIME_OUT	0x40
#define SDHSTS_CRC16_ERROR	0x20
#define SDHSTS_CRC7_ERROR	0x10
#define SDHSTS_FIFO_ERROR	0x08
#define SDHSTS_DATA_FLAG	0x01
#define SDHSTS_W1C_ALL		0x7f8
#define SDHSTS_ERROR_MASK	(SDHSTS_CMD_TIME_OUT | SDHSTS_REW_TIME_OUT | \
				 SDHSTS_CRC7_ERROR | SDHSTS_CRC16_ERROR | \
				 SDHSTS_FIFO_ERROR)

/* ===== SDVDD ====================================================== */
#define SDVDD_POWER_OFF		0
#define SDVDD_POWER_ON		1

/* ===== SDEDM bits ================================================= */
#define SDEDM_FSM_MASK			0xf
#define SDEDM_FSM_IDENTMODE		0x0
#define SDEDM_FSM_DATAMODE		0x1
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
#define SDHCFG_BUSY_IRPT_EN	BIT(10)
#define SDHCFG_BLOCK_IRPT_EN	BIT(8)
#define SDHCFG_SDIO_IRPT_EN	BIT(5)
#define SDHCFG_DATA_IRPT_EN	BIT(4)
#define SDHCFG_SLOW_CARD	BIT(3)
#define SDHCFG_WIDE_EXT_BUS	BIT(2)
#define SDHCFG_WIDE_INT_BUS	BIT(1)
#define SDHCFG_REL_CMD_LINE	BIT(0)

/* ===== FIFO / timing constants ==================================== */
#define SDDATA_FIFO_WORDS	16
#define FIFO_READ_THRESHOLD	4
#define FIFO_WRITE_THRESHOLD	4
#define SDTOUT_RESET_VALUE	0x00f00000
#define SDHOST_POWER_SETTLE_MS	20

/* Caps reported via get_host_props. Mirrors Linux's bcm2835_add_host:
 * f_max defaults to clk_in (the VPU core_freq), f_min is clk_in /
 * SDCDIV_MAX_CDIV, and max_blk_size is 1024 (encoded as 1 in the
 * Zephyr SDHC API). We cap f_max at 50 MHz since the HS SD spec runs
 * at 50 MHz and we haven't verified anything faster.
 */
#define SDHOST_F_MAX_HZ		50000000

/* Default command-completion poll timeout. CMDs at the slow 400 kHz
 * card-ID clock can take a few ms; 100 ms is comfortably enough for
 * any non-busy command. Callers can override via cmd->timeout_ms.
 */
#define SDHOST_CMD_TIMEOUT_MS	100

struct sdhc_bcm2835_sdhost_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pincfg;
	uint32_t clock_freq;
	uint8_t bus_width;
};

struct sdhc_bcm2835_sdhost_data {
	DEVICE_MMIO_RAM;
	uint32_t hcfg;
	uint32_t cdiv;
};

/* ===== Soft reset ==================================================
 * Mirrors Linux's bcm2835_reset_internal() at bcm2835.c:241.
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
	sys_write32(SDHSTS_W1C_ALL, base + SDHSTS);
	sys_write32(0, base + SDHCFG);
	sys_write32(0, base + SDHBCT);
	sys_write32(0, base + SDHBLC);

	edm = sys_read32(base + SDEDM);
	edm &= ~((SDEDM_THRESHOLD_MASK << SDEDM_READ_THRESHOLD_SHIFT) |
		 (SDEDM_THRESHOLD_MASK << SDEDM_WRITE_THRESHOLD_SHIFT));
	edm |= (FIFO_READ_THRESHOLD << SDEDM_READ_THRESHOLD_SHIFT) |
	       (FIFO_WRITE_THRESHOLD << SDEDM_WRITE_THRESHOLD_SHIFT);
	sys_write32(edm, base + SDEDM);

	k_msleep(SDHOST_POWER_SETTLE_MS);
	sys_write32(SDVDD_POWER_ON, base + SDVDD);
	k_msleep(SDHOST_POWER_SETTLE_MS);

	data->hcfg = 0;
	data->cdiv = SDCDIV_MAX_CDIV;
	sys_write32(data->hcfg, base + SDHCFG);
	sys_write32(data->cdiv, base + SDCDIV);
}

/* ===== Command-completion poll ===================================== */
static int sdhost_wait_cmd_done(const struct device *dev, int timeout_ms)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (sys_read32(base + SDCMD) & SDCMD_NEW_FLAG) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}
	return 0;
}

/* ===== Clock divider math =========================================
 * SDHost bus clock = clk_in / (SDCDIV + 2). To get the largest bus
 * clock <= target_hz we ceil-divide clk_in/target_hz and subtract 2.
 * Mirrors Linux's bcm2835_set_clock at bcm2835.c:1116-1134.
 */
static uint32_t sdhost_calc_clk_div(uint32_t clk_in, uint32_t target_hz)
{
	uint32_t div;

	if (target_hz == 0 || target_hz >= clk_in / 2) {
		return 0;
	}
	div = clk_in / target_hz;
	if (div < 2) {
		div = 2;
	}
	if ((clk_in / div) > target_hz) {
		div++;
	}
	div -= 2;
	if (div > SDCDIV_MAX_CDIV) {
		div = SDCDIV_MAX_CDIV;
	}
	return div;
}

static int sdhc_bcm2835_sdhost_reset(const struct device *dev)
{
	sdhost_soft_reset(dev);
	return 0;
}

static int sdhc_bcm2835_sdhost_set_io(const struct device *dev,
				      struct sdhc_io *ios)
{
	const struct sdhc_bcm2835_sdhost_config *cfg = dev->config;
	struct sdhc_bcm2835_sdhost_data *data = dev->data;
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t div;

	/* Clock divider. SDCDIV gates and re-arms the bus clock; SDHost
	 * has no separate enable bit. Target 0 -> slowest divider.
	 */
	div = (ios->clock == 0) ? SDCDIV_MAX_CDIV
				: sdhost_calc_clk_div(cfg->clock_freq, ios->clock);
	data->cdiv = div;
	sys_write32(div, base + SDCDIV);

	/* Power: bit 0 of SDVDD. SDHost is 3.3V-only; signal_voltage
	 * field is ignored (the only meaningful value is SD_VOL_3_3_V).
	 */
	sys_write32(ios->power_mode == SDHC_POWER_ON ? SDVDD_POWER_ON
						     : SDVDD_POWER_OFF,
		    base + SDVDD);

	/* Bus width + SLOW_CARD. WIDE_EXT_BUS is the external (slot)
	 * 4-bit toggle; WIDE_INT_BUS is for the (unused) internal
	 * variant. SLOW_CARD forces the ident-clock divisor at all
	 * times -- per Linux: "Disable clever clock switching, to cope
	 * with fast core clocks". Without it, data-mode uses a coarser
	 * 3-bit divisor that's wrong above core_freq=250.
	 */
	data->hcfg &= ~SDHCFG_WIDE_EXT_BUS;
	if (ios->bus_width == SDHC_BUS_WIDTH4BIT) {
		data->hcfg |= SDHCFG_WIDE_EXT_BUS;
	}
	data->hcfg |= SDHCFG_SLOW_CARD;
	sys_write32(data->hcfg, base + SDHCFG);

	return 0;
}

static int sdhc_bcm2835_sdhost_request(const struct device *dev,
				       struct sdhc_command *cmd,
				       struct sdhc_data *data)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);
	uint32_t sdcmd;
	uint32_t sdhsts;
	uint32_t rsp;
	int cmd_timeout_ms;
	int ret;

	if (cmd == NULL) {
		return -EINVAL;
	}
	if (data != NULL) {
		/* PIO data path lands in sub-task D. */
		return -ENOSYS;
	}

	cmd_timeout_ms = cmd->timeout_ms ? cmd->timeout_ms
					 : SDHOST_CMD_TIMEOUT_MS;

	/* Make sure any previous command finished arming. The controller
	 * latches SDCMD on NEW_FLAG transitions; a stale NEW_FLAG means
	 * the previous command never completed -- recover by failing
	 * fast rather than overwriting it.
	 */
	ret = sdhost_wait_cmd_done(dev, cmd_timeout_ms);
	if (ret != 0) {
		LOG_ERR("%s CMD%u: previous command never completed",
			dev->name, cmd->opcode);
		return ret;
	}

	/* Clear any stale error bits before issuing the new command, so
	 * the post-completion error check only sees fresh state.
	 */
	sdhsts = sys_read32(base + SDHSTS);
	if (sdhsts & SDHSTS_ERROR_MASK) {
		sys_write32(sdhsts & SDHSTS_W1C_ALL, base + SDHSTS);
	}

	sdcmd = cmd->opcode & SDCMD_CMD_MASK;
	rsp = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	switch (rsp) {
	case SD_RSP_TYPE_NONE:
		sdcmd |= SDCMD_NO_RESPONSE;
		break;
	case SD_RSP_TYPE_R2:
		sdcmd |= SDCMD_LONG_RESPONSE;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		/* Short 48-bit response. SDHost handles CRC + index check
		 * decisions automatically based on the opcode; we don't
		 * need explicit toggles like SDHCI's CMDTM_CRC_CHECK /
		 * INDEX_CHECK. */
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		/* R1b/R5b would set SDCMD_BUSYWAIT, which makes the
		 * hardware spin on DAT0 until the card releases busy.
		 * Without IRQ-driven completion + a busy timeout that
		 * can hang. Refuse for now -- matches the Arasan v1
		 * driver. Callers (e.g. CMD7) can request R1; the next
		 * command will inhibit until the card finishes whatever
		 * R1b would have waited for. */
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	sys_write32(cmd->arg, base + SDARG);
	sys_write32(sdcmd | SDCMD_NEW_FLAG, base + SDCMD);

	ret = sdhost_wait_cmd_done(dev, cmd_timeout_ms);
	if (ret != 0) {
		LOG_ERR("%s CMD%u arg=0x%08x: NEW_FLAG never cleared",
			dev->name, cmd->opcode, cmd->arg);
		return ret;
	}

	/* Re-read SDCMD; FAIL_FLAG signals the controller hit a CRC /
	 * timeout / etc. error during the command. */
	sdcmd = sys_read32(base + SDCMD);
	if (sdcmd & SDCMD_FAIL_FLAG) {
		sdhsts = sys_read32(base + SDHSTS);
		sys_write32(sdhsts & SDHSTS_W1C_ALL, base + SDHSTS);

		if (sdhsts & SDHSTS_CMD_TIME_OUT) {
			/* CMD8 on legacy SD 1.x cards times out by spec;
			 * the sd subsystem retries with the legacy path.
			 * Don't spam ERR for the expected case. */
			LOG_DBG("%s CMD%u arg=0x%08x: timeout (sdhsts=0x%08x)",
				dev->name, cmd->opcode, cmd->arg, sdhsts);
			return -ETIMEDOUT;
		}
		LOG_ERR("%s CMD%u arg=0x%08x: failed sdhsts=0x%08x sdcmd=0x%08x",
			dev->name, cmd->opcode, cmd->arg, sdhsts, sdcmd);
		return -EIO;
	}

	/* Read the response in straight order: response[0] = SDRSP0 =
	 * card's [31:0]. Matches the Arasan driver's convention; opposite
	 * of Linux (which swaps for the MMC layer's high-bits-first
	 * response[] array).
	 */
	if (rsp != SD_RSP_TYPE_NONE) {
		cmd->response[0] = sys_read32(base + SDRSP0);
		if (rsp == SD_RSP_TYPE_R2) {
			cmd->response[1] = sys_read32(base + SDRSP1);
			cmd->response[2] = sys_read32(base + SDRSP2);
			cmd->response[3] = sys_read32(base + SDRSP3);
		}
	}

	return 0;
}

static int sdhc_bcm2835_sdhost_get_card_present(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* Pi Zero 2W's microSD slot has no card-detect line wired to a
	 * GPIO. Linux's bcm2835.c does the same -- relies on CMD0
	 * timeout (or CMD8/ACMD41 timeout) to signal an empty slot.
	 */
	return 1;
}

static int sdhc_bcm2835_sdhost_card_busy(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* Full version reads DAT0 line state via SDEDM FSM; defer to
	 * a later sub-task. R1b commands return -ENOTSUP for now so
	 * the SD subsystem never needs to poll us between commands.
	 */
	return 0;
}

static int sdhc_bcm2835_sdhost_get_host_props(const struct device *dev,
					      struct sdhc_host_props *props)
{
	const struct sdhc_bcm2835_sdhost_config *cfg = dev->config;

	memset(props, 0, sizeof(*props));
	props->f_min = cfg->clock_freq / SDCDIV_MAX_CDIV;
	props->f_max = MIN(cfg->clock_freq / 2, SDHOST_F_MAX_HZ);
	props->host_caps.max_blk_len = 1;	/* 1 = 1024-byte blocks */
	props->host_caps.high_spd_support = true;
	props->host_caps.vol_330_support = true;
	props->bus_4_bit_support = (cfg->bus_width == 4);
	props->is_spi = false;
	return 0;
}

#ifdef CONFIG_SDHC_BCM2835_SDHOST_SELFTEST
/* Bring-up self-test: fires CMD0 -> CMD8 -> ACMD41 OCR loop -> CMD2
 * (CID) -> CMD3 (RCA) -> CMD9 (CSD) -> CMD7 (SELECT) through the
 * driver's own request() path. Logs each step and computes capacity
 * from the CSD. Failures log-only so init still returns 0 (the device
 * stays available for later layers). Drop the Kconfig once the SD
 * subsystem is wired up and exercises the same path.
 *
 * CMD7 is sent as R1 (not R1b) because the driver doesn't support
 * R1b yet -- same pattern as the Arasan SDIO selftest.
 */
static void sdhost_selftest(const struct device *dev)
{
	struct sdhc_command cmd;
	struct sdhc_io ios;
	uint32_t ocr;
	uint16_t rca;
	bool sd_v2;
	uint32_t c_size;
	uint64_t capacity_mb;
	int i;
	int ret;

	LOG_INF("--- SDHost SD-card bring-up self-test ---");

	/* Bring the bus up to safe ident state: 400 kHz, 1-bit, 3.3V. */
	ios = (struct sdhc_io){
		.clock = SDMMC_CLOCK_400KHZ,
		.bus_width = SDHC_BUS_WIDTH1BIT,
		.power_mode = SDHC_POWER_ON,
		.signal_voltage = SD_VOL_3_3_V,
	};
	ret = sdhc_bcm2835_sdhost_set_io(dev, &ios);
	if (ret != 0) {
		LOG_ERR("set_io(400kHz/1-bit) failed: %d", ret);
		return;
	}

	/* CMD0 GO_IDLE_STATE: no response */
	cmd = (struct sdhc_command){
		.opcode = SD_GO_IDLE_STATE,
		.response_type = SD_RSP_TYPE_NONE,
	};
	ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD0 GO_IDLE failed: %d (no card?)", ret);
		return;
	}
	LOG_INF("CMD0 GO_IDLE: ok");

	/* CMD8 SEND_IF_COND: SD 2.0 distinguisher. Arg = VHS=1 +
	 * check-pattern 0xAA. SD 2.0 cards echo back; SD 1.x times out.
	 */
	cmd = (struct sdhc_command){
		.opcode = SD_SEND_IF_COND,
		.arg = 0x1AA,
		.response_type = SD_RSP_TYPE_R7,
	};
	ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
	if (ret == -ETIMEDOUT) {
		LOG_INF("CMD8 SEND_IF_COND: timeout (legacy SD 1.x card)");
		sd_v2 = false;
	} else if (ret != 0) {
		LOG_ERR("CMD8 SEND_IF_COND failed: %d", ret);
		return;
	} else {
		LOG_INF("CMD8 SEND_IF_COND: resp=0x%08x (SD 2.0)",
			cmd.response[0]);
		sd_v2 = true;
	}

	/* ACMD41 = CMD55 (APP_CMD) + CMD41 (SD_SEND_OP_COND). HCS=1
	 * advertises host high-capacity support; voltage window
	 * 0xFF8000 spans 2.7-3.6V. Loop until BUSY bit (bit 31) clears.
	 */
	for (i = 0; i < 100; i++) {
		cmd = (struct sdhc_command){
			.opcode = SD_APP_CMD,
			.arg = 0,
			.response_type = SD_RSP_TYPE_R1,
		};
		ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
		if (ret != 0) {
			LOG_ERR("CMD55 APP_CMD failed @ iter %d: %d", i, ret);
			return;
		}
		cmd = (struct sdhc_command){
			.opcode = SD_APP_SEND_OP_COND,
			.arg = (sd_v2 ? 0x40FF8000U : 0x00FF8000U),
			.response_type = SD_RSP_TYPE_R3,
		};
		ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
		if (ret != 0) {
			LOG_ERR("ACMD41 failed @ iter %d: %d", i, ret);
			return;
		}
		if (cmd.response[0] & 0x80000000U) {
			break;
		}
		k_msleep(10);
	}
	if (!(cmd.response[0] & 0x80000000U)) {
		LOG_ERR("ACMD41: card never reported ready (last OCR=0x%08x)",
			cmd.response[0]);
		return;
	}
	ocr = cmd.response[0];
	LOG_INF("ACMD41 SEND_OP_COND: OCR=0x%08x ready (CCS=%u)",
		ocr, !!(ocr & 0x40000000U));

	/* CMD2 ALL_SEND_CID: R2 long response, CID in response[0..3] */
	cmd = (struct sdhc_command){
		.opcode = SD_ALL_SEND_CID,
		.response_type = SD_RSP_TYPE_R2,
	};
	ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD2 ALL_SEND_CID failed: %d", ret);
		return;
	}
	LOG_INF("CMD2 ALL_SEND_CID: CID=0x%08x_%08x_%08x_%08x (MID=0x%02x)",
		cmd.response[3], cmd.response[2], cmd.response[1],
		cmd.response[0], (cmd.response[3] >> 24) & 0xff);

	/* CMD3 SEND_RELATIVE_ADDR: R6, RCA in response[0] bits 31:16 */
	cmd = (struct sdhc_command){
		.opcode = SD_SEND_RELATIVE_ADDR,
		.response_type = SD_RSP_TYPE_R6,
	};
	ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD3 SEND_RELATIVE_ADDR failed: %d", ret);
		return;
	}
	rca = cmd.response[0] >> 16;
	LOG_INF("CMD3 SEND_RELATIVE_ADDR: RCA=0x%04x", rca);

	/* CMD9 SEND_CSD: R2 long response, CSD in response[0..3].
	 * CSD layout (straight order, response[3] = high bits):
	 *   csd_v1: response[3] bits [29:24] = 0
	 *   csd_v2: response[3] bits [29:24] = 1 (== 0x40000000 set)
	 *
	 * For csd_v2 (SDHC/SDXC):
	 *   C_SIZE is CSD bits [69:48], spanning response[1] bits 31:16
	 *   and response[2] bits 5:0. Capacity = (C_SIZE + 1) * 512 KB.
	 *
	 * For csd_v1: more involved (C_SIZE_MULT + READ_BL_LEN); skip
	 * full decode in the selftest -- just log the raw CSD.
	 */
	cmd = (struct sdhc_command){
		.opcode = SD_SEND_CSD,
		.arg = ((uint32_t)rca) << 16,
		.response_type = SD_RSP_TYPE_R2,
	};
	ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD9 SEND_CSD failed: %d", ret);
		return;
	}
	LOG_INF("CMD9 SEND_CSD: CSD=0x%08x_%08x_%08x_%08x",
		cmd.response[3], cmd.response[2], cmd.response[1],
		cmd.response[0]);
	if ((cmd.response[3] >> 30) == 1) {
		c_size = ((cmd.response[2] & 0x3F) << 16) |
			 (cmd.response[1] >> 16);
		capacity_mb = ((uint64_t)c_size + 1) * 512ULL / 1024ULL;
		LOG_INF("  csd_v2: C_SIZE=%u -> capacity ~ %llu MiB",
			c_size, capacity_mb);
	} else {
		LOG_INF("  csd_v1: full decode skipped in selftest");
	}

	/* CMD7 SELECT_CARD: R1 (not R1b -- see driver header). After
	 * SELECT the card enters transfer state and is ready for data.
	 */
	cmd = (struct sdhc_command){
		.opcode = SD_SELECT_CARD,
		.arg = ((uint32_t)rca) << 16,
		.response_type = SD_RSP_TYPE_R1,
	};
	ret = sdhc_bcm2835_sdhost_request(dev, &cmd, NULL);
	if (ret != 0) {
		LOG_ERR("CMD7 SELECT_CARD failed: %d", ret);
		return;
	}
	LOG_INF("CMD7 SELECT_CARD: ok (status=0x%08x)", cmd.response[0]);

	/* Bus up to 25 MHz / 4-bit for the next layer. Not strictly
	 * required for the selftest -- this just exercises the set_io
	 * path one more time with non-default args.
	 */
	ios.clock = SD_CLOCK_25MHZ;
	ios.bus_width = SDHC_BUS_WIDTH4BIT;
	ret = sdhc_bcm2835_sdhost_set_io(dev, &ios);
	if (ret != 0) {
		LOG_ERR("set_io(25MHz/4-bit) failed: %d", ret);
		return;
	}
	LOG_INF("set_io: 25 MHz, 4-bit, 3.3V");

	LOG_INF("--- selftest complete ---");
}
#endif /* CONFIG_SDHC_BCM2835_SDHOST_SELFTEST */

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

#ifdef CONFIG_SDHC_BCM2835_SDHOST_SELFTEST
	sdhost_selftest(dev);
#endif

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
