/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner SMHC SD/MMC host controller, PIO mode.
 *
 * H6-generation register layout (H616/H618): THLDC at 0x100, sample
 * delay at 0x144, FIFO at 0x200. The IP always runs new timing mode
 * (NTSR bit 31); SDR card clocks map 1:1 to the CCU module clock, so
 * everything here is sourced from OSC24M and 24 MHz is the ceiling.
 * Register/flow references: U-Boot drivers/mmc/sunxi_mmc.c, Linux
 * drivers/mmc/host/sunxi-mmc.c.
 */

#define DT_DRV_COMPAT allwinner_sunxi_smhc

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(sdhc_sunxi, CONFIG_SDHC_LOG_LEVEL);

#define SMHC_GCTRL		0x00
#define SMHC_CLKCR		0x04
#define SMHC_TMOUT		0x08
#define SMHC_WIDTH		0x0c
#define SMHC_BLKSZ		0x10
#define SMHC_BYTECNT		0x14
#define SMHC_CMD		0x18
#define SMHC_ARG		0x1c
#define SMHC_RESP0		0x20
#define SMHC_RESP1		0x24
#define SMHC_RESP2		0x28
#define SMHC_RESP3		0x2c
#define SMHC_IMASK		0x30
#define SMHC_RINT		0x38
#define SMHC_STATUS		0x3c
#define SMHC_FTRGLEVEL		0x40
#define SMHC_NTSR		0x5c
#define SMHC_THLDC		0x100
#define SMHC_FIFO		0x200

#define GCTRL_SOFT_RESET	BIT(0)
#define GCTRL_FIFO_RESET	BIT(1)
#define GCTRL_DMA_RESET		BIT(2)
#define GCTRL_RESET_ALL		(GCTRL_SOFT_RESET | GCTRL_FIFO_RESET | \
				 GCTRL_DMA_RESET)
#define GCTRL_ACCESS_BY_AHB	BIT(31)

#define CLKCR_ENABLE		BIT(16)

#define CMD_RESP_EXPIRE		BIT(6)
#define CMD_LONG_RESPONSE	BIT(7)
#define CMD_CHK_RESPONSE_CRC	BIT(8)
#define CMD_DATA_EXPIRE		BIT(9)
#define CMD_WRITE		BIT(10)
#define CMD_AUTO_STOP		BIT(12)
#define CMD_WAIT_PRE_OVER	BIT(13)
#define CMD_SEND_INIT_SEQ	BIT(15)
#define CMD_UPCLK_ONLY		BIT(21)
#define CMD_START		BIT(31)

#define RINT_RESP_ERROR		BIT(1)
#define RINT_COMMAND_DONE	BIT(2)
#define RINT_DATA_OVER		BIT(3)
#define RINT_RESP_CRC_ERROR	BIT(6)
#define RINT_DATA_CRC_ERROR	BIT(7)
#define RINT_RESP_TIMEOUT	BIT(8)
#define RINT_DATA_TIMEOUT	BIT(9)
#define RINT_FIFO_RUN_ERROR	BIT(11)
#define RINT_HARD_WARE_LOCKED	BIT(12)
#define RINT_START_BIT_ERROR	BIT(13)
#define RINT_AUTO_COMMAND_DONE	BIT(14)
#define RINT_END_BIT_ERROR	BIT(15)
#define RINT_ERROR_MASK		(RINT_RESP_ERROR | RINT_RESP_CRC_ERROR | \
				 RINT_DATA_CRC_ERROR | RINT_RESP_TIMEOUT | \
				 RINT_DATA_TIMEOUT | RINT_FIFO_RUN_ERROR | \
				 RINT_HARD_WARE_LOCKED | \
				 RINT_START_BIT_ERROR | RINT_END_BIT_ERROR)

#define STATUS_FIFO_FULL	BIT(3)
#define STATUS_CARD_DATA_BUSY	BIT(9)
#define STATUS_FIFO_LEVEL(r)	(((r) >> 17) & 0x3fff)

#define NTSR_MODE_SEL_NEW	BIT(31)

#define THLDC_READ_EN		BIT(0)
#define THLDC_BSY_CLR_INT_EN	BIT(1)
#define THLDC_READ_THLD(x)	(((x) & 0xfff) << 16)

#define PIO_CFG0		0x00
#define PIO_DRV0		0x14
#define PIO_PULL0		0x1c

#define CMD_TIMEOUT_MS		1000
#define DATA_TIMEOUT_MS		2000
#define BUSY_TIMEOUT_MS		2000

struct sunxi_smhc_config {
	DEVICE_MMIO_ROM;
	const struct device *ccu;
	clock_control_subsys_t ahb_clk;
	clock_control_subsys_t mod_clk;
	struct reset_dt_spec rst;
	uintptr_t pio_bank;
	uint32_t max_freq;
};

struct sunxi_smhc_data {
	DEVICE_MMIO_RAM;
	struct sdhc_io io;
};

static int smhc_poll(mm_reg_t base, uint32_t reg, uint32_t mask,
		     bool set, int32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	for (;;) {
		uint32_t val = sys_read32(base + reg);

		if (set ? (val & mask) : !(val & mask)) {
			return 0;
		}
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}
}

static int smhc_update_clk(mm_reg_t base)
{
	sys_write32(CMD_START | CMD_UPCLK_ONLY | CMD_WAIT_PRE_OVER,
		    base + SMHC_CMD);

	if (smhc_poll(base, SMHC_CMD, CMD_START, false, CMD_TIMEOUT_MS)) {
		return -ETIMEDOUT;
	}
	sys_write32(sys_read32(base + SMHC_RINT), base + SMHC_RINT);
	return 0;
}

static int smhc_set_clock(const struct device *dev, uint32_t hz)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	const struct sunxi_smhc_config *cfg = dev->config;
	uint32_t got = 0;
	int ret;

	sys_write32(sys_read32(base + SMHC_CLKCR) & ~CLKCR_ENABLE,
		    base + SMHC_CLKCR);
	if (smhc_update_clk(base)) {
		return -ETIMEDOUT;
	}

	ret = clock_control_set_rate(cfg->ccu, cfg->mod_clk,
				     (clock_control_subsys_rate_t)(uintptr_t)hz);
	if (ret) {
		return ret;
	}

	sys_write32(sys_read32(base + SMHC_CLKCR) | CLKCR_ENABLE,
		    base + SMHC_CLKCR);
	if (smhc_update_clk(base)) {
		return -ETIMEDOUT;
	}

	(void)clock_control_get_rate(cfg->ccu, cfg->mod_clk, &got);
	LOG_DBG("clock %u Hz (asked %u)", got, hz);
	return 0;
}

static void smhc_recover(mm_reg_t base)
{
	sys_write32(sys_read32(base + SMHC_GCTRL) | GCTRL_RESET_ALL,
		    base + SMHC_GCTRL);
	(void)smhc_poll(base, SMHC_GCTRL, GCTRL_RESET_ALL, false,
			CMD_TIMEOUT_MS);
	(void)smhc_update_clk(base);
	sys_write32(0xffffffff, base + SMHC_RINT);
}

static int smhc_pio_xfer(mm_reg_t base, struct sdhc_data *sd_data,
			 bool is_read)
{
	uint32_t *buf = sd_data->data;
	size_t words = sd_data->blocks * sd_data->block_size / 4;
	int64_t deadline = k_uptime_get() +
		MAX(DATA_TIMEOUT_MS, sd_data->timeout_ms);

	while (words > 0) {
		uint32_t status = sys_read32(base + SMHC_STATUS);

		if (is_read) {
			uint32_t level = STATUS_FIFO_LEVEL(status);

			level = MIN(level, words);
			for (uint32_t i = 0; i < level; i++) {
				*buf++ = sys_read32(base + SMHC_FIFO);
			}
			words -= level;
			if (level > 0) {
				continue;
			}
		} else {
			if (!(status & STATUS_FIFO_FULL)) {
				sys_write32(*buf++, base + SMHC_FIFO);
				words--;
				continue;
			}
		}

		if (sys_read32(base + SMHC_RINT) & RINT_ERROR_MASK) {
			return -EIO;
		}
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int sdhc_sunxi_request(const struct device *dev,
			      struct sdhc_command *cmd,
			      struct sdhc_data *sd_data)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint32_t cmdval = CMD_START;
	uint32_t rsp = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;
	bool is_read = true;
	int ret;

	switch (rsp) {
	case SD_RSP_TYPE_NONE:
		break;
	case SD_RSP_TYPE_R2:
		cmdval |= CMD_RESP_EXPIRE | CMD_LONG_RESPONSE |
			  CMD_CHK_RESPONSE_CRC;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		cmdval |= CMD_RESP_EXPIRE;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R5b:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		cmdval |= CMD_RESP_EXPIRE | CMD_CHK_RESPONSE_CRC;
		break;
	default:
		return -EINVAL;
	}

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		cmdval |= CMD_SEND_INIT_SEQ;
	}

	if (sd_data != NULL) {
		is_read = (cmd->opcode != SD_WRITE_SINGLE_BLOCK &&
			   cmd->opcode != SD_WRITE_MULTIPLE_BLOCK);
		cmdval |= CMD_DATA_EXPIRE | CMD_WAIT_PRE_OVER;
		if (!is_read) {
			cmdval |= CMD_WRITE;
		}
		if (sd_data->blocks > 1) {
			cmdval |= CMD_AUTO_STOP;
		}
		sys_write32(sd_data->block_size, base + SMHC_BLKSZ);
		sys_write32(sd_data->blocks * sd_data->block_size,
			    base + SMHC_BYTECNT);
		sys_write32(sys_read32(base + SMHC_GCTRL) |
			    GCTRL_ACCESS_BY_AHB, base + SMHC_GCTRL);
	}

	sys_write32(cmd->arg, base + SMHC_ARG);
	sys_write32(cmdval | cmd->opcode, base + SMHC_CMD);

	if (sd_data != NULL) {
		ret = smhc_pio_xfer(base, sd_data, is_read);
		if (ret) {
			goto error;
		}
	}

	ret = smhc_poll(base, SMHC_RINT, RINT_COMMAND_DONE, true,
			CMD_TIMEOUT_MS);
	if (ret) {
		goto error;
	}
	if (sys_read32(base + SMHC_RINT) & RINT_ERROR_MASK) {
		ret = (sys_read32(base + SMHC_RINT) & RINT_RESP_TIMEOUT) ?
			-ETIMEDOUT : -EIO;
		goto error;
	}

	if (sd_data != NULL) {
		uint32_t done = (sd_data->blocks > 1) ?
			RINT_AUTO_COMMAND_DONE : RINT_DATA_OVER;

		ret = smhc_poll(base, SMHC_RINT, done, true,
				MAX(DATA_TIMEOUT_MS, sd_data->timeout_ms));
		if (ret) {
			goto error;
		}
		if (sys_read32(base + SMHC_RINT) & RINT_ERROR_MASK) {
			ret = -EIO;
			goto error;
		}
	}

	if (rsp == SD_RSP_TYPE_R1b || rsp == SD_RSP_TYPE_R5b) {
		ret = smhc_poll(base, SMHC_STATUS, STATUS_CARD_DATA_BUSY,
				false, BUSY_TIMEOUT_MS);
		if (ret) {
			goto error;
		}
	}

	if (rsp != SD_RSP_TYPE_NONE) {
		cmd->response[0] = sys_read32(base + SMHC_RESP0);
		if (rsp == SD_RSP_TYPE_R2) {
			cmd->response[1] = sys_read32(base + SMHC_RESP1);
			cmd->response[2] = sys_read32(base + SMHC_RESP2);
			cmd->response[3] = sys_read32(base + SMHC_RESP3);
		}
	}

	sys_write32(0xffffffff, base + SMHC_RINT);
	return 0;

error:
	if (ret == -ETIMEDOUT &&
	    !(sys_read32(base + SMHC_RINT) & ~RINT_RESP_TIMEOUT)) {
		LOG_DBG("cmd%u timeout", cmd->opcode);
	} else {
		LOG_ERR("cmd%u failed: rint=0x%08x ret=%d", cmd->opcode,
			sys_read32(base + SMHC_RINT), ret);
	}
	smhc_recover(base);
	return ret;
}

static int sdhc_sunxi_reset(const struct device *dev)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);

	sys_write32(GCTRL_RESET_ALL, base + SMHC_GCTRL);
	if (smhc_poll(base, SMHC_GCTRL, GCTRL_RESET_ALL, false,
		      CMD_TIMEOUT_MS)) {
		return -ETIMEDOUT;
	}

	sys_write32(sys_read32(base + SMHC_NTSR) | NTSR_MODE_SEL_NEW,
		    base + SMHC_NTSR);
	sys_write32(0xffffffff, base + SMHC_TMOUT);
	sys_write32(THLDC_READ_EN | THLDC_BSY_CLR_INT_EN |
		    THLDC_READ_THLD(512), base + SMHC_THLDC);
	sys_write32(0, base + SMHC_IMASK);
	sys_write32(0xffffffff, base + SMHC_RINT);

	return 0;
}

static int sdhc_sunxi_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sunxi_smhc_data *data = dev->data;
	const struct sunxi_smhc_config *cfg = dev->config;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	int ret;

	if (ios->clock != 0 && ios->clock != data->io.clock) {
		if (ios->clock < 0 || (uint32_t)ios->clock > cfg->max_freq) {
			return -ENOTSUP;
		}
		ret = smhc_set_clock(dev, ios->clock);
		if (ret) {
			return ret;
		}
		data->io.clock = ios->clock;
	}

	if (ios->bus_width != data->io.bus_width) {
		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			sys_write32(0, base + SMHC_WIDTH);
			break;
		case SDHC_BUS_WIDTH4BIT:
			sys_write32(1, base + SMHC_WIDTH);
			break;
		default:
			return -ENOTSUP;
		}
		data->io.bus_width = ios->bus_width;
	}

	return 0;
}

static int sdhc_sunxi_card_busy(const struct device *dev)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);

	return (sys_read32(base + SMHC_STATUS) & STATUS_CARD_DATA_BUSY) ?
		1 : 0;
}

static int sdhc_sunxi_get_card_present(const struct device *dev)
{
	/* No card-detect wiring on the supported boards; the card is
	 * the boot medium, so it is present by construction.
	 */
	ARG_UNUSED(dev);
	return 1;
}

static int sdhc_sunxi_get_host_props(const struct device *dev,
				     struct sdhc_host_props *props)
{
	const struct sunxi_smhc_config *cfg = dev->config;

	memset(props, 0, sizeof(*props));
	props->f_min = SDMMC_CLOCK_400KHZ;
	props->f_max = cfg->max_freq;
	props->power_delay = 5;
	props->host_caps.high_spd_support = true;
	props->host_caps.vol_330_support = true;
	props->bus_4_bit_support = true;

	return 0;
}

static void smhc_pinmux(uintptr_t bank_phys)
{
	mm_reg_t bank;
	uint32_t val;

	device_map(&bank, bank_phys, 0x24, K_MEM_CACHE_NONE);

	/* pins 0..5 -> function 2 (mmc), pull-up, drive level 2 */
	val = sys_read32(bank + PIO_CFG0);
	val = (val & 0xff000000) | 0x00222222;
	sys_write32(val, bank + PIO_CFG0);

	val = sys_read32(bank + PIO_PULL0);
	val = (val & ~GENMASK(11, 0)) | 0x555;
	sys_write32(val, bank + PIO_PULL0);

	val = sys_read32(bank + PIO_DRV0);
	val = (val & ~GENMASK(11, 0)) | 0xaaa;
	sys_write32(val, bank + PIO_DRV0);
}

static int sdhc_sunxi_init(const struct device *dev)
{
	const struct sunxi_smhc_config *cfg = dev->config;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->ccu) || !device_is_ready(cfg->rst.dev)) {
		return -ENODEV;
	}

	if (cfg->pio_bank != 0) {
		smhc_pinmux(cfg->pio_bank);
	}

	/* deassert reset, then open the AHB gate */
	ret = reset_line_deassert(cfg->rst.dev, cfg->rst.id);
	if (ret == 0) {
		ret = clock_control_on(cfg->ccu, cfg->ahb_clk);
	}

	/*
	 * The controller cannot complete a soft reset without a running
	 * module clock, and the watchdog reset used for FEL re-entry
	 * returns the CCU to defaults (clock off). 400 kHz from OSC24M
	 * until set_io programs the real rate.
	 */
	if (ret == 0) {
		ret = clock_control_set_rate(cfg->ccu, cfg->mod_clk,
				(clock_control_subsys_rate_t)400000);
	}
	if (ret) {
		return ret;
	}

	return sdhc_sunxi_reset(dev);
}

static DEVICE_API(sdhc, sdhc_sunxi_api) = {
	.reset = sdhc_sunxi_reset,
	.request = sdhc_sunxi_request,
	.set_io = sdhc_sunxi_set_io,
	.card_busy = sdhc_sunxi_card_busy,
	.get_card_present = sdhc_sunxi_get_card_present,
	.get_host_props = sdhc_sunxi_get_host_props,
};

#define SDHC_SUNXI_INIT(inst)						\
	static const struct sunxi_smhc_config sunxi_smhc_config_##inst = { \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),		\
		.ccu = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, ahb)), \
		.ahb_clk = (clock_control_subsys_t)			\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, ahb, id),	\
		.mod_clk = (clock_control_subsys_t)			\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, mod, id),	\
		.rst = RESET_DT_SPEC_INST_GET(inst),			\
		.pio_bank = DT_INST_PROP_OR(inst, allwinner_pio_bank_reg, 0), \
		.max_freq = DT_INST_PROP(inst, max_bus_freq),		\
	};								\
	static struct sunxi_smhc_data sunxi_smhc_data_##inst;		\
									\
	DEVICE_DT_INST_DEFINE(inst, sdhc_sunxi_init, NULL,		\
			      &sunxi_smhc_data_##inst,			\
			      &sunxi_smhc_config_##inst, POST_KERNEL,	\
			      CONFIG_SDHC_INIT_PRIORITY, &sdhc_sunxi_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_SUNXI_INIT)
