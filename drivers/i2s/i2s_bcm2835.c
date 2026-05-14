/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 / BCM2710 / BCM2837 PCM / I2S controller.
 *
 * Implements the Zephyr I2S API for the PCM audio interface on the
 * Raspberry Pi 40-pin header (GPIO 18..21, ALT0). The peripheral is a
 * continuously-clocked serial engine with 64-entry TX and RX FIFOs; it
 * is fed and drained through the BCM2835 DMA controller, with the PCM
 * TX/RX DREQ lines pacing the transfers so the CPU only wakes at buffer
 * boundaries.
 *
 * Scope (first cut): master mode (the peripheral drives the bit clock
 * and frame sync), I2S (Philips) format, 16-bit stereo, frame-packed
 * (two 16-bit samples per 32-bit FIFO word). This covers
 * samples/drivers/i2s/echo. Target (slave) clocking, non-I2S formats,
 * word sizes other than 16, and the peripheral's own TXW/RXR/error IRQ
 * are not implemented -- under/overrun is surfaced through the DMA
 * completion path instead.
 *
 * Three things shape the driver:
 *
 *   - The bit clock is not in this peripheral. In master mode the
 *     BCLK comes from the SoC clock manager (CPRMAN), a separate
 *     register block. cm_pcm_set_rate() programs that divider directly
 *     off the 19.2 MHz oscillator -- there is no Zephyr clock-control
 *     driver for CPRMAN, so the I2S driver owns this for now.
 *
 *   - DMA is single-block. The BCM2835 DMA driver runs one control
 *     block per transfer; this driver ping-pongs by re-arming from the
 *     DMA completion callback (dma_reload + dma_start). The 64-entry
 *     FIFO absorbs the re-arm latency. True gapless double-buffering
 *     would need chained control blocks in the DMA driver.
 *
 *   - Buffer ownership follows the Zephyr I2S contract: TX buffers
 *     arrive from the application via i2s_write() and are freed once
 *     transmitted; RX buffers are allocated from the slab, filled by
 *     DMA, and handed back via i2s_read(). A pair of k_msgq queues per
 *     direction tracks them -- the same shape as i2s_mcux_sai.c.
 *
 * Reference: Linux sound/soc/bcm/bcm2835-i2s.c (register map, frame
 * geometry, FIFO-clear handshake) and drivers/clk/bcm/clk-bcm2835.c
 * (PCM clock divider); BCM2835 ARM Peripherals datasheet ch. 8 (PCM).
 */

#define DT_DRV_COMPAT brcm_bcm2835_i2s

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2s_bcm2835, CONFIG_I2S_LOG_LEVEL);

/* PCM register offsets (BCM2835 ARM Peripherals datasheet ch. 8). */
#define I2S_CS_A     0x00U /* control and status */
#define I2S_FIFO_A   0x04U /* TX / RX data FIFO port */
#define I2S_MODE_A   0x08U /* mode: frame + clock format */
#define I2S_RXC_A    0x0cU /* RX channel 1 / 2 config */
#define I2S_TXC_A    0x10U /* TX channel 1 / 2 config */
#define I2S_DREQ_A   0x14U /* DMA request thresholds */
#define I2S_INTEN_A  0x18U /* interrupt enables */
#define I2S_INTSTC_A 0x1cU /* interrupt status (W1C) */
#define I2S_GRAY     0x20U /* GRAY-code mode */

/* CS_A bits. */
#define CS_EN       BIT(0)      /* enable the PCM block */
#define CS_RXON     BIT(1)      /* receive on */
#define CS_TXON     BIT(2)      /* transmit on */
#define CS_TXCLR    BIT(3)      /* clear TX FIFO (2 PCM cycles) */
#define CS_RXCLR    BIT(4)      /* clear RX FIFO (2 PCM cycles) */
#define CS_TXTHR(v) ((v) << 5)  /* TX FIFO DMA-request threshold */
#define CS_RXTHR(v) ((v) << 7)  /* RX FIFO DMA-request threshold */
#define CS_DMAEN    BIT(9)      /* generate TX/RX DREQs */
#define CS_TXERR    BIT(15)     /* TX FIFO error (W1C) */
#define CS_RXERR    BIT(16)     /* RX FIFO error (W1C) */
#define CS_SYNC     BIT(24)     /* sync flag -- FIFO-clear timing handshake */
#define CS_STBY     BIT(25)     /* set to leave standby (4 PCM cycles) */

/* MODE_A bits. */
#define MODE_FSLEN(v) ((v) << 0)  /* frame sync pulse length */
#define MODE_FLEN(v)  ((v) << 10) /* frame length - 1 */
#define MODE_FSI      BIT(20)     /* frame sync inverted (falling edge) */
#define MODE_FSM      BIT(21)     /* frame sync target (slave) */
#define MODE_CLKI     BIT(22)     /* clock inverted */
#define MODE_CLKM     BIT(23)     /* bit clock target (slave) */
#define MODE_FTXP     BIT(24)     /* TX frame-packed (2ch / 32-bit word) */
#define MODE_FRXP     BIT(25)     /* RX frame-packed */

/* RXC_A / TXC_A: channel 1 in bits 31..16, channel 2 in bits 15..0. */
#define CHWID(v) ((v) << 0)  /* channel width: data_bits - 8 */
#define CHPOS(v) ((v) << 4)  /* channel data position within the frame */
#define CHEN     BIT(14)     /* channel enable */
#define CH1(v)   ((v) << 16)
#define CH2(v)   ((v) << 0)

/* DREQ_A: FIFO levels at which a DREQ / panic is asserted. */
#define DREQ_RX(v)       ((v) << 0)
#define DREQ_TX(v)       ((v) << 8)
#define DREQ_RX_PANIC(v) ((v) << 16)
#define DREQ_TX_PANIC(v) ((v) << 24)

/* FIFO DMA-request tuning, mirrored from Linux bcm2835-i2s.c. */
#define I2S_TX_PANIC 0x10U
#define I2S_RX_PANIC 0x30U
#define I2S_TX_DREQ  0x30U
#define I2S_RX_DREQ  0x20U

/* Fixed frame geometry for 16-bit I2S, frame-packed -- the only config
 * this first cut accepts. Two 16-bit slots make a 32-bit frame; the I2S
 * MSB lands one bit clock after the frame-sync edge. Derivation follows
 * Linux bcm2835-i2s.c hw_params for data_length 16 in I2S format.
 */
#define I2S_WORD_BITS  16U
#define I2S_FRAME_BITS 32U /* 2 channels * 16 bits */
#define I2S_CHWID_VAL  8U  /* data_bits - 8 */
#define I2S_CH1_POS    1U  /* MSB one bit clock after frame start */
#define I2S_CH2_POS    17U /* second slot */

/* SoC clock manager (CPRMAN). In master mode the PCM bit clock is
 * generated here, not in the PCM peripheral: CM_PCMCTL / CM_PCMDIV at
 * ARM-physical 0x3f101000 + 0x98 / 0x9c. Every write needs the password
 * in the top byte. The divisor is 12.12 fixed point; a MASH-1 clock
 * uses the fractional part. Sourced from the 19.2 MHz oscillator -- a
 * fixed SoC constant, always running, with no dependency on
 * VPU-programmed PLL state. Reference: Linux drivers/clk/bcm/clk-bcm2835.c.
 */
#define CM_BASE          0x3f101000U
#define CM_SIZE          0x1000U
#define CM_PCMCTL        0x98U
#define CM_PCMDIV        0x9cU
#define CM_PASSWORD      0x5a000000U
#define CM_CTL_SRC_OSC   1U
#define CM_CTL_ENABLE    BIT(4)
#define CM_CTL_BUSY      BIT(7)
#define CM_CTL_FRAC      BIT(9) /* MASH-1: use the fractional divider */
#define CM_DIV_FRAC_BITS 12U
#define CM_OSC_HZ        19200000U

struct i2s_q_entry {
	void *mem_block;
	size_t size;
};

struct i2s_bcm2835_stream {
	enum i2s_state state;
	struct i2s_config cfg;
	uint32_t dma_channel;
	uint32_t dma_dreq;
	bool dma_channel_valid;
	bool last_block; /* STOP requested: stop after the current block */
	struct dma_config dma_cfg;
	struct dma_block_config dma_block;
	struct k_msgq in_queue;
	struct k_msgq out_queue;
};

struct i2s_bcm2835_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pcfg;
	const struct device *dma_dev;
	uint32_t fifo_phys; /* ARM-physical FIFO_A address, for the DMA engine */
	uint32_t tx_dreq;
	uint32_t rx_dreq;
};

struct i2s_bcm2835_data {
	DEVICE_MMIO_RAM;
	mm_reg_t cm_base; /* clock manager window, mapped at init */
	struct i2s_bcm2835_stream tx;
	struct i2s_bcm2835_stream rx;
	struct i2s_q_entry tx_in_msgs[CONFIG_I2S_BCM2835_TX_BLOCK_COUNT];
	struct i2s_q_entry tx_out_msgs[CONFIG_I2S_BCM2835_TX_BLOCK_COUNT];
	struct i2s_q_entry rx_in_msgs[CONFIG_I2S_BCM2835_RX_BLOCK_COUNT];
	struct i2s_q_entry rx_out_msgs[CONFIG_I2S_BCM2835_RX_BLOCK_COUNT];
};

#define DEV_CFG(dev)  ((const struct i2s_bcm2835_config *)(dev)->config)
#define DEV_DATA(dev) ((struct i2s_bcm2835_data *)(dev)->data)

static inline uint32_t i2s_rd(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + off);
}

static inline void i2s_wr(const struct device *dev, uint32_t off, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + off);
}

/* Program the clock manager's PCM clock to bclk_hz off the 19.2 MHz
 * oscillator and enable it. MASH-1 fractional division covers any audio
 * bit-clock rate to within ~0.01%. The stop-before-program sequence
 * mirrors Linux clk-bcm2835.c::bcm2835_clock_set_rate_and_parent.
 */
static void cm_pcm_set_rate(const struct device *dev, uint32_t bclk_hz)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);
	mm_reg_t cm = data->cm_base;
	uint32_t div, divi, divf, ctl;

	div = (uint32_t)(((uint64_t)CM_OSC_HZ << CM_DIV_FRAC_BITS) / bclk_hz);
	divi = div >> CM_DIV_FRAC_BITS;
	divf = div & BIT_MASK(CM_DIV_FRAC_BITS);

	/* A MASH clock clamps the integer divider to a minimum of 2. */
	if (divi < 2U) {
		divi = 2U;
		divf = 0U;
	}

	/* Stop the clock if it is already running and wait for the
	 * divider to finish its current cycle (glitchless stop).
	 */
	ctl = sys_read32(cm + CM_PCMCTL);
	if (ctl & CM_CTL_ENABLE) {
		sys_write32(CM_PASSWORD | (ctl & ~CM_CTL_ENABLE),
			    cm + CM_PCMCTL);
		while (sys_read32(cm + CM_PCMCTL) & CM_CTL_BUSY) {
		}
	}

	/* Source = oscillator; FRAC selects MASH-1 when there is a
	 * fractional part. Program the control and divider, then enable.
	 */
	ctl = CM_CTL_SRC_OSC | (divf ? CM_CTL_FRAC : 0U);
	sys_write32(CM_PASSWORD | ctl, cm + CM_PCMCTL);
	sys_write32(CM_PASSWORD | (divi << CM_DIV_FRAC_BITS) | divf,
		    cm + CM_PCMDIV);
	sys_write32(CM_PASSWORD | ctl | CM_CTL_ENABLE, cm + CM_PCMCTL);
}

static void cm_pcm_stop(const struct device *dev)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);
	mm_reg_t cm = data->cm_base;
	uint32_t ctl = sys_read32(cm + CM_PCMCTL);

	sys_write32(CM_PASSWORD | (ctl & ~CM_CTL_ENABLE), cm + CM_PCMCTL);
	while (sys_read32(cm + CM_PCMCTL) & CM_CTL_BUSY) {
	}
}

/* Clear the requested FIFO(s). The hardware needs ~2 PCM clock cycles
 * for a clear to land; toggling SYNC and waiting for the new value to
 * read back is a hardware-timed delay of exactly that length. Ported
 * from Linux bcm2835-i2s.c::bcm2835_i2s_clear_fifos. The PCM clock must
 * be running -- callers ensure it is.
 */
static void i2s_bcm2835_clear_fifos(const struct device *dev, bool tx, bool rx)
{
	uint32_t cs = i2s_rd(dev, I2S_CS_A);
	uint32_t active = cs & (CS_TXON | CS_RXON);
	uint32_t off = (tx ? CS_TXON : 0U) | (rx ? CS_RXON : 0U);
	uint32_t clr = (tx ? CS_TXCLR : 0U) | (rx ? CS_RXCLR : 0U);
	uint32_t sync;
	int timeout = 1000;

	/* Stop the requested direction(s), then strobe the clear bits. */
	i2s_wr(dev, I2S_CS_A, cs & ~off);
	i2s_wr(dev, I2S_CS_A, i2s_rd(dev, I2S_CS_A) | clr);

	sync = i2s_rd(dev, I2S_CS_A) & CS_SYNC;
	i2s_wr(dev, I2S_CS_A, i2s_rd(dev, I2S_CS_A) ^ CS_SYNC);
	while (--timeout > 0) {
		if ((i2s_rd(dev, I2S_CS_A) & CS_SYNC) != sync) {
			break;
		}
	}
	if (timeout == 0) {
		LOG_WRN("FIFO clear: SYNC handshake timed out");
	}

	/* Restore the TX/RX-on state. */
	i2s_wr(dev, I2S_CS_A,
	       (i2s_rd(dev, I2S_CS_A) & ~(CS_TXON | CS_RXON)) | active);
}

static void i2s_purge_queues(struct i2s_bcm2835_stream *strm)
{
	struct i2s_q_entry entry;

	while (k_msgq_get(&strm->in_queue, &entry, K_NO_WAIT) == 0) {
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
	}
	while (k_msgq_get(&strm->out_queue, &entry, K_NO_WAIT) == 0) {
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
	}
}

static void i2s_tx_disable(const struct device *dev, bool purge)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);

	i2s_wr(dev, I2S_CS_A, i2s_rd(dev, I2S_CS_A) & ~CS_TXON);
	if (data->tx.dma_channel_valid) {
		dma_stop(DEV_CFG(dev)->dma_dev, data->tx.dma_channel);
	}
	if (purge) {
		i2s_purge_queues(&data->tx);
	}
}

static void i2s_rx_disable(const struct device *dev, bool purge)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);

	i2s_wr(dev, I2S_CS_A, i2s_rd(dev, I2S_CS_A) & ~CS_RXON);
	if (data->rx.dma_channel_valid) {
		dma_stop(DEV_CFG(dev)->dma_dev, data->rx.dma_channel);
	}
	if (purge) {
		i2s_purge_queues(&data->rx);
	}
}

/* DMA completion callback for the TX stream. Runs in the DMA ISR: free
 * the block that just finished, pull the next from the queue, and
 * re-arm. An empty queue is a drain completing or an underrun.
 */
static void i2s_dma_tx_cb(const struct device *dma_dev, void *user_data,
			  uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct i2s_bcm2835_config *cfg = DEV_CFG(dev);
	struct i2s_bcm2835_stream *strm = &DEV_DATA(dev)->tx;
	struct i2s_q_entry entry;
	int ret;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	/* Release the block that just finished transmitting. */
	if (k_msgq_get(&strm->out_queue, &entry, K_NO_WAIT) == 0) {
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
	}

	if (status != DMA_STATUS_COMPLETE) {
		LOG_ERR("TX DMA error %d", status);
		strm->state = I2S_STATE_ERROR;
		i2s_tx_disable(dev, false);
		return;
	}

	/* STOP: finish here. */
	if (strm->last_block) {
		strm->state = I2S_STATE_READY;
		i2s_tx_disable(dev, false);
		return;
	}

	/* Pull the next queued block and re-arm the DMA. */
	if (k_msgq_get(&strm->in_queue, &entry, K_NO_WAIT) == 0) {
		ret = dma_reload(cfg->dma_dev, strm->dma_channel,
				 (uint32_t)(uintptr_t)entry.mem_block,
				 cfg->fifo_phys, entry.size);
		if (ret == 0) {
			ret = k_msgq_put(&strm->out_queue, &entry, K_NO_WAIT);
		}
		if (ret == 0) {
			ret = dma_start(cfg->dma_dev, strm->dma_channel);
		}
		if (ret != 0) {
			LOG_ERR("TX re-arm failed: %d", ret);
			k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
			strm->state = I2S_STATE_ERROR;
			i2s_tx_disable(dev, false);
		}
		return;
	}

	/* Queue empty: a drain has completed, or the application failed
	 * to supply the next block in time (underrun).
	 */
	if (strm->state == I2S_STATE_STOPPING) {
		strm->state = I2S_STATE_READY;
	} else {
		LOG_ERR("TX underrun");
		strm->state = I2S_STATE_ERROR;
	}
	i2s_tx_disable(dev, false);
}

/* DMA completion callback for the RX stream. Runs in the DMA ISR: hand
 * the just-filled block to the application, allocate a fresh one, and
 * re-arm. A full out-queue or an exhausted slab is an overrun.
 */
static void i2s_dma_rx_cb(const struct device *dma_dev, void *user_data,
			  uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct i2s_bcm2835_config *cfg = DEV_CFG(dev);
	struct i2s_bcm2835_stream *strm = &DEV_DATA(dev)->rx;
	struct i2s_q_entry entry;
	int ret;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	/* The block the DMA just filled. */
	if (k_msgq_get(&strm->in_queue, &entry, K_NO_WAIT) != 0) {
		LOG_ERR("RX callback with no in-flight block");
		return;
	}

	if (status != DMA_STATUS_COMPLETE) {
		LOG_ERR("RX DMA error %d", status);
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		strm->state = I2S_STATE_ERROR;
		i2s_rx_disable(dev, false);
		return;
	}

	/* Hand the filled block to the application. */
	if (k_msgq_put(&strm->out_queue, &entry, K_NO_WAIT) != 0) {
		LOG_ERR("RX overrun (application not reading)");
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		strm->state = I2S_STATE_ERROR;
		i2s_rx_disable(dev, false);
		return;
	}

	/* STOP / DRAIN: do not re-arm. */
	if (strm->last_block) {
		strm->state = I2S_STATE_READY;
		i2s_rx_disable(dev, false);
		return;
	}

	/* Allocate a fresh block and re-arm the DMA. */
	if (k_mem_slab_alloc(strm->cfg.mem_slab, &entry.mem_block,
			     K_NO_WAIT) != 0) {
		LOG_ERR("RX overrun (slab exhausted)");
		strm->state = I2S_STATE_ERROR;
		i2s_rx_disable(dev, false);
		return;
	}
	entry.size = strm->cfg.block_size;

	ret = dma_reload(cfg->dma_dev, strm->dma_channel, cfg->fifo_phys,
			 (uint32_t)(uintptr_t)entry.mem_block, entry.size);
	if (ret == 0) {
		ret = k_msgq_put(&strm->in_queue, &entry, K_NO_WAIT);
	}
	if (ret == 0) {
		ret = dma_start(cfg->dma_dev, strm->dma_channel);
	}
	if (ret != 0) {
		LOG_ERR("RX re-arm failed: %d", ret);
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		strm->state = I2S_STATE_ERROR;
		i2s_rx_disable(dev, false);
	}
}

static int i2s_tx_stream_start(const struct device *dev)
{
	const struct i2s_bcm2835_config *cfg = DEV_CFG(dev);
	struct i2s_bcm2835_stream *strm = &DEV_DATA(dev)->tx;
	struct i2s_q_entry entry;
	int ret;

	/* The application must have queued at least one block. */
	if (k_msgq_get(&strm->in_queue, &entry, K_NO_WAIT) != 0) {
		LOG_ERR("TX start: no block queued");
		return -EIO;
	}

	memset(&strm->dma_block, 0, sizeof(strm->dma_block));
	strm->dma_block.source_address = (uint32_t)(uintptr_t)entry.mem_block;
	strm->dma_block.dest_address = cfg->fifo_phys;
	strm->dma_block.block_size = entry.size;
	strm->dma_block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	strm->dma_block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	memset(&strm->dma_cfg, 0, sizeof(strm->dma_cfg));
	strm->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	strm->dma_cfg.dma_slot = strm->dma_dreq;
	strm->dma_cfg.complete_callback_en = 1;
	strm->dma_cfg.block_count = 1;
	strm->dma_cfg.head_block = &strm->dma_block;
	strm->dma_cfg.dma_callback = i2s_dma_tx_cb;
	strm->dma_cfg.user_data = (void *)dev;

	ret = dma_config(cfg->dma_dev, strm->dma_channel, &strm->dma_cfg);
	if (ret != 0) {
		LOG_ERR("TX dma_config failed: %d", ret);
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		return ret;
	}

	ret = k_msgq_put(&strm->out_queue, &entry, K_NO_WAIT);
	if (ret != 0) {
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		return ret;
	}

	ret = dma_start(cfg->dma_dev, strm->dma_channel);
	if (ret != 0) {
		LOG_ERR("TX dma_start failed: %d", ret);
		return ret;
	}

	return 0;
}

static int i2s_rx_stream_start(const struct device *dev)
{
	const struct i2s_bcm2835_config *cfg = DEV_CFG(dev);
	struct i2s_bcm2835_stream *strm = &DEV_DATA(dev)->rx;
	struct i2s_q_entry entry;
	int ret;

	ret = k_mem_slab_alloc(strm->cfg.mem_slab, &entry.mem_block, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("RX start: slab alloc failed: %d", ret);
		return -ENOMEM;
	}
	entry.size = strm->cfg.block_size;

	memset(&strm->dma_block, 0, sizeof(strm->dma_block));
	strm->dma_block.source_address = cfg->fifo_phys;
	strm->dma_block.dest_address = (uint32_t)(uintptr_t)entry.mem_block;
	strm->dma_block.block_size = entry.size;
	strm->dma_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	strm->dma_block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	memset(&strm->dma_cfg, 0, sizeof(strm->dma_cfg));
	strm->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	strm->dma_cfg.dma_slot = strm->dma_dreq;
	strm->dma_cfg.complete_callback_en = 1;
	strm->dma_cfg.block_count = 1;
	strm->dma_cfg.head_block = &strm->dma_block;
	strm->dma_cfg.dma_callback = i2s_dma_rx_cb;
	strm->dma_cfg.user_data = (void *)dev;

	ret = dma_config(cfg->dma_dev, strm->dma_channel, &strm->dma_cfg);
	if (ret != 0) {
		LOG_ERR("RX dma_config failed: %d", ret);
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		return ret;
	}

	ret = k_msgq_put(&strm->in_queue, &entry, K_NO_WAIT);
	if (ret != 0) {
		k_mem_slab_free(strm->cfg.mem_slab, entry.mem_block);
		return ret;
	}

	ret = dma_start(cfg->dma_dev, strm->dma_channel);
	if (ret != 0) {
		LOG_ERR("RX dma_start failed: %d", ret);
		return ret;
	}

	return 0;
}

static int i2s_bcm2835_configure(const struct device *dev, enum i2s_dir dir,
				 const struct i2s_config *cfg)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);
	struct i2s_bcm2835_stream *streams[2];
	uint32_t bclk_hz, mode, chcfg;
	int n = 0;

	/* Which stream(s) this call configures. */
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		streams[n++] = &data->tx;
	}
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		streams[n++] = &data->rx;
	}
	if (n == 0) {
		return -EINVAL;
	}

	/* (Re)configuration is only allowed from NOT_READY or READY. */
	for (int i = 0; i < n; i++) {
		if (streams[i]->state != I2S_STATE_NOT_READY &&
		    streams[i]->state != I2S_STATE_READY) {
			LOG_ERR("configure: stream busy (state %d)",
				streams[i]->state);
			return -EINVAL;
		}
	}

	/* frame_clk_freq 0 tears the stream(s) down to NOT_READY. */
	if (cfg->frame_clk_freq == 0U) {
		for (int i = 0; i < n; i++) {
			i2s_purge_queues(streams[i]);
			streams[i]->state = I2S_STATE_NOT_READY;
		}
		if (data->tx.state == I2S_STATE_NOT_READY &&
		    data->rx.state == I2S_STATE_NOT_READY) {
			cm_pcm_stop(dev);
		}
		return 0;
	}

	/* First cut: 16-bit stereo I2S, master, no loopback / ping-pong. */
	if (cfg->word_size != I2S_WORD_BITS) {
		LOG_ERR("only %u-bit word size supported", I2S_WORD_BITS);
		return -EINVAL;
	}
	if (cfg->channels != 2U) {
		LOG_ERR("only 2 channels supported");
		return -EINVAL;
	}
	if ((cfg->format & I2S_FMT_DATA_FORMAT_MASK) !=
	    I2S_FMT_DATA_FORMAT_I2S) {
		LOG_ERR("only I2S (Philips) format supported");
		return -EINVAL;
	}
	if (cfg->options &
	    (I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET)) {
		LOG_ERR("only controller (master) clocking supported");
		return -EINVAL;
	}
	if (cfg->options & (I2S_OPT_LOOPBACK | I2S_OPT_PINGPONG)) {
		LOG_ERR("loopback / ping-pong mode not supported");
		return -EINVAL;
	}
	if (cfg->mem_slab == NULL || cfg->block_size == 0U ||
	    (cfg->block_size % sizeof(uint32_t)) != 0U) {
		LOG_ERR("invalid mem_slab / block_size");
		return -EINVAL;
	}

	/* Bit clock = sample rate * 32 (two 16-bit slots per frame).
	 *
	 * The shared clock and frame format are programmed on every
	 * configure() call. For I2S_DIR_BOTH that is once; for a split
	 * RX-then-TX sequence the second call rewrites identical values
	 * (the format does not depend on direction here) while neither
	 * stream is running, so it is harmless.
	 */
	bclk_hz = cfg->frame_clk_freq * I2S_FRAME_BITS;
	cm_pcm_set_rate(dev, bclk_hz);

	/* MODE_A: 32-bit frame, 16-bit (50% duty) frame sync, frame-packed,
	 * normal clocking, frame sync on the falling edge (I2S). Master
	 * mode leaves CLKM / FSM clear. Matches Linux bcm2835-i2s.c.
	 */
	mode = MODE_FLEN(I2S_FRAME_BITS - 1U) |
	       MODE_FSLEN(I2S_FRAME_BITS / 2U) | MODE_FTXP | MODE_FRXP |
	       MODE_CLKI | MODE_FSI;
	i2s_wr(dev, I2S_MODE_A, mode);

	/* RXC_A / TXC_A: both channels enabled, 16 bits wide, positioned
	 * at bit 1 and bit 17 of the frame.
	 */
	chcfg = CH1(CHEN | CHWID(I2S_CHWID_VAL) | CHPOS(I2S_CH1_POS)) |
		CH2(CHEN | CHWID(I2S_CHWID_VAL) | CHPOS(I2S_CH2_POS));
	i2s_wr(dev, I2S_TXC_A, chcfg);
	i2s_wr(dev, I2S_RXC_A, chcfg);

	/* FIFO DMA-request thresholds and panic levels. */
	i2s_wr(dev, I2S_DREQ_A,
	       DREQ_TX_PANIC(I2S_TX_PANIC) | DREQ_RX_PANIC(I2S_RX_PANIC) |
		       DREQ_TX(I2S_TX_DREQ) | DREQ_RX(I2S_RX_DREQ));

	/* Enable the block and leave standby (needs >= 4 PCM cycles). */
	i2s_wr(dev, I2S_CS_A, CS_EN);
	i2s_wr(dev, I2S_CS_A, i2s_rd(dev, I2S_CS_A) | CS_STBY);
	k_busy_wait(DIV_ROUND_UP(4U * USEC_PER_SEC, bclk_hz) + 1U);

	/* Clear both FIFOs (the PCM clock is running now). */
	i2s_bcm2835_clear_fifos(dev, true, true);

	/* Arm the FIFO DMA-request machinery. */
	i2s_wr(dev, I2S_CS_A,
	       i2s_rd(dev, I2S_CS_A) | CS_TXTHR(1) | CS_RXTHR(1) | CS_DMAEN);

	for (int i = 0; i < n; i++) {
		streams[i]->cfg = *cfg;
		streams[i]->last_block = false;
		streams[i]->state = I2S_STATE_READY;
	}

	return 0;
}

static int i2s_bcm2835_trigger(const struct device *dev, enum i2s_dir dir,
			       enum i2s_trigger_cmd cmd)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);
	bool do_tx = (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH);
	bool do_rx = (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH);
	unsigned int key;
	int ret = 0;

	if (!do_tx && !do_rx) {
		return -EINVAL;
	}

	key = irq_lock();

	switch (cmd) {
	case I2S_TRIGGER_START:
		if ((do_tx && data->tx.state != I2S_STATE_READY) ||
		    (do_rx && data->rx.state != I2S_STATE_READY)) {
			LOG_ERR("START: stream not in READY state");
			ret = -EIO;
			break;
		}

		/* Re-clear the requested FIFO(s): a prior DROP may have
		 * left stale samples behind.
		 */
		i2s_bcm2835_clear_fifos(dev, do_tx, do_rx);

		if (do_tx) {
			ret = i2s_tx_stream_start(dev);
			if (ret != 0) {
				break;
			}
		}
		if (do_rx) {
			ret = i2s_rx_stream_start(dev);
			if (ret != 0) {
				if (do_tx) {
					i2s_tx_disable(dev, false);
				}
				break;
			}
		}

		/* Switch the requested FIFO(s) on together. */
		i2s_wr(dev, I2S_CS_A,
		       i2s_rd(dev, I2S_CS_A) | (do_tx ? CS_TXON : 0U) |
			       (do_rx ? CS_RXON : 0U));

		if (do_tx) {
			data->tx.last_block = false;
			data->tx.state = I2S_STATE_RUNNING;
		}
		if (do_rx) {
			data->rx.last_block = false;
			data->rx.state = I2S_STATE_RUNNING;
		}
		break;

	case I2S_TRIGGER_STOP:
		if ((do_tx && data->tx.state != I2S_STATE_RUNNING) ||
		    (do_rx && data->rx.state != I2S_STATE_RUNNING)) {
			ret = -EIO;
			break;
		}
		/* Stop after the block currently in flight. */
		if (do_tx) {
			data->tx.last_block = true;
		}
		if (do_rx) {
			data->rx.last_block = true;
		}
		break;

	case I2S_TRIGGER_DRAIN:
		if ((do_tx && data->tx.state != I2S_STATE_RUNNING) ||
		    (do_rx && data->rx.state != I2S_STATE_RUNNING)) {
			ret = -EIO;
			break;
		}
		/* TX drains its queue before stopping; RX has nothing to
		 * drain, so DRAIN is equivalent to STOP for it.
		 */
		if (do_tx) {
			data->tx.state = I2S_STATE_STOPPING;
		}
		if (do_rx) {
			data->rx.last_block = true;
		}
		break;

	case I2S_TRIGGER_DROP:
		if ((do_tx && data->tx.state == I2S_STATE_NOT_READY) ||
		    (do_rx && data->rx.state == I2S_STATE_NOT_READY)) {
			ret = -EIO;
			break;
		}
		if (do_tx) {
			i2s_tx_disable(dev, true);
			data->tx.last_block = false;
			data->tx.state = I2S_STATE_READY;
		}
		if (do_rx) {
			i2s_rx_disable(dev, true);
			data->rx.last_block = false;
			data->rx.state = I2S_STATE_READY;
		}
		break;

	case I2S_TRIGGER_PREPARE:
		if ((do_tx && data->tx.state != I2S_STATE_ERROR) ||
		    (do_rx && data->rx.state != I2S_STATE_ERROR)) {
			ret = -EIO;
			break;
		}
		if (do_tx) {
			i2s_tx_disable(dev, true);
			data->tx.last_block = false;
			data->tx.state = I2S_STATE_READY;
		}
		if (do_rx) {
			i2s_rx_disable(dev, true);
			data->rx.last_block = false;
			data->rx.state = I2S_STATE_READY;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	irq_unlock(key);
	return ret;
}

static int i2s_bcm2835_read(const struct device *dev, void **mem_block,
			    size_t *size)
{
	struct i2s_bcm2835_stream *strm = &DEV_DATA(dev)->rx;
	struct i2s_q_entry entry;
	int ret;

	if (strm->state == I2S_STATE_NOT_READY) {
		LOG_ERR("read: RX stream not configured");
		return -EIO;
	}

	/* In the ERROR state, drain whatever is still queued without
	 * blocking; report -EIO once it is empty.
	 */
	ret = k_msgq_get(&strm->out_queue, &entry,
			 (strm->state == I2S_STATE_ERROR)
				 ? K_NO_WAIT
				 : SYS_TIMEOUT_MS(strm->cfg.timeout));
	if (ret != 0) {
		return (strm->state == I2S_STATE_ERROR) ? -EIO : -EAGAIN;
	}

	*mem_block = entry.mem_block;
	*size = entry.size;
	return 0;
}

static int i2s_bcm2835_write(const struct device *dev, void *mem_block,
			     size_t size)
{
	struct i2s_bcm2835_stream *strm = &DEV_DATA(dev)->tx;
	struct i2s_q_entry entry = {.mem_block = mem_block, .size = size};

	if (strm->state != I2S_STATE_READY &&
	    strm->state != I2S_STATE_RUNNING) {
		LOG_ERR("write: TX stream not in READY/RUNNING (state %d)",
			strm->state);
		return -EIO;
	}
	if (size > strm->cfg.block_size) {
		LOG_ERR("write: size %zu exceeds block_size %zu", size,
			strm->cfg.block_size);
		return -EIO;
	}

	return k_msgq_put(&strm->in_queue, &entry,
			  SYS_TIMEOUT_MS(strm->cfg.timeout));
}

static const struct i2s_config *
i2s_bcm2835_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_bcm2835_data *data = DEV_DATA(dev);
	struct i2s_bcm2835_stream *strm =
		(dir == I2S_DIR_RX) ? &data->rx : &data->tx;

	if (strm->state == I2S_STATE_NOT_READY) {
		return NULL;
	}
	return &strm->cfg;
}

static int i2s_bcm2835_init(const struct device *dev)
{
	const struct i2s_bcm2835_config *cfg = DEV_CFG(dev);
	struct i2s_bcm2835_data *data = DEV_DATA(dev);
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* The PCM bit-clock divider lives in the clock manager, a
	 * separate register block; map a window over it.
	 */
	device_map(&data->cm_base, CM_BASE, CM_SIZE, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("DMA controller not ready");
		return -ENODEV;
	}

	/* pinctrl-0 is optional: a board may mux GPIO 18..21 to ALT0
	 * here, or omit it and mux imperatively at runtime (the layered
	 * board-config model, as for SPI0).
	 */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("pinctrl apply failed: %d", ret);
			return ret;
		}
	}

	/* One DMA channel per direction, taken from the channels the DMA
	 * driver's channel mask leaves free.
	 */
	ret = dma_request_channel(cfg->dma_dev, NULL);
	if (ret < 0) {
		LOG_ERR("no DMA channel available for TX: %d", ret);
		return ret;
	}
	data->tx.dma_channel = (uint32_t)ret;
	data->tx.dma_channel_valid = true;
	data->tx.dma_dreq = cfg->tx_dreq;

	ret = dma_request_channel(cfg->dma_dev, NULL);
	if (ret < 0) {
		LOG_ERR("no DMA channel available for RX: %d", ret);
		dma_release_channel(cfg->dma_dev, data->tx.dma_channel);
		data->tx.dma_channel_valid = false;
		return ret;
	}
	data->rx.dma_channel = (uint32_t)ret;
	data->rx.dma_channel_valid = true;
	data->rx.dma_dreq = cfg->rx_dreq;

	k_msgq_init(&data->tx.in_queue, (char *)data->tx_in_msgs,
		    sizeof(struct i2s_q_entry),
		    CONFIG_I2S_BCM2835_TX_BLOCK_COUNT);
	k_msgq_init(&data->tx.out_queue, (char *)data->tx_out_msgs,
		    sizeof(struct i2s_q_entry),
		    CONFIG_I2S_BCM2835_TX_BLOCK_COUNT);
	k_msgq_init(&data->rx.in_queue, (char *)data->rx_in_msgs,
		    sizeof(struct i2s_q_entry),
		    CONFIG_I2S_BCM2835_RX_BLOCK_COUNT);
	k_msgq_init(&data->rx.out_queue, (char *)data->rx_out_msgs,
		    sizeof(struct i2s_q_entry),
		    CONFIG_I2S_BCM2835_RX_BLOCK_COUNT);

	data->tx.state = I2S_STATE_NOT_READY;
	data->rx.state = I2S_STATE_NOT_READY;

	/* Park the block disabled; configure() brings it up. */
	i2s_wr(dev, I2S_CS_A, 0U);

	return 0;
}

static DEVICE_API(i2s, i2s_bcm2835_driver_api) = {
	.configure = i2s_bcm2835_configure,
	.config_get = i2s_bcm2835_config_get,
	.read = i2s_bcm2835_read,
	.write = i2s_bcm2835_write,
	.trigger = i2s_bcm2835_trigger,
};

#define I2S_BCM2835_INIT(n)                                                    \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),                       \
		    (PINCTRL_DT_INST_DEFINE(n);), ())                          \
                                                                               \
	static struct i2s_bcm2835_data i2s_bcm2835_data_##n;                   \
                                                                               \
	static const struct i2s_bcm2835_config i2s_bcm2835_config_##n = {      \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                          \
		.pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),        \
				    (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),        \
				    (NULL)),                                   \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),     \
		.fifo_phys = DT_INST_REG_ADDR(n) + I2S_FIFO_A,                  \
		.tx_dreq = DT_INST_DMAS_CELL_BY_NAME(n, tx, dreq),             \
		.rx_dreq = DT_INST_DMAS_CELL_BY_NAME(n, rx, dreq),             \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, i2s_bcm2835_init, NULL, &i2s_bcm2835_data_##n, \
			      &i2s_bcm2835_config_##n, POST_KERNEL,            \
			      CONFIG_I2S_INIT_PRIORITY,                        \
			      &i2s_bcm2835_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_BCM2835_INIT)
