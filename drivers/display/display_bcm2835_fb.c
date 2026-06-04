/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2835/2710 VideoCore-firmware framebuffer display driver.
 *
 * The VideoCore GPU owns HDMI output on the Pi: it brings up the PHY
 * from config.txt at boot, drives the HVS, and continuously scans a
 * framebuffer out to the cable. The ARM side asks for that framebuffer
 * via the VideoCore firmware property-tag mailbox interface (rpi_fw)
 * and writes pixels into the returned memory.
 *
 * Init sequence (matches Linux drivers/video/fbdev/bcm2708_fb.c):
 *
 *   1. SET_PHYS_WH    -- scanout / monitor resolution
 *   2. SET_VIRT_WH    -- framebuffer resolution (= phys for no scrolling)
 *   3. SET_DEPTH      -- bits per pixel (32 = ARGB_8888, 16 = RGB_565)
 *   4. SET_PIXEL_ORDER-- 0=BGR, 1=RGB (1 matches both Zephyr formats)
 *   5. ALLOCATE_BUFFER-- materialises the FB; returns VC-bus base + size
 *   6. GET_PITCH      -- actual bytes-per-row (VC may pad beyond W*BPP)
 *
 * These tags must arrive as one chained property request -- VC clamps
 * SET_VIRT_WH / SET_DEPTH to minimums when they arrive separately --
 * so the chain is built by rpi_fw_fb_setup(). The framebuffer lives in
 * VideoCore-shared DRAM. ALLOCATE_BUFFER returns a VC-bus address in
 * the 0xC0000000-alias range; the ARM physical address is
 * `bus & 0x3FFFFFFF`. Mapped K_MEM_CACHE_NONE so CPU writes land in the
 * buffer without separate cache maintenance, matching VC's uncached
 * scanout view.
 *
 * Selectable pixel format: `pixel-format` DT property picks either
 * PANEL_PIXEL_FORMAT_ARGB_8888 (default, depth 32) or
 * PANEL_PIXEL_FORMAT_RGB_565 (depth 16). 16 bpp halves DRAM bandwidth
 * per frame and lets small embedded renderers (which natively emit
 * RGB565) drop their output straight into the scanout buffer with no
 * per-pixel conversion.
 *
 * Optional HVS scaling: `render-width` / `render-height` DT properties
 * (when non-zero) allocate a virtual framebuffer smaller than the
 * scanout resolution. VC's hardware scaler upscales it to the monitor
 * native size on the fly, so a software renderer can paint a quarter-
 * area buffer (16 bpp ⇒ ~900 KB at 912x492 instead of 3.5 MiB at
 * 1824x984) while the display still receives the full native mode.
 * No scaling when these are absent / 0.
 */

#define DT_DRV_COMPAT brcm_bcm2835_fb

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/dt-bindings/display/panel.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>

#include <rpi_fw.h>

LOG_MODULE_REGISTER(bcm2835_fb, CONFIG_DISPLAY_LOG_LEVEL);

/* Pixel order 1 = RGB. With depth 32 this puts R in bits [23:16] of
 * the ARGB8888 word; with depth 16 it puts R in the top 5 bits of the
 * RGB565 halfword -- both matching the Zephyr PIXEL_FORMAT_* layouts.
 */
#define BCM2835_FB_PIXEL_ORDER_RGB 1U

/* VC-bus alias mask: lower 30 bits = ARM physical address. */
#define BCM2835_VC_BUS_MASK 0x3FFFFFFFU

struct bcm2835_fb_config {
	/* Requested physical (scanout) resolution from DT. 0 / 0 means
	 * "ask VC for the monitor's native size" -- normal case for HDMI.
	 */
	uint32_t req_width;
	uint32_t req_height;
	/* Optional render resolution: when non-zero, VC scales this
	 * framebuffer up to phys on scanout (HVS hardware scaler).
	 * Lets the CPU paint a smaller buffer while the monitor still
	 * receives native-resolution video. 0 / 0 = render at phys.
	 */
	uint32_t render_width;
	uint32_t render_height;
	/* PANEL_PIXEL_FORMAT_* from dt-bindings/display/panel.h. */
	uint32_t panel_format;
};

struct bcm2835_fb_data {
	uint8_t *fb;
	mm_reg_t fb_map;
	uintptr_t fb_phys;       /* ARM physical address of framebuffer */
	uint32_t width;          /* actual; either DT-provided or auto-detected */
	uint32_t height;
	uint32_t pitch;          /* bytes per row (incl. VC end-of-row padding) */
	uint32_t fb_size;
	uint8_t bpp;             /* bytes per pixel: 4 (ARGB8888) or 2 (RGB565) */
	enum display_pixel_format format;
	/* Optional DMA path. dma_dev != NULL && dma_channel valid =>
	 * display_write() uses memory-to-memory DMA on the contiguous
	 * full-frame fast path; CPU memcpy is the fallback.
	 */
	const struct device *dma_dev;
	int dma_channel;
	struct k_sem dma_done;
};

static int bcm2835_fb_set_pixel_format(const struct device *dev,
				       const enum display_pixel_format format)
{
	const struct bcm2835_fb_data *data = dev->data;

	/* Format is fixed at allocate-time (VC picks the buffer size and
	 * pitch from the depth tag). Accept a no-op set to the configured
	 * format, reject anything else.
	 */
	return (format == data->format) ? 0 : -ENOTSUP;
}

static int bcm2835_fb_set_orientation(const struct device *dev,
				      const enum display_orientation orientation)
{
	ARG_UNUSED(dev);
	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static void bcm2835_fb_get_capabilities(const struct device *dev,
					struct display_capabilities *caps)
{
	const struct bcm2835_fb_data *data = dev->data;

	caps->x_resolution = data->width;
	caps->y_resolution = data->height;
	caps->supported_pixel_formats = data->format;
	caps->screen_info = 0;
	caps->current_pixel_format = data->format;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static void bcm2835_fb_dma_cb(const struct device *dma_dev, void *user_data,
			      uint32_t channel, int status)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(status);
	struct bcm2835_fb_data *data = user_data;

	k_sem_give(&data->dma_done);
}

/* Single contiguous DMA copy of the entire blit (source and destination
 * strides match the framebuffer pitch, so all rows are back-to-back in
 * both buffers). Blocks until the DMA completion IRQ fires.
 */
static int bcm2835_fb_dma_blit(const struct device *dev, uintptr_t src,
			       uintptr_t dst, size_t bytes)
{
	struct bcm2835_fb_data *data = dev->data;
	struct dma_block_config blk = {
		.source_address = src,
		.dest_address = dst,
		.block_size = (uint32_t)bytes,
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	};
	struct dma_config dcfg = {
		.channel_direction = MEMORY_TO_MEMORY,
		.block_count = 1U,
		.head_block = &blk,
		.dma_callback = bcm2835_fb_dma_cb,
		.user_data = data,
	};
	int err;

	err = dma_config(data->dma_dev, (uint32_t)data->dma_channel, &dcfg);
	if (err < 0) {
		LOG_ERR("dma_config: %d", err);
		return err;
	}
	err = dma_start(data->dma_dev, (uint32_t)data->dma_channel);
	if (err < 0) {
		LOG_ERR("dma_start: %d", err);
		return err;
	}
	/* Generous timeout -- a 3.5 MiB blit at the lite-channel floor
	 * (~50 MB/s) is ~70 ms; 250 ms covers worst-case contention.
	 */
	return k_sem_take(&data->dma_done, K_MSEC(250));
}

static int bcm2835_fb_write(const struct device *dev, uint16_t x, uint16_t y,
			    const struct display_buffer_descriptor *desc,
			    const void *buf)
{
	struct bcm2835_fb_data *data = dev->data;
	const size_t row_bytes = (size_t)desc->width * data->bpp;
	const size_t src_stride = (size_t)desc->pitch * data->bpp;
	uint8_t *dst;
	const uint8_t *src;

	if ((x + desc->width > data->width) || (y + desc->height > data->height) ||
	    desc->pitch < desc->width) {
		return -EINVAL;
	}
	if (desc->buf_size < src_stride * desc->height) {
		return -EINVAL;
	}

	/* DMA fast path: full-frame contiguous blit (source and
	 * destination strides match the fb pitch, source rows have no
	 * padding gap). Single memory-to-memory DMA -- no per-row CPU
	 * memcpy. Falls through to the loop below if any precondition
	 * isn't met (partial region update, etc.) or DMA isn't wired.
	 */
	if (data->dma_dev != NULL && x == 0 && row_bytes == data->pitch &&
	    src_stride == data->pitch) {
		const uintptr_t src_arm = (uintptr_t)buf;
		const uintptr_t dst_arm = data->fb_phys +
					  (size_t)y * data->pitch;
		const size_t bytes = (size_t)desc->height * data->pitch;
		int err = bcm2835_fb_dma_blit(dev, src_arm, dst_arm, bytes);

		if (err == 0) {
			return 0;
		}
		LOG_WRN("DMA blit failed (%d), falling back to memcpy", err);
		/* Fall through to the CPU path -- the framebuffer state
		 * is undefined after a half-finished DMA, but a memcpy
		 * over the same region restores it.
		 */
	}

	dst = data->fb + (size_t)x * data->bpp + (size_t)y * data->pitch;
	src = buf;

	for (uint32_t row = 0; row < desc->height; row++) {
		memcpy(dst, src, row_bytes);
		dst += data->pitch;
		src += src_stride;
	}

	return 0;
}

static int bcm2835_fb_read(const struct device *dev, uint16_t x, uint16_t y,
			   const struct display_buffer_descriptor *desc, void *buf)
{
	struct bcm2835_fb_data *data = dev->data;
	const size_t row_bytes = (size_t)desc->width * data->bpp;
	const size_t dst_stride = (size_t)desc->pitch * data->bpp;
	const uint8_t *src;
	uint8_t *dst;

	if ((x + desc->width > data->width) || (y + desc->height > data->height) ||
	    desc->pitch < desc->width) {
		return -EINVAL;
	}
	if (desc->buf_size < dst_stride * desc->height) {
		return -EINVAL;
	}

	src = data->fb + (size_t)x * data->bpp + (size_t)y * data->pitch;
	dst = buf;

	for (uint32_t row = 0; row < desc->height; row++) {
		memcpy(dst, src, row_bytes);
		src += data->pitch;
		dst += dst_stride;
	}

	return 0;
}

static DEVICE_API(display, bcm2835_fb_api) = {
	.write = bcm2835_fb_write,
	.read = bcm2835_fb_read,
	.get_capabilities = bcm2835_fb_get_capabilities,
	.set_pixel_format = bcm2835_fb_set_pixel_format,
	.set_orientation = bcm2835_fb_set_orientation,
};

static int bcm2835_fb_init(const struct device *dev)
{
	const struct bcm2835_fb_config *cfg = dev->config;
	struct bcm2835_fb_data *data = dev->data;
	const struct device *fw = DEVICE_DT_GET_ONE(raspberrypi_bcm283x_firmware);
	uint32_t phys_w = cfg->req_width;
	uint32_t phys_h = cfg->req_height;
	uint32_t virt_w = cfg->render_width;   /* 0 = match phys */
	uint32_t virt_h = cfg->render_height;
	uint32_t depth;
	uint32_t order = BCM2835_FB_PIXEL_ORDER_RGB;
	uintptr_t fb_bus = 0;
	uintptr_t fb_phys;
	uint32_t pitch = 0;
	int err;

	switch (cfg->panel_format) {
	case PANEL_PIXEL_FORMAT_ARGB_8888:
		data->format = PIXEL_FORMAT_ARGB_8888;
		data->bpp = 4U;
		depth = 32U;
		break;
	case PANEL_PIXEL_FORMAT_RGB_565:
		data->format = PIXEL_FORMAT_RGB_565;
		data->bpp = 2U;
		depth = 16U;
		break;
	default:
		LOG_ERR("unsupported pixel-format 0x%x (expected ARGB_8888 or RGB_565)",
			cfg->panel_format);
		return -ENOTSUP;
	}

	if (!device_is_ready(fw)) {
		LOG_ERR("VC firmware not ready");
		return -ENODEV;
	}

	/* DT didn't specify a phys resolution -- ask VC what the monitor
	 * negotiated at boot (via EDID). The same value is what
	 * `vcgencmd get_lcd_info` reports under Linux. A single-tag GET
	 * doesn't suffer the clamping that forces the FB-setup chain.
	 */
	if (phys_w == 0 || phys_h == 0) {
		uint32_t wh[2] = {0U, 0U};

		err = rpi_fw_transfer(fw, RPI_FW_TAG_FB_GET_PHYSICAL_SIZE, wh, sizeof(wh));
		if (err < 0) {
			LOG_ERR("fb get size failed: %d", err);
			return err;
		}
		phys_w = wh[0];
		phys_h = wh[1];
		if (phys_w == 0 || phys_h == 0) {
			LOG_ERR("VC reports no display (HDMI not connected? "
				"check config.txt: hdmi_force_hotplug=1)");
			return -ENODEV;
		}
	}

	err = rpi_fw_fb_setup(fw, &phys_w, &phys_h, &virt_w, &virt_h,
			      &depth, &order, 256U,
			      &fb_bus, &data->fb_size, &pitch);
	if (err < 0) {
		LOG_ERR("fb setup failed: %d", err);
		return err;
	}
	if (fb_bus == 0 || data->fb_size == 0) {
		LOG_ERR("VC returned empty FB (HDMI not connected? "
			"check config.txt: hdmi_force_hotplug=1)");
		return -EIO;
	}
	if (depth != (uint32_t)data->bpp * 8U) {
		LOG_ERR("VC refused %u bpp (got %u)", data->bpp * 8U, depth);
		return -EIO;
	}
	if (pitch == 0U || (pitch % data->bpp) != 0U) {
		LOG_ERR("bad pitch %u", pitch);
		return -EIO;
	}
	/* The framebuffer geometry we expose to the display API is the
	 * VIRTUAL size -- that's the memory the caller writes into. VC
	 * scales it up to phys for scanout.
	 */
	data->width = virt_w;
	data->height = virt_h;
	data->pitch = pitch;
	if (virt_w != phys_w || virt_h != phys_h) {
		LOG_INF("HVS scaling: %ux%u virt -> %ux%u phys",
			virt_w, virt_h, phys_w, phys_h);
	}

	/* VC-bus to ARM-physical translation: VC firmware returns
	 * addresses in the 0xC0000000-aliased range (L2-coherent view);
	 * stripping the alias gives the actual DRAM address the ARM MMU
	 * can map. Both 0x40000000 and 0xC0000000 aliases land at the
	 * same physical RAM on BCM2710, so masking the low 30 bits is
	 * always safe.
	 */
	fb_phys = (uintptr_t)fb_bus & BCM2835_VC_BUS_MASK;
	data->fb_phys = fb_phys;

	/* Map Normal Non-Cacheable, not Device-nGnRnE. ARM writes still
	 * bypass the data caches (VC's scanout sees them immediately
	 * from its own bus master), but Normal-NC lets the AXI master
	 * keep multiple writes outstanding and combine bursts -- with
	 * Device-nGnRnE every write must fully reach DRAM before the
	 * next can be issued, capping a DMA-driven blit at ~30 MB/s
	 * regardless of burst settings. Normal-NC unlocks the full
	 * DRAM write throughput.
	 */
	device_map(&data->fb_map, fb_phys, data->fb_size, K_MEM_ARM_NORMAL_NC);
	data->fb = (uint8_t *)data->fb_map;

	k_sem_init(&data->dma_done, 0, 1);
	data->dma_dev = NULL;
	data->dma_channel = -1;

	/* Optional fast path: if a BCM2835 DMA controller is present
	 * and enabled in DT, request any free channel from its mask
	 * for the backbuffer->framebuffer blit. Falls back to CPU
	 * memcpy if no DMA / channel exhaustion / dma_request_channel
	 * fails.
	 */
	const struct device *dma = DEVICE_DT_GET_ANY(brcm_bcm2835_dma);

	if (dma != NULL && device_is_ready(dma)) {
		int ch = dma_request_channel(dma, NULL);

		if (ch >= 0) {
			data->dma_dev = dma;
			data->dma_channel = ch;
			LOG_INF("DMA blit on %s channel %d", dma->name, ch);
		} else {
			LOG_WRN("DMA available but no free channel (%d) -- CPU blit",
				ch);
		}
	}

	return 0;
}

#define BCM2835_FB_INIT(inst)                                                      \
	static struct bcm2835_fb_data bcm2835_fb_data_##inst;                      \
	static const struct bcm2835_fb_config bcm2835_fb_cfg_##inst = {            \
		.req_width = DT_INST_PROP(inst, width),                            \
		.req_height = DT_INST_PROP(inst, height),                          \
		.render_width = DT_INST_PROP(inst, render_width),                  \
		.render_height = DT_INST_PROP(inst, render_height),                \
		.panel_format = DT_INST_PROP(inst, pixel_format),                  \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, bcm2835_fb_init, NULL,                         \
			      &bcm2835_fb_data_##inst, &bcm2835_fb_cfg_##inst,     \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,           \
			      &bcm2835_fb_api);

DT_INST_FOREACH_STATUS_OKAY(BCM2835_FB_INIT)
