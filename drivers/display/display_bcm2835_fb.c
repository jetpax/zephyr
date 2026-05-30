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
 *   3. SET_DEPTH      -- bits per pixel (32 for ARGB_8888)
 *   4. SET_PIXEL_ORDER-- 0=BGR, 1=RGB (1 matches Zephyr ARGB_8888)
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
 */

#define DT_DRV_COMPAT brcm_bcm2835_fb

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>

#include <rpi_fw.h>

LOG_MODULE_REGISTER(bcm2835_fb, CONFIG_DISPLAY_LOG_LEVEL);

#define BPP 4U

/* Pixel order 1 = RGB, matching Zephyr's PIXEL_FORMAT_ARGB_8888. */
#define BCM2835_FB_PIXEL_ORDER_RGB 1U

/* VC-bus alias mask: lower 30 bits = ARM physical address. */
#define BCM2835_VC_BUS_MASK 0x3FFFFFFFU

struct bcm2835_fb_config {
	/* Requested resolution from DT. 0 / 0 means "ask VC for the
	 * monitor's native size" -- normal case for HDMI.
	 */
	uint32_t req_width;
	uint32_t req_height;
};

struct bcm2835_fb_data {
	uint32_t *fb;
	mm_reg_t fb_map;
	uint32_t width;          /* actual; either DT-provided or auto-detected */
	uint32_t height;
	uint32_t pitch_px;       /* pitch / BPP, for u32 row stride */
	uint32_t fb_size;
};

static int bcm2835_fb_set_pixel_format(const struct device *dev,
				       const enum display_pixel_format format)
{
	ARG_UNUSED(dev);
	switch (format) {
	case PIXEL_FORMAT_ARGB_8888:
		return 0;
	default:
		return -ENOTSUP;
	}
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
	caps->supported_pixel_formats = PIXEL_FORMAT_ARGB_8888;
	caps->screen_info = 0;
	caps->current_pixel_format = PIXEL_FORMAT_ARGB_8888;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int bcm2835_fb_write(const struct device *dev, uint16_t x, uint16_t y,
			    const struct display_buffer_descriptor *desc,
			    const void *buf)
{
	struct bcm2835_fb_data *data = dev->data;
	uint32_t *dst;
	const uint32_t *src;

	if ((x + desc->width > data->width) || (y + desc->height > data->height) ||
	    desc->pitch < desc->width) {
		return -EINVAL;
	}
	if (desc->buf_size < ((size_t)desc->pitch * desc->height * BPP)) {
		return -EINVAL;
	}

	dst = data->fb + x + ((size_t)y * data->pitch_px);
	src = (const uint32_t *)buf;

	for (uint32_t row = 0; row < desc->height; row++) {
		memcpy(dst, src, (size_t)desc->width * BPP);
		dst += data->pitch_px;
		src += desc->pitch;
	}

	return 0;
}

static int bcm2835_fb_read(const struct device *dev, uint16_t x, uint16_t y,
			   const struct display_buffer_descriptor *desc, void *buf)
{
	struct bcm2835_fb_data *data = dev->data;
	uint32_t *src;
	uint32_t *dst;

	if ((x + desc->width > data->width) || (y + desc->height > data->height) ||
	    desc->pitch < desc->width) {
		return -EINVAL;
	}
	if (desc->buf_size < ((size_t)desc->pitch * desc->height * BPP)) {
		return -EINVAL;
	}

	src = data->fb + x + ((size_t)y * data->pitch_px);
	dst = (uint32_t *)buf;

	for (uint32_t row = 0; row < desc->height; row++) {
		memcpy(dst, src, (size_t)desc->width * BPP);
		src += data->pitch_px;
		dst += desc->pitch;
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
	uint32_t w = cfg->req_width;
	uint32_t h = cfg->req_height;
	uint32_t depth = 32U;
	uint32_t order = BCM2835_FB_PIXEL_ORDER_RGB;
	uintptr_t fb_bus = 0;
	uintptr_t fb_phys;
	uint32_t pitch = 0;
	int err;

	if (!device_is_ready(fw)) {
		LOG_ERR("VC firmware not ready");
		return -ENODEV;
	}

	/* DT didn't specify a resolution -- ask VC what the monitor
	 * negotiated at boot (via EDID). The same value is what
	 * `vcgencmd get_lcd_info` reports under Linux. A single-tag GET
	 * doesn't suffer the clamping that forces the FB-setup chain.
	 */
	if (w == 0 || h == 0) {
		uint32_t wh[2] = {0U, 0U};

		err = rpi_fw_transfer(fw, RPI_FW_TAG_FB_GET_PHYSICAL_SIZE, wh, sizeof(wh));
		if (err < 0) {
			LOG_ERR("fb get size failed: %d", err);
			return err;
		}
		w = wh[0];
		h = wh[1];
		if (w == 0 || h == 0) {
			LOG_ERR("VC reports no display (HDMI not connected? "
				"check config.txt: hdmi_force_hotplug=1)");
			return -ENODEV;
		}
	}

	err = rpi_fw_fb_setup(fw, &w, &h, &depth, &order,
			      256U, &fb_bus, &data->fb_size, &pitch);
	if (err < 0) {
		LOG_ERR("fb setup failed: %d", err);
		return err;
	}
	if (fb_bus == 0 || data->fb_size == 0) {
		LOG_ERR("VC returned empty FB (HDMI not connected? "
			"check config.txt: hdmi_force_hotplug=1)");
		return -EIO;
	}
	if (depth != 32U) {
		LOG_ERR("VC refused 32 bpp (got %u)", depth);
		return -EIO;
	}
	if (pitch == 0U || (pitch % BPP) != 0U) {
		LOG_ERR("bad pitch %u", pitch);
		return -EIO;
	}
	data->width = w;
	data->height = h;
	data->pitch_px = pitch / BPP;

	/* VC-bus to ARM-physical translation: VC firmware returns
	 * addresses in the 0xC0000000-aliased range (L2-coherent view);
	 * stripping the alias gives the actual DRAM address the ARM MMU
	 * can map. Both 0x40000000 and 0xC0000000 aliases land at the
	 * same physical RAM on BCM2710, so masking the low 30 bits is
	 * always safe.
	 */
	fb_phys = (uintptr_t)fb_bus & BCM2835_VC_BUS_MASK;

	device_map(&data->fb_map, fb_phys, data->fb_size, K_MEM_CACHE_NONE);
	data->fb = (uint32_t *)data->fb_map;

	return 0;
}

#define BCM2835_FB_INIT(inst)                                                      \
	static struct bcm2835_fb_data bcm2835_fb_data_##inst;                      \
	static const struct bcm2835_fb_config bcm2835_fb_cfg_##inst = {            \
		.req_width = DT_INST_PROP(inst, width),                            \
		.req_height = DT_INST_PROP(inst, height),                          \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, bcm2835_fb_init, NULL,                         \
			      &bcm2835_fb_data_##inst, &bcm2835_fb_cfg_##inst,     \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,           \
			      &bcm2835_fb_api);

DT_INST_FOREACH_STATUS_OKAY(BCM2835_FB_INIT)
