/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HDMI smoke test for the BCM2835/2710 VideoCore-firmware framebuffer.
 *
 * Eight full-intensity vertical bars -- Cyan, Magenta, Yellow, Key
 * (black), White, Red, Green, Blue -- sized to fill whatever
 * resolution VC negotiated with the monitor at boot. Drawn once and
 * held; the static image is the smoke test.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hdmi_smoke, LOG_LEVEL_INF);

/* Row scratch buffer -- one scanline of ARGB pixels. Sized for the
 * widest mode VC will hand us on a Pi 3 (1920 px).
 */
#define MAX_W 1920U
static uint32_t row_buf[MAX_W];

/* CMYKWRGB at full intensity. */
static const uint32_t bar_colors[] = {
	0xFF00FFFFU, /* Cyan */
	0xFFFF00FFU, /* Magenta */
	0xFFFFFF00U, /* Yellow */
	0xFF000000U, /* Key (black) */
	0xFFFFFFFFU, /* White */
	0xFFFF0000U, /* Red */
	0xFF00FF00U, /* Green */
	0xFF0000FFU, /* Blue */
};

static void draw_bars(const struct device *dev, uint16_t w, uint16_t h)
{
	const struct display_buffer_descriptor desc = {
		.buf_size = (size_t)w * sizeof(uint32_t),
		.width    = w,
		.height   = 1,
		.pitch    = w,
	};
	const size_t n = ARRAY_SIZE(bar_colors);

	/* Pre-compute one scanline; reuse it for every row. */
	for (uint16_t x = 0; x < w; x++) {
		const size_t band = ((size_t)x * n) / w;

		row_buf[x] = bar_colors[band];
	}

	for (uint16_t y = 0; y < h; y++) {
		(void)display_write(dev, 0, y, &desc, row_buf);
	}
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	struct display_capabilities caps;
	uint16_t w, h;

	if (!device_is_ready(dev)) {
		LOG_ERR("display device not ready");
		return -ENODEV;
	}

	display_get_capabilities(dev, &caps);
	w = caps.x_resolution;
	h = caps.y_resolution;

	if (w == 0 || h == 0) {
		LOG_ERR("display reports zero resolution");
		return -EINVAL;
	}
	if (w > MAX_W) {
		LOG_ERR("display width %u exceeds row buffer (%u)", w, MAX_W);
		return -ENOMEM;
	}

	LOG_INF("drawing CMYKWRGB bars at %u x %u", w, h);
	draw_bars(dev, w, h);
	LOG_INF("done -- holding image");

	k_sleep(K_FOREVER);
	return 0;
}
