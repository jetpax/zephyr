/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * P5-M1 proof pattern. The layout diagnoses scanout faults by eye:
 * - white 8px border: clipping / overscan / geometry
 * - corner squares (TL red, TR green, BL blue, BR white): origin,
 *   mirroring, pitch errors (show as shear or wrapped corners)
 * - eight vertical color bars: pixel format and channel order
 * - horizontal grayscale gradient: bit depth, banding, RGB skew
 */

#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(hdmi_pattern, LOG_LEVEL_INF);

/*
 * PHY I2C post-mortem: when display init fails, dump the clock and
 * I2C-master state, rebuild the master setup from scratch (config-port
 * mux, soft reset, SCL dividers, slave address), then drive one PHY
 * write by hand and report every readback. All truth lands in one
 * uart.log pass.
 */
static void hdmi_phy_diag(void)
{
	mm_reg_t ccu, hdmi;
	uint8_t st;

	device_map(&ccu, 0x03001000, 0x1000, K_MEM_CACHE_NONE);
	device_map(&hdmi, 0x06000000, 0x8000, K_MEM_CACHE_NONE);

	LOG_INF("CCU hdmi=0x%08x slow=0x%08x bus/rst=0x%08x",
		sys_read32(ccu + 0xb00), sys_read32(ccu + 0xb04),
		sys_read32(ccu + 0xb1c));
	LOG_INF("jtagcfg=0x%02x softrstz=0x%02x ss_scl=%02x %02x %02x %02x",
		sys_read8(hdmi + 0x3034), sys_read8(hdmi + 0x302a),
		sys_read8(hdmi + 0x302b), sys_read8(hdmi + 0x302c),
		sys_read8(hdmi + 0x302d), sys_read8(hdmi + 0x302e));
	LOG_INF("int=0x%02x ctlint=0x%02x conf0=0x%02x stat0=0x%02x slave=0x%02x",
		sys_read8(hdmi + 0x3027), sys_read8(hdmi + 0x3028),
		sys_read8(hdmi + 0x3000), sys_read8(hdmi + 0x3004),
		sys_read8(hdmi + 0x3020));

	/* full master setup from scratch */
	sys_write8(0x10, hdmi + 0x3034);	/* config port -> I2C */
	sys_write8(0x00, hdmi + 0x302a);	/* master soft reset pulse */
	k_busy_wait(10);
	sys_write8(0x01, hdmi + 0x302a);
	sys_write8(0x00, hdmi + 0x302b);	/* SS SCL HCNT = 96 (4us @24M) */
	sys_write8(0x60, hdmi + 0x302c);
	sys_write8(0x00, hdmi + 0x302d);	/* SS SCL LCNT = 113 (4.7us) */
	sys_write8(0x71, hdmi + 0x302e);
	sys_write8(0x00, hdmi + 0x3029);	/* DIV: standard mode */
	sys_write8(0x20, hdmi + 0x3001);	/* TSTCLR */
	sys_write8(0x69, hdmi + 0x3020);
	sys_write8(0x00, hdmi + 0x3001);
	LOG_INF("re-armed: jtagcfg=0x%02x softrstz=0x%02x slave=0x%02x",
		sys_read8(hdmi + 0x3034), sys_read8(hdmi + 0x302a),
		sys_read8(hdmi + 0x3020));

	/* one manual write: PHY reg 0x06 <= 0x0051 */
	sys_write8(0xff, hdmi + 0x0108);
	sys_write8(0x06, hdmi + 0x3021);
	sys_write8(0x00, hdmi + 0x3022);
	sys_write8(0x51, hdmi + 0x3023);
	sys_write8(0x10, hdmi + 0x3026);
	st = 0;
	for (int ms = 0; ms < 200; ms++) {
		st = sys_read8(hdmi + 0x0108) & 0x3;
		if (st) {
			break;
		}
		k_busy_wait(1000);
	}
	LOG_INF("manual xfer: stat=0x%02x op=0x%02x (2=done 1=err 0=stuck)",
		st, sys_read8(hdmi + 0x3026));
}

#define BORDER 8
#define CORNER 64

static const uint32_t bars[8] = {
	0xffffffff, 0xffffff00, 0xff00ffff, 0xff00ff00,
	0xffff00ff, 0xffff0000, 0xff0000ff, 0xff000000,
};

static uint32_t line[1920];

static void put_row(const struct device *dev, uint16_t y, uint16_t w)
{
	struct display_buffer_descriptor desc = {
		.buf_size = w * 4U,
		.width = w,
		.height = 1,
		.pitch = w,
	};

	display_write(dev, 0, y, &desc, line);
}

int main(void)
{
	const struct device *dev =
		DEVICE_DT_GET_ANY(allwinner_sun50i_h616_display);
	struct display_capabilities caps;
	uint16_t w, h;

	if (dev == NULL || !device_is_ready(dev)) {
		LOG_ERR("display not ready");
		hdmi_phy_diag();
		return 0;
	}
	display_get_capabilities(dev, &caps);
	w = caps.x_resolution;
	h = caps.y_resolution;
	LOG_INF("painting %ux%u", w, h);

	for (uint16_t y = 0; y < h; y++) {
		uint16_t gband_top = h / 2;
		uint16_t gband_bot = 3 * h / 4;

		for (uint16_t x = 0; x < w; x++) {
			uint32_t px;

			if (y < BORDER || y >= h - BORDER || x < BORDER ||
			    x >= w - BORDER) {
				px = 0xffffffff;
			} else if (y < CORNER && x < CORNER) {
				px = 0xffff0000;
			} else if (y < CORNER && x >= w - CORNER) {
				px = 0xff00ff00;
			} else if (y >= h - CORNER && x < CORNER) {
				px = 0xff0000ff;
			} else if (y >= h - CORNER && x >= w - CORNER) {
				px = 0xffffffff;
			} else if (y >= gband_top && y < gband_bot) {
				uint32_t g = (x * 255U) / w;

				px = 0xff000000 | (g << 16) | (g << 8) | g;
			} else {
				px = bars[(x * 8U) / w];
			}
			line[x] = px;
		}
		put_row(dev, y, w);
	}

	LOG_INF("pattern done");

	/* Watch the DE33 state: log any change in the channel/blender
	 * registers or the first framebuffer word, to timestamp the
	 * flash-then-black wipe.
	 */
	{
		mm_reg_t de;
		uint32_t prev[4] = { 0 }, cur[4];
		static const uint32_t off[4] = {
			0x1c1000, 0x1c1010, 0x281000, 0x8100,
		};

		device_map(&de, 0x01000000, 0x400000, K_MEM_CACHE_NONE);
		for (int t = 0; ; t++) {
			for (int i = 0; i < 4; i++) {
				cur[i] = sys_read32(de + off[i]);
			}
			if (memcmp(cur, prev, sizeof(cur)) != 0) {
				LOG_INF("t=%d attr=%08x laddr=%08x pipe=%08x glb=%08x",
					t, cur[0], cur[1], cur[2], cur[3]);
				memcpy(prev, cur, sizeof(cur));
			}
			k_msleep(500);
		}
	}
	return 0;
}
