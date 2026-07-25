/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner H616/H618 display pipeline: DE33 mixer (one UI channel,
 * XRGB8888) -> TCON-TV0 -> DesignWare HDMI 1.4 TX + Allwinner
 * (H6-style) PHY, fixed mode, no EDID/HPD dependence.
 *
 * Register truth: mainline sun4i-drm (sun4i_tcon TV channel,
 * sun8i_tcon_top, dw-hdmi) plus the out-of-tree DE33 series
 * (sun8i_mixer de33 paths, sun50i_fmt) and the H616 PHY tables;
 * cross-checked against the vendor sun50iw9 disp2 driver and against a
 * regmap-ftrace capture of a working Linux boot on this box. Distilled
 * recipes with derivations: SS/notes/wo-orange-1-m1-recipes.md.
 *
 * Hardware facts that shape this file:
 * - DW-HDMI core registers are BYTES at 1-byte stride; the PHY
 *   wrapper next door is 32-bit at 4-byte stride.
 * - The DE33 mixer is three disjoint register files, not one: layers at
 *   DE + 0x100000, top at DE + 0x8100, display (blender, formatter) at
 *   DE + 0x280000.
 * - DE33 blender routes by LOGICAL channel index (1) while the UI
 *   channel registers live at the PHYSICAL channel base (6).
 * - Mixer state is double-buffered: nothing takes effect until the
 *   commit write to the top block, auto-update clock notwithstanding.
 * - The formatter is part of the scanout path, not optional colour
 *   management: left disabled, the display block emits nothing.
 * - TCON GCTL bit 1 (PAD_SEL) is H616-specific and mandatory, or the
 *   signal never reaches the HDMI pads.
 */

#define DT_DRV_COMPAT allwinner_sun50i_h616_display

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/clock/sun50i-h616-ccu.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(display_de33, CONFIG_DISPLAY_LOG_LEVEL);

/* DE33-internal clock controller (mainline ccu-sun8i-de2 h616/de33
 * layout, at DE base + 0x8000): mod gate 0x00, bus gate 0x04, resets
 * 0x08, mixer0 divider 0x0c, plus the H616 probe quirk pair 0x24/0x28
 * (channel-to-core and port-to-channel routing). Without the quirk
 * pair, the channel/blender register files are disconnected: writes
 * vanish, reads return zero, output stays black. HW-verified live.
 */
#define DE_CLK_MOD_GATE		0x8000
#define DE_CLK_BUS_GATE		0x8004
#define DE_CLK_RESETS		0x8008
#define DE_CLK_MIXER0_DIV	0x800c
#define DE2TCON_MUX		0x8010
#define DE_CHN2CORE_MUX		0x8024
#define DE_PORT2CHN_MUX		0x8028
#define DE_PORT2CHN_VAL		0x0000a980

/* syscon: SRAM C must be mapped to the DE (clear bit 24), or the DE
 * register SRAM is CPU-owned and the DE side reads garbage
 */
#define SYSCON_BASE		0x03000000
#define SYSCON_SRAM_CTRL1	0x04
#define SYSCON_SRAM_C_CPU	BIT(24)
/* DE33 "top" register file (mixer node reg[1], DE base + 0x8100, 0x40
 * bytes): global ctl/status/size/clk plus the double-buffer commit.
 */
#define RTMX_GLB_CTL		0x8100
#define RTMX_GLB_STS		0x8104
#define RTMX_OUT_SIZE		0x8108
#define RTMX_AUTO_CLK		0x810c
#define RTMX_DBUFF		0x8110

/* DE33 formatter, in the display block (reg[2] + 0x5000). Mandatory
 * output stage: the golden kernel programs it with the mode size and
 * full-range (0..4095) limits per channel and enables it last.
 */
#define FMT_BASE		0x285000
#define FMT_CTRL		0x00
#define FMT_SIZE		0x04
#define FMT_LIMIT(c)		(0x20 + 0x04 * (c))
#define FMT_LIMIT_FULL		0x0fff0000

/* blender, at DE base + 0x281000 (display block + DE2_BLD_BASE) */
#define BLD_BASE		0x281000
#define BLD_PIPE_CTL		0x00
#define BLD_FCOLOR(p)		(0x04 + 0x10 * (p))
#define BLD_INSIZE(p)		(0x08 + 0x10 * (p))
#define BLD_COORD(p)		(0x0c + 0x10 * (p))
#define BLD_ROUTE		0x80
#define BLD_BKCOLOR		0x88
#define BLD_OUTSIZE		0x8c
#define BLD_MODE(p)		(0x90 + 0x04 * (p))
#define BLD_OUTCTL		0xfc

/* first UI channel: logical route index 1, physical channel 6. The
 * channel's scaler sits 0x800 below its register file.
 */
#define UI_CH_BASE		(0x100000 + 0x1000 + 6 * 0x20000)

/* DE33 has NO UI scaler: a UI channel scales through the VI scaler
 * (VSU) at channel_base + 0x3000, unlike DE2/DE3 which put a GSU 0x800
 * below the channel. Sizes use the mixer encoding, steps are
 * source/destination ratios in a 20-bit fraction, and the horizontal
 * filter is 8 taps split across two coefficient banks while the
 * vertical is 4 taps in one, over 32 phase entries each.
 */
#define VSU_BASE		(UI_CH_BASE + 0x3000)
#define VSU_CTRL		0x00
#define VSU_CTRL_EN		BIT(0)
#define VSU_CTRL_COEFF_RDY	BIT(4)
#define VSU_SCALE_MODE		0x10
#define VSU_SCALE_MODE_UI	0
#define VSU_OUTSIZE		0x40
#define VSU_YINSIZE		0x80
#define VSU_YHSTEP		0x88
#define VSU_YVSTEP		0x8c
#define VSU_YHPHASE		0x90
#define VSU_YVPHASE		0x98
#define VSU_CINSIZE		0xc0
#define VSU_CHSTEP		0xc8
#define VSU_CVSTEP		0xcc
#define VSU_CHPHASE		0xd0
#define VSU_CVPHASE		0xd8
#define VSU_YHCOEFF0(i)		(0x200 + 0x04 * (i))
#define VSU_YHCOEFF1(i)		(0x300 + 0x04 * (i))
#define VSU_YVCOEFF(i)		(0x400 + 0x04 * (i))
#define VSU_CHCOEFF0(i)		(0x600 + 0x04 * (i))
#define VSU_CHCOEFF1(i)		(0x700 + 0x04 * (i))
#define VSU_CVCOEFF(i)		(0x800 + 0x04 * (i))
#define VSU_COEFF_COUNT		32
#define VSU_STEP_FRAC		20
/* Nearest neighbour: one whole source pixel per phase, which is
 * pixel-exact for an integer factor and matches the software path
 * output bit for bit. Unity is 0x40 in a 6-bit fraction, and it sits in
 * a different tap position in the horizontal and vertical banks.
 * Replacing these three constants with a polyphase table is all it
 * would take to get a smoothing filter.
 */
#define VSU_COEFF_H_UNITY	0x40000000
#define VSU_COEFF_H_ZERO	0x00000000
#define VSU_COEFF_V_UNITY	0x00004000

/* DE2/DE3 keep the UI scaler 0x800 into the channel unit. DE33 does not
 * use it, but the golden kernel still clears it, so keep parity.
 */
#define UI_GSU_LEGACY_CTRL	(UI_CH_BASE - 0x800)
#define UI_ATTR			0x00
#define UI_SIZE			0x04
#define UI_COORD		0x08
#define UI_PITCH		0x0c
#define UI_TOP_LADDR		0x10
#define UI_OVL_SIZE		0x88

#define UI_ATTR_EN		BIT(0)
#define UI_FMT_XRGB8888		(4U << 8)
#define UI_ALPHA(a)		((uint32_t)(a) << 24)

/* The two blocks encode size in OPPOSITE halves: the DE33 mixer packs
 * height high, the TCON packs width high (sun4i SUN4I_TCON1_BASIC0_X is
 * bits 31:16). Using the mixer macro on TCON1_BASIC0/1/2 programs a
 * 1080x1920 active area into a 2200x1125 raster: syncs and pixel clock
 * stay correct, so the PHY locks and the line counter runs, but no
 * valid frame is ever produced and the sink stays dark.
 */
#define DE_SIZE(w, h)		((((h) - 1) << 16) | ((w) - 1))
#define TCON_SIZE(w, h)		((((w) - 1) << 16) | ((h) - 1))

/* TCON-TOP */
#define TOP_PORT_SEL		0x1c
#define TOP_GATE_SRC		0x20

/* TCON-TV (sun4i tcon, TV/channel-1 path) */
#define TCON_GCTL		0x00
#define TCON_GCTL_EN		BIT(31)
#define TCON_GCTL_PAD_SEL	BIT(1)
#define TCON_GCTL_IOMAP_TCON1	BIT(0)
#define TCON_GINT0		0x04
#define TCON_GINT1		0x08
#define TCON0_IO_POL		0x88
#define TCON0_IO_TRI		0x8c
#define TCON1_CTL		0x90
#define TCON1_CTL_EN		BIT(31)
#define TCON1_CTL_CLK_DELAY(d)	(((d) & 0x1f) << 4)
#define TCON1_CTL_SRC_MASK	GENMASK(1, 0)
#define TCON1_CTL_SRC_DE	0
#define TCON1_CTL_SRC_COLORBAR	1
#define TCON1_BASIC0		0x94
#define TCON1_BASIC1		0x98
#define TCON1_BASIC2		0x9c
#define TCON1_BASIC3		0xa0
#define TCON1_BASIC4		0xa4
#define TCON1_BASIC5		0xa8
#define TCON1_IO_TRI		0xf4

/* DW-HDMI core (byte registers) */
#define HDMI_DESIGN_ID		0x0000
#define HDMI_CONFIG2_ID		0x0006
#define HDMI_IH_FC_STAT0	0x0100
#define HDMI_IH_FC_STAT1	0x0101
#define HDMI_IH_FC_STAT2	0x0102
#define HDMI_IH_I2CMPHY_STAT0	0x0108
#define HDMI_IH_MUTE_FC_STAT2	0x0182
#define HDMI_IH_MUTE		0x01ff
#define HDMI_TX_INVID0		0x0200
#define HDMI_TX_INSTUFFING	0x0201
#define HDMI_TX_GYDATA0		0x0202
#define HDMI_VP_PR_CD		0x0801
#define HDMI_VP_STUFF		0x0802
#define HDMI_VP_REMAP		0x0803
#define HDMI_VP_CONF		0x0804
#define HDMI_VP_MASK		0x0807
#define HDMI_FC_INVIDCONF	0x1000
#define HDMI_FC_INHACTV0	0x1001
#define HDMI_FC_INHACTV1	0x1002
#define HDMI_FC_INHBLANK0	0x1003
#define HDMI_FC_INHBLANK1	0x1004
#define HDMI_FC_INVACTV0	0x1005
#define HDMI_FC_INVACTV1	0x1006
#define HDMI_FC_INVBLANK	0x1007
#define HDMI_FC_HSYNCINDELAY0	0x1008
#define HDMI_FC_HSYNCINDELAY1	0x1009
#define HDMI_FC_HSYNCINWIDTH0	0x100a
#define HDMI_FC_HSYNCINWIDTH1	0x100b
#define HDMI_FC_VSYNCINDELAY	0x100c
#define HDMI_FC_VSYNCINWIDTH	0x100d
#define HDMI_FC_CTRLDUR		0x1011
#define HDMI_FC_EXCTRLDUR	0x1012
#define HDMI_FC_EXCTRLSPAC	0x1013
#define HDMI_FC_CH0PREAM	0x1014
#define HDMI_FC_CH1PREAM	0x1015
#define HDMI_FC_CH2PREAM	0x1016
#define HDMI_FC_AVICONF3	0x1017
#define HDMI_FC_GCP		0x1018
#define HDMI_FC_AVICONF0	0x1019
#define HDMI_FC_AVICONF1	0x101a
#define HDMI_FC_AVICONF2	0x101b
#define HDMI_FC_AVIVID		0x101c
#define HDMI_FC_MASK0		0x10d2
#define HDMI_FC_MASK1		0x10d6
#define HDMI_FC_MASK2		0x10da
#define HDMI_FC_PRCONF		0x10e0
#define HDMI_FC_DATAUTO0	0x10b3
#define HDMI_FC_DATAUTO1	0x10b4
#define HDMI_FC_DATAUTO2	0x10b5
#define HDMI_FC_DATAUTO3	0x10b7
#define HDMI_PHY_CONF0		0x3000
#define HDMI_PHY_TST0		0x3001
#define HDMI_PHY_STAT0		0x3004
#define HDMI_PHY_MASK0		0x3006
#define HDMI_PHY_I2CM_SLAVE	0x3020
#define HDMI_PHY_I2CM_ADDRESS	0x3021
#define HDMI_PHY_I2CM_DATAO_1	0x3022
#define HDMI_PHY_I2CM_DATAO_0	0x3023
#define HDMI_PHY_I2CM_OPERATION	0x3026
#define HDMI_PHY_I2CM_INT	0x3027
#define HDMI_PHY_I2CM_CTLINT	0x3028
#define HDMI_PHY_I2CM_SOFTRSTZ	0x302a
/* vendor JTAG_PHY_CONFIG 0xC0D0 / 4: bit4 I2C_JTAGZ routes the PHY
 * config port to the I2C master (cold boots default to JTAG; Linux
 * gets away without this only because U-Boot's bootlogo sets it)
 */
#define HDMI_JTAG_PHY_CONFIG	0x3034
#define JTAG_PHY_CONFIG_I2C	0x10
#define HDMI_MC_CLKDIS		0x4001
#define HDMI_MC_SWRSTZ		0x4002
#define HDMI_MC_FLOWCTRL	0x4004
#define HDMI_MC_PHYRSTZ		0x4005
#define HDMI_MC_LOCKONCLOCK	0x4006
#define HDMI_MC_HEACPHY_RST	0x4007
#define HDMI_A_HDCPCFG0		0x5000
#define HDMI_A_HDCPCFG1		0x5001
#define HDMI_A_VIDPOLCFG	0x5009

#define PHY_CONF0_SELDATAENPOL	BIT(1)
#define PHY_CONF0_SELDIPIF	BIT(0)
#define PHY_CONF0_TXPWRON	BIT(3)
#define PHY_CONF0_PDDQ		BIT(4)
#define PHY_TST0_TSTCLR		BIT(5)
#define PHY_STAT0_TX_LOCK	BIT(0)
#define PHY_I2CM_SLAVE_GEN2	0x69
#define PHY_I2CM_OP_WRITE	0x10
#define PHY_I2CM_OP_READ	0x01
#define HDMI_PHY_I2CM_DATAI_1	0x3024
#define HDMI_PHY_I2CM_DATAI_0	0x3025
#define PHY_CONF0_SVSRET	BIT(5)

/* Synopsys gen2 PHY register addresses (behind the I2C-over-MMIO) */
#define PHYREG_CKCALCTRL	0x05
#define PHYREG_CPCE_CTRL	0x06
#define PHYREG_CKSYMTXCTRL	0x09
#define PHYREG_VLEVCTRL		0x0e
#define PHYREG_CURRCTRL		0x10
#define PHYREG_PLLPHBYCTRL	0x13
#define PHYREG_GMPCTRL		0x15
#define PHYREG_MSM_CTRL		0x17
#define PHYREG_TXTERM		0x19

/* sun8i PHY wrapper (32-bit registers) */
#define SUN8I_PHY_REXT_CTRL	0x0004
#define SUN8I_PHY_REXT_VALUE	0x80c00000

struct de33_mode {
	uint16_t w, h;
	uint16_t hsync, hfp, hbp;
	uint16_t vsync, vfp, vbp;
	uint32_t pixclk;
	uint8_t vic;
};

/* index = DT `mode` enum: 0 = 1080p60, 1 = 720p60, 2 = 576p50 */
static const struct de33_mode de33_modes[] = {
	{ 1920, 1080, 44, 88, 148, 5, 4, 36, 148500000, 16 },
	{ 1280, 720, 40, 110, 220, 5, 5, 20, 74250000, 4 },
	{ 720, 576, 64, 12, 68, 5, 5, 39, 27000000, 17 },
};

/* H616 PHY tables (mpll cpce/gmp, current, term/sym/vlev) per rate */
struct de33_phy_cfg {
	uint32_t pixclk;
	uint16_t cpce, gmp, curr, term, sym, vlev;
};

static const struct de33_phy_cfg de33_phy_cfgs[] = {
	{ 27000000, 0x00b3, 0x0003, 0x0012, 0x0007, 0x8009, 0x02b0 },
	{ 74250000, 0x0072, 0x0003, 0x0013, 0x0004, 0x8019, 0x0290 },
	{ 148500000, 0x0051, 0x0003, 0x0019, 0x0004, 0x8019, 0x0290 },
};

struct de33_config {
	uintptr_t de_phys, top_phys, tcon_phys, hdmi_phys, phy_phys;
	size_t de_size, top_size, tcon_size, hdmi_size, phy_size;
	const struct device *ccu;
	struct reset_dt_spec rst_de, rst_top, rst_tcon, rst_ctrl, rst_phy;
	uint8_t mode_idx;
	bool tcon_pattern;
	uint16_t render_w, render_h;
};

struct de33_data {
	mm_reg_t de, top, tcon, hdmi, phy;
	const struct de33_mode *mode;
	enum display_pixel_format format;
	/* render surface: what callers draw into, scaled up to the scanout
	 * mode by an integer factor and centred (see de33_write)
	 */
	uint16_t render_w, render_h;
	uint16_t scale;
	/* framebuffer geometry: the scanout buffer is render-sized when the
	 * hardware scales, mode-sized when the CPU does
	 */
	uint16_t fb_w, sw_scale, off_x, off_y;
	bool hw_scaled;
	uintptr_t fb_phys[2];
	uint8_t front;
};

/* One instance per SoC; 1080p is the largest supported mode. */
/*
 * Two of them: the DE33 latches the channel's register set at a frame
 * boundary when armed (see RTMX_DBUFF), so pointing the channel at the
 * buffer just finished and arming the commit is an atomic page flip in
 * hardware, with no vblank interrupt and no tearing. Writing a live
 * scanout buffer instead shows a seam on any frame that takes longer
 * than a scanout period, which at 1080p60 is most of them.
 */
static uint32_t de33_fb[2][1920 * 1080] __aligned(64);

/* ---- CCU shorthand -------------------------------------------------- */

#define CCU_CLK(inst, name) \
	(clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(inst, name, id)

static int ccu_on(const struct device *ccu, uint32_t id)
{
	return clock_control_on(ccu, (clock_control_subsys_t)(uintptr_t)id);
}

static int ccu_rate(const struct device *ccu, uint32_t id, uint32_t hz)
{
	return clock_control_set_rate(ccu,
			(clock_control_subsys_t)(uintptr_t)id,
			(clock_control_subsys_rate_t)(uintptr_t)hz);
}

/* ---- DE33 ----------------------------------------------------------- */

static void de33_mixer_init(struct de33_data *data, uintptr_t fb_phys)
{
	const struct de33_mode *m = data->mode;
	mm_reg_t de = data->de;
	mm_reg_t bld = de + BLD_BASE;
	mm_reg_t ui = de + UI_CH_BASE;
	mm_reg_t vsu = de + VSU_BASE;
	mm_reg_t fmt = de + FMT_BASE;
	uint32_t size = DE_SIZE(m->w, m->h);

	/* de33-clk block: gates, reset, divider, then the H616 routing
	 * quirk pair, then the tcon mux at its golden value
	 */
	sys_write32(1, de + DE_CLK_MOD_GATE);
	sys_write32(1, de + DE_CLK_BUS_GATE);
	sys_write32(1, de + DE_CLK_RESETS);
	sys_write32(0, de + DE_CLK_MIXER0_DIV);
	sys_write32(0, de + DE_CHN2CORE_MUX);
	sys_write32(DE_PORT2CHN_VAL, de + DE_PORT2CHN_MUX);
	sys_write32(0xe4, de + DE2TCON_MUX);

	/* global enable + the DE33 auto-update clock (mixer_init) */
	sys_write32(1, de + RTMX_GLB_CTL);
	sys_write32(1, de + RTMX_AUTO_CLK);
	sys_write32(0xff000000, bld + BLD_BKCOLOR);
	sys_write32(BIT(0), bld + BLD_PIPE_CTL);
	sys_write32(0xff000000, bld + BLD_FCOLOR(0));
	for (int p = 0; p < 4; p++) {
		sys_write32(0x03010301, bld + BLD_MODE(p));
	}

	/* mode set: output size, then the formatter */
	sys_write32(size, de + RTMX_OUT_SIZE);
	sys_write32(size, bld + BLD_OUTSIZE);
	sys_write32(sys_read32(bld + BLD_OUTCTL) & ~BIT(1), bld + BLD_OUTCTL);
	sys_write32(0xff000000, bld + BLD_BKCOLOR);
	sys_write32(0xff000000, bld + BLD_FCOLOR(0));

	sys_write32(0, fmt + FMT_CTRL);
	sys_write32(size, fmt + FMT_SIZE);
	for (uint32_t r = 0x08; r <= 0x14; r += 4) {
		sys_write32(0, fmt + r);
	}
	for (int c = 0; c < 3; c++) {
		sys_write32(FMT_LIMIT_FULL, fmt + FMT_LIMIT(c));
	}
	sys_write32(1, fmt + FMT_CTRL);

	/*
	 * plane: the channel always describes the SOURCE, the blender the
	 * destination rectangle. Without scaling those are the same full
	 * screen; with the scaler on, the channel reads the small render
	 * surface and the blender places the scaled result.
	 */
	sys_write32(0, de + UI_GSU_LEGACY_CTRL);

	if (data->hw_scaled) {
		uint32_t out_w = data->render_w * data->scale;
		uint32_t out_h = data->render_h * data->scale;
		uint32_t src = DE_SIZE(data->render_w, data->render_h);
		uint32_t dst = DE_SIZE(out_w, out_h);
		uint32_t step = BIT(VSU_STEP_FRAC) / data->scale;

		sys_write32(src, ui + UI_SIZE);
		sys_write32(src, ui + UI_OVL_SIZE);
		sys_write32(((uint32_t)data->off_y << 16) | data->off_x,
			    bld + BLD_COORD(0));
		sys_write32(dst, bld + BLD_INSIZE(0));

		/* RGB source: UI scale mode, and the chroma registers track
		 * luma because there is no subsampling
		 */
		sys_write32(VSU_SCALE_MODE_UI, vsu + VSU_SCALE_MODE);
		sys_write32(dst, vsu + VSU_OUTSIZE);
		sys_write32(src, vsu + VSU_YINSIZE);
		sys_write32(step, vsu + VSU_YHSTEP);
		sys_write32(step, vsu + VSU_YVSTEP);
		sys_write32(0, vsu + VSU_YHPHASE);
		sys_write32(0, vsu + VSU_YVPHASE);
		sys_write32(src, vsu + VSU_CINSIZE);
		sys_write32(step, vsu + VSU_CHSTEP);
		sys_write32(step, vsu + VSU_CVSTEP);
		sys_write32(0, vsu + VSU_CHPHASE);
		sys_write32(0, vsu + VSU_CVPHASE);
		for (int i = 0; i < VSU_COEFF_COUNT; i++) {
			sys_write32(VSU_COEFF_H_UNITY, vsu + VSU_YHCOEFF0(i));
			sys_write32(VSU_COEFF_H_ZERO, vsu + VSU_YHCOEFF1(i));
			sys_write32(VSU_COEFF_V_UNITY, vsu + VSU_YVCOEFF(i));
			sys_write32(VSU_COEFF_H_UNITY, vsu + VSU_CHCOEFF0(i));
			sys_write32(VSU_COEFF_H_ZERO, vsu + VSU_CHCOEFF1(i));
			sys_write32(VSU_COEFF_V_UNITY, vsu + VSU_CVCOEFF(i));
		}
		sys_write32(VSU_CTRL_EN | VSU_CTRL_COEFF_RDY, vsu + VSU_CTRL);
		sys_write32(data->render_w * 4U, ui + UI_PITCH);
	} else {
		sys_write32(size, ui + UI_SIZE);
		sys_write32(size, ui + UI_OVL_SIZE);
		sys_write32(0, bld + BLD_COORD(0));
		sys_write32(size, bld + BLD_INSIZE(0));
		sys_write32(0, vsu + VSU_CTRL);
		sys_write32(m->w * 4U, ui + UI_PITCH);
	}
	sys_write32((uint32_t)fb_phys, ui + UI_TOP_LADDR);
	sys_write32(UI_ALPHA(0xff) | UI_FMT_XRGB8888 | UI_ATTR_EN,
		    ui + UI_ATTR);
	/* pipe 0 <- logical UI port 1 (physical channel 6) */
	sys_write32(1, bld + BLD_ROUTE);
	sys_write32(BIT(8) | BIT(0), bld + BLD_PIPE_CTL);
	/* DE33 does have a double-buffer commit, in the top block: nothing
	 * written above reaches the active register set without it
	 */
	sys_write32(1, de + RTMX_DBUFF);

	LOG_INF("de: attr=%08x pipe=%08x fmt=%08x", sys_read32(ui + UI_ATTR),
		sys_read32(bld + BLD_PIPE_CTL), sys_read32(fmt + FMT_CTRL));
}

/* ---- TCON ----------------------------------------------------------- */

static void de33_tcon_init(const struct de33_config *cfg,
			   struct de33_data *data)
{
	const struct de33_mode *m = data->mode;
	mm_reg_t top = data->top;
	mm_reg_t tcon = data->tcon;
	uint32_t htotal = m->w + m->hfp + m->hsync + m->hbp;
	uint32_t vtotal = m->h + m->vfp + m->vsync + m->vbp;
	uint32_t bp_h = m->hsync + m->hbp;
	uint32_t bp_v = m->vsync + m->vbp;
	uint32_t delay = MIN(vtotal - m->h - 2, 30);
	uint32_t src = cfg->tcon_pattern ? TCON1_CTL_SRC_COLORBAR :
					   TCON1_CTL_SRC_DE;

	/* tcon-top routing, equivalent to what mainline sun8i_tcon_top
	 * computes. H6-family TCON numbering puts TCON_TV0 at index 2, so
	 * PORT_SEL DE0 (bits 1:0) = 2 (sun8i_tcon_top_de_config: field =
	 * tcon) and GATE_SRC HDMI_SRC (bits 29:28) = 1
	 * (sun8i_tcon_top_set_hdmi_src: field = tcon - 1, and it rejects
	 * anything but tcon 2 or 3). Bit 20 is the TCON_TV0 gate, which
	 * mainline drives from a registered clk gate rather than from the
	 * DRM path. All three are load-bearing: zeroing them stops the
	 * TCON scanning and starves the DW of clocks (proven live). A
	 * regmap-ftrace capture of a working kernel shows none of this
	 * block, because tcon_top uses raw writel; do not read a golden
	 * dump of zeros here as golden's real state.
	 */
	sys_write32(0, top + TOP_PORT_SEL);
	sys_write32(0, top + TOP_GATE_SRC);
	sys_write32(2, top + TOP_PORT_SEL);
	sys_write32((1U << 28) | BIT(20), top + TOP_GATE_SRC);

	/* known state + H616 pad select (mandatory). Both IO_TRI files
	 * stay fully tri-stated: the TV channel feeds the HDMI block
	 * internally, it drives no pads. This is what the golden stream
	 * writes and never revisits; the earlier "un-tristate or no video"
	 * reading was drawn while BASIC0/1/2 were mis-encoded below.
	 */
	sys_write32(0, tcon + TCON_GCTL);
	sys_write32(0, tcon + TCON_GINT0);
	sys_write32(0, tcon + TCON_GINT1);
	sys_write32(0xffffffff, tcon + TCON0_IO_TRI);
	sys_write32(0xffffffff, tcon + TCON1_IO_TRI);
	sys_write32(TCON_GCTL_PAD_SEL, tcon + TCON_GCTL);

	sys_write32(TCON1_CTL_CLK_DELAY(delay) | src, tcon + TCON1_CTL);
	/* input, upscaling and output resolution, all width-high */
	sys_write32(TCON_SIZE(m->w, m->h), tcon + TCON1_BASIC0);
	sys_write32(TCON_SIZE(m->w, m->h), tcon + TCON1_BASIC1);
	sys_write32(TCON_SIZE(m->w, m->h), tcon + TCON1_BASIC2);
	sys_write32(((htotal - 1) << 16) | (bp_h - 1), tcon + TCON1_BASIC3);
	/* V_TOTAL is doubled and NOT minus-one encoded */
	sys_write32((vtotal * 2) << 16 | (bp_v - 1), tcon + TCON1_BASIC4);
	sys_write32(((m->hsync - 1) << 16) | (m->vsync - 1),
		    tcon + TCON1_BASIC5);
	/* golden dump: the working pipeline drives the TV channel with
	 * both syncs negative at the TCON (IO_POL = 0) and matching
	 * low-active polarity bits in the DW frame composer, even for
	 * CEA modes; the monitor accepts it and it is what this die's
	 * path is proven with. Copy it exactly.
	 */
	sys_write32(0, tcon + TCON0_IO_POL);

	sys_write32(TCON_GCTL_PAD_SEL | TCON_GCTL_IOMAP_TCON1, tcon + TCON_GCTL);
	sys_write32(TCON_GCTL_PAD_SEL | TCON_GCTL_IOMAP_TCON1 | TCON_GCTL_EN,
		    tcon + TCON_GCTL);
	sys_write32(sys_read32(tcon + TCON1_CTL) | TCON1_CTL_EN,
		    tcon + TCON1_CTL);
}

/* ---- DW-HDMI (byte registers) --------------------------------------- */

static void hdmi_w8(struct de33_data *data, uint32_t off, uint8_t val)
{
	sys_write8(val, data->hdmi + off);
}

static uint8_t hdmi_r8(struct de33_data *data, uint32_t off)
{
	return sys_read8(data->hdmi + off);
}

/*
 * The PHY I2C master signals completion in IH_I2CMPHY_STAT0 (bit0 done,
 * bit1 error), but only once its own interrupt sources are unmasked:
 * PHY_I2CM_INT/CTLINT must already hold 0x08/0x88 when the transfer
 * runs. Configuring the PHY with those still at 0xff is what produced
 * the long-standing "this controller never latches done" reading. Poll
 * the latch with a bounded timeout and keep the readback check: some
 * PHY registers hold self-clearing or read-only fields, and the final
 * authority on the whole configuration is TX_PHY_LOCK either way.
 */
static bool phy_i2cm_stat_latched;

static uint8_t hdmi_phy_i2c_wait(struct de33_data *data)
{
	for (int us = 0; us < 2000; us += 20) {
		uint8_t stat = hdmi_r8(data, HDMI_IH_I2CMPHY_STAT0) & 0x3;

		if (stat) {
			phy_i2cm_stat_latched = true;
			hdmi_w8(data, HDMI_IH_I2CMPHY_STAT0, stat);
			return stat;
		}
		k_busy_wait(20);
	}
	return 0;
}

static uint16_t hdmi_phy_i2c_read(struct de33_data *data, uint8_t addr)
{
	hdmi_w8(data, HDMI_IH_I2CMPHY_STAT0, 0xff);
	hdmi_w8(data, HDMI_PHY_I2CM_ADDRESS, addr);
	hdmi_w8(data, HDMI_PHY_I2CM_OPERATION, PHY_I2CM_OP_READ);
	hdmi_phy_i2c_wait(data);
	return ((uint16_t)hdmi_r8(data, HDMI_PHY_I2CM_DATAI_1) << 8) |
	       hdmi_r8(data, HDMI_PHY_I2CM_DATAI_0);
}

static int hdmi_phy_i2c_write(struct de33_data *data, uint8_t addr,
			      uint16_t val)
{
	uint16_t got = 0;

	for (int try = 0; try < 3; try++) {
		hdmi_w8(data, HDMI_IH_I2CMPHY_STAT0, 0xff);
		hdmi_w8(data, HDMI_PHY_I2CM_ADDRESS, addr);
		hdmi_w8(data, HDMI_PHY_I2CM_DATAO_1, val >> 8);
		hdmi_w8(data, HDMI_PHY_I2CM_DATAO_0, val & 0xff);
		hdmi_w8(data, HDMI_PHY_I2CM_OPERATION, PHY_I2CM_OP_WRITE);
		hdmi_phy_i2c_wait(data);
		got = hdmi_phy_i2c_read(data, addr);
		if (got == val) {
			return 0;
		}
	}
	/* Some registers hold self-clearing or read-only fields; a
	 * mismatch is worth a log line but must not abort: PHY lock
	 * is the real gate.
	 */
	LOG_WRN("PHY[0x%02x]: wrote 0x%04x reads 0x%04x", addr, val, got);
	return 0;
}

static int hdmi_phy_configure(struct de33_data *data,
			      const struct de33_phy_cfg *pc)
{
	uint8_t conf0;
	int ret;

	/* data-enable polarity, direct interface */
	conf0 = hdmi_r8(data, HDMI_PHY_CONF0);
	conf0 |= PHY_CONF0_SELDATAENPOL;
	conf0 &= ~PHY_CONF0_SELDIPIF;
	hdmi_w8(data, HDMI_PHY_CONF0, conf0);

	/* gen2 power down */
	conf0 &= ~PHY_CONF0_TXPWRON;
	hdmi_w8(data, HDMI_PHY_CONF0, conf0);
	for (int i = 0; i < 5; i++) {
		if (!(hdmi_r8(data, HDMI_PHY_STAT0) & PHY_STAT0_TX_LOCK)) {
			break;
		}
		k_busy_wait(1000);
	}
	conf0 |= PHY_CONF0_PDDQ;
	hdmi_w8(data, HDMI_PHY_CONF0, conf0);

	/* config2 0xf3 = DWC HDMI 2.0 TX PHY, which has SVSRET. Mainline
	 * (= the golden cold-start on this box) asserts SVSRET to leave
	 * retention BEFORE the gen2 reset pulse; no 1->0->1 toggle.
	 */
	conf0 |= PHY_CONF0_SVSRET;
	hdmi_w8(data, HDMI_PHY_CONF0, conf0);
	hdmi_w8(data, HDMI_MC_PHYRSTZ, 0x01);
	hdmi_w8(data, HDMI_MC_PHYRSTZ, 0x00);
	hdmi_w8(data, HDMI_MC_HEACPHY_RST, 0x01);

	/* route the PHY config port to the I2C master and release the
	 * master's soft reset, then latch the gen2 slave address
	 */
	hdmi_w8(data, HDMI_JTAG_PHY_CONFIG, JTAG_PHY_CONFIG_I2C);
	hdmi_w8(data, HDMI_PHY_I2CM_SOFTRSTZ, 0x01);
	hdmi_w8(data, HDMI_PHY_TST0, PHY_TST0_TSTCLR);
	hdmi_w8(data, HDMI_PHY_I2CM_SLAVE, PHY_I2CM_SLAVE_GEN2);
	hdmi_w8(data, HDMI_PHY_TST0, 0);

	{
		static const struct {
			uint8_t addr;
			uint16_t val;
		} *w, writes[9] = {
			{ PHYREG_CPCE_CTRL, 0 }, { PHYREG_GMPCTRL, 0 },
			{ PHYREG_CURRCTRL, 0 }, { PHYREG_PLLPHBYCTRL, 0x0000 },
			{ PHYREG_MSM_CTRL, 0x0006 }, { PHYREG_TXTERM, 0 },
			{ PHYREG_CKSYMTXCTRL, 0 }, { PHYREG_VLEVCTRL, 0 },
			{ PHYREG_CKCALCTRL, 0x8000 },
		};
		const uint16_t vals[9] = {
			pc->cpce, pc->gmp, pc->curr, 0x0000, 0x0006,
			pc->term, pc->sym, pc->vlev, 0x8000,
		};

		LOG_INF("pre-i2c conf0=0x%02x stat0=0x%02x",
			hdmi_r8(data, HDMI_PHY_CONF0),
			hdmi_r8(data, HDMI_PHY_STAT0));
		for (int i = 0; i < 9; i++) {
			w = &writes[i];
			ret = hdmi_phy_i2c_write(data, w->addr, vals[i]);
			if (ret) {
				return ret;
			}
		}
	}

	/* gen2 power up, wait for TX PLL lock. Mainline order: TXPWRON
	 * rises first, PDDQ drops in a separate write.
	 */
	conf0 = hdmi_r8(data, HDMI_PHY_CONF0);
	conf0 |= PHY_CONF0_TXPWRON;
	hdmi_w8(data, HDMI_PHY_CONF0, conf0);
	conf0 &= ~PHY_CONF0_PDDQ;
	hdmi_w8(data, HDMI_PHY_CONF0, conf0);

	for (int i = 0; i < 10; i++) {
		if (hdmi_r8(data, HDMI_PHY_STAT0) & PHY_STAT0_TX_LOCK) {
			return 0;
		}
		k_busy_wait(1000);
	}
	return -ETIMEDOUT;
}

static int de33_hdmi_init(struct de33_data *data)
{
	const struct de33_mode *m = data->mode;
	const struct de33_phy_cfg *pc = NULL;
	uint32_t hblank = m->hfp + m->hsync + m->hbp;
	uint32_t vblank = m->vfp + m->vsync + m->vbp;
	int ret = -EINVAL;

	for (size_t i = 0; i < ARRAY_SIZE(de33_phy_cfgs); i++) {
		if (de33_phy_cfgs[i].pixclk == m->pixclk) {
			pc = &de33_phy_cfgs[i];
		}
	}
	if (pc == NULL) {
		return -ENOTSUP;
	}

	LOG_INF("dw-hdmi design 0x%02x rev 0x%02x config2 0x%02x",
		hdmi_r8(data, HDMI_DESIGN_ID), hdmi_r8(data, 0x0001),
		hdmi_r8(data, HDMI_CONFIG2_ID));

	/* PHY wrapper: the whole H616 glue init is this one write */
	sys_write32(SUN8I_PHY_REXT_VALUE, data->phy + SUN8I_PHY_REXT_CTRL);

	/* mask everything; polled bring-up, no HDMI ISR */
	hdmi_w8(data, HDMI_IH_MUTE, hdmi_r8(data, HDMI_IH_MUTE) | 0x3);
	hdmi_w8(data, HDMI_VP_MASK, 0xff);
	hdmi_w8(data, HDMI_FC_MASK0, 0xff);
	hdmi_w8(data, HDMI_FC_MASK1, 0xff);
	hdmi_w8(data, HDMI_FC_MASK2, 0xff);
	hdmi_w8(data, HDMI_PHY_MASK0, 0xff);
	/* not 0xff: the PHY i2c master's done/error events must be
	 * unmasked before hdmi_phy_configure runs, or its completion
	 * latch stays dead for the whole configuration (golden writes
	 * these at probe, before touching the PHY)
	 */
	hdmi_w8(data, HDMI_PHY_I2CM_INT, 0x08);
	hdmi_w8(data, HDMI_PHY_I2CM_CTLINT, 0x88);
	for (uint32_t r = 0x0180; r <= 0x0189; r++) {
		hdmi_w8(data, r, 0xff);
	}
	hdmi_w8(data, HDMI_IH_MUTE_FC_STAT2, 0x03);

	/* frame composer before PHY power-on; 0x18 = HDMI mode, low
	 * active syncs (matches TCON IO_POL = 0), DE high, progressive
	 * (golden dump value)
	 */
	hdmi_w8(data, HDMI_FC_INVIDCONF, 0x18);
	hdmi_w8(data, HDMI_FC_INHACTV0, m->w & 0xff);
	hdmi_w8(data, HDMI_FC_INHACTV1, m->w >> 8);
	hdmi_w8(data, HDMI_FC_INHBLANK0, hblank & 0xff);
	hdmi_w8(data, HDMI_FC_INHBLANK1, hblank >> 8);
	hdmi_w8(data, HDMI_FC_INVACTV0, m->h & 0xff);
	hdmi_w8(data, HDMI_FC_INVACTV1, m->h >> 8);
	hdmi_w8(data, HDMI_FC_INVBLANK, vblank);
	hdmi_w8(data, HDMI_FC_HSYNCINDELAY0, m->hfp & 0xff);
	hdmi_w8(data, HDMI_FC_HSYNCINDELAY1, m->hfp >> 8);
	hdmi_w8(data, HDMI_FC_HSYNCINWIDTH0, m->hsync & 0xff);
	hdmi_w8(data, HDMI_FC_HSYNCINWIDTH1, m->hsync >> 8);
	hdmi_w8(data, HDMI_FC_VSYNCINDELAY, m->vfp);
	hdmi_w8(data, HDMI_FC_VSYNCINWIDTH, m->vsync);

	/* PHY configure runs twice per the Synopsys note */
	for (int pass = 0; pass < 2; pass++) {
		ret = hdmi_phy_configure(data, pc);
		if (ret) {
			LOG_ERR("PHY configure pass %d: %d", pass, ret);
			return ret;
		}
	}
	LOG_INF("PHY TX PLL locked");

	/* video path enable after lock */
	hdmi_w8(data, HDMI_FC_CTRLDUR, 12);
	hdmi_w8(data, HDMI_FC_EXCTRLDUR, 32);
	hdmi_w8(data, HDMI_FC_EXCTRLSPAC, 1);
	hdmi_w8(data, HDMI_FC_CH0PREAM, 0x0b);
	hdmi_w8(data, HDMI_FC_CH1PREAM, 0x16);
	hdmi_w8(data, HDMI_FC_CH2PREAM, 0x21);
	hdmi_w8(data, HDMI_MC_CLKDIS, 0x7e);
	hdmi_w8(data, HDMI_MC_CLKDIS, 0x7c);
	hdmi_w8(data, HDMI_MC_FLOWCTRL, 0x00);

	/* packetizer + sampler, RGB888 passthrough */
	hdmi_w8(data, HDMI_VP_PR_CD, 0x40);
	hdmi_w8(data, HDMI_FC_DATAUTO3,
		hdmi_r8(data, HDMI_FC_DATAUTO3) & ~0x04);
	hdmi_w8(data, HDMI_VP_STUFF, 0x27);
	hdmi_w8(data, HDMI_VP_CONF, 0x47);
	hdmi_w8(data, HDMI_VP_REMAP, 0x00);
	hdmi_w8(data, HDMI_TX_INVID0, 0x01);
	hdmi_w8(data, HDMI_TX_INSTUFFING, 0x07);
	for (uint32_t r = HDMI_TX_GYDATA0; r <= 0x0207; r++) {
		hdmi_w8(data, r, 0x00);
	}

	/* HDCP off, DE polarity high */
	hdmi_w8(data, HDMI_A_HDCPCFG0,
		hdmi_r8(data, HDMI_A_HDCPCFG0) & ~0x04);
	hdmi_w8(data, HDMI_A_VIDPOLCFG,
		hdmi_r8(data, HDMI_A_VIDPOLCFG) | 0x10);
	hdmi_w8(data, HDMI_A_HDCPCFG1,
		hdmi_r8(data, HDMI_A_HDCPCFG1) | 0x02);

	/* HDMI mode: AVI infoframe + clear AVMUTE via GCP, plus the FC
	 * packet auto-scheduler (DATAUTO0..3). Golden byte values from
	 * the working pipeline. Without DATAUTO the configured packets
	 * are never transmitted: sinks hold a live link with the panel
	 * dark (no AVI, no GCP = treated as muted/no-video).
	 */
	hdmi_w8(data, HDMI_FC_AVICONF0, 0x60);
	hdmi_w8(data, HDMI_FC_AVICONF1, 0x08);
	hdmi_w8(data, HDMI_FC_AVICONF2, 0x08);
	hdmi_w8(data, HDMI_FC_AVIVID, 0x00);
	hdmi_w8(data, HDMI_FC_AVICONF3, 0x04);
	hdmi_w8(data, HDMI_FC_PRCONF, 0x10);
	hdmi_w8(data, HDMI_FC_GCP, 0x01);
	hdmi_w8(data, HDMI_FC_DATAUTO0, 0x08);
	hdmi_w8(data, HDMI_FC_DATAUTO1, 0x01);
	hdmi_w8(data, HDMI_FC_DATAUTO2, 0x11);
	hdmi_w8(data, HDMI_FC_DATAUTO3, 0x7b);

	/* Golden parity (full DW byte-diff vs the working Linux boot,
	 * 2026-07-19): every writable byte where the golden dump
	 * disagreed with ours, copied verbatim, unbisected. Candidates
	 * of interest: 0x3034 bit0 (golden 0x11, JTAG_PHY_CONFIG) and
	 * undocumented 0x3102 = 0x0C (past the documented PHY block).
	 */
	for (uint32_t r = 0x0100; r <= 0x010a; r++) {
		hdmi_w8(data, r, 0xff);
	}
	hdmi_w8(data, 0x0184, 0x02);
	hdmi_w8(data, 0x0185, 0x03);
	hdmi_w8(data, HDMI_IH_MUTE, 0x00);
	hdmi_w8(data, 0x1029, 0x03);
	hdmi_w8(data, 0x102a, 0x04);
	hdmi_w8(data, 0x1030, 0x0c);
	hdmi_w8(data, HDMI_PHY_MASK0, 0x01);
	hdmi_w8(data, 0x3007, 0x00);
	hdmi_w8(data, HDMI_JTAG_PHY_CONFIG, 0x11);
	hdmi_w8(data, 0x3102, 0x0c);
	hdmi_w8(data, 0x7e00, 0x50);
	hdmi_w8(data, 0x7e01, 0xff);
	hdmi_w8(data, 0x7e03, 0x30);
	hdmi_w8(data, 0x7e05, 0x00);
	hdmi_w8(data, 0x7e07, 0x00);
	hdmi_w8(data, 0x7e27, 0x30);

	/* TMDS soft reset pulse, then re-latch the frame composer */
	hdmi_w8(data, HDMI_MC_SWRSTZ, 0xfd);
	hdmi_w8(data, HDMI_FC_INVIDCONF, hdmi_r8(data, HDMI_FC_INVIDCONF));

	/* Live-path probe: the IH FC status registers latch a bit each
	 * time the FC actually sends the matching packet (GCP/AVI/ACR run
	 * every frame once DATAUTO is scheduled), independent of mutes.
	 * Clear them, give the pipe ~6 frames, and read back: nonzero
	 * stat1 = the FC is really composing and transmitting frames;
	 * all-zero = the FC never cycles even though every config
	 * register is golden. Same probe works on the golden boot via
	 * devmem for a side-by-side.
	 */
	hdmi_w8(data, HDMI_IH_FC_STAT0, 0xff);
	hdmi_w8(data, HDMI_IH_FC_STAT1, 0xff);
	hdmi_w8(data, HDMI_IH_FC_STAT2, 0xff);
	k_msleep(100);
	LOG_INF("FC live: ih_fc %02x/%02x/%02x lockonclock %02x "
		"phystat %02x i2cmphy-latch %s",
		hdmi_r8(data, HDMI_IH_FC_STAT0),
		hdmi_r8(data, HDMI_IH_FC_STAT1),
		hdmi_r8(data, HDMI_IH_FC_STAT2),
		hdmi_r8(data, HDMI_MC_LOCKONCLOCK),
		hdmi_r8(data, HDMI_PHY_STAT0),
		phy_i2cm_stat_latched ? "YES" : "no");

	return 0;
}

#ifdef CONFIG_DISPLAY_SUNXI_DE33_DW_DUMP
/* Output format must byte-match the busybox devmem loops used on the
 * golden Linux boot (lowercase absolute address, uppercase value) so
 * the two dumps diff mechanically.
 */
static void dw_dump_nonzero(struct de33_data *data, uintptr_t phys)
{
	k_msleep(100);
	printk("DWDUMP-BEGIN\n");
	for (uint32_t off = 0; off < 0x8000; off++) {
		uint8_t v = hdmi_r8(data, off);

		if (v != 0U) {
			printk("%08lx: 0x%02X\n", (unsigned long)(phys + off),
			       v);
		}
	}
	printk("WRAP-BEGIN\n");
	for (uint32_t off = 0; off < 0x800; off += 4) {
		printk("WRAP %03x: 0x%08X\n", off,
		       sys_read32(data->phy + off));
	}
	printk("PHY-BEGIN\n");
	for (uint32_t a = 0; a <= 0x7f; a++) {
		printk("PHY %02x: 0x%04X\n", a,
		       hdmi_phy_i2c_read(data, (uint8_t)a));
	}
	/* Blocks never parity-checked against golden: tcon-top past
	 * 0x30, syscon words, PRCM. Same label format for mechanical
	 * diff against the busybox loops on the golden boot.
	 */
	printk("TOP-BEGIN\n");
	for (uint32_t off = 0; off < 0x100; off += 4) {
		printk("TOP %03x: 0x%08X\n", off,
		       sys_read32(data->top + off));
	}
	{
		mm_reg_t blk;

		printk("SYSCON-BEGIN\n");
		device_map(&blk, SYSCON_BASE, 0x100, K_MEM_CACHE_NONE);
		for (uint32_t off = 0; off < 0x50; off += 4) {
			printk("SYSCON %02x: 0x%08X\n", off,
			       sys_read32(blk + off));
		}
		printk("PRCM-BEGIN\n");
		device_map(&blk, 0x07010000, 0x1000, K_MEM_CACHE_NONE);
		for (uint32_t off = 0; off < 0x200; off += 4) {
			printk("PRCM %03x: 0x%08X\n", off,
			       sys_read32(blk + off));
		}
	}
	printk("DWDUMP-END\n");
}
#endif

/* ---- display API ----------------------------------------------------- */

/*
 * Arm the frame-boundary latch and wait for the previous one to be
 * consumed, so the caller never draws into a buffer the DE is still
 * scanning out. The bit self-clears when the hardware takes the shadow
 * register set; the timeout is a few frame periods, after which we
 * proceed anyway rather than wedge a display that has stopped scanning.
 */
static void de33_flip(struct de33_data *data, uint8_t buf)
{
	mm_reg_t ui = data->de + UI_CH_BASE;

	for (int us = 0; us < 50000; us += 100) {
		if (sys_read32(data->de + RTMX_DBUFF) == 0U) {
			break;
		}
		k_busy_wait(100);
	}

	sys_write32((uint32_t)data->fb_phys[buf], ui + UI_TOP_LADDR);
	sys_write32(1, data->de + RTMX_DBUFF);
	data->front = buf;
}

/*
 * Scanout is a fixed CEA mode, but the applications that matter here
 * (game and emulator ports through the SDL2 shim) render at their own
 * native size: 320x200, 512x342 and friends. Rather than make every one
 * of them scale, the driver presents a render surface of the size given
 * by the render-width / render-height properties and blows it up to the
 * mode by the largest integer factor that fits, centred, black around
 * it. Integer-only keeps the source aspect exact and every output pixel
 * a copy of exactly one source pixel, which is what pixel art wants.
 *
 * This is the fallback path, used when the hardware scaler is disabled
 * or the factor is 1. Its cost is a fixed ~31 ms per frame for 320x200
 * at x5 to 1080p, which is why CONFIG_DISPLAY_SUNXI_DE33_HW_SCALER
 * exists; either way callers see the same contract, "capabilities
 * report the render size".
 */
static int de33_write(const struct device *dev, const uint16_t x,
		      const uint16_t y,
		      const struct display_buffer_descriptor *desc,
		      const void *buf)
{
	struct de33_data *data = dev->data;
	uint32_t s = data->sw_scale;
	uint16_t fb_w = data->fb_w;
	bool full = (x == 0U && y == 0U && desc->width == data->render_w &&
		     desc->height == data->render_h);
	uint8_t back = data->front ^ 1U;
	size_t first_row;
	int passes;

	if (x + desc->width > data->render_w ||
	    y + desc->height > data->render_h) {
		return -EINVAL;
	}

	first_row = (size_t)data->off_y + (size_t)y * s;

	/*
	 * A full-surface write goes to the back buffer and is flipped. A
	 * partial one has no previous content in the back buffer to build
	 * on, so it is applied to both: the visible copy updates in place
	 * (as before, so a partial update can still tear in its own small
	 * region) and the other stays coherent for the next flip.
	 */
	passes = full ? 1 : 2;
	for (int pass = 0; pass < passes; pass++) {
		uint32_t *fb = de33_fb[full ? back : (uint8_t)(data->front ^
							       (uint8_t)pass)];
		const uint32_t *src = buf;

		if (s == 1U && data->off_x == 0U && data->off_y == 0U) {
			uint32_t *dst = &fb[(size_t)y * fb_w + x];

			for (uint16_t row = 0; row < desc->height; row++) {
				memcpy(dst, src, desc->width * 4U);
				src += desc->pitch;
				dst += fb_w;
			}
		} else {
			for (uint16_t row = 0; row < desc->height; row++) {
				uint32_t *out = &fb[(first_row +
						     (size_t)row * s) * fb_w +
						    data->off_x +
						    (size_t)x * s];
				uint32_t *dst = out;

				for (uint16_t col = 0; col < desc->width; col++) {
					uint32_t px = src[col];

					for (uint32_t rep = 0; rep < s; rep++) {
						*dst++ = px;
					}
				}
				/* the remaining s-1 output rows are copies */
				for (uint32_t rep = 1; rep < s; rep++) {
					memcpy(out + (size_t)rep * fb_w, out,
					       (size_t)desc->width * s * 4U);
				}
				src += desc->pitch;
			}
		}

		sys_cache_data_flush_range(&fb[first_row * fb_w],
					   (size_t)desc->height * s * fb_w * 4U);
	}

	if (full) {
		de33_flip(data, back);
	}
	return 0;
}

static void de33_get_capabilities(const struct device *dev,
				  struct display_capabilities *caps)
{
	struct de33_data *data = dev->data;

	memset(caps, 0, sizeof(*caps));
	caps->x_resolution = data->render_w;
	caps->y_resolution = data->render_h;
	caps->supported_pixel_formats = PIXEL_FORMAT_ARGB_8888;
	caps->current_pixel_format = PIXEL_FORMAT_ARGB_8888;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int de33_set_pixel_format(const struct device *dev,
				 const enum display_pixel_format format)
{
	return (format == PIXEL_FORMAT_ARGB_8888) ? 0 : -ENOTSUP;
}

static int de33_blanking(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

/* ---- init ------------------------------------------------------------ */

static int de33_init(const struct device *dev)
{
	const struct de33_config *cfg = dev->config;
	struct de33_data *data = dev->data;
	const struct de33_mode *m = &de33_modes[cfg->mode_idx];
	uintptr_t fb_phys;
	uint32_t rate;
	int ret;

	data->mode = m;
	data->render_w = cfg->render_w ? cfg->render_w : m->w;
	data->render_h = cfg->render_h ? cfg->render_h : m->h;
	if (data->render_w > m->w || data->render_h > m->h) {
		LOG_ERR("render %ux%u exceeds mode %ux%u", data->render_w,
			data->render_h, m->w, m->h);
		return -EINVAL;
	}
	data->scale = MIN(m->w / data->render_w, m->h / data->render_h);
	data->off_x = (m->w - data->render_w * data->scale) / 2U;
	data->off_y = (m->h - data->render_h * data->scale) / 2U;
	/*
	 * With the scaler on, the framebuffer is the render surface itself
	 * and the offsets are a blender placement; without it, callers get
	 * expanded into a mode-sized buffer at those offsets by the CPU,
	 * which costs a fixed ~31 ms per frame at 320x200 x5 to 1080p.
	 */
	data->hw_scaled = IS_ENABLED(CONFIG_DISPLAY_SUNXI_DE33_HW_SCALER) &&
			  data->scale > 1U;
	if (data->hw_scaled) {
		data->fb_w = data->render_w;
		data->sw_scale = 1U;
	} else {
		data->fb_w = m->w;
		data->sw_scale = data->scale;
	}

	if (!device_is_ready(cfg->ccu)) {
		return -ENODEV;
	}

	{
		mm_reg_t syscon;

		device_map(&syscon, SYSCON_BASE, 0x100, K_MEM_CACHE_NONE);
		sys_write32(sys_read32(syscon + SYSCON_SRAM_CTRL1) &
			    ~SYSCON_SRAM_C_CPU, syscon + SYSCON_SRAM_CTRL1);
		/* syscon 0x24 bits 12-13 clear: golden parity, same
		 * SRAM-mapping family as the load-bearing SRAM-C bit above
		 */
		sys_write32(sys_read32(syscon + 0x24) & ~(BIT(13) | BIT(12)),
			    syscon + 0x24);
	}

	device_map(&data->de, cfg->de_phys, cfg->de_size, K_MEM_CACHE_NONE);
	device_map(&data->top, cfg->top_phys, cfg->top_size, K_MEM_CACHE_NONE);
	device_map(&data->tcon, cfg->tcon_phys, cfg->tcon_size,
		   K_MEM_CACHE_NONE);
	device_map(&data->hdmi, cfg->hdmi_phys, cfg->hdmi_size,
		   K_MEM_CACHE_NONE);
	device_map(&data->phy, cfg->phy_phys, cfg->phy_size, K_MEM_CACHE_NONE);

	/* DE: reset, bus gate, 600 MHz module clock (mainline: rate is
	 * load-bearing for the mixer). Golden runs CLK_DE from PLL_DE at
	 * div 1, so bring the PLL up first; with it live the mux planner
	 * picks it over periph0-2x/2 (golden CCU parity).
	 */
	ret = reset_line_deassert(cfg->rst_de.dev, cfg->rst_de.id);
	ret |= ccu_on(cfg->ccu, DT_INST_CLOCKS_CELL_BY_NAME(0, bus_de, id));
	ret |= ccu_rate(cfg->ccu, CLK_PLL_DE, 600000000);
	ret |= ccu_rate(cfg->ccu, DT_INST_CLOCKS_CELL_BY_NAME(0, de, id),
			600000000);

	/* TCON path: top then tv, module clock = pixel clock (this
	 * plans PLL_VIDEO0)
	 */
	ret |= reset_line_deassert(cfg->rst_top.dev, cfg->rst_top.id);
	ret |= ccu_on(cfg->ccu,
		      DT_INST_CLOCKS_CELL_BY_NAME(0, bus_tcon_top, id));
	ret |= reset_line_deassert(cfg->rst_tcon.dev, cfg->rst_tcon.id);
	ret |= ccu_on(cfg->ccu,
		      DT_INST_CLOCKS_CELL_BY_NAME(0, bus_tcon_tv, id));
	ret |= ccu_rate(cfg->ccu, DT_INST_CLOCKS_CELL_BY_NAME(0, tcon_tv, id),
			m->pixclk);

	/* HDMI: golden (sun8i_dw_hdmi_bind + sun8i_hdmi_phy_init) order —
	 * resets released BEFORE their clocks start: ctrl reset, tmds
	 * clock, phy reset, bus clock, slow clock. Our old order (all
	 * clocks first, then both resets) left the PHY-i2c-master IH
	 * event latch dead: identical end-state registers, done-event
	 * latches on golden and never on us (probed live 2026-07-22).
	 */
	ret |= reset_line_deassert(cfg->rst_ctrl.dev, cfg->rst_ctrl.id);
	ret |= ccu_rate(cfg->ccu, DT_INST_CLOCKS_CELL_BY_NAME(0, hdmi, id),
			m->pixclk);
	ret |= reset_line_deassert(cfg->rst_phy.dev, cfg->rst_phy.id);
	ret |= ccu_on(cfg->ccu, DT_INST_CLOCKS_CELL_BY_NAME(0, bus_hdmi, id));
	ret |= ccu_on(cfg->ccu, DT_INST_CLOCKS_CELL_BY_NAME(0, hdmi_slow, id));
	if (ret) {
		LOG_ERR("clock/reset bring-up failed");
		return -EIO;
	}

	if (!clock_control_get_rate(cfg->ccu,
			(clock_control_subsys_t)
			DT_INST_CLOCKS_CELL_BY_NAME(0, tcon_tv, id), &rate)) {
		LOG_INF("tcon-tv %u Hz (want %u)", rate, m->pixclk);
	}

	memset(de33_fb, 0, sizeof(de33_fb));
	sys_cache_data_flush_range(de33_fb, sizeof(de33_fb));
	data->fb_phys[0] = k_mem_phys_addr(de33_fb[0]);
	data->fb_phys[1] = k_mem_phys_addr(de33_fb[1]);
	data->front = 0U;
	fb_phys = data->fb_phys[0];

	de33_mixer_init(data, fb_phys);
	de33_tcon_init(cfg, data);
	ret = de33_hdmi_init(data);
	if (ret) {
		return ret;
	}


	LOG_INF("%ux%u@60 up, render %ux%u x%u at +%u+%u (%s scaled), "
		"fb %p (phys 0x%lx)%s",
		m->w, m->h, data->render_w, data->render_h, data->scale,
		data->off_x, data->off_y, data->hw_scaled ? "VSU" : "CPU",
		(void *)de33_fb, (unsigned long)fb_phys,
		cfg->tcon_pattern ? " [TCON colorbar]" : "");

#ifdef CONFIG_DISPLAY_SUNXI_DE33_DW_DUMP
	dw_dump_nonzero(data, cfg->hdmi_phys);
#endif
	return 0;
}

static DEVICE_API(display, de33_api) = {
	.blanking_on = de33_blanking,
	.blanking_off = de33_blanking,
	.write = de33_write,
	.get_capabilities = de33_get_capabilities,
	.set_pixel_format = de33_set_pixel_format,
};

#define DE33_INIT(inst)							\
	static const struct de33_config de33_config_##inst = {		\
		.de_phys = DT_INST_REG_ADDR_BY_NAME(inst, de),		\
		.de_size = DT_INST_REG_SIZE_BY_NAME(inst, de),		\
		.top_phys = DT_INST_REG_ADDR_BY_NAME(inst, tcon_top),	\
		.top_size = DT_INST_REG_SIZE_BY_NAME(inst, tcon_top),	\
		.tcon_phys = DT_INST_REG_ADDR_BY_NAME(inst, tcon_tv),	\
		.tcon_size = DT_INST_REG_SIZE_BY_NAME(inst, tcon_tv),	\
		.hdmi_phys = DT_INST_REG_ADDR_BY_NAME(inst, hdmi),	\
		.hdmi_size = DT_INST_REG_SIZE_BY_NAME(inst, hdmi),	\
		.phy_phys = DT_INST_REG_ADDR_BY_NAME(inst, hdmi_phy),	\
		.phy_size = DT_INST_REG_SIZE_BY_NAME(inst, hdmi_phy),	\
		.ccu = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, de)), \
		.rst_de = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 0),	\
		.rst_top = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 1),	\
		.rst_tcon = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 2),	\
		.rst_ctrl = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 3),	\
		.rst_phy = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 4),	\
		.mode_idx = DT_INST_ENUM_IDX(inst, mode),		\
		.tcon_pattern = DT_INST_PROP(inst, tcon_test_pattern),	\
		.render_w = DT_INST_PROP_OR(inst, render_width, 0),	\
		.render_h = DT_INST_PROP_OR(inst, render_height, 0),	\
	};								\
	static struct de33_data de33_data_##inst;			\
									\
	DEVICE_DT_INST_DEFINE(inst, de33_init, NULL, &de33_data_##inst,	\
			      &de33_config_##inst, POST_KERNEL,		\
			      CONFIG_DISPLAY_INIT_PRIORITY, &de33_api);

DT_INST_FOREACH_STATUS_OKAY(DE33_INIT)
