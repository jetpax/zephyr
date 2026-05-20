/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phase 2.1 minimal CDC ACM bring-up for rpi_zero_2w.
 *
 * The USB stack initializes itself at SYS_INIT (we set
 * CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=y) and the
 * `zephyr,cdc-acm-uart` DT node is bound by the class driver to look
 * like a regular UART. The app code is just a write loop: it pushes
 * a counted "hello USB" line every 500 ms via the standard UART
 * poll-out API. The CDC class translates that into bulk-IN packets
 * on the chip's endpoint 1 IN; the host PC sees them on its
 * /dev/cu.usbmodem* (macOS) / /dev/ttyACM0 (Linux) / COMn (Windows).
 *
 * We also LOG_INF every iteration so the PL011 console (1 Mbaud on
 * GPIO 14/15) gives us a side-channel view of device-side liveness
 * regardless of whether the host has the USB port open. This is the
 * key debugging feature for first bring-up: if the host never sees
 * /dev/cu.usbmodem* but the PL011 log keeps ticking, the device-side
 * stack is healthy and the failure is enumeration. If the PL011 log
 * itself stops at boot, we have an init-time wedge to investigate.
 */

#include <stdio.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_cdc_min, LOG_LEVEL_INF);

static const struct device *const cdc_dev =
	DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

/* Bus-state probe — reads the DWC2 registers that tell us whether the
 * chip thinks a host is electrically attached. Useful when "/dev/cu.usbmodem*
 * never appears" so we can tell apart:
 *   - VBUS not sensed by the chip → no B-session valid → no pullup drive
 *   - VBUS sensed but pullup not on → soft-disconnect still asserted
 *   - Pullup on but host not enumerating → cable / port physical issue
 */
#define DWC2_PA       0x3F980000UL
#define DWC2_LEN      0x100UL

#define GOTGCTL       0x000
#define GAHBCFG       0x008
#define GUSBCFG       0x00C
#define GINTSTS       0x014
#define DCFG          0x800
#define DCTL          0x804
#define DSTS          0x808
#define PCGCCTL       0xE00

static void probe_bus_state(mm_reg_t base, const char *label)
{
	uint32_t gotgctl = sys_read32(base + GOTGCTL);
	uint32_t gintsts = sys_read32(base + GINTSTS);
	uint32_t dctl    = sys_read32(base + DCTL);
	uint32_t dsts    = sys_read32(base + DSTS);
	uint32_t dcfg    = sys_read32(base + DCFG);

	LOG_INF("--- bus probe (%s) ---", label);
	LOG_INF("GOTGCTL = 0x%08x  BvalidOvEn=%u BvalidOvVal=%u "
		"ConIDSts=%u BSesVld=%u",
		gotgctl,
		(gotgctl >> 6) & 1,   /* BvalidOvEn */
		(gotgctl >> 7) & 1,   /* BvalidOvVal */
		(gotgctl >> 16) & 1,  /* ConIDSts: 1=device, 0=host */
		(gotgctl >> 19) & 1); /* BSesVld */
	LOG_INF("GINTSTS = 0x%08x  CurMod=%u (1=host 0=device)",
		gintsts, gintsts & 1);
	LOG_INF("DCTL    = 0x%08x  SftDiscon=%u (want 0)",
		dctl, (dctl >> 1) & 1);
	LOG_INF("DSTS    = 0x%08x  EnumSpd=%u SuspSts=%u",
		dsts,
		(dsts >> 1) & 3,
		dsts & 1);
	LOG_INF("DCFG    = 0x%08x  DevAddr=%u",
		dcfg, (dcfg >> 4) & 0x7f);
	uint32_t pcgcctl = sys_read32(base + PCGCCTL);
	LOG_INF("PCGCCTL = 0x%08x  StopPClk=%u GateHclk=%u PwrClmp=%u "
		"RstPdwn=%u PhySleep=%u",
		pcgcctl,
		pcgcctl & 1,        /* StopPClk */
		(pcgcctl >> 1) & 1, /* GateHclk */
		(pcgcctl >> 2) & 1, /* PwrClmp */
		(pcgcctl >> 3) & 1, /* RstPdwnModule */
		(pcgcctl >> 6) & 1);/* PhySleep */
}

int main(void)
{
	mm_reg_t dwc2_base;

	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM device not ready -- driver bind failed");
		return -1;
	}

	LOG_INF("CDC ACM device ready: %s", cdc_dev->name);

	device_map(&dwc2_base, DWC2_PA, DWC2_LEN, K_MEM_CACHE_NONE);

	/* Give init a moment to settle, then probe. */
	k_msleep(200);
	probe_bus_state(dwc2_base, "post-init (no host expected)");

	LOG_INF("");
	LOG_INF("** Unplug and replug the USB cable now -- bus probes will");
	LOG_INF("** be re-printed every 2 seconds. Look for changes in:");
	LOG_INF("**   BSesVld 0->1  = chip electrically detects host VBUS");
	LOG_INF("**   ConIDSts      = micro-USB OTG ID-pin state");
	LOG_INF("**   SftDiscon 0   = D+ pullup is being driven");
	LOG_INF("**   EnumSpd       = bus-detected speed (0=HS 1=FS 2=LS 3=FS)");
	LOG_INF("");

	unsigned int n = 0;

	while (1) {
		char buf[64];
		int len = snprintf(buf, sizeof(buf),
				   "hello USB #%u\r\n", n);

		for (int i = 0; i < len; i++) {
			uart_poll_out(cdc_dev, buf[i]);
		}

		if ((n % 4) == 0) {
			probe_bus_state(dwc2_base, "loop");
		}
		n++;
		k_msleep(500);
	}

	return 0;
}
