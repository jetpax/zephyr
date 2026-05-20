/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_BRCM_BCM2835_USB_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_BRCM_BCM2835_USB_H

#if defined(CONFIG_BCM2835_FIRMWARE)
#include <zephyr/drivers/firmware/bcm2835.h>
#endif

/*
 * BCM2710 (Pi Zero 2 W / Pi 3 family) DWC2 vendor quirks.
 *
 * Two quirks now:
 *
 *  1. `pre_enable` - turn on the VideoCore-controlled USB analog PHY
 *     power island via the firmware property mailbox. The DWC2
 *     digital register block is ARM-accessible out of VC boot, but
 *     D+/D- aren't electrically driven until SET_POWER_STATE(USB_HCD,
 *     on) flips the island on. Linux's DT models this as
 *     `power-domains = <&power RPI_POWER_DOMAIN_USB>`; we just call
 *     the firmware helper directly. SoC-level power-on persists
 *     across DWC2's own soft reset inside init_controller (it only
 *     resets the controller's internal state machines), so running
 *     this from pre_enable is the right place.
 *
 *  2. `caps` - declare HS support. The generic driver reads
 *     GHWCFG2.HSPhyType but doesn't propagate it into `udc_data
 *     .caps.hs`, so without this callback the device only advertises
 *     FS speeds and the host never tries HS enumeration. Phase 2.0
 *     of the bring-up confirmed UTMI+ HS PHY (GHWCFG2.HSPhyType = 1).
 *
 * Other quirks considered but not needed here:
 *   - host->device role flip:  unnecessary; chip defaults to device
 *     mode when nothing is plugged into the OTG receptacle (ID
 *     floats) and the driver's GUSBCFG_FORCEDEVMODE write inside
 *     init_controller takes care of the rest.
 *   - GDFIFOCFG / GRXFSIZ priming: BCM2710 leaves both at zero after
 *     the driver's own soft reset, which would clamp the RX FIFO to
 *     0 via the generic driver's MIN logic. Tried priming in
 *     `pre_enable` -- doesn't work because init_controller's soft
 *     reset wipes the values. The fix lives in init_controller
 *     itself: fall back to GHWCFG3.DfifoDepth when GDFIFOCFG reads
 *     0, and fall back to default_depth when GRXFSIZ reads 0. Small
 *     patch with general applicability, intended for upstream.
 */

static inline int brcm_bcm2835_usb_pre_enable(const struct device *dev)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_BCM2835_FIRMWARE)
	/* Failure path is logged by the firmware driver; we just
	 * propagate the errno so the DWC2 core aborts cleanly.
	 */
	return bcm2835_property_set_power_state(
		BCM2835_POWER_DEVICE_USB_HCD, true);
#else
	/* No mailbox path compiled in -- without it, D+/D- aren't
	 * driven. Fail fast so the user notices the missing config.
	 */
	return -ENODEV;
#endif
}

static inline int brcm_bcm2835_usb_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.hs = true;

	return 0;
}

#define QUIRK_BRCM_BCM2835_USB_DEFINE(n)					\
	static const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {	\
		.pre_enable = brcm_bcm2835_usb_pre_enable,			\
		.caps = brcm_bcm2835_usb_caps,					\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_BRCM_BCM2835_USB_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_BRCM_BCM2835_USB_H */
