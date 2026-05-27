/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_BCM2835_H_
#define ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_BCM2835_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VideoCore power-domain device IDs (the "old" / legacy interface
 * still used by the SET_POWER_STATE tag — index space the firmware
 * tracks internally, distinct from Linux's RPI_POWER_DOMAIN_* DT
 * indices which are off-by-one and only used by the bcm2835-power
 * pmdomain shim). Values are 0..7.
 */
#define BCM2835_POWER_DEVICE_SDCARD     0
#define BCM2835_POWER_DEVICE_UART0      1
#define BCM2835_POWER_DEVICE_UART1      2
#define BCM2835_POWER_DEVICE_USB_HCD    3
#define BCM2835_POWER_DEVICE_I2C0       4
#define BCM2835_POWER_DEVICE_I2C1       5
#define BCM2835_POWER_DEVICE_I2C2       6
#define BCM2835_POWER_DEVICE_SPI        7

/**
 * @brief Set VideoCore-controlled power state for a SoC peripheral.
 *
 * Issues a SET_POWER_STATE (tag 0x00028001) property request over
 * the VideoCore mailbox. Synchronous: blocks until the firmware
 * acknowledges. Callable from thread context or from an early-init
 * (SYS_INIT) function — uses polled I/O so the scheduler isn't
 * needed.
 *
 * @param device_id  One of BCM2835_POWER_DEVICE_* above.
 * @param on         true to power on, false to power off.
 *
 * @retval 0        Power state changed (or already in requested state).
 * @retval -ENODEV  Firmware driver not enabled / not initialised.
 * @retval -EIO     Mailbox transport error or firmware-reported failure.
 */
int bcm2835_property_set_power_state(uint32_t device_id, bool on);

/**
 * @brief Fetch the 64-bit board serial number from VideoCore OTP.
 *
 * Each Raspberry Pi has a unique 64-bit serial number burned into
 * one-time-programmable storage at factory; queried via the
 * GET_BOARD_SERIAL property tag (0x00010004). Returned as 8 raw
 * bytes in the wire-format order (low u32 first, high u32 second —
 * matches the on-RAM layout VC writes).
 *
 * @param out  Caller-allocated 8-byte buffer; receives the serial.
 *
 * @retval 0        Serial number written into @p out.
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENODEV  Firmware driver not enabled / not initialised.
 * @retval -EIO     Mailbox transport error or firmware-reported failure.
 */
int bcm2835_property_get_board_serial(uint8_t *out);

/**
 * @brief Read the SoC temperature via the VideoCore firmware.
 *
 * Issues a GET_TEMPERATURE property tag (0x00030006) for sensor 0 --
 * the same source as `vcgencmd measure_temp`.
 *
 * @param out_millideg  Receives the SoC temperature in millidegrees Celsius.
 *
 * @retval 0        Temperature written into @p out_millideg.
 * @retval -EINVAL  @p out_millideg is NULL.
 * @retval -ENODEV  Firmware driver not enabled / not initialised.
 * @retval -EIO     Mailbox transport error or firmware-reported failure.
 */
int bcm2835_property_get_temperature(int32_t *out_millideg);

/*
 * VideoCore clock IDs (for the GET_CLOCK_RATE tag). Values match the
 * RPi firmware mailbox property interface; ARM is the Cortex-A CPU
 * clock.
 */
#define BCM2835_CLOCK_ARM 0x00000003U

/**
 * @brief Read a VideoCore-managed clock rate.
 *
 * Issues a GET_CLOCK_RATE property tag (0x00030002) for the given
 * clock ID. Returns the configured rate in Hz -- for BCM2835_CLOCK_ARM
 * this is the Cortex-A53 core frequency (the same value `vcgencmd
 * measure_clock arm` reports).
 *
 * @param clock_id  One of BCM2835_CLOCK_* above.
 * @param out_hz    Receives the clock rate in Hz (0 if the clock does
 *                  not exist or is not running).
 *
 * @retval 0        Rate written into @p out_hz.
 * @retval -EINVAL  @p out_hz is NULL.
 * @retval -ENODEV  Firmware driver not enabled / not initialised.
 * @retval -EIO     Mailbox transport error or firmware-reported failure.
 */
int bcm2835_property_get_clock_rate(uint32_t clock_id, uint32_t *out_hz);

/*
 * Framebuffer pixel order constants (the value of the SET_PIXEL_ORDER
 * tag). At depth=32 + RGB, pixels are stored as u32 0xAARRGGBB --
 * little-endian memory order [B, G, R, A] -- matching DRM_FORMAT_ARGB8888
 * and Zephyr's PIXEL_FORMAT_ARGB_8888.
 */
#define BCM2835_FB_PIXEL_ORDER_BGR 0U
#define BCM2835_FB_PIXEL_ORDER_RGB 1U

/**
 * @brief Read VideoCore's current display dimensions.
 *
 * Returns the resolution VC negotiated with the monitor's EDID at
 * boot (driven by the `hdmi_*` settings in config.txt and the EDID
 * the monitor advertises). Returns 0 x 0 if no display is detected.
 *
 * Used by the framebuffer driver to auto-select a native mode when
 * DT doesn't specify an explicit width / height.
 *
 * @param width   Receives the current display width in pixels.
 * @param height  Receives the current display height in pixels.
 *
 * @retval 0        Size read; @p width / @p height hold the result.
 * @retval -EINVAL  @p width or @p height is NULL.
 * @retval -ENODEV  Firmware driver not enabled / not initialised.
 * @retval -EIO     Mailbox transport error or firmware-reported failure.
 */
int bcm2835_property_fb_get_size(uint32_t *width, uint32_t *height);

/**
 * @brief Configure and allocate the VideoCore framebuffer in one atomic call.
 *
 * Issues a single chained property request that sets the physical and
 * virtual size, depth, and pixel order; allocates the buffer; and
 * reads the actual row pitch back. VC requires these tags to arrive
 * in a single request -- when split across multiple calls, VC silently
 * clamps virt to 2x2 and depth to a minimum.
 *
 * All in/out arguments are updated with the values VC actually applied,
 * which may differ from what was requested if the firmware clamped (for
 * instance, to a mode the connected monitor supports).
 *
 * @param width        in/out: requested width; actual on return.
 * @param height       in/out: requested height; actual on return.
 * @param depth        in/out: requested bpp (16, 24, 32); actual on return.
 * @param pixel_order  in/out: BCM2835_FB_PIXEL_ORDER_BGR or _RGB; actual.
 * @param alignment    Buffer alignment in bytes (256 is conventional).
 * @param fb_bus       Receives the VC-bus framebuffer base address.
 *                     The ARM physical address is `*fb_bus & 0x3FFFFFFF`.
 * @param fb_size      Receives the buffer size in bytes.
 * @param pitch        Receives the row pitch in bytes (may exceed width*BPP
 *                     if VC padded for alignment).
 *
 * @retval 0        Configured and allocated.
 * @retval -EINVAL  Any pointer argument is NULL.
 * @retval -ENODEV  Firmware driver not enabled / not initialised.
 * @retval -EIO     Mailbox transport error or firmware-reported failure
 *                  (also returned if HDMI is not active -- check that
 *                  `hdmi_force_hotplug=1` is set in config.txt).
 */
int bcm2835_property_fb_setup(uint32_t *width, uint32_t *height,
			      uint32_t *depth, uint32_t *pixel_order,
			      uint32_t alignment,
			      uintptr_t *fb_bus, uint32_t *fb_size,
			      uint32_t *pitch);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_BCM2835_H_ */
