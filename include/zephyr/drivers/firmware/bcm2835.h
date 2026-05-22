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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_BCM2835_H_ */
