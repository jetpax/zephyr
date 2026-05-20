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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_BCM2835_H_ */
