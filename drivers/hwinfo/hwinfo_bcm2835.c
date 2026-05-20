/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raspberry Pi (BCM283x family) hwinfo: returns the 64-bit board
 * serial number from VideoCore OTP via the firmware property-tag
 * interface. Used by the Zephyr USB device stack to populate
 * iSerialNumber, by MCUboot for device-unique seeding, etc.
 *
 * Each call issues a fresh mailbox round-trip; that's microseconds
 * on this SoC and matches Linux's behaviour. No caching layer — if
 * a callsite ever needs to read the serial on a hot path, add one
 * here behind a small mutex.
 */

#include <string.h>

#include <zephyr/drivers/firmware/bcm2835.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/util.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t serial[8];
	int err;

	err = bcm2835_property_get_board_serial(serial);
	if (err < 0) {
		return err;
	}

	const size_t copy = MIN(length, sizeof(serial));

	memcpy(buffer, serial, copy);
	return (ssize_t)copy;
}
