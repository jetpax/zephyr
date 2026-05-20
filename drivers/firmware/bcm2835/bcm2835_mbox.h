/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FIRMWARE_BCM2835_BCM2835_MBOX_H_
#define ZEPHYR_DRIVERS_FIRMWARE_BCM2835_BCM2835_MBOX_H_

#include <stdint.h>
#include <zephyr/device.h>

/* VideoCore mailbox channel for property tags (the only channel
 * this driver tree uses today).
 */
#define BCM2835_MBOX_CHAN_PROPERTY  8

/**
 * @brief Single round-trip mailbox call.
 *
 * Writes (data | channel) to MAIL1, then polls MAIL0 until a reply
 * appears with the matching channel in the low 4 bits. Replies on
 * other channels are discarded — first-cut driver, no demux, no IRQ.
 *
 * Internally serialised by a per-device mutex so it is safe to call
 * from multiple threads. Polled, so safe from SYS_INIT context too.
 *
 * @param dev      The mailbox device (DT_INST_GET on the
 *                 brcm,bcm2835-mbox compatible node).
 * @param channel  0..15.
 * @param data     Payload — bits 31..4. Low 4 bits MUST be zero
 *                 (caller has already masked the buffer address
 *                 etc.); driver does not re-mask.
 * @param reply    Optional, filled with payload bits 31..4 of the
 *                 reply word. May be NULL.
 *
 * @retval 0       Reply received on the expected channel.
 * @retval -EINVAL channel out of range, or data has non-zero low
 *                 4 bits.
 * @retval -EIO    Timeout waiting for transmit/receive FIFO.
 */
int bcm2835_mbox_call(const struct device *dev, uint8_t channel,
		      uint32_t data, uint32_t *reply);

#endif /* ZEPHYR_DRIVERS_FIRMWARE_BCM2835_BCM2835_MBOX_H_ */
