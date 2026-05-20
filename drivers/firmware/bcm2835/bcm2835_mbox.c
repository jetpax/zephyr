/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835-family ARM <-> VideoCore mailbox driver.
 *
 * Register layout (offsets from MMIO base 0x3F00B880):
 *   0x00  MAIL0_RD     ARM reads replies from VC
 *   0x18  MAIL0_STA    Rx FIFO status; bit 30 EMPTY, bit 31 FULL
 *   0x1c  MAIL0_CNF    Rx IRQ enable (not used by this driver)
 *   0x20  MAIL1_WRT    ARM writes requests to VC
 *   0x38  MAIL1_STA    Tx FIFO status; bit 31 FULL
 *
 * Message encoding: channel in bits 3..0, payload (typically the
 * 16-byte-aligned bus address of a request buffer) in bits 31..4.
 *
 * Polled. No IRQ. Single-shot RPC pattern — burns CPU during the
 * (microsecond-scale) wait, which is fine for boot-time and config
 * calls. See plan-of-record doc for the polling-vs-IRQ decision.
 */

#define DT_DRV_COMPAT brcm_bcm2835_mbox

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/mm/system_mm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "bcm2835_mbox.h"

LOG_MODULE_REGISTER(bcm2835_mbox, CONFIG_BCM2835_MBOX_LOG_LEVEL);

#define MBOX_MAIL0_RD   0x00
#define MBOX_MAIL0_STA  0x18
#define MBOX_MAIL1_WRT  0x20
#define MBOX_MAIL1_STA  0x38

#define MBOX_STATUS_FULL   BIT(31)
#define MBOX_STATUS_EMPTY  BIT(30)

#define MBOX_CHANNEL_MASK  0xfU

/* Generous: a healthy property call returns in microseconds.
 * 1 s is a "VC firmware has wedged" backstop.
 */
#define MBOX_TIMEOUT_US    1000000U

struct bcm2835_mbox_config {
	DEVICE_MMIO_NAMED_ROM(base_addr);
};

struct bcm2835_mbox_data {
	DEVICE_MMIO_NAMED_RAM(base_addr);
	struct k_mutex lock;
};

#define DEV_CFG(dev)  ((const struct bcm2835_mbox_config *)(dev)->config)
#define DEV_DATA(dev) ((struct bcm2835_mbox_data *)(dev)->data)

static int wait_until_clear(mem_addr_t reg, uint32_t mask)
{
	uint32_t start = k_cycle_get_32();

	while (sys_read32(reg) & mask) {
		if (k_cyc_to_us_near32(k_cycle_get_32() - start) >
		    MBOX_TIMEOUT_US) {
			return -EIO;
		}
	}
	return 0;
}

int bcm2835_mbox_call(const struct device *dev, uint8_t channel,
		      uint32_t data, uint32_t *reply)
{
	const mem_addr_t base = DEVICE_MMIO_NAMED_GET(dev, base_addr);
	struct bcm2835_mbox_data *drv_data = dev->data;
	uint32_t msg, recv;
	int err;

	if (channel > 15U || (data & MBOX_CHANNEL_MASK) != 0U) {
		return -EINVAL;
	}

	msg = data | (channel & MBOX_CHANNEL_MASK);

	(void)k_mutex_lock(&drv_data->lock, K_FOREVER);

	err = wait_until_clear(base + MBOX_MAIL1_STA, MBOX_STATUS_FULL);
	if (err < 0) {
		LOG_ERR("Tx FIFO full, VC not draining");
		goto out;
	}

	sys_write32(msg, base + MBOX_MAIL1_WRT);

	/* Spin until a reply on our channel pops out of MAIL0. Replies
	 * on other channels are discarded — no other consumer in this
	 * driver tree today.
	 */
	uint32_t start = k_cycle_get_32();
	for (;;) {
		uint32_t status = sys_read32(base + MBOX_MAIL0_STA);

		if (!(status & MBOX_STATUS_EMPTY)) {
			recv = sys_read32(base + MBOX_MAIL0_RD);
			if ((recv & MBOX_CHANNEL_MASK) == channel) {
				if (reply != NULL) {
					*reply = recv & ~MBOX_CHANNEL_MASK;
				}
				err = 0;
				goto out;
			}
			LOG_DBG("dropped reply on channel %u",
				recv & MBOX_CHANNEL_MASK);
			continue;
		}
		if (k_cyc_to_us_near32(k_cycle_get_32() - start) >
		    MBOX_TIMEOUT_US) {
			LOG_ERR("Rx timeout waiting for channel %u reply",
				channel);
			err = -EIO;
			goto out;
		}
	}

out:
	k_mutex_unlock(&drv_data->lock);
	return err;
}

static int bcm2835_mbox_init(const struct device *dev)
{
	struct bcm2835_mbox_data *drv_data = dev->data;

	DEVICE_MMIO_NAMED_MAP(dev, base_addr, K_MEM_CACHE_NONE);
	k_mutex_init(&drv_data->lock);
	return 0;
}

#define BCM2835_MBOX_INIT(n)                                                   \
	static struct bcm2835_mbox_data bcm2835_mbox_data_##n;                 \
	static const struct bcm2835_mbox_config bcm2835_mbox_config_##n = {    \
		DEVICE_MMIO_NAMED_ROM_INIT(base_addr, DT_DRV_INST(n)),         \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(n, bcm2835_mbox_init, NULL,                      \
			      &bcm2835_mbox_data_##n,                          \
			      &bcm2835_mbox_config_##n,                        \
			      POST_KERNEL,                                     \
			      CONFIG_BCM2835_MBOX_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BCM2835_MBOX_INIT)
