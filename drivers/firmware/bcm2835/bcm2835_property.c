/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raspberry Pi VideoCore firmware property-tag interface.
 *
 * Sits on top of bcm2835_mbox.c. Builds a property request buffer
 * (single tag, fixed layout), flushes the data cache, hands the
 * 16-byte-aligned ARM-physical address to the mailbox on channel 8,
 * waits for the reply, invalidates the data cache, parses the
 * status code, returns.
 *
 * Wire format (matches Linux drivers/firmware/raspberrypi.c):
 *
 *   word 0   total buffer size in bytes (incl. this word)
 *   word 1   request/response code (REQUEST=0, RESPONSE=0x80000000+)
 *   word 2   tag id
 *   word 3   tag value-buffer size
 *   word 4   tag request/response size (response bit 31 set on reply)
 *   word 5+  tag payload (variable)
 *   word N   end tag (0)
 *
 * Single global request buffer with a per-call mutex — VC mailbox is
 * a serialised resource at the property layer anyway. Buffer is sized
 * for the largest tag we expect to send (SET_POWER_STATE is 32 bytes
 * including framing); future tags larger than that bump REQ_BUF_SIZE.
 */

#define DT_DRV_COMPAT brcm_bcm2835_firmware

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/firmware/bcm2835.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "bcm2835_mbox.h"

LOG_MODULE_REGISTER(bcm2835_firmware, CONFIG_BCM2835_FIRMWARE_LOG_LEVEL);

/* Tag IDs (from RPi firmware mailbox property interface wiki). */
#define RPI_FW_TAG_GET_BOARD_SERIAL 0x00010004U
#define RPI_FW_TAG_SET_POWER_STATE  0x00028001U
#define RPI_FW_TAG_GET_TEMPERATURE  0x00030006U

/* Top-level request/response codes. */
#define RPI_FW_REQUEST              0x00000000U
#define RPI_FW_RESPONSE_SUCCESS     0x80000000U

/* Response bit OR'd into the per-tag req/resp size on reply. */
#define RPI_FW_TAG_RESPONSE_BIT     0x80000000U

#define RPI_FW_END_TAG              0x00000000U

/* Logical request size for the SET_POWER_STATE tag — what we tell
 * VC in word 0 of the buffer. 8 words = 32 bytes covers the
 * 5-word framing (size/status/tag-id/buf-size/req-resp-size) plus
 * the 2-word payload plus the 1-word end tag.
 */
#define REQ_TAG_WORDS  8U
#define REQ_BUF_SIZE   (REQ_TAG_WORDS * sizeof(uint32_t))

/* Physical allocation. The buffer crosses the ARM/VC trust boundary;
 * VC firmware DMAs it. Aligned AND sized to a full Cortex-A53 cache
 * line (64 B) because `sys_cache_data_invd_range` operates on whole
 * lines: a smaller allocation lets the linker place co-tenant BSS
 * (e.g. `logging_thread`) on the same line, and our post-call invd
 * would silently discard any cached writes to that neighbor. First
 * bring-up hit exactly this — `logging_thread` landed 32 B after a
 * 32-B req_buf, the invd lost the scheduler's cached updates to the
 * log thread, and the next LOG_INF call dispatched a corrupted
 * thread entry pointer into .rodata (PXN fault on EL1 instruction
 * fetch).
 */
#define REQ_BUF_WORDS  16U
static uint32_t req_buf[REQ_BUF_WORDS] __aligned(64);
BUILD_ASSERT(REQ_TAG_WORDS <= REQ_BUF_WORDS,
	     "tag won't fit in cache-line-sized buffer");

static K_MUTEX_DEFINE(req_buf_lock);

#if DT_HAS_COMPAT_STATUS_OKAY(brcm_bcm2835_firmware)

#define FW_NODE       DT_INST(0, brcm_bcm2835_firmware)
#define FW_MBOX_NODE  DT_PHANDLE_BY_IDX(FW_NODE, mboxes, 0)

static const struct device *const fw_mbox = DEVICE_DT_GET(FW_MBOX_NODE);

static inline bool fw_ready(void)
{
	return device_is_ready(fw_mbox);
}

#else  /* !DT_HAS_COMPAT_STATUS_OKAY(brcm_bcm2835_firmware) */

static const struct device *const fw_mbox;  /* NULL */
static inline bool fw_ready(void) { return false; }

#endif

/*
 * Build a one-tag property request in req_buf, submit it, parse the
 * response. Caller holds req_buf_lock.
 *
 * payload_words: number of u32s of request payload following the tag
 *                header. Same number is allocated for the response.
 */
static int property_call_locked(uint32_t tag_id, uint32_t payload_words)
{
	const uint32_t tag_buf_bytes = payload_words * sizeof(uint32_t);
	uint32_t reply;
	int err;

	req_buf[0] = REQ_BUF_SIZE;
	req_buf[1] = RPI_FW_REQUEST;
	req_buf[2] = tag_id;
	req_buf[3] = tag_buf_bytes;
	req_buf[4] = tag_buf_bytes;
	/* req_buf[5..5+payload_words-1] = payload — filled by caller */
	req_buf[5 + payload_words] = RPI_FW_END_TAG;

	/* Push our writes out to RAM so VC's DMA sees them. */
	sys_cache_data_flush_range(req_buf, sizeof(req_buf));

	err = bcm2835_mbox_call(fw_mbox, BCM2835_MBOX_CHAN_PROPERTY,
				(uint32_t)(uintptr_t)req_buf, &reply);
	if (err < 0) {
		LOG_ERR("mbox call failed: %d", err);
		return err;
	}

	/* VC has written the response back; drop any cached copy of the
	 * buffer so subsequent reads come from RAM.
	 */
	sys_cache_data_invd_range(req_buf, sizeof(req_buf));

	if (req_buf[1] != RPI_FW_RESPONSE_SUCCESS) {
		LOG_ERR("firmware rejected tag 0x%08x, status 0x%08x",
			tag_id, req_buf[1]);
		return -EIO;
	}
	if (!(req_buf[4] & RPI_FW_TAG_RESPONSE_BIT)) {
		LOG_ERR("tag 0x%08x: no response bit set", tag_id);
		return -EIO;
	}

	return 0;
}

int bcm2835_property_set_power_state(uint32_t device_id, bool on)
{
	int err;

	if (!fw_ready()) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&req_buf_lock, K_FOREVER);

	/* Payload: device_id, state-flags (bit 0 = on). */
	req_buf[5] = device_id;
	req_buf[6] = on ? 1U : 0U;

	err = property_call_locked(RPI_FW_TAG_SET_POWER_STATE, 2);
	if (err == 0) {
		LOG_INF("device %u power -> %s (state=0x%08x)",
			device_id, on ? "ON" : "OFF", req_buf[6]);
	}

	k_mutex_unlock(&req_buf_lock);
	return err;
}

int bcm2835_property_get_board_serial(uint8_t *out)
{
	int err;

	if (out == NULL) {
		return -EINVAL;
	}
	if (!fw_ready()) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&req_buf_lock, K_FOREVER);

	/* GET_BOARD_SERIAL has no request payload — VC writes 2 u32
	 * of serial into the value-buffer slot. Zero the slot so a
	 * firmware that does inspect it sees a clean request.
	 */
	req_buf[5] = 0;
	req_buf[6] = 0;

	err = property_call_locked(RPI_FW_TAG_GET_BOARD_SERIAL, 2);
	if (err == 0) {
		memcpy(out, &req_buf[5], 8);
		LOG_INF("board serial: %08x%08x", req_buf[6], req_buf[5]);
	}

	k_mutex_unlock(&req_buf_lock);
	return err;
}

int bcm2835_property_get_temperature(int32_t *out_millideg)
{
	int err;

	if (out_millideg == NULL) {
		return -EINVAL;
	}
	if (!fw_ready()) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&req_buf_lock, K_FOREVER);

	/* GET_TEMPERATURE value buffer is {u32 sensor_id, u32 temp}.
	 * Request sensor 0; VC writes the temperature (millidegrees C)
	 * back into the second word.
	 */
	req_buf[5] = 0;
	req_buf[6] = 0;

	err = property_call_locked(RPI_FW_TAG_GET_TEMPERATURE, 2);
	if (err == 0) {
		*out_millideg = (int32_t)req_buf[6];
		/* DBG, not INF: this tag is polled for the status bar. */
		LOG_DBG("SoC temperature: %d millideg C", *out_millideg);
	}

	k_mutex_unlock(&req_buf_lock);
	return err;
}
