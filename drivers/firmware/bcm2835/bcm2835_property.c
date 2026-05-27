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
#define RPI_FW_TAG_GET_CLOCK_RATE   0x00030002U
#define RPI_FW_TAG_GET_TEMPERATURE  0x00030006U
#define RPI_FW_TAG_FB_ALLOCATE      0x00040001U
#define RPI_FW_TAG_FB_GET_PHYS_WH   0x00040003U
#define RPI_FW_TAG_FB_GET_PITCH     0x00040008U
#define RPI_FW_TAG_FB_SET_PHYS_WH   0x00048003U
#define RPI_FW_TAG_FB_SET_VIRT_WH   0x00048004U
#define RPI_FW_TAG_FB_SET_DEPTH     0x00048005U
#define RPI_FW_TAG_FB_SET_PIXEL_ORDER 0x00048006U

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
 *
 * 64 words / 256 bytes / 4 cache lines: large enough to hold the
 * framebuffer-setup tag chain (6 tags + outer header + end tag = 30
 * words / 120 bytes) with headroom for any future chained requests.
 */
#define REQ_BUF_WORDS  64U
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

int bcm2835_property_get_clock_rate(uint32_t clock_id, uint32_t *out_hz)
{
	int err;

	if (out_hz == NULL) {
		return -EINVAL;
	}
	if (!fw_ready()) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&req_buf_lock, K_FOREVER);

	/* GET_CLOCK_RATE value buffer is {u32 clock_id, u32 rate}.
	 * Request the clock; VC writes the rate (Hz) into the second word.
	 */
	req_buf[5] = clock_id;
	req_buf[6] = 0;

	err = property_call_locked(RPI_FW_TAG_GET_CLOCK_RATE, 2);
	if (err == 0) {
		*out_hz = req_buf[6];
		LOG_DBG("clock %u rate: %u Hz", clock_id, *out_hz);
	}

	k_mutex_unlock(&req_buf_lock);
	return err;
}

int bcm2835_property_fb_get_size(uint32_t *width, uint32_t *height)
{
	int err;

	if (width == NULL || height == NULL) {
		return -EINVAL;
	}
	if (!fw_ready()) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&req_buf_lock, K_FOREVER);

	/* GET_PHYSICAL_WIDTH_HEIGHT: request payload is empty; VC writes
	 * the current scanout {W, H} into the value buffer. Single-tag
	 * GET calls don't suffer the SET-clamping issue that forced the
	 * FB-setup tags into one chained request.
	 */
	req_buf[5] = 0;
	req_buf[6] = 0;

	err = property_call_locked(RPI_FW_TAG_FB_GET_PHYS_WH, 2);
	if (err == 0) {
		*width = req_buf[5];
		*height = req_buf[6];
		LOG_INF("vc display size: %u x %u", *width, *height);
	}

	k_mutex_unlock(&req_buf_lock);
	return err;
}

/*
 * Framebuffer setup -- atomic, single chained property call.
 *
 * VC owns HDMI output: the GPU brings up the PHY based on config.txt at
 * boot, scans out a framebuffer continuously, and lets the ARM side
 * request/configure that buffer via the property interface. The
 * configuration tags must arrive as a single chained request -- VC
 * processes each property call atomically, and SET_VIRT_WH / SET_DEPTH
 * are silently clamped to minimums when they arrive in a call separate
 * from ALLOCATE_BUFFER (observed 2026-05-27: 6 separate calls landed
 * VC at virt=2x2, depth=16 even though phys was accepted at 640x480).
 *
 * Matches Linux drivers/video/fbdev/bcm2708_fb.c and u-boot
 * drivers/video/bcm2835.c, both of which chain.
 *
 * Chain layout (offsets in u32 words from req_buf base):
 *
 *   [0]  total bytes
 *   [1]  request code
 *   [2..6]   SET_PHYS_WH  : id, buf=8, req=8, W, H
 *   [7..11]  SET_VIRT_WH  : id, buf=8, req=8, W, H
 *   [12..15] SET_DEPTH    : id, buf=4, req=4, bpp
 *   [16..19] SET_PIXEL_ORDER: id, buf=4, req=4, order
 *   [20..24] ALLOCATE_BUFFER: id, buf=8, req=4, alignment, (size after)
 *   [25..28] GET_PITCH    : id, buf=4, req=0, (pitch after)
 *   [29] END_TAG
 *
 * VC writes responses back into the same words: phys at [5..6], virt at
 * [10..11], depth at [15], order at [19], FB base + size at [23..24],
 * pitch at [28].
 */

int bcm2835_property_fb_setup(uint32_t *width, uint32_t *height,
			      uint32_t *depth, uint32_t *pixel_order,
			      uint32_t alignment,
			      uintptr_t *fb_bus, uint32_t *fb_size,
			      uint32_t *pitch)
{
	uint32_t reply;
	int err;

	if (width == NULL || height == NULL || depth == NULL ||
	    pixel_order == NULL || fb_bus == NULL || fb_size == NULL ||
	    pitch == NULL) {
		return -EINVAL;
	}
	if (!fw_ready()) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&req_buf_lock, K_FOREVER);

	/* Outer wrapper -- total bytes filled in below. */
	req_buf[1] = RPI_FW_REQUEST;

	/* SET_PHYS_WH (req=8, resp=8, value_buf=8). */
	req_buf[2] = RPI_FW_TAG_FB_SET_PHYS_WH;
	req_buf[3] = 8U;
	req_buf[4] = 8U;
	req_buf[5] = *width;
	req_buf[6] = *height;

	/* SET_VIRT_WH (req=8, resp=8, value_buf=8). */
	req_buf[7] = RPI_FW_TAG_FB_SET_VIRT_WH;
	req_buf[8] = 8U;
	req_buf[9] = 8U;
	req_buf[10] = *width;
	req_buf[11] = *height;

	/* SET_DEPTH (req=4, resp=4, value_buf=4). */
	req_buf[12] = RPI_FW_TAG_FB_SET_DEPTH;
	req_buf[13] = 4U;
	req_buf[14] = 4U;
	req_buf[15] = *depth;

	/* SET_PIXEL_ORDER (req=4, resp=4, value_buf=4). */
	req_buf[16] = RPI_FW_TAG_FB_SET_PIXEL_ORDER;
	req_buf[17] = 4U;
	req_buf[18] = 4U;
	req_buf[19] = *pixel_order;

	/* ALLOCATE_BUFFER (req=4 alignment, resp=8 {base,size}, value_buf=8). */
	req_buf[20] = RPI_FW_TAG_FB_ALLOCATE;
	req_buf[21] = 8U;
	req_buf[22] = 4U;
	req_buf[23] = alignment;
	req_buf[24] = 0U;

	/* GET_PITCH (req=0, resp=4, value_buf=4). */
	req_buf[25] = RPI_FW_TAG_FB_GET_PITCH;
	req_buf[26] = 4U;
	req_buf[27] = 0U;
	req_buf[28] = 0U;

	req_buf[29] = RPI_FW_END_TAG;

	req_buf[0] = 30U * sizeof(uint32_t);

	sys_cache_data_flush_range(req_buf, sizeof(req_buf));

	err = bcm2835_mbox_call(fw_mbox, BCM2835_MBOX_CHAN_PROPERTY,
				(uint32_t)(uintptr_t)req_buf, &reply);
	if (err < 0) {
		LOG_ERR("fb mbox call failed: %d", err);
		goto unlock;
	}

	sys_cache_data_invd_range(req_buf, sizeof(req_buf));

	if (req_buf[1] != RPI_FW_RESPONSE_SUCCESS) {
		LOG_ERR("VC rejected fb chain, status 0x%08x", req_buf[1]);
		err = -EIO;
		goto unlock;
	}

	*width = req_buf[10];          /* virt W -- what the FB actually is */
	*height = req_buf[11];         /* virt H */
	*depth = req_buf[15];
	*pixel_order = req_buf[19];
	*fb_bus = req_buf[23];
	*fb_size = req_buf[24];
	*pitch = req_buf[28];

	LOG_INF("fb: phys=%u x %u virt=%u x %u %u bpp %s base=0x%08x size=%u pitch=%u",
		req_buf[5], req_buf[6], *width, *height, *depth,
		*pixel_order ? "RGB" : "BGR",
		(uint32_t)*fb_bus, *fb_size, *pitch);

unlock:
	k_mutex_unlock(&req_buf_lock);
	return err;
}
