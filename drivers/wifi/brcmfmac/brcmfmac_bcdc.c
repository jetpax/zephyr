/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCDC (Broadcom Comm Driver Codec) protocol over SDPCM/SDIO F2.
 *
 * Phase 4.4: dedicated RX kthread drains F2 and dispatches frames
 * by SDPCM channel:
 *   chan=0 (control) -> match by reqid, copy payload into the
 *                       waiting query_dcmd caller's buffer, signal
 *                       its completion semaphore.
 *   chan=1 (event)   -> raw frame body handed to the registered
 *                       event callback (if any).
 *   chan=2 (data)    -> logged + discarded (net_if arrives in 4.5).
 *
 * query_dcmd is now async at the protocol level: TX, then block on
 * a per-request semaphore until the RX thread signals or timeout.
 * F2's CIS rdy_timeout is overridden to 0 so sdio_enable_func polls
 * IOR directly instead of sleeping ~2 s per chip CIS request.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sd/sdio.h>

#include "brcmfmac_priv.h"

LOG_MODULE_DECLARE(brcmfmac, CONFIG_WIFI_LOG_LEVEL);

#define BRCMFMAC_BCDC_RX_BUF       512
#define BRCMFMAC_BCDC_TIMEOUT_MS  2000
#define BRCMFMAC_RX_IDLE_SLEEP_MS    1

/* Single-instance stack + thread storage. DT only describes one
 * brcmfmac node today; if multi-instance is wanted later, move
 * these into a per-instance macro expansion in core.c.
 */
K_KERNEL_STACK_DEFINE(brcmfmac_rx_stack, CONFIG_WIFI_BRCMFMAC_RX_THREAD_STACK_SIZE);
static struct k_thread brcmfmac_rx_thread;

static void brcmfmac_handle_ctrl(struct brcmfmac_data *data,
				 struct cdc_hdr *rcdc, uint16_t outlen)
{
	uint16_t reqid = (uint16_t)(rcdc->flags >> BCDC_REQ_ID_SHIFT);

	if (!data->pending.active || reqid != data->pending.reqid) {
		LOG_DBG("rx ctrl: unmatched reqid=%u (pending.active=%d pending.reqid=%u)",
			reqid, data->pending.active, data->pending.reqid);
		return;
	}

	if (rcdc->flags & BCDC_FLAG_ERROR) {
		LOG_WRN("rx ctrl: chip BCDC error (status=%u reqid=%u)",
			rcdc->status, reqid);
		data->pending.status = -EIO;
		data->pending.out_copied = 0;
	} else {
		uint16_t want = outlen;
		if (want > data->pending.out_capacity) {
			want = data->pending.out_capacity;
		}
		if (want > 0 && data->pending.out_buf != NULL) {
			memcpy(data->pending.out_buf,
			       (uint8_t *)rcdc + sizeof(*rcdc), want);
		}
		data->pending.out_copied = want;
		data->pending.status = 0;
	}

	k_sem_give(&data->pending.done);
}

static void brcmfmac_handle_event(struct brcmfmac_data *data,
				  struct sdpcm_sw_hdr *sw,
				  uint16_t total_len)
{
	uint16_t hdr_len = sw->hdrlen;
	if (hdr_len > total_len) {
		LOG_WRN("rx event: hdrlen=%u > total=%u", hdr_len, total_len);
		return;
	}
	uint16_t body_len = (uint16_t)(total_len - hdr_len);
	const uint8_t *body = (const uint8_t *)sw + (hdr_len - sizeof(struct sdpcm_frame_hdr));

	LOG_DBG("rx event: body_len=%u", body_len);
	if (data->event_cb) {
		data->event_cb(data, body, body_len);
	}
}

static void brcmfmac_rx_thread_fn(void *p1, void *p2, void *p3)
{
	struct brcmfmac_data *data = p1;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	static uint8_t rx_buf[BRCMFMAC_BCDC_RX_BUF] __aligned(4);

	LOG_INF("rx thread started");

	while (1) {
		int ret = sdio_read_addr(&data->radio, BRCMFMAC_F2_FIFO_ADDR,
					 rx_buf, sizeof(rx_buf));
		if (ret != 0) {
			LOG_DBG("rx thread: F2 read failed: %d", ret);
			k_msleep(BRCMFMAC_RX_IDLE_SLEEP_MS);
			continue;
		}

		struct sdpcm_frame_hdr *fh = (void *)rx_buf;
		struct sdpcm_sw_hdr *sw = (void *)(rx_buf + sizeof(*fh));

		uint16_t xor_check = (uint16_t)(fh->len ^ fh->notlen);
		if (xor_check != 0xFFFFu) {
			/* No valid frame queued -- chip's F2 returned junk. */
			k_msleep(BRCMFMAC_RX_IDLE_SLEEP_MS);
			continue;
		}
		if (fh->len < sizeof(*fh) + sizeof(*sw)) {
			LOG_WRN("rx thread: short frame (len=%u)", fh->len);
			k_msleep(BRCMFMAC_RX_IDLE_SLEEP_MS);
			continue;
		}

		switch (sw->chan) {
		case SDPCM_CHAN_CTRL: {
			if (fh->len < sizeof(*fh) + sizeof(*sw) + sizeof(struct cdc_hdr)) {
				LOG_WRN("rx ctrl: undersized (len=%u)", fh->len);
				break;
			}
			struct cdc_hdr *rcdc = (void *)((uint8_t *)sw + sizeof(*sw));
			uint16_t payload_avail = (uint16_t)(fh->len - sizeof(*fh)
							     - sizeof(*sw)
							     - sizeof(*rcdc));
			uint16_t outlen = rcdc->outlen;
			if (outlen > payload_avail) {
				outlen = payload_avail;
			}
			brcmfmac_handle_ctrl(data, rcdc, outlen);
			break;
		}
		case SDPCM_CHAN_EVENT:
			brcmfmac_handle_event(data, sw, fh->len);
			break;
		case SDPCM_CHAN_DATA:
			LOG_DBG("rx data: len=%u (net_if TODO)", fh->len);
			break;
		default:
			LOG_DBG("rx: unknown chan=%u len=%u", sw->chan, fh->len);
			break;
		}
	}
}

/* Enable F2 + wait for IOR ourselves. Zephyr's sdio_enable_func sleeps
 * for the full CIS-advertised rdy_timeout (~2 s on this chip) before
 * its first IOR poll; setting rdy_timeout=0 just makes it poll too
 * fast (CONFIG_SD_RETRY_COUNT × ~0 ms = ~3 ms total, chip is not
 * ready that quickly). Linux brcmfmac polls IOR with ~10 ms gaps up
 * to ~500 ms; we mirror that.
 */
static int brcmfmac_bcdc_enable_f2(struct brcmfmac_data *data)
{
	uint8_t reg;
	int ret = sdio_read_byte(&data->card.func0, SDIO_CCCR_IO_EN, &reg);
	if (ret != 0) {
		return ret;
	}
	reg |= BIT(SDIO_FUNC_NUM_2);
	ret = sdio_write_byte(&data->card.func0, SDIO_CCCR_IO_EN, reg);
	if (ret != 0) {
		return ret;
	}

	int64_t t0 = k_uptime_get();
	for (int i = 0; i < 50; i++) {
		ret = sdio_read_byte(&data->card.func0, SDIO_CCCR_IO_RD, &reg);
		if (ret != 0) {
			return ret;
		}
		if (reg & BIT(SDIO_FUNC_NUM_2)) {
			LOG_INF("F2 IOR up after %lld ms",
				(long long)(k_uptime_get() - t0));
			return 0;
		}
		k_msleep(10);
	}
	LOG_ERR("F2 IOR timeout after %lld ms (IOR=0x%02x)",
		(long long)(k_uptime_get() - t0), reg);
	return -ETIMEDOUT;
}

int brcmfmac_bcdc_init(struct brcmfmac_data *data)
{
	int ret = sdio_init_func(&data->card, &data->radio, SDIO_FUNC_NUM_2);
	if (ret != 0) {
		LOG_ERR("sdio_init_func(F2) failed: %d", ret);
		return ret;
	}

	ret = brcmfmac_bcdc_enable_f2(data);
	if (ret != 0) {
		LOG_ERR("F2 enable failed: %d", ret);
		return ret;
	}

	ret = sdio_set_block_size(&data->radio, BRCMFMAC_F2_BLOCK_SIZE);
	if (ret != 0) {
		LOG_ERR("sdio_set_block_size(F2, %u) failed: %d",
			BRCMFMAC_F2_BLOCK_SIZE, ret);
		return ret;
	}

	k_mutex_init(&data->bcdc_mutex);
	k_sem_init(&data->pending.done, 0, 1);
	data->pending.active = false;

	k_thread_create(&brcmfmac_rx_thread, brcmfmac_rx_stack,
			K_KERNEL_STACK_SIZEOF(brcmfmac_rx_stack),
			brcmfmac_rx_thread_fn, data, NULL, NULL,
			CONFIG_WIFI_BRCMFMAC_RX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&brcmfmac_rx_thread, "brcmfmac_rx");

	data->f2_ready = true;
	LOG_INF("F2 claimed (block_size=%u), rx thread up",
		BRCMFMAC_F2_BLOCK_SIZE);
	return 0;
}

void brcmfmac_bcdc_set_event_cb(struct brcmfmac_data *data,
				brcmfmac_event_cb_t cb)
{
	data->event_cb = cb;
}

/* Build SDPCM headers in-place at the start of `frame` and TX `total`
 * bytes (padded to 4) via incrementing CMD53 on F2.
 */
static int brcmfmac_bcdc_tx(struct brcmfmac_data *data, uint8_t chan,
			    uint8_t *frame, uint16_t total)
{
	struct sdpcm_frame_hdr *fh = (void *)frame;
	struct sdpcm_sw_hdr *sw = (void *)(frame + sizeof(*fh));

	fh->len = total;
	fh->notlen = (uint16_t)~total;

	sw->seq = data->sdpcm_txseq++;
	sw->chan = chan;
	sw->nextlen = 0;
	sw->hdrlen = (uint8_t)(sizeof(*fh) + sizeof(*sw));
	sw->flow = 0;
	sw->credit = 0;
	sw->reserved[0] = 0;
	sw->reserved[1] = 0;

	uint16_t padded = (total + 3) & ~3u;
	return sdio_write_addr(&data->radio, BRCMFMAC_F2_FIFO_ADDR, frame, padded);
}

int brcmfmac_bcdc_query_dcmd(struct brcmfmac_data *data, uint32_t cmd,
			     const uint8_t *tx_payload, uint16_t tx_len,
			     uint8_t *rx_buf, uint16_t rx_capacity)
{
	if (!data->f2_ready) {
		return -EAGAIN;
	}

	int rc = k_mutex_lock(&data->bcdc_mutex, K_FOREVER);
	if (rc != 0) {
		return rc;
	}

	static uint8_t tx_buf[256] __aligned(4);

	const size_t hdr_len = sizeof(struct sdpcm_frame_hdr)
			     + sizeof(struct sdpcm_sw_hdr)
			     + sizeof(struct cdc_hdr);

	if (hdr_len + tx_len > sizeof(tx_buf)) {
		rc = -EMSGSIZE;
		goto out;
	}

	memset(tx_buf, 0, sizeof(tx_buf));

	struct cdc_hdr *cdc = (void *)(tx_buf + sizeof(struct sdpcm_frame_hdr)
					      + sizeof(struct sdpcm_sw_hdr));

	cdc->cmd    = cmd;
	cdc->outlen = tx_len;
	cdc->inlen  = 0;
	data->bcdc_reqid++;
	cdc->flags  = ((uint32_t)data->bcdc_reqid << BCDC_REQ_ID_SHIFT);
	cdc->status = 0;

	if (tx_payload != NULL && tx_len > 0) {
		memcpy((uint8_t *)cdc + sizeof(*cdc), tx_payload, tx_len);
	}

	uint16_t total = (uint16_t)(hdr_len + tx_len);

	/* Publish the waiter context BEFORE TX -- the RX thread may race
	 * us to the response (chip can be fast).
	 */
	data->pending.reqid = data->bcdc_reqid;
	data->pending.out_buf = rx_buf;
	data->pending.out_capacity = rx_capacity;
	data->pending.out_copied = 0;
	data->pending.status = 0;
	k_sem_reset(&data->pending.done);
	data->pending.active = true;

	rc = brcmfmac_bcdc_tx(data, SDPCM_CHAN_CTRL, tx_buf, total);
	if (rc != 0) {
		LOG_ERR("bcdc_query: TX failed: %d", rc);
		data->pending.active = false;
		goto out;
	}

	rc = k_sem_take(&data->pending.done, K_MSEC(BRCMFMAC_BCDC_TIMEOUT_MS));
	data->pending.active = false;

	if (rc == -EAGAIN) {
		LOG_ERR("bcdc_query: timeout waiting on reqid=%u",
			data->pending.reqid);
		rc = -ETIMEDOUT;
		goto out;
	}
	if (rc != 0) {
		goto out;
	}

	rc = (data->pending.status != 0)
		? data->pending.status
		: (int)data->pending.out_copied;

out:
	k_mutex_unlock(&data->bcdc_mutex);
	return rc;
}

int brcmfmac_bcdc_iovar_get(struct brcmfmac_data *data, const char *name,
			    uint8_t *buf, uint16_t len)
{
	size_t name_len = strlen(name) + 1;
	static uint8_t scratch[64] __aligned(4);

	if (name_len > sizeof(scratch)) {
		return -EMSGSIZE;
	}

	memset(scratch, 0, sizeof(scratch));
	memcpy(scratch, name, name_len);

	return brcmfmac_bcdc_query_dcmd(data, BRCMFMAC_WLC_GET_VAR,
					scratch, (uint16_t)name_len,
					buf, len);
}
