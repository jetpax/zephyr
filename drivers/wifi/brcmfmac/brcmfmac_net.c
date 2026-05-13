/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 *
 * net_if + wifi_mgmt glue + RX dispatch hooks for chan=1 (event) and
 * chan=2 (data) frames coming from the brcmfmac RX thread.
 *
 * Phase 4.5a: iface_api.init + iface_api.send, RX -> net_recv_data,
 * wifi_mgmt_ops.scan via the "escan" IOVAR. Connect/disconnect and
 * full event parsing (WLC_E_LINK / WLC_E_AUTH / WLC_E_DISASSOC_IND)
 * land in 4.5b.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sys/byteorder.h>

#include "brcmfmac_priv.h"

LOG_MODULE_DECLARE(brcmfmac, CONFIG_WIFI_LOG_LEVEL);

#define BRCMFMAC_ETH_FRAME_BUF_SIZE   1600

/* === net_if init =========================================================== */

void brcmfmac_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct brcmfmac_data *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	data->iface = iface;

	if (net_if_set_link_addr(iface, data->chip_mac, 6, NET_LINK_ETHERNET) != 0) {
		LOG_ERR("iface_init: net_if_set_link_addr failed");
	}

	ethernet_init(iface);
	net_if_dormant_on(iface);

	LOG_INF("iface up: MAC %02x:%02x:%02x:%02x:%02x:%02x (dormant until assoc)",
		data->chip_mac[0], data->chip_mac[1], data->chip_mac[2],
		data->chip_mac[3], data->chip_mac[4], data->chip_mac[5]);
}

/* === TX path (L2 frame -> BDC + SDPCM -> F2 chan=2) ======================== */

int brcmfmac_iface_send(const struct device *dev, struct net_pkt *pkt)
{
	struct brcmfmac_data *data = dev->data;
	size_t pkt_len = net_pkt_get_len(pkt);
	static uint8_t tx_buf[BRCMFMAC_ETH_FRAME_BUF_SIZE] __aligned(4);

	if (!data->probed || !data->f2_ready) {
		return -ENETDOWN;
	}

	const size_t hdr_len = sizeof(struct sdpcm_frame_hdr)
			     + sizeof(struct sdpcm_sw_hdr)
			     + sizeof(struct bdc_hdr);

	if (hdr_len + pkt_len > sizeof(tx_buf)) {
		LOG_ERR("iface_send: oversize (%zu+%zu > %zu)",
			hdr_len, pkt_len, sizeof(tx_buf));
		return -EMSGSIZE;
	}

	if (net_pkt_read(pkt, tx_buf + hdr_len, pkt_len) < 0) {
		LOG_ERR("iface_send: net_pkt_read failed");
		return -EIO;
	}

	struct bdc_hdr *bdc = (void *)(tx_buf + sizeof(struct sdpcm_frame_hdr)
					      + sizeof(struct sdpcm_sw_hdr));
	bdc->flags = BDC_PROTO_VER << BDC_PROTO_VER_SHIFT;
	bdc->priority = 0;
	bdc->flags2 = 0;
	bdc->data_offset = 0;

	uint16_t total = (uint16_t)(hdr_len + pkt_len);

	k_mutex_lock(&data->bcdc_mutex, K_FOREVER);
	int ret = brcmfmac_bcdc_tx_frame(data, SDPCM_CHAN_DATA, tx_buf, total);
	k_mutex_unlock(&data->bcdc_mutex);

	if (ret != 0) {
		LOG_ERR("iface_send: F2 TX failed: %d", ret);
		return -EIO;
	}
	return 0;
}

/* === RX dispatch helpers (called from bcdc.c RX thread) ==================== */

/* Strip BDC header + data_offset padding; return pointer/length of the L2
 * frame within `body`. Returns NULL on malformed input.
 */
static const uint8_t *bdc_strip(const uint8_t *body, uint16_t body_len,
				uint16_t *l2_len_out)
{
	if (body_len < BDC_HEADER_LEN) {
		return NULL;
	}
	const struct bdc_hdr *bdc = (const void *)body;
	uint16_t skip = BDC_HEADER_LEN + (uint16_t)(bdc->data_offset * 4);
	if (skip > body_len) {
		return NULL;
	}
	*l2_len_out = body_len - skip;
	return body + skip;
}

void brcmfmac_net_rx_data(struct brcmfmac_data *data,
			  const uint8_t *body, uint16_t body_len)
{
	uint16_t l2_len = 0;
	const uint8_t *l2 = bdc_strip(body, body_len, &l2_len);
	if (l2 == NULL) {
		LOG_WRN("rx data: BDC strip failed (body_len=%u)", body_len);
		return;
	}

	if (data->iface == NULL || !net_if_flag_is_set(data->iface, NET_IF_UP)) {
		/* Iface not up yet (e.g. pre-assoc). Drop. */
		return;
	}

	struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(data->iface, l2_len,
							    AF_UNSPEC, 0,
							    K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("rx data: net_pkt alloc failed (len=%u)", l2_len);
		return;
	}

	if (net_pkt_write(pkt, l2, l2_len) < 0) {
		LOG_ERR("rx data: net_pkt_write failed");
		net_pkt_unref(pkt);
		return;
	}

	if (net_recv_data(data->iface, pkt) < 0) {
		LOG_WRN("rx data: net_recv_data rejected");
		net_pkt_unref(pkt);
	}
}

/* === Event parsing (chan=1) ================================================
 *
 * Body layout after BDC strip:
 *   ethhdr (14)            -- dst[6] src[6] type[2 BE] (type == 0x886C)
 *   brcm_ethhdr (10)
 *   brcmf_event_msg_be (36 BE)
 *   event-specific payload
 */
#define EVENT_FIXED_HEADER_LEN  (14 + sizeof(struct brcm_ethhdr) + sizeof(struct brcmf_event_msg_be))

static void brcmfmac_handle_escan_event(struct brcmfmac_data *data,
					uint32_t status, uint32_t datalen,
					const uint8_t *payload)
{
	if (data->scan_cb == NULL || data->iface == NULL) {
		return;
	}

	if (status == BRCMF_E_STATUS_PARTIAL) {
		if (datalen < sizeof(struct brcmf_escan_result_le)) {
			LOG_WRN("escan partial: undersized payload (%u)", datalen);
			return;
		}
		const struct brcmf_escan_result_le *res = (const void *)payload;
		const struct brcmf_bss_info_le *bss = &res->bss_info_le;

		struct wifi_scan_result entry = {0};

		uint8_t ssid_len = bss->SSID_len;
		if (ssid_len > WIFI_SSID_MAX_LEN) {
			ssid_len = WIFI_SSID_MAX_LEN;
		}
		memcpy(entry.ssid, bss->SSID, ssid_len);
		entry.ssid_length = ssid_len;

		memcpy(entry.mac, bss->BSSID, 6);
		entry.mac_length = 6;

		/* BCM43430A1 is 2.4 GHz only -- low byte of chanspec is
		 * the control channel.
		 */
		entry.band = WIFI_FREQ_BAND_2_4_GHZ;
		entry.channel = (uint8_t)(bss->chanspec & 0xFF);

		/* Defer real RSN/WPA-IE parsing to 4.5c. For 4.5a, mark
		 * security UNKNOWN so the shell prints something visible
		 * but no assumption is made.
		 */
		entry.security = WIFI_SECURITY_TYPE_UNKNOWN;
		entry.mfp = WIFI_MFP_DISABLE;

		int16_t rssi = (int16_t)bss->RSSI;
		entry.rssi = (int8_t)rssi;

		data->scan_cb(data->iface, 0, &entry);
	} else if (status == BRCMF_E_STATUS_SUCCESS) {
		LOG_INF("escan complete");
		data->scan_cb(data->iface, 0, NULL);
		data->scan_cb = NULL;
	} else {
		LOG_WRN("escan event status=%u (treating as failure)", status);
		data->scan_cb(data->iface, -EIO, NULL);
		data->scan_cb = NULL;
	}
}

void brcmfmac_net_rx_event(struct brcmfmac_data *data,
			   const uint8_t *body, uint16_t body_len)
{
	uint16_t l2_len = 0;
	const uint8_t *l2 = bdc_strip(body, body_len, &l2_len);
	if (l2 == NULL || l2_len < EVENT_FIXED_HEADER_LEN) {
		LOG_DBG("rx event: undersized (body_len=%u)", body_len);
		return;
	}

	/* Ethernet type at offset 12 (after src+dst MACs). */
	uint16_t ethertype = sys_get_be16(l2 + 12);
	if (ethertype != BRCM_ETHERTYPE_EVENT) {
		LOG_DBG("rx event: non-Broadcom ethertype 0x%04x", ethertype);
		return;
	}

	const struct brcmf_event_msg_be *msg =
		(const void *)(l2 + 14 + sizeof(struct brcm_ethhdr));

	uint32_t event_type = sys_be32_to_cpu(msg->event_type);
	uint32_t status     = sys_be32_to_cpu(msg->status);
	uint32_t datalen    = sys_be32_to_cpu(msg->datalen);

	const uint8_t *payload = (const uint8_t *)msg + sizeof(*msg);
	uint16_t payload_avail = (uint16_t)(l2_len - EVENT_FIXED_HEADER_LEN);
	if (datalen > payload_avail) {
		datalen = payload_avail;
	}

	switch (event_type) {
	case WLC_E_ESCAN_RESULT:
		brcmfmac_handle_escan_event(data, status, datalen, payload);
		break;
	default:
		LOG_DBG("event %u status=%u datalen=%u (no handler)",
			event_type, status, datalen);
		break;
	}
}

/* === wifi_mgmt_ops.scan ==================================================== */

int brcmfmac_mgmt_scan(const struct device *dev, struct net_if *iface,
		       struct wifi_scan_params *params, scan_result_cb_t cb)
{
	struct brcmfmac_data *data = dev->data;
	ARG_UNUSED(iface);
	ARG_UNUSED(params);

	if (!data->probed) {
		return -ENETDOWN;
	}
	if (data->scan_cb != NULL) {
		return -EINPROGRESS;
	}

	struct brcmf_escan_params_le esc;
	memset(&esc, 0, sizeof(esc));
	esc.version = BRCMF_SCAN_PARAMS_VERSION;
	esc.action  = WL_ESCAN_ACTION_START;
	esc.sync_id = 1;

	esc.params_le.bss_type     = 2;             /* DOT11_BSSTYPE_ANY */
	esc.params_le.scan_type    = 0;             /* active */
	esc.params_le.nprobes      = (uint32_t)-1;  /* defaults */
	esc.params_le.active_time  = (uint32_t)-1;
	esc.params_le.passive_time = (uint32_t)-1;
	esc.params_le.home_time    = (uint32_t)-1;
	esc.params_le.channel_num  = 0;             /* all channels */
	memset(esc.params_le.bssid, 0xFF, 6);       /* broadcast */
	esc.params_le.ssid_le.SSID_len = 0;         /* broadcast SSID */

	/* Park the cb BEFORE firing the IOVAR so an event arriving fast
	 * (which we've observed -- the chip can have queued frames) has
	 * a place to land.
	 */
	data->scan_cb = cb;

	int ret = brcmfmac_bcdc_iovar_set(data, "escan",
					   (const uint8_t *)&esc, sizeof(esc));
	if (ret != 0) {
		LOG_ERR("escan iovar_set failed: %d", ret);
		data->scan_cb = NULL;
		return ret;
	}
	LOG_INF("escan started -- results streaming via WLC_E_ESCAN_RESULT");
	return 0;
}
