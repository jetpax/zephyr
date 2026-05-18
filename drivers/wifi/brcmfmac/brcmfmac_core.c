/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM43xxx SDIO Wi-Fi driver (brcmfmac protocol).
 *
 * Phase 4.5b: WPA2-PSK association + event-driven link state + DHCP
 * autostart on top of 4.5a's net_if + scan. Connect IOCTL sequence
 * (mpc/auth/wsec/wpa_auth/wsec_pmk/WLC_SET_SSID) and chan=1 event
 * parsing (WLC_E_AUTH / ASSOC / LINK / DISASSOC_IND -> wifi_mgmt
 * raise calls) live in brcmfmac_net.c. On link-up: net_if_dormant_off
 * + net_dhcpv4_restart drive the iface to UP and acquire an IP.
 */

#define DT_DRV_COMPAT brcm_bcm43xxx_sdio

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>

#include "brcmfmac_priv.h"

LOG_MODULE_REGISTER(brcmfmac, CONFIG_WIFI_LOG_LEVEL);

static int brcmfmac_bringup(struct brcmfmac_data *data)
{
	int ret = brcmfmac_chip_read_id(data);
	if (ret != 0) {
		return ret;
	}
	if (data->chip_id == 43430 && data->chip_rev == 1) {
		LOG_INF("  -> matches BCM43430A1");
	}

	ret = brcmfmac_chip_pmu_setup(data);
	if (ret != 0) {
		return ret;
	}

	ret = brcmfmac_chip_erom_scan(data);
	if (ret != 0) {
		return ret;
	}

	ret = brcmfmac_chip_set_passive(data);
	if (ret != 0) {
		return ret;
	}

	ret = brcmfmac_sdio_fw_upload(data);
	if (ret != 0) {
		return ret;
	}

	ret = brcmfmac_sdio_nvram_upload(data);
	if (ret != 0) {
		return ret;
	}

	return brcmfmac_chip_set_active(data);
}

static int brcmfmac_probe_sdio(const struct device *dev)
{
	const struct brcmfmac_config *cfg = dev->config;
	struct brcmfmac_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->sdhc)) {
		LOG_ERR("SDHC parent %s not ready", cfg->sdhc->name);
		return -ENODEV;
	}

	ret = sd_init(cfg->sdhc, &data->card);
	if (ret != 0) {
		LOG_ERR("sd_init failed: %d", ret);
		return ret;
	}
	LOG_INF("sd_init ok: num_io=%u rca=0x%04x bus_width=%u",
		(unsigned int)data->card.num_io,
		data->card.relative_addr,
		data->card.bus_io.bus_width);

	ret = sdio_init_func(&data->card, &data->backplane, SDIO_FUNC_NUM_1);
	if (ret != 0) {
		LOG_ERR("sdio_init_func(F1) failed: %d", ret);
		return ret;
	}
	ret = sdio_enable_func(&data->backplane);
	if (ret != 0) {
		LOG_ERR("sdio_enable_func(F1) failed: %d", ret);
		return ret;
	}
	/* sdio_init_func leaves block_size=0; subsys helper then divides by
	 * zero on the block-mode branch. brcmfmac canonical for F1 is 64.
	 */
	ret = sdio_set_block_size(&data->backplane, 64);
	if (ret != 0) {
		LOG_ERR("sdio_set_block_size(F1, 64) failed: %d", ret);
		return ret;
	}
	LOG_INF("F1 claimed (max_blk=%u)", data->backplane.cis.max_blk_size);
	return 0;
}

static int brcmfmac_init(const struct device *dev)
{
	struct brcmfmac_data *data = dev->data;

	int ret = brcmfmac_probe_sdio(dev);
	if (ret != 0) {
		return ret;
	}

	int64_t t0 = k_uptime_get();
	ret = brcmfmac_bringup(data);
	if (ret != 0) {
		LOG_ERR("bring-up failed: %d", ret);
		return ret;
	}

	ret = brcmfmac_bcdc_init(data);
	if (ret != 0) {
		LOG_ERR("BCDC init failed: %d", ret);
		return ret;
	}

	int got = brcmfmac_bcdc_iovar_get(data, "cur_etheraddr",
					  data->chip_mac, sizeof(data->chip_mac));
	if (got < (int)sizeof(data->chip_mac)) {
		LOG_ERR("cur_etheraddr read returned %d (want >=6)", got);
		return (got < 0) ? got : -EIO;
	}
	LOG_INF("chip MAC = %02x:%02x:%02x:%02x:%02x:%02x",
		data->chip_mac[0], data->chip_mac[1], data->chip_mac[2],
		data->chip_mac[3], data->chip_mac[4], data->chip_mac[5]);

	/* WLC_UP: bring the MAC layer up. Most write IOCTLs (notably the
	 * "escan" IOVAR) return BCME_NOTUP (-4) until this fires.
	 */
	ret = brcmfmac_bcdc_set_dcmd(data, BRCMFMAC_WLC_UP, NULL, 0);
	if (ret != 0) {
		LOG_ERR("WLC_UP failed: %d", ret);
		return ret;
	}
	LOG_INF("WLC_UP ok");

	/* Enable the events we care about in the chip's event mask. Read
	 * current mask first so we don't clobber chip defaults. Events the
	 * chip leaves disabled by default but we need:
	 *   - WLC_E_ESCAN_RESULT  (scan results stream)
	 *   - WLC_E_AUTH          (auth attempt outcome)
	 *   - WLC_E_ASSOC         (assoc attempt outcome)
	 *   - WLC_E_LINK          (link up/down -- triggers dormant_off + DHCP)
	 *   - WLC_E_DISASSOC_IND  (disconnect notice)
	 *   - WLC_E_SET_SSID      (echo of WLC_SET_SSID outcome)
	 */
	uint8_t event_mask[BRCMFMAC_EVENTING_MASK_LEN] = {0};
	int em_got = brcmfmac_bcdc_iovar_get(data, "event_msgs",
					     event_mask, sizeof(event_mask));
	if (em_got < (int)sizeof(event_mask)) {
		LOG_WRN("event_msgs get returned %d; starting from zero", em_got);
		memset(event_mask, 0, sizeof(event_mask));
	}
#define ENABLE_EVENT(ev) \
	(event_mask[(ev) / 8] |= (uint8_t)(1u << ((ev) % 8)))
	ENABLE_EVENT(WLC_E_ESCAN_RESULT);
	ENABLE_EVENT(WLC_E_AUTH);
	ENABLE_EVENT(WLC_E_ASSOC);
	ENABLE_EVENT(WLC_E_LINK);
	ENABLE_EVENT(WLC_E_DISASSOC_IND);
	ENABLE_EVENT(WLC_E_DEAUTH);
	ENABLE_EVENT(WLC_E_DEAUTH_IND);
	ENABLE_EVENT(WLC_E_AUTH_FAIL);
	ENABLE_EVENT(WLC_E_PSK_SUP);
	ENABLE_EVENT(WLC_E_SET_SSID);
#undef ENABLE_EVENT
	ret = brcmfmac_bcdc_iovar_set(data, "event_msgs",
				      event_mask, sizeof(event_mask));
	if (ret != 0) {
		LOG_ERR("event_msgs set failed: %d", ret);
		return ret;
	}
	LOG_DBG("event_msgs set (escan + auth/assoc/link/disassoc/set_ssid)");

	/* Mirror Linux brcmf_dongle_roam + brcmf_cfg80211_set_power_mgmt
	 * setup. Without these the firmware's defaults left us self-deauthing
	 * after ~10 Mbit/s UDP bursts (chip emitted WLC_E_LINK reason=2
	 * BRCMF_E_REASON_DEAUTH without an inbound deauth frame -- i.e. the
	 * firmware itself decided the AP was unreachable).
	 *
	 * Specifically:
	 *   - roam_off=1 disables firmware-internal roaming (we have one AP)
	 *   - bcn_timeout=4 matches Linux's roam-off default
	 *   - pm=PM_FAST + pm2_sleep_ret=2000 matches Linux's default
	 *     "power save on, but wake immediately on TX activity"
	 *
	 * Best-effort: errors are logged but non-fatal -- the chip still
	 * brings up, just with looser defaults.
	 */
	{
		const uint32_t roam_off    = 1;     /* no firmware-side roaming */
		const uint32_t bcn_timeout = 4;     /* seconds without beacons */
		const uint32_t pm_mode     = 2;     /* PM_FAST */
		const uint32_t pm2_sleep   = 2000;  /* ms */
		int rret;

		rret = brcmfmac_bcdc_iovar_set(data, "roam_off",
					       (const uint8_t *)&roam_off, 4);
		if (rret != 0) {
			LOG_WRN("roam_off=1 set failed: %d (best-effort)", rret);
		}
		rret = brcmfmac_bcdc_iovar_set(data, "bcn_timeout",
					       (const uint8_t *)&bcn_timeout, 4);
		if (rret != 0) {
			LOG_WRN("bcn_timeout=4 set failed: %d (best-effort)", rret);
		}
		rret = brcmfmac_bcdc_set_dcmd(data, BRCMFMAC_WLC_SET_PM,
					      (const uint8_t *)&pm_mode, 4);
		if (rret != 0) {
			LOG_WRN("WLC_SET_PM=FAST set failed: %d (best-effort)", rret);
		}
		rret = brcmfmac_bcdc_iovar_set(data, "pm2_sleep_ret",
					       (const uint8_t *)&pm2_sleep, 4);
		if (rret != 0) {
			LOG_WRN("pm2_sleep_ret=2000 set failed: %d (best-effort)", rret);
		}
		LOG_INF("post-up tuning: roam_off=1 bcn_timeout=4s pm=FAST pm2_sleep_ret=2000ms");
	}

	data->probed = true;
	LOG_INF("bring-up complete in %lld ms; awaiting iface_init",
		(long long)(k_uptime_get() - t0));
	return 0;
}

/* === wifi_mgmt + ethernet_api wiring ====================================== */

static const struct wifi_mgmt_ops brcmfmac_mgmt_ops = {
	.scan         = brcmfmac_mgmt_scan,
	.connect      = brcmfmac_mgmt_connect,
	.disconnect   = brcmfmac_mgmt_disconnect,
	.iface_status = brcmfmac_mgmt_iface_status,
};

static const struct net_wifi_mgmt_offload brcmfmac_api = {
	.wifi_iface.iface_api.init = brcmfmac_iface_init,
	.wifi_iface.send           = brcmfmac_iface_send,
	.wifi_mgmt_api             = &brcmfmac_mgmt_ops,
};

/* Single-instance: the brcmfmac DT binding describes one chip on the
 * SDHC parent. Multi-instance would need a per-inst api + data + config
 * (a la DT_INST_FOREACH_STATUS_OKAY); deferred until we actually need it.
 */
static const struct brcmfmac_config brcmfmac_config_0 = {
	.sdhc          = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.reg_on        = GPIO_DT_SPEC_INST_GET_OR(0, wifi_reg_on_gpios, {0}),
	.firmware_name = DT_INST_PROP_OR(0, firmware_name, ""),
};

static struct brcmfmac_data brcmfmac_data_0;

NET_DEVICE_DT_INST_DEFINE(0, brcmfmac_init, NULL,
			  &brcmfmac_data_0, &brcmfmac_config_0,
			  CONFIG_WIFI_INIT_PRIORITY, &brcmfmac_api,
			  ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2),
			  NET_ETH_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
