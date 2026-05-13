/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM43xxx SDIO Wi-Fi driver (brcmfmac protocol).
 *
 * Phase 4.5a: net_if registration (ETHERNET_L2) + iface_api.send/recv
 * + wifi_mgmt_ops.scan via the "escan" IOVAR. Iface stays dormant
 * until association lands in 4.5b. Connect/disconnect, event-driven
 * link state, and DHCP integration are 4.5b's job.
 */

#define DT_DRV_COMPAT brcm_bcm43xxx_sdio

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
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

	/* Enable WLC_E_ESCAN_RESULT in the chip's event mask, otherwise
	 * scan results never reach chan=1. Read current mask first so we
	 * don't clobber chip defaults.
	 */
	uint8_t event_mask[BRCMFMAC_EVENTING_MASK_LEN] = {0};
	int em_got = brcmfmac_bcdc_iovar_get(data, "event_msgs",
					     event_mask, sizeof(event_mask));
	if (em_got < (int)sizeof(event_mask)) {
		LOG_WRN("event_msgs get returned %d; starting from zero", em_got);
		memset(event_mask, 0, sizeof(event_mask));
	}
	event_mask[WLC_E_ESCAN_RESULT / 8] |= (1u << (WLC_E_ESCAN_RESULT % 8));
	ret = brcmfmac_bcdc_iovar_set(data, "event_msgs",
				      event_mask, sizeof(event_mask));
	if (ret != 0) {
		LOG_ERR("event_msgs set failed: %d", ret);
		return ret;
	}
	LOG_INF("event_msgs set (WLC_E_ESCAN_RESULT enabled)");

	data->probed = true;
	LOG_INF("Phase 4.5a: bring-up + BCDC + RX thread complete in %lld ms; iface_init pending",
		(long long)(k_uptime_get() - t0));
	return 0;
}

/* Phase 4.5a verification scaffold: trigger a single escan a few seconds
 * after boot and log results. Disposable -- goes away when MP
 * network.WLAN (Phase 4.6) provides a real entry point.
 */
static void brcmfmac_phase45a_scan_log_cb(struct net_if *iface, int status,
					  struct wifi_scan_result *entry)
{
	ARG_UNUSED(iface);
	if (entry == NULL) {
		LOG_INF("phase 4.5a scan complete (status=%d)", status);
		return;
	}
	LOG_INF("scan: \"%.*s\"  bssid=%02x:%02x:%02x:%02x:%02x:%02x  rssi=%d  ch=%u",
		entry->ssid_length, entry->ssid,
		entry->mac[0], entry->mac[1], entry->mac[2],
		entry->mac[3], entry->mac[4], entry->mac[5],
		entry->rssi, entry->channel);
}

static void brcmfmac_phase45a_scan_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	const struct device *dev = DEVICE_DT_INST_GET(0);
	LOG_INF("phase 4.5a: triggering test escan");
	int ret = brcmfmac_mgmt_scan(dev, NULL, NULL, brcmfmac_phase45a_scan_log_cb);
	if (ret != 0) {
		LOG_ERR("phase 4.5a scan failed to start: %d", ret);
	}
}

static K_WORK_DELAYABLE_DEFINE(brcmfmac_phase45a_scan_work,
			       brcmfmac_phase45a_scan_work_fn);

static int brcmfmac_phase45a_arm_test(void)
{
	k_work_schedule(&brcmfmac_phase45a_scan_work, K_SECONDS(3));
	return 0;
}
SYS_INIT(brcmfmac_phase45a_arm_test, APPLICATION, 99);

/* === wifi_mgmt + ethernet_api wiring ====================================== */

static const struct wifi_mgmt_ops brcmfmac_mgmt_ops = {
	.scan = brcmfmac_mgmt_scan,
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
