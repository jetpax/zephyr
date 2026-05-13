/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM43xxx SDIO Wi-Fi driver (brcmfmac protocol).
 *
 * Phase 4.4: RX kthread + event dispatch + per-reqid semaphore land.
 * F2 ready_timeout overridden to 0 so the ~2 s sdio_enable_func wait
 * goes away. Boot from POR through MAC read drops back into the
 * sub-second range.
 *
 * Sub-phases to come: net_if + wifi_mgmt (4.5), MP network.WLAN (4.6).
 */

#define DT_DRV_COMPAT brcm_bcm43xxx_sdio

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>

#include "brcmfmac_priv.h"

LOG_MODULE_REGISTER(brcmfmac, CONFIG_WIFI_LOG_LEVEL);

/* Phase 4.4 placeholder event handler: log chan=1 frames as they arrive
 * from the chip. Structured event parsing (WLC_E_LINK, WLC_E_AUTH, ...)
 * happens in 4.5 when wifi_mgmt needs to translate them into Zephyr's
 * wifi_mgmt_raise_*_event calls.
 */
static void brcmfmac_phase44_event_log(struct brcmfmac_data *data,
				       const uint8_t *frame, uint16_t len)
{
	ARG_UNUSED(data);
	LOG_INF("event frame: len=%u  hdr0..3=%02x %02x %02x %02x",
		len,
		len > 0 ? frame[0] : 0u,
		len > 1 ? frame[1] : 0u,
		len > 2 ? frame[2] : 0u,
		len > 3 ? frame[3] : 0u);
}

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

	brcmfmac_bcdc_set_event_cb(data, brcmfmac_phase44_event_log);

	int got = brcmfmac_bcdc_iovar_get(data, "cur_etheraddr",
					  data->chip_mac, sizeof(data->chip_mac));
	if (got < (int)sizeof(data->chip_mac)) {
		LOG_ERR("cur_etheraddr read returned %d (want >=6)", got);
		return (got < 0) ? got : -EIO;
	}
	LOG_INF("chip MAC = %02x:%02x:%02x:%02x:%02x:%02x",
		data->chip_mac[0], data->chip_mac[1], data->chip_mac[2],
		data->chip_mac[3], data->chip_mac[4], data->chip_mac[5]);

	data->probed = true;
	LOG_INF("Phase 4.4: bring-up + BCDC + RX thread complete in %lld ms; net_if TODO",
		(long long)(k_uptime_get() - t0));
	return 0;
}

#define BRCMFMAC_DEVICE_INIT(inst)                                          \
	static const struct brcmfmac_config brcmfmac_config_##inst = {       \
		.sdhc          = DEVICE_DT_GET(DT_INST_PARENT(inst)),         \
		.reg_on        = GPIO_DT_SPEC_INST_GET_OR(inst,               \
				 wifi_reg_on_gpios, {0}),                     \
		.firmware_name = DT_INST_PROP_OR(inst, firmware_name, ""),    \
	};                                                                    \
	static struct brcmfmac_data brcmfmac_data_##inst;                     \
	DEVICE_DT_INST_DEFINE(inst, brcmfmac_init, NULL,                      \
			      &brcmfmac_data_##inst,                          \
			      &brcmfmac_config_##inst,                        \
			      POST_KERNEL, CONFIG_WIFI_INIT_PRIORITY,         \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(BRCMFMAC_DEVICE_INIT)
