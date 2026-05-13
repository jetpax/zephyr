# Copyright (c) 2026 jetpax
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_WIFI_BRCMFMAC
  brcmfmac_core.c
  brcmfmac_chip.c
  brcmfmac_sdio.c
  brcmfmac_bcdc.c
  firmware/brcmfmac_fw.c
  firmware/brcmfmac_nvram.c
)
