/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm64/arm_mmu.h>

/*
 * The interrupt controllers are accessed by the SoC IRQ glue before any
 * driver init level runs, so they need a static MMU mapping rather than
 * a driver-time DEVICE_MMIO_MAP.
 */
static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("BCM2836_L1_INTC",
			      DT_REG_ADDR(DT_INST(0, brcm_bcm2836_l1_intc)),
			      DT_REG_SIZE(DT_INST(0, brcm_bcm2836_l1_intc)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("BCM2835_ARMCTRL_IC",
			      DT_REG_ADDR(DT_INST(0, brcm_bcm2835_armctrl_ic)),
			      DT_REG_SIZE(DT_INST(0, brcm_bcm2835_armctrl_ic)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),

	/* V3D / ASB peripheral windows handled at driver runtime via
	 * device_map(K_MEM_CACHE_NONE). Static MMU entries with
	 * non-64KB-aligned bases (e.g. ASB at 0x3F00A000) interact badly
	 * with CONFIG_MMU_PAGE_SIZE=0x10000 and silently break early boot.
	 *
	 * The PM block below is the exception: reboot.c does identity
	 * sys_write32 on it (no device_map), and its 0x3f100000 base IS
	 * 64KB-aligned, so the static entry is safe.
	 */
	MMU_REGION_FLAT_ENTRY("BCM2835_PM",
			      0x3f100000UL,
			      0x1000,
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
