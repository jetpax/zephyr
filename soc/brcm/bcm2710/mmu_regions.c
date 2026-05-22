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
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
