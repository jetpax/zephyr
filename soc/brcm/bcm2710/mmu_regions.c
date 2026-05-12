/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm64/arm_mmu.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("BCM2836_L1_INTC",
			      DT_REG_ADDR(DT_INST(0, brcm_bcm2836_l1_intc)),
			      DT_REG_SIZE(DT_INST(0, brcm_bcm2836_l1_intc)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("BCM2835_ARMCTRL_IC",
			      DT_REG_ADDR(DT_INST(0, brcm_bcm2835_armctrl_ic)),
			      DT_REG_SIZE(DT_INST(0, brcm_bcm2835_armctrl_ic)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),

	/* Mailbox scratch buffer at a fixed physical address well above
	 * MP heap (heap is 64 MiB starting ~0x300000; this is at 240 MiB).
	 * Mapped Device-nGnRE so ARM writes go straight to DRAM without
	 * L1 caching -- avoids the otherwise-needed explicit cache flush
	 * before the VPU reads the buffer via the bus alias 0xC0000000+x.
	 * Used by MP REPL property-channel mailbox calls for SDHCI
	 * power-state probing.
	 */
	MMU_REGION_FLAT_ENTRY("MBOX_SCRATCH",
			      0x0F000000UL,
			      0x1000UL,
			      MT_DEVICE_nGnRE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),

	/* Arasan SDHCI registers at 0x3F300000.
	 *
	 * Explicitly mapped as Device-nGnRE so the identity-mapped page
	 * table entry has the right attribute from boot, before the SDHC
	 * driver's DEVICE_MMIO_MAP runs at POST_KERNEL. Pi-downstream Linux
	 * maps BCM283x peripheral space the same way.
	 */
	MMU_REGION_FLAT_ENTRY("ARASAN_SDHCI",
			      0x3F300000UL,
			      0x1000UL,
			      MT_DEVICE_nGnRE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
