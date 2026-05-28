/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2835 static MMU regions. arm_mmu.c maps the Zephyr image RAM
 * (text/data/rodata) automatically. The peripheral MMIO block is added here
 * as device memory: the ARMC interrupt controller (soc_irq.c) and the
 * BCM2835 system timer access their registers by physical address, before /
 * without a driver-time DEVICE_MMIO_MAP, so the whole 0x20000000 block must
 * be present in the static map.
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>

static const struct arm_mmu_region mmu_regions[] = {
	/*
	 * The exception vector table is copied to 0x0 by reset.S (ARM1176 has
	 * no VBAR, low vectors). It must be mapped executable, or the first
	 * interrupt after the MMU is enabled prefetch-aborts. A 4 KiB page,
	 * not a 1 MiB section -- 0x0 shares its section with the image at
	 * 0x8000, which arm_mmu.c maps via an L2 table.
	 */
	MMU_REGION_FLAT_ENTRY("vectors",
			      0x0U, 0x1000U,
			      MT_NORMAL | MATTR_SHARED |
			      MATTR_CACHE_OUTER_WB_nWA | MATTR_CACHE_INNER_WB_nWA |
			      MPERM_R | MPERM_X),

	MMU_REGION_FLAT_ENTRY("bcm2835_peripherals",
			      0x20000000U, 0x01000000U,
			      MT_DEVICE | MPERM_R | MPERM_W),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
