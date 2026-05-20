/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2710 DWC2 register probe.
 *
 * One-shot diagnostic for Phase 2.0 of the USB CDC bring-up: read the
 * Synopsys-identity and hardware-configuration registers from the
 * on-SoC DWC2 OTG controller so we can backfill the real GHWCFG1/2/4
 * values into bcm2710.dtsi (currently set to ESP32-S3 placeholders).
 *
 * Also dumps GUSBCFG and GINTSTS so we can confirm:
 *   - VideoCore left the controller in host mode (FDMod=0, FHMod=1
 *     in GUSBCFG; CurMod=1 in GINTSTS bit 0);
 *   - the MMIO is reachable at all (GSNPSID matches a known DWC2
 *     signature; not 0xffffffff).
 *
 * Phys 0x3F980000 on BCM2710 / BCM2837 (Pi 3 / Pi Zero 2 W). Same IP
 * block on BCM2835/2836 at 0x20980000.
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/device_mmio.h>

#define DWC2_PA       0x3F980000UL
#define DWC2_LEN      0x100UL

/* Synopsys USB 2.0 OTG global registers (OTG_DataBook 2.x) */
#define GUSBCFG       0x00C
#define GINTSTS       0x014
#define GSNPSID       0x040
#define GHWCFG1       0x044
#define GHWCFG2       0x048
#define GHWCFG3       0x04C
#define GHWCFG4       0x050

int main(void)
{
	mm_reg_t base;

	device_map(&base, DWC2_PA, DWC2_LEN, K_MEM_CACHE_NONE);

	uint32_t snpsid  = sys_read32(base + GSNPSID);
	uint32_t gusbcfg = sys_read32(base + GUSBCFG);
	uint32_t gintsts = sys_read32(base + GINTSTS);
	uint32_t ghwcfg1 = sys_read32(base + GHWCFG1);
	uint32_t ghwcfg2 = sys_read32(base + GHWCFG2);
	uint32_t ghwcfg3 = sys_read32(base + GHWCFG3);
	uint32_t ghwcfg4 = sys_read32(base + GHWCFG4);

	printk("\n");
	printk("=== BCM2710 DWC2 probe ===\n");
	printk("base PA   = 0x%08lx\n", (unsigned long)DWC2_PA);
	printk("GSNPSID   = 0x%08x  (Synopsys DWC2 OTG id; expect 0x4f5x)\n", snpsid);
	printk("GUSBCFG   = 0x%08x  (bit 30 FDMod, bit 29 FHMod)\n", gusbcfg);
	printk("GINTSTS   = 0x%08x  (bit 0 CurMod: 1=host, 0=device)\n", gintsts);
	printk("GHWCFG1   = 0x%08x\n", ghwcfg1);
	printk("GHWCFG2   = 0x%08x\n", ghwcfg2);
	printk("GHWCFG3   = 0x%08x\n", ghwcfg3);
	printk("GHWCFG4   = 0x%08x\n", ghwcfg4);
	printk("=== end of DWC2 probe ===\n");

	return 0;
}
