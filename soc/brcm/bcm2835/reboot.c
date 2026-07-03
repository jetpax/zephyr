/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2835 SoC reboot via the PM block's watchdog full-reset path. Mirrors
 * the BCM2710 implementation (soc/brcm/bcm2710/reboot.c) but the PM block
 * lives at 0x20100000 on BCM2835 instead of 0x3f100000 on BCM2710.
 *
 * The peripheral block (0x20000000 + 0x01000000) is already mapped device
 * memory by mmu_regions.c, so the PM register window needs no separate
 * MMU entry.
 *
 *   1. Arm PM_WDOG with a short timeout (10 ticks, ~150 us).
 *   2. Read PM_RSTC, clear the WRCFG field, set WRCFG_FULL_RESET, write
 *      back. Every write to the PM block must include PM_PASSWORD (the
 *      0x5a000000 magic the Broadcom PM controller requires).
 *
 * Watchdog expiry asserts SoC reset; the VC ROM bootloader picks up and
 * re-loads kernel.img from the SD card. This provides a strong override
 * of Zephyr's __weak sys_arch_reboot() so sys_reboot(SYS_REBOOT_COLD)
 * becomes functional on this SoC.
 */

#include <stdint.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/toolchain.h>

#define BCM2835_PM_BASE         0x20100000UL
#define BCM2835_PM_RSTC         (BCM2835_PM_BASE + 0x1cUL)
#define BCM2835_PM_WDOG         (BCM2835_PM_BASE + 0x24UL)

#define BCM2835_PM_PASSWORD     0x5a000000U
#define BCM2835_PM_RSTC_WRCFG_CLR        0xffffffcfU
#define BCM2835_PM_RSTC_WRCFG_FULL_RESET 0x00000020U

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	sys_write32(BCM2835_PM_PASSWORD | 10U, BCM2835_PM_WDOG);

	uint32_t cur = sys_read32(BCM2835_PM_RSTC);

	sys_write32(BCM2835_PM_PASSWORD |
			(cur & BCM2835_PM_RSTC_WRCFG_CLR) |
			BCM2835_PM_RSTC_WRCFG_FULL_RESET,
		    BCM2835_PM_RSTC);

	for (;;) {
		__asm__ volatile("wfe");
	}
}
