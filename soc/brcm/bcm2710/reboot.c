/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2710 / BCM2837 SoC reboot via the PM block's watchdog full-reset
 * path. Mirrors the sequence used by the Linux RPi downstream kernel in
 * drivers/watchdog/bcm2835_wdt.c (__bcm2835_restart):
 *
 *   1. Arm PM_WDOG with a short timeout (10 ticks, ~150 us).
 *   2. Read PM_RSTC, clear the WRCFG field, set WRCFG_FULL_RESET, write
 *      back. Every write to the PM block must include PM_PASSWORD (the
 *      0x5a000000 magic the Broadcom PM controller requires).
 *
 * The watchdog expiry asserts SoC reset; the VC ROM bootloader picks up
 * and re-loads zephyr.bin from /boot/PIZZA. Wake-time is the regular
 * boot path (~1.0 s to "sketch started" on a Pi Zero 2 W).
 *
 * This provides a strong override of Zephyr's __weak sys_arch_reboot()
 * so sys_reboot(SYS_REBOOT_COLD) becomes functional on this SoC.
 *
 * Requires an MMU mapping of the PM block (see mmu_regions.c).
 */

#include <stdint.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/toolchain.h>

#define BCM2710_PM_BASE         0x3f100000UL
#define BCM2710_PM_RSTC         (BCM2710_PM_BASE + 0x1cUL)
#define BCM2710_PM_WDOG         (BCM2710_PM_BASE + 0x24UL)

#define BCM2710_PM_PASSWORD     0x5a000000U
#define BCM2710_PM_RSTC_WRCFG_CLR        0xffffffcfU
#define BCM2710_PM_RSTC_WRCFG_FULL_RESET 0x00000020U

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	sys_write32(BCM2710_PM_PASSWORD | 10U, BCM2710_PM_WDOG);

	uint32_t cur = sys_read32(BCM2710_PM_RSTC);

	sys_write32(BCM2710_PM_PASSWORD |
			(cur & BCM2710_PM_RSTC_WRCFG_CLR) |
			BCM2710_PM_RSTC_WRCFG_FULL_RESET,
		    BCM2710_PM_RSTC);

	for (;;) {
		__asm__ volatile("wfe");
	}
}
