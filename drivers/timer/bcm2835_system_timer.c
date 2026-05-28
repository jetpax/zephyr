/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom BCM2835 free-running system timer (1 MHz). A compare-channel
 * match raises an interrupt; the ISR reprograms the next compare for a
 * periodic (non-tickless) kernel tick. Channels 0 and 2 are reserved by
 * the VideoCore firmware, so channel 3 is used.
 */

#define DT_DRV_COMPAT brcm_bcm2835_system_timer

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys/sys_io.h>

#define TIMER_BASE  DT_INST_REG_ADDR(0)
#define TIMER_IRQ   DT_INST_IRQN(0)
#define TIMER_CH    3

#define ST_CS       (TIMER_BASE + 0x00)              /* control/status   */
#define ST_CLO      (TIMER_BASE + 0x04)              /* counter low      */
#define ST_COMPARE  (TIMER_BASE + 0x0c + TIMER_CH * 4)
#define ST_MATCH    BIT(TIMER_CH)

#define CYC_PER_TICK ((uint32_t)k_ticks_to_cyc_floor32(1))

static void bcm2835_timer_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/*
	 * Acknowledge the match, then schedule the next tick relative to the
	 * previous compare value so the period does not drift.
	 */
	sys_write32(ST_MATCH, ST_CS);
	sys_write32(sys_read32(ST_COMPARE) + CYC_PER_TICK, ST_COMPARE);

	sys_clock_announce(1);
}

/* Tickless kernel is not supported by this driver. */
uint32_t sys_clock_elapsed(void)
{
	return 0;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return sys_read32(ST_CLO);
}

static int bcm2835_timer_init(void)
{
	IRQ_CONNECT(TIMER_IRQ, 0, bcm2835_timer_isr, NULL, 0);

	sys_write32(sys_read32(ST_CLO) + CYC_PER_TICK, ST_COMPARE);
	sys_write32(ST_MATCH, ST_CS);   /* clear any stale match */
	irq_enable(TIMER_IRQ);

	return 0;
}

SYS_INIT(bcm2835_timer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
