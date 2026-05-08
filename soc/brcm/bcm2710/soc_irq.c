/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2710 SoC IRQ glue. With CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER
 * selected, the arm64 arch core calls these instead of the GIC
 * driver (see arch/arm64/core/irq_init.c and irq_manage.c).
 *
 * Phase 1b.0: stubs only. No actual IRQ delivery yet -- this commit
 * only flips the wiring from GIC to the BCM intc nodes so the build
 * compiles against the new device tree. Phase 1b.1 fills in dispatch
 * to the BCM2836 ARM-local intc (root) and the BCM2835 ARMC cascade.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util_macro.h>

void z_soc_irq_init(void)
{
	/* TODO Phase 1b.1: probe + init BCM2836 ARM-local intc and the
	 * BCM2835 ARMC cascade child.
	 */
}

void z_soc_irq_enable(unsigned int irq)
{
	ARG_UNUSED(irq);
	/* TODO Phase 1b.1 */
}

void z_soc_irq_disable(unsigned int irq)
{
	ARG_UNUSED(irq);
	/* TODO Phase 1b.1 */
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	ARG_UNUSED(irq);
	return 0;
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio,
			    unsigned int flags)
{
	ARG_UNUSED(irq);
	ARG_UNUSED(prio);
	ARG_UNUSED(flags);
	/* The BCM2835/2836 intc pair has no per-IRQ priority register. */
}

unsigned int z_soc_irq_get_active(void)
{
	/* TODO Phase 1b.1: read pending source from local intc, with
	 * cascade fall-through to ARMC PEND1/PEND2 if it's the GPU IRQ.
	 */
	return 0;
}

void z_soc_irq_eoi(unsigned int irq)
{
	ARG_UNUSED(irq);
	/* BCM intc lines are level-driven by the source peripheral; no
	 * separate EOI register exists.
	 */
}
