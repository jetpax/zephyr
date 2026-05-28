/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2835 SoC IRQ glue. With CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER
 * selected, the ARMv6 ISR wrapper calls these z_soc_irq_* hooks instead
 * of a GIC driver.
 *
 * Unlike the BCM2836/BCM2710 (Pi 2/3 / Zero 2 W), the original BCM2835
 * (Pi 1 / Zero / Zero W) has NO per-core ARM-local interrupt controller
 * (0x40000000). There is only the ARMC peripheral aggregator at
 * 0x2000b200, so the Zephyr IRQ space is the single ARMC range:
 *
 *     32..39   ARMC basic bank   (IRQ_BASIC_PENDING bits 0..7)
 *     64..95   ARMC bank 1 (PEND1, GPU 0..31)
 *     96..127  ARMC bank 2 (PEND2, GPU 32..63)
 *
 * The 0..31 range is reserved (would be the BCM2836 ARM-local sources)
 * so the BCM2835_IRQ_* binding numbers are shared with the BCM2710 port.
 *
 * The ARMC has three decode quirks (BCM2835 ARM Peripherals datasheet
 * ch. 7):
 *   1. Bank-1/2 IRQs with a bank-0 "shortcut" set ONLY their shortcut
 *      bit; the per-bank pending bit and bank-0 bits 8/9 do not fire.
 *   2. Bank-0 bits 8/9 cannot be masked.
 *   3. Shortcut enable/disable bits in bank 0 are ignored; the real
 *      bank-1/2 enable register must be used.
 *
 * Single-core only.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

/* ----- BCM2835 ARMC peripheral intc (base 0x2000b200) ----- */
#define ARMC_BASE               DT_REG_ADDR(DT_INST(0, brcm_bcm2835_armctrl_ic))
#define ARMC_IRQ_BASIC_PENDING  (ARMC_BASE + 0x00)
#define ARMC_IRQ_PENDING1       (ARMC_BASE + 0x04)
#define ARMC_IRQ_PENDING2       (ARMC_BASE + 0x08)
#define ARMC_FIQ_CONTROL        (ARMC_BASE + 0x0c)
#define ARMC_ENABLE_IRQS_1      (ARMC_BASE + 0x10)
#define ARMC_ENABLE_IRQS_2      (ARMC_BASE + 0x14)
#define ARMC_ENABLE_BASIC_IRQS  (ARMC_BASE + 0x18)
#define ARMC_DISABLE_IRQS_1     (ARMC_BASE + 0x1c)
#define ARMC_DISABLE_IRQS_2     (ARMC_BASE + 0x20)
#define ARMC_DISABLE_BASIC_IRQS (ARMC_BASE + 0x24)

/* IRQ_BASIC_PENDING register layout */
#define BANK0_BASIC_MASK        0xff          /* bits 0..7 = ARMC basic IRQs */
#define BANK1_PENDING_BIT       BIT(8)        /* "bank 1 has more pending" */
#define BANK2_PENDING_BIT       BIT(9)        /* "bank 2 has more pending" */
#define SHORTCUT1_MASK          0x00007c00    /* bits 10..14 -> bank 1 IRQs */
#define SHORTCUT2_MASK          0x001f8000    /* bits 15..20 -> bank 2 IRQs */
#define SHORTCUT_SHIFT          10
#define BANK0_VALID_MASK        (BANK0_BASIC_MASK | BANK1_PENDING_BIT | \
				 BANK2_PENDING_BIT | SHORTCUT1_MASK | \
				 SHORTCUT2_MASK)

/*
 * Bank-0 shortcut bits 10..20 each map to one bank-1/bank-2 IRQ.
 * Indices 0..4 hold the bank-1 IRQ numbers (bits 10..14); indices 5..10
 * the bank-2 IRQ numbers (bits 15..20).
 */
static const uint8_t shortcuts[] = {
	7, 9, 10, 18, 19,                /* SHORTCUT1 (bank 1) */
	21, 22, 23, 24, 25, 30           /* SHORTCUT2 (bank 2) */
};

/* ----- IRQ-space layout ----- */
#define ARMC_IRQ_BASE           32
#define IRQ_IS_ARMC(irq)        ((irq) >= ARMC_IRQ_BASE && \
				 (irq) < ARMC_IRQ_BASE + 96)
#define ARMC_IRQ_ENC(bank, n)   (ARMC_IRQ_BASE + ((bank) << 5) + (n))
#define ARMC_IRQ_BANK(irq)      (((irq) - ARMC_IRQ_BASE) >> 5)
#define ARMC_IRQ_BIT(irq)       (((irq) - ARMC_IRQ_BASE) & 0x1f)

static const uintptr_t armc_enable_reg[3] = {
	ARMC_ENABLE_BASIC_IRQS, ARMC_ENABLE_IRQS_1, ARMC_ENABLE_IRQS_2,
};
static const uintptr_t armc_disable_reg[3] = {
	ARMC_DISABLE_BASIC_IRQS, ARMC_DISABLE_IRQS_1, ARMC_DISABLE_IRQS_2,
};

void z_soc_irq_init(void)
{
	/* Mask everything on the ARMC (write-1-to-disable). */
	sys_write32(0xffffffff, ARMC_DISABLE_IRQS_1);
	sys_write32(0xffffffff, ARMC_DISABLE_IRQS_2);
	sys_write32(0xff,       ARMC_DISABLE_BASIC_IRQS);

	/* Cancel any FIQ left enabled by the boot firmware. */
	sys_write32(0, ARMC_FIQ_CONTROL);
}

void z_soc_irq_enable(unsigned int irq)
{
	if (IRQ_IS_ARMC(irq)) {
		sys_write32(BIT(ARMC_IRQ_BIT(irq)),
			    armc_enable_reg[ARMC_IRQ_BANK(irq)]);
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	if (IRQ_IS_ARMC(irq)) {
		sys_write32(BIT(ARMC_IRQ_BIT(irq)),
			    armc_disable_reg[ARMC_IRQ_BANK(irq)]);
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	if (IRQ_IS_ARMC(irq)) {
		return !!(sys_read32(armc_enable_reg[ARMC_IRQ_BANK(irq)])
			  & BIT(ARMC_IRQ_BIT(irq)));
	}
	return 0;
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio,
			    unsigned int flags)
{
	/* No per-IRQ priority register on the ARMC -- intentional no-op. */
	ARG_UNUSED(irq);
	ARG_UNUSED(prio);
	ARG_UNUSED(flags);
}

/*
 * Identify the firing IRQ. Returns CONFIG_NUM_IRQS for spurious so the
 * ISR wrapper's bounds check dispatches to nothing.
 */
static unsigned int decode_active(void)
{
	uint32_t basic = sys_read32(ARMC_IRQ_BASIC_PENDING) & BANK0_VALID_MASK;

	if (basic == 0) {
		return CONFIG_NUM_IRQS;
	}
	if (basic & BANK0_BASIC_MASK) {
		return ARMC_IRQ_ENC(0, __builtin_ctz(basic & BANK0_BASIC_MASK));
	}
	if (basic & SHORTCUT1_MASK) {
		return ARMC_IRQ_ENC(1, shortcuts[__builtin_ctz(basic >> SHORTCUT_SHIFT)]);
	}
	if (basic & SHORTCUT2_MASK) {
		return ARMC_IRQ_ENC(2, shortcuts[__builtin_ctz(basic >> SHORTCUT_SHIFT)]);
	}
	if (basic & BANK1_PENDING_BIT) {
		uint32_t p = sys_read32(ARMC_IRQ_PENDING1);

		if (p) {
			return ARMC_IRQ_ENC(1, __builtin_ctz(p));
		}
	}
	if (basic & BANK2_PENDING_BIT) {
		uint32_t p = sys_read32(ARMC_IRQ_PENDING2);

		if (p) {
			return ARMC_IRQ_ENC(2, __builtin_ctz(p));
		}
	}
	return CONFIG_NUM_IRQS;
}

unsigned int z_soc_irq_get_active(void)
{
	unsigned int irq = decode_active();

	/*
	 * The BCM2835 ARMC is level-triggered with no ack-on-read: a source
	 * stays asserted until the peripheral clears it. Mask it here so it
	 * cannot re-fire before its ISR runs; z_soc_irq_eoi re-enables it.
	 */
	if (irq < CONFIG_NUM_IRQS) {
		z_soc_irq_disable(irq);
	}
	return irq;
}

void z_soc_irq_eoi(unsigned int irq)
{
	if (irq < CONFIG_NUM_IRQS) {
		z_soc_irq_enable(irq);
	}
}
