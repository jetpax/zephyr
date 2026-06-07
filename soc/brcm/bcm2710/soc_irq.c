/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2710 SoC IRQ glue. With CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER
 * selected, the arm64 ISR wrapper (arch/arm64/core/isr_wrapper.S)
 * calls these functions instead of the GIC driver:
 *   z_soc_irq_get_active() identifies the firing IRQ
 *   _sw_isr_table[irq]     dispatches to the registered ISR
 *   z_soc_irq_eoi(irq)     signals end-of-interrupt
 * and the per-IRQ enable/disable/priority hooks back the
 * irq_enable() / IRQ_CONNECT() abstractions from drivers.
 *
 * IRQ-number layout (single Zephyr IRQ space backed by two
 * physical interrupt controllers):
 *
 *     0..9     BCM2836 ARM-local intc per-core sources
 *              (0..3 = timers, 4..7 = mailboxes, 8 = GPU
 *               cascade [hidden, see below], 9 = PMU)
 *     32..39   BCM2835 ARMC basic bank   (IRQ_BASIC_PENDING bits 0..7)
 *     64..95   BCM2835 ARMC bank 1 (PEND1, GPU 0..31)
 *     96..127  BCM2835 ARMC bank 2 (PEND2, GPU 32..63)
 *
 * IRQ 8 is the cascade entry: when the L1 IRQ_SOURCE register shows
 * only the GPU bit, z_soc_irq_get_active walks down through the
 * BCM2835 ARMC pending registers and returns the actual peripheral
 * IRQ in the 32..127 range. ISRs are therefore never registered at
 * index 8 -- the cascade is invisible to the rest of Zephyr.
 *
 * The BCM2835 ARMC peripheral controller has three decode quirks
 * (BCM2835 ARM Peripherals datasheet ch. 7):
 *
 *   1. Bank-1/2 IRQs that have a "shortcut" in bank 0 set ONLY
 *      their shortcut bit -- their per-bank pending bit is suppressed
 *      and bank 0 bits 8/9 ("bank 1/2 has more pending") do NOT fire.
 *   2. Bank 0 bits 8/9 cannot be masked.
 *   3. The shortcut bits in bank 0 enable/disable registers are
 *      ignored; the actual bank 1/2 enable register must be used.
 *
 * SMP: the per-core IRQ register reads use this_core() so each CPU
 * sees its own pending mask, and the scheduler IPI raises mailbox 0
 * of the addressed core (see send_ipi() / soc_sched_ipi()). Peripheral
 * and GPU interrupts stay routed to core 0 in this prototype --
 * extending to dynamic GPU-IRQ routing is a later step.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

/* ----- BCM2836 ARM-local intc (base 0x40000000) ----- */
#define L1_BASE                 DT_REG_ADDR(DT_INST(0, brcm_bcm2836_l1_intc))
#define L1_GPU_INT_ROUTING      (L1_BASE + 0x0c)
#define L1_PMU_ROUTING_SET      (L1_BASE + 0x10)
#define L1_PMU_ROUTING_CLR      (L1_BASE + 0x14)
#define L1_TIMER_INT_CTRL(c)    (L1_BASE + 0x40 + (c) * 4)
#define L1_MBOX_INT_CTRL(c)     (L1_BASE + 0x50 + (c) * 4)
#define L1_IRQ_SOURCE(c)        (L1_BASE + 0x60 + (c) * 4)

/* L1 IRQ_SOURCE register bit positions (= Zephyr IRQ # for L1 sources) */
#define L1_SRC_TIMER_MASK       0x000f  /* bits 0..3 */
#define L1_SRC_MBOX_MASK        0x00f0  /* bits 4..7 */
#define L1_SRC_GPU_BIT          8
#define L1_SRC_PMU_BIT          9

/* ----- BCM2835 ARMC peripheral intc (base 0x3f00b200) ----- */
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
 * Bank-0 shortcut bits 10..20 each map directly to one bank-1/bank-2
 * IRQ. Indices 0..4 hold the bank-1 IRQ numbers reachable via bits
 * 10..14; indices 5..10 the bank-2 IRQ numbers reachable via bits
 * 15..20.
 */
static const uint8_t shortcuts[] = {
	7, 9, 10, 18, 19,                /* SHORTCUT1 (bank 1) */
	21, 22, 23, 24, 25, 30           /* SHORTCUT2 (bank 2) */
};

/* ----- IRQ-space layout ----- */
#define ARMC_IRQ_BASE           32
#define IRQ_IS_L1(irq)          ((irq) < ARMC_IRQ_BASE)
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

/* Current physical core id from MPIDR (flat single-cluster A53). */
static inline unsigned int this_core(void)
{
	uint64_t mpidr;

	__asm__ volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
	return (unsigned int)(mpidr & 0xffU);
}

/* ----- SMP scheduler IPI over the BCM2836 per-core mailboxes ----- */
#define L1_MBOX_SET(c)          (L1_BASE + 0x80 + (c) * 0x10)  /* mbox0 write-set */
#define L1_MBOX_RDCLR(c)        (L1_BASE + 0xc0 + (c) * 0x10)  /* mbox0 read/clear */
#define IPI_IRQ                 4  /* BCM2836_LOCAL_IRQ_MAILBOX0 -> scheduler IPI */

extern void sched_ipi_handler(const void *unused);

static void mbox0_ipi_isr(const void *arg)
{
	ARG_UNUSED(arg);
	/*
	 * Read the set mailbox bits, then write the same value back to
	 * ack ONLY those bits. A concurrent raise from another core that
	 * lands between this read and the write is preserved: its bit
	 * stays set, the mailbox source stays asserted at the L1 intc,
	 * and the level-triggered IRQ refires once z_soc_irq_eoi()
	 * re-enables MBOX_INT_CTRL. Blanket-clearing 0xffffffff drops
	 * any such mid-isr raise on the floor.
	 *
	 * Same pattern as Linux's drivers/irqchip/irq-bcm2836.c:
	 *   bcm2836_arm_irqchip_handle_ipi() reads MAILBOX0_CLR + ffs(),
	 *   bcm2836_arm_irqchip_ipi_ack()    writes BIT(d->hwirq) back.
	 */
	uint32_t mbox = sys_read32(L1_MBOX_RDCLR(this_core()));

	if (mbox != 0U) {
		sys_write32(mbox, L1_MBOX_RDCLR(this_core()));
	}
	sched_ipi_handler(NULL);
}

/* Raise the scheduler IPI on the target core by setting its mailbox 0. */
void soc_sched_ipi(uint64_t target_mpidr)
{
	unsigned int core = (unsigned int)(target_mpidr & 0xffU);

	sys_write32(BIT(0), L1_MBOX_SET(core));
}

/*
 * Strong override of the __weak arch_spin_relax in kernel/idle.c.
 * The default asserts !arch_cpu_irqs_are_enabled(), which is the right
 * invariant for the in-tree callers (k_spin_lock, z_smp_global_lock,
 * thread_halt_spin, z_sched_switch_spin all relax with IRQs masked).
 * The arm64 GIC build supplies a non-asserting variant under
 * CONFIG_FPU_SHARING (arch/arm64/core/smp.c) to drain the FPU IPI
 * during a contended spin; this prototype runs FPU_SHARING off, so
 * the __weak default applies and its assertion fires on any path
 * that relaxes outside an irq lock. Provide a plain relax that
 * mirrors the FPU_SHARING shape but without the GIC-specific
 * bookkeeping (which doesn't exist here).
 */
void arch_spin_relax(void)
{
	arch_nop();
}

/* Per-core: enable this core's mailbox-0 IPI (handler wired in z_soc_irq_init). */
void soc_per_core_init_hook(void)
{
	irq_enable(IPI_IRQ);
}

void z_soc_irq_init(void)
{
	/*
	 * Mask all locally-routed L1 sources for core 0. The GPU IRQ
	 * routing register defaults to core 0 after reset; force it so
	 * the value is deterministic across firmware variants.
	 */
	sys_write32(0, L1_TIMER_INT_CTRL(this_core()));
	sys_write32(0, L1_MBOX_INT_CTRL(this_core()));
	sys_write32(0, L1_GPU_INT_ROUTING);

	/*
	 * Mask everything on the BCM2835 ARMC. Writing 1 disables; the
	 * register is write-1-to-disable, not r/w mask.
	 */
	sys_write32(0xffffffff, ARMC_DISABLE_IRQS_1);
	sys_write32(0xffffffff, ARMC_DISABLE_IRQS_2);
	sys_write32(0xff,       ARMC_DISABLE_BASIC_IRQS);

	/* Cancel any FIQ left enabled by the boot firmware. */
	sys_write32(0, ARMC_FIQ_CONTROL);

	/* Register the scheduler-IPI handler (each core's mailbox 0). */
	IRQ_CONNECT(IPI_IRQ, 0, mbox0_ipi_isr, NULL, 0);
}

void z_soc_irq_enable(unsigned int irq)
{
	if (IRQ_IS_L1(irq)) {
		if (irq <= 3) {
			sys_write32(sys_read32(L1_TIMER_INT_CTRL(this_core())) | BIT(irq),
				    L1_TIMER_INT_CTRL(this_core()));
		} else if (irq <= 7) {
			sys_write32(sys_read32(L1_MBOX_INT_CTRL(this_core())) | BIT(irq - 4),
				    L1_MBOX_INT_CTRL(this_core()));
		} else if (irq == L1_SRC_PMU_BIT) {
			sys_write32(BIT(this_core()), L1_PMU_ROUTING_SET);
		}
		/*
		 * IRQ 8 (GPU cascade) is implicit -- the L1 routing
		 * register already steers it to this_core(); no per-IRQ
		 * enable bit exists.
		 */
		return;
	}
	if (IRQ_IS_ARMC(irq)) {
		sys_write32(BIT(ARMC_IRQ_BIT(irq)),
			    armc_enable_reg[ARMC_IRQ_BANK(irq)]);
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	if (IRQ_IS_L1(irq)) {
		if (irq <= 3) {
			sys_write32(sys_read32(L1_TIMER_INT_CTRL(this_core())) & ~BIT(irq),
				    L1_TIMER_INT_CTRL(this_core()));
		} else if (irq <= 7) {
			sys_write32(sys_read32(L1_MBOX_INT_CTRL(this_core())) & ~BIT(irq - 4),
				    L1_MBOX_INT_CTRL(this_core()));
		} else if (irq == L1_SRC_PMU_BIT) {
			sys_write32(BIT(this_core()), L1_PMU_ROUTING_CLR);
		}
		return;
	}
	if (IRQ_IS_ARMC(irq)) {
		sys_write32(BIT(ARMC_IRQ_BIT(irq)),
			    armc_disable_reg[ARMC_IRQ_BANK(irq)]);
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	if (IRQ_IS_L1(irq)) {
		if (irq <= 3) {
			return !!(sys_read32(L1_TIMER_INT_CTRL(this_core())) & BIT(irq));
		}
		if (irq <= 7) {
			return !!(sys_read32(L1_MBOX_INT_CTRL(this_core())) & BIT(irq - 4));
		}
		if (irq == L1_SRC_GPU_BIT) {
			/* No mask for the cascade -- always live. */
			return 1;
		}
		/* PMU has no readback path; report disabled conservatively. */
		return 0;
	}
	if (IRQ_IS_ARMC(irq)) {
		return !!(sys_read32(armc_enable_reg[ARMC_IRQ_BANK(irq)])
			  & BIT(ARMC_IRQ_BIT(irq)));
	}
	return 0;
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio,
			    unsigned int flags)
{
	/* No per-IRQ priority register on either intc -- intentional no-op. */
	ARG_UNUSED(irq);
	ARG_UNUSED(prio);
	ARG_UNUSED(flags);
}

/*
 * Identify the firing IRQ. Returns CONFIG_NUM_IRQS for spurious so the
 * arm64 isr_wrapper's bounds check (`irq > NUM_IRQS - 1 -> spurious`)
 * dispatches to nothing.
 */
static unsigned int decode_active(void)
{
	uint32_t l1 = sys_read32(L1_IRQ_SOURCE(this_core()));

	if (l1 == 0) {
		return CONFIG_NUM_IRQS;
	}
	if (l1 & L1_SRC_TIMER_MASK) {
		return __builtin_ctz(l1 & L1_SRC_TIMER_MASK);    /* 0..3 */
	}
	if (l1 & L1_SRC_MBOX_MASK) {
		return __builtin_ctz(l1 & L1_SRC_MBOX_MASK);     /* 4..7 */
	}
	if (l1 & BIT(L1_SRC_PMU_BIT)) {
		return L1_SRC_PMU_BIT;                            /* 9 */
	}

	/*
	 * Only the GPU cascade bit can be set. Walk the ARMC pending
	 * registers; the bank-0 shortcut bits (Quirk 1) are checked
	 * before the bank-1/2 fall-through.
	 */
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
	 * The arm64 isr_wrapper unmasks IRQs globally (`daifclr`) around
	 * the ISR call to support nested handlers. With a GIC that's
	 * fine -- ack-on-read prevents the same source from re-entering
	 * the handler. The BCM2835/2836 intc pair has no such ack: a
	 * level-triggered source stays asserted until the peripheral's
	 * own state machine clears it, so unmasking globally would
	 * re-fire the same IRQ before its ISR has had a chance to run.
	 *
	 * Workaround: mask the source here at the SoC intc, let the ISR
	 * run, then re-enable in z_soc_irq_eoi. This brackets the ISR
	 * with a per-source mask the way GIC's running-priority would.
	 */
	if (irq < CONFIG_NUM_IRQS) {
		z_soc_irq_disable(irq);
	}
	return irq;
}

void z_soc_irq_eoi(unsigned int irq)
{
	/*
	 * Re-enable the source masked by z_soc_irq_get_active. By now
	 * either the ISR cleared the underlying peripheral and the
	 * source is no longer asserted (normal case) or we're returning
	 * from a spurious dispatch and there's nothing to re-fire.
	 */
	if (irq < CONFIG_NUM_IRQS) {
		z_soc_irq_enable(irq);
	}
}
