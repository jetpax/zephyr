/*
 * Copyright (c) 2025 Muhammad Waleed Badar
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_pinctrl

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/bcm2711-pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>

/* BCM2711 PINCTRL Base Address */
#define BCM2711_PINCTRL_BASE_ADDR DT_REG_ADDR(DT_DRV_INST(0))

/* Whether the parent SoC uses the legacy GPPUD/GPPUDCLK pull-control
 * sequence (BCM2710 / BCM2835 family) instead of the modern 0xE4
 * single-register PUP_PDN_CNTRL (BCM2711). Compile-time constant; the
 * compiler folds the unused branch.
 */
#define BCM2711_PINCTRL_LEGACY_PULL DT_INST_PROP(0, legacy_pull_control)

/* Function Select Registers (3 bits per pin, 10 pins per register) */
#define GPFSEL_OFFSET(pin) (((pin) / 10) * 4)
#define GPFSEL_SHIFT(pin)  (((pin) % 10) * 3)
#define GPFSEL_MASK        0x7

/* Pull-up/down Control Registers (2 bits per pin, 16 pins per register) --
 * BCM2711 modern path.
 */
#define GPIO_PUP_PDN_OFFSET(pin) (0xE4 + ((pin) / 16) * 4)
#define GPIO_PUP_PDN_SHIFT(pin)  (((pin) % 16) * 2)
#define GPIO_PUP_PDN_MASK        0x3

/* Legacy pull-control registers (BCM2710 / BCM2835 family). The 0xE4
 * register doesn't exist on this silicon -- writes there are silently
 * misrouted. The pull mode is selected via this sequence (matches
 * circle's gpiopull on p9arch.cpp lines 60-76, the canonical bare-metal
 * reference for the Pi Zero 2 W WLAN bring-up):
 *   1. write GPPUD = mode
 *   2. wait 5 us  ("1 us should be enough, but to be sure" -- circle)
 *   3. write GPPUDCLK[bank] = bit-mask of target pin(s)
 *   4. wait 5 us
 *   5. write GPPUD = 0           (clear the mode while strobe is still asserted)
 *   6. write GPPUDCLK[bank] = 0  (clear the strobe last)
 *
 * Linux's pinctrl-bcm2835 uses udelay(1) and skips step 5; it gets away
 * with that because Linux's udelay rounds up to a loop-calibrated lower
 * bound (typically >= 1 us, often longer), whereas Zephyr's system-timer
 * busy-wait can exit on the next 1 MHz tick boundary -- so a request for
 * 1 us can be satisfied by 0..1 us of actual wait. Empirically the 1 us
 * version of this routine failed to latch pull-up on DAT0..3 on this
 * silicon (verified 2026-05-12 by forcing the pull-up manually from MP
 * REPL with this exact sequence and watching 4-bit CMD53 succeed where
 * it previously fired DATA_CRC).
 *
 * Note that the legacy mode encoding (1=DOWN, 2=UP) is the SWAP of the
 * BCM2711 0xE4 encoding (1=UP, 2=DOWN); we translate explicitly.
 */
#define GPPUD_OFFSET             0x94
#define GPPUDCLK_OFFSET(pin)     (0x98 + ((pin) / 32) * 4)
#define LEGACY_PULL_OFF          0x0
#define LEGACY_PULL_DOWN         0x1
#define LEGACY_PULL_UP           0x2

/* BCM2835 System Timer at 0x3F003000, register CLO at +0x04: a 32-bit
 * free-running 1 MHz counter driven by the VPU since power-on. Used
 * here instead of k_busy_wait() because pinctrl_apply_state can run
 * at POST_KERNEL before Zephyr's generic-timer-based timing is
 * usable on this SoC, in which case k_busy_wait(1) returns early
 * and the GPPUD setup/hold windows don't actually elapse. Matches
 * circle's CTimer::SimpleusDelay (lib/timer.cpp:653-668).
 */
#define BCM2835_ST_CLO 0x3F003004UL

static inline void bcm2835_st_busy_wait_us(uint32_t us)
{
	uint32_t start = sys_read32(BCM2835_ST_CLO);

	while ((sys_read32(BCM2835_ST_CLO) - start) < us) {
		/* spin */
	}
}

static inline uint32_t bcm2711_pinctrl_read(uintptr_t base, uint32_t offset)
{
	return sys_read32(base + offset);
}

static inline void bcm2711_pinctrl_write(uintptr_t base, uint32_t offset, uint32_t val)
{
	sys_write32(val, base + offset);
}

static void bcm2711_pinctrl_set_func(uintptr_t base, uint8_t pin, uint8_t func)
{
	uint32_t offset = GPFSEL_OFFSET(pin);
	uint32_t shift = GPFSEL_SHIFT(pin);
	uint32_t reg_val;

	reg_val = bcm2711_pinctrl_read(base, offset);
	reg_val &= ~(GPFSEL_MASK << shift);
	reg_val |= (func & GPFSEL_MASK) << shift;
	bcm2711_pinctrl_write(base, offset, reg_val);
}

static void bcm2711_pinctrl_set_pull(uintptr_t base, uint8_t pin, uint8_t pull)
{
	uint32_t offset = GPIO_PUP_PDN_OFFSET(pin);
	uint32_t shift = GPIO_PUP_PDN_SHIFT(pin);
	uint32_t reg_val;

	reg_val = bcm2711_pinctrl_read(base, offset);
	reg_val &= ~(GPIO_PUP_PDN_MASK << shift);
	reg_val |= (pull & GPIO_PUP_PDN_MASK) << shift;
	bcm2711_pinctrl_write(base, offset, reg_val);
}

static void bcm2711_pinctrl_set_pull_legacy(uintptr_t base, uint8_t pin,
					    uint8_t pull)
{
	uint32_t pud_offset = GPPUDCLK_OFFSET(pin);
	uint32_t pin_bit = BIT(pin & 0x1F);
	uint32_t pud;

	switch (pull) {
	case BCM2711_PULL_UP:
		pud = LEGACY_PULL_UP;
		break;
	case BCM2711_PULL_DOWN:
		pud = LEGACY_PULL_DOWN;
		break;
	default:
		pud = LEGACY_PULL_OFF;
		break;
	}

	bcm2711_pinctrl_write(base, GPPUD_OFFSET, pud);
	bcm2835_st_busy_wait_us(5);
	bcm2711_pinctrl_write(base, pud_offset, pin_bit);
	bcm2835_st_busy_wait_us(5);
	bcm2711_pinctrl_write(base, GPPUD_OFFSET, 0);
	bcm2711_pinctrl_write(base, pud_offset, 0);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	uintptr_t base;

	if (!pins || pin_cnt == 0) {
		return -EINVAL;
	}

	device_map(&base, BCM2711_PINCTRL_BASE_ADDR, 0x100, K_MEM_CACHE_NONE);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		uint8_t pin = pins[i].pin;
		uint8_t pull = pins[i].pull;

		if (pin >= BCM2711_NUM_GPIO) {
			return -EINVAL;
		}

		bcm2711_pinctrl_set_func(base, pin, pins[i].func);

		/* BCM2711_PULL_KEEP: bias unspecified at the DT group
		 * level. Don't touch the pull state -- preserves whatever
		 * the firmware (or a prior pinctrl state) configured.
		 */
		if (pull != BCM2711_PULL_KEEP) {
			if (BCM2711_PINCTRL_LEGACY_PULL) {
				bcm2711_pinctrl_set_pull_legacy(base, pin, pull);
			} else {
				bcm2711_pinctrl_set_pull(base, pin, pull);
			}
		}
	}

	return 0;
}
