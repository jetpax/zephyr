/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCM2710 (Pi 3 / Pi Zero 2 W) pinctrl SoC header.
 *
 * The BCM283x GPIO/pinctrl block is register-compatible across BCM2710,
 * BCM2711 and BCM2837, so we deliberately reuse the existing
 * <zephyr/dt-bindings/pinctrl/bcm2711-pinctrl.h> dt-bindings and the
 * brcm,bcm2711-pinctrl driver. The names are slightly misleading
 * (they're really BCM283x-family) but renaming the upstream dt-binding
 * header is out of scope for this port.
 */

#ifndef ZEPHYR_SOC_ARM64_BCM2710_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM64_BCM2710_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/dt-bindings/pinctrl/bcm2711-pinctrl.h>

/**
 * @brief Type to hold a pin's pinctrl configuration.
 */
typedef struct bcm2710_pinctrl_soc_pin {
	/** GPIO pin number (0-57) */
	uint8_t pin;
	/** Function select (0-7: IN, OUT, ALT0-ALT5) */
	uint8_t func;
	/** Pull resistor configuration (0=none, 1=up, 2=down) */
	uint8_t pull;
} pinctrl_soc_pin_t;

/**
 * @brief Get pull configuration from DT node properties
 *
 * @param node_id Node identifier (the group node).
 */
#define BCM2710_GET_PULL(node_id)                                                                  \
	COND_CASE_1(DT_PROP(node_id, bias_disable), (BCM2711_PULL_NONE),                           \
		    DT_PROP(node_id, bias_pull_up), (BCM2711_PULL_UP),                         \
		    DT_PROP(node_id, bias_pull_down), (BCM2711_PULL_DOWN), (BCM2711_PULL_NONE))

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier (the group node).
 * @param prop Property name (should be "pinmux").
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{.pin = BCM2711_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),                               \
	 .func = BCM2711_GET_FUNC(DT_PROP_BY_IDX(node_id, prop, idx)),                             \
	 .pull = BCM2710_GET_PULL(node_id)},

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier (the pinctrl state node, e.g., uart0_default).
 * @param prop Property name describing state pins (should be "pinctrl-0").
 *
 * Walks one phandle's children. Zephyr's pinctrl convention is "one
 * phandle per state": pinctrl-0 = <&uart0_default> with all pins for
 * that state collected inside the &uart0_default node's children. This
 * is the same shape as ESP32's pinctrl_soc.h and the upstream rpi_pico
 * one. Linux's "<&group_a &group_b>" multi-phandle pattern doesn't
 * translate; merge those into one group when porting from Linux DTS.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,           \
				Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_ARM64_BCM2710_PINCTRL_SOC_H_ */
