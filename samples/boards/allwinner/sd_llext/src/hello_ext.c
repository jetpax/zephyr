/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/llext/symbol.h>
#include <zephyr/sys/printk.h>

void hello_world(void)
{
	printk("hello from an llext loaded off the SD card\n");
}
EXPORT_SYMBOL(hello_world);
