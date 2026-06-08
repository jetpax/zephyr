/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Smoke test for the bcm2835_v3d driver. Verifies, in increasing
 * order of "things that might be wrong":
 *
 *   1. Driver probes -- V3D is powered + out of reset, IDENT0 reads
 *      back 0x02443356 ("V3D\x02" with bytes V,3,D,02 in LE order).
 *   2. Firmware mailbox memory tags work -- we can allocate a small
 *      GPU-coherent buffer, write a sentinel from ARM, read it back
 *      via the same mapping, and release it.
 *
 * No QPU kernel execution yet -- that's the next bring-up commit
 * once kernel_init / kernel_execute are implemented in the driver.
 *
 * Expected boot log (success) -- driver init runs first, then main:
 *
 *     <inf> bcm2835_v3d: V3D up: IDENT0=0x02443356 IDENT1=0xc1102431 IDENT2=0x00000121
 *     *** Booting Zephyr OS ... ***
 *     [v3d-smoke] starting
 *     [v3d-smoke] device ready
 *     [v3d-smoke] alloc_coherent(4096, 16, 0x4) ok: bus=0xXXXXXXXX cpu=0xXXXXXXXX
 *     [v3d-smoke] sentinel write/read round-trip OK (0xdeadbeef)
 *     [v3d-smoke] free_coherent ok
 *     [v3d-smoke] smoke test PASS
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/misc/bcm2835_v3d.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(v3d_smoke, LOG_LEVEL_INF);

#define V3D_NODE DT_NODELABEL(v3d)
BUILD_ASSERT(DT_NODE_HAS_STATUS(V3D_NODE, okay), "v3d node not enabled");

/* APPLICATION-priority SYS_INIT runs AFTER POST_KERNEL drivers and
 * BEFORE main(). If we see this line in the log but no "[v3d-smoke]
 * starting" later, the main thread is hung or not invoked.
 */
static int v3d_smoke_app_marker(void)
{
	LOG_INF("APPLICATION init reached -- main() should fire next");
	return 0;
}
SYS_INIT(v3d_smoke_app_marker, APPLICATION, 99);

int main(void)
{
	LOG_INF("main() entered");
	printk("[v3d-smoke] starting\n");

	const struct device *v3d = DEVICE_DT_GET(V3D_NODE);

	if (!device_is_ready(v3d)) {
		printk("[v3d-smoke] FAIL: device_is_ready returned false\n");
		return -ENODEV;
	}
	printk("[v3d-smoke] device ready\n");

	/* Step 2: round-trip a sentinel through a GPU-coherent buffer.
	 * If the mailbox ALLOCATE + LOCK chain works and the ARM-side
	 * device_map of the returned bus address yields a usable
	 * pointer, ARM-side writes survive the cache-bypassed mapping
	 * and read back cleanly.
	 */
	uintptr_t bus  = 0;
	void     *cpu  = NULL;
	uint32_t  handle = 0;
	int rc = bcm2835_v3d_alloc_coherent(v3d, 4096, 16, 0x4,
	                                    &bus, &cpu, &handle);
	if (rc) {
		printk("[v3d-smoke] FAIL: alloc_coherent rc=%d\n", rc);
		return rc;
	}
	printk("[v3d-smoke] alloc_coherent(4096, 16, 0x4) ok: bus=0x%lx cpu=%p handle=0x%x\n",
	       (unsigned long)bus, cpu, handle);

	volatile uint32_t *p = (volatile uint32_t *)cpu;
	const uint32_t sentinel = 0xDEADBEEFu;

	p[0] = sentinel;
	p[1] = ~sentinel;
	p[2] = 0x55555555u;
	p[3] = 0xAAAAAAAAu;

	if (p[0] != sentinel || p[1] != ~sentinel ||
	    p[2] != 0x55555555u || p[3] != 0xAAAAAAAAu) {
		printk("[v3d-smoke] FAIL: sentinel readback mismatch: "
		       "%08x %08x %08x %08x\n",
		       p[0], p[1], p[2], p[3]);
		(void)bcm2835_v3d_free_coherent(v3d, handle);
		return -EIO;
	}
	printk("[v3d-smoke] sentinel write/read round-trip OK (0x%08x)\n", sentinel);

	rc = bcm2835_v3d_free_coherent(v3d, handle);
	if (rc) {
		printk("[v3d-smoke] FAIL: free_coherent rc=%d\n", rc);
		return rc;
	}
	printk("[v3d-smoke] free_coherent ok\n");

	printk("[v3d-smoke] smoke test PASS\n");
	return 0;
}
