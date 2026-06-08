/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Smoke test for the bcm2835_v3d driver. Two steps in order:
 *
 *   1. Coherent-memory sanity: alloc a 4 KiB GPU-coherent buffer via
 *      the firmware mailbox, write a sentinel from ARM, read it back
 *      via the same cache-bypassed mapping, release it.
 *
 *   2. End-to-end QPU execution: launch the qpu_write shader on
 *      NUM_QPUS QPUs. Each QPU writes its index to its slice of the
 *      output buffer; ARM verifies every word matches its QPU id.
 *      Shader (bundled as qpu_write_code.{c,h}) is rpi_os's labs
 *      qpu_write reference, vc4asm-precompiled.
 *
 * Expected boot log on success:
 *
 *     <inf> bcm2835_v3d: V3D up: IDENT0=0x02443356 ...
 *     *** Booting Zephyr OS ... ***
 *     [v3d-smoke] starting
 *     [v3d-smoke] step 1: coherent-memory round-trip OK
 *     [v3d-smoke] step 2: qpu_write 12 QPUs x 64 ints in N us
 *     [v3d-smoke] step 2: verified 768/768 words match qpu_id
 *     [v3d-smoke] PASS
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/misc/bcm2835_v3d.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "qpu_write_code.h"

LOG_MODULE_REGISTER(v3d_smoke, LOG_LEVEL_INF);

#define V3D_NODE DT_NODELABEL(v3d)
BUILD_ASSERT(DT_NODE_HAS_STATUS(V3D_NODE, okay), "v3d node not enabled");

/* qpu_write shader contract: each QPU's inner loop runs `ITERATIONS/4`
 * times and DMAs SIMD_WIDTH*4 = 64 ints per iteration -- so each QPU
 * fills `ITERATIONS*SIMD_WIDTH` ints with its qpu_id. Stay small for
 * the first smoke; we can scale ITERATIONS up later for benchmarking.
 */
#define NUM_QPUS      12u
#define NUM_UNIFS     3u
#define ITERATIONS    4u
#define SIMD_WIDTH    16u
#define ELEMS_PER_QPU (ITERATIONS * SIMD_WIDTH)
#define N_ELEMS       (NUM_QPUS * ELEMS_PER_QPU)
#define V3D_MEM_FLAG_L1_NONALLOC 0x4u

/* Sanity-step 1: prove the coherent-memory plumbing (mailbox alloc +
 * lock + device_map) round-trips a sentinel cleanly. If this fails,
 * the kernel step has no chance.
 */
static int step1_coherent_roundtrip(const struct device *v3d)
{
	uintptr_t bus = 0;
	void     *cpu = NULL;
	uint32_t  handle = 0;
	int rc = bcm2835_v3d_alloc_coherent(v3d, 4096u, 16u,
	                                    V3D_MEM_FLAG_L1_NONALLOC,
	                                    &bus, &cpu, &handle);
	if (rc) {
		printk("[v3d-smoke] step 1 FAIL: alloc_coherent rc=%d\n", rc);
		return rc;
	}

	volatile uint32_t *p = (volatile uint32_t *)cpu;
	const uint32_t s = 0xDEADBEEFu;

	p[0] = s;
	p[1] = ~s;
	p[2] = 0x55555555u;
	p[3] = 0xAAAAAAAAu;

	if (p[0] != s || p[1] != ~s || p[2] != 0x55555555u || p[3] != 0xAAAAAAAAu) {
		printk("[v3d-smoke] step 1 FAIL: sentinel mismatch "
		       "%08x %08x %08x %08x\n", p[0], p[1], p[2], p[3]);
		(void)bcm2835_v3d_free_coherent(v3d, handle);
		return -EIO;
	}

	rc = bcm2835_v3d_free_coherent(v3d, handle);
	if (rc) {
		printk("[v3d-smoke] step 1 FAIL: free_coherent rc=%d\n", rc);
		return rc;
	}

	printk("[v3d-smoke] step 1: coherent-memory round-trip OK\n");
	return 0;
}

/* Sanity-step 2: launch the qpu_write shader and verify. */
static int step2_qpu_write(const struct device *v3d)
{
	/* Output buffer -- separate coherent allocation so we can scrub
	 * it with a sentinel before the kick (proves the writes are
	 * actually from V3D and not just leftover memory state).
	 */
	uintptr_t out_bus = 0;
	void     *out_cpu = NULL;
	uint32_t  out_handle = 0;
	int rc = bcm2835_v3d_alloc_coherent(v3d,
	                                    N_ELEMS * sizeof(uint32_t),
	                                    16u,
	                                    V3D_MEM_FLAG_L1_NONALLOC,
	                                    &out_bus, &out_cpu, &out_handle);
	if (rc) {
		printk("[v3d-smoke] step 2 FAIL: output alloc rc=%d\n", rc);
		return rc;
	}

	volatile uint32_t *out = (volatile uint32_t *)out_cpu;

	for (uint32_t i = 0; i < N_ELEMS; i++) {
		out[i] = 0xDEADBEEFu;
	}

	struct bcm2835_v3d_kernel k;

	rc = bcm2835_v3d_kernel_init(v3d, &k, NUM_QPUS, NUM_UNIFS,
	                             qpu_write_code, sizeof(qpu_write_code));
	if (rc) {
		printk("[v3d-smoke] step 2 FAIL: kernel_init rc=%d\n", rc);
		(void)bcm2835_v3d_free_coherent(v3d, out_handle);
		return rc;
	}

	for (uint32_t i = 0; i < NUM_QPUS; i++) {
		uint32_t slice_bus = (uint32_t)out_bus +
		                     i * ELEMS_PER_QPU * sizeof(uint32_t);
		(void)bcm2835_v3d_kernel_load_unif_u32(&k, i, slice_bus);
		(void)bcm2835_v3d_kernel_load_unif_u32(&k, i, ITERATIONS / 4u);
		(void)bcm2835_v3d_kernel_load_unif_u32(&k, i, i);
	}

	uint32_t t0 = k_cycle_get_32();

	rc = bcm2835_v3d_kernel_execute(v3d, &k);

	uint32_t cycles = k_cycle_get_32() - t0;
	uint32_t cps    = (uint32_t)sys_clock_hw_cycles_per_sec();
	uint32_t us     = (uint32_t)(((uint64_t)cycles * 1000000ULL) / cps);

	if (rc) {
		printk("[v3d-smoke] step 2 FAIL: kernel_execute rc=%d "
		       "(SRQCS poll timed out)\n", rc);
		(void)bcm2835_v3d_kernel_free(v3d, &k);
		(void)bcm2835_v3d_free_coherent(v3d, out_handle);
		return rc;
	}

	printk("[v3d-smoke] step 2: qpu_write %u QPUs x %u ints in %u us\n",
	       NUM_QPUS, ELEMS_PER_QPU, us);

	/* Verify: out[q*ELEMS_PER_QPU + j] should equal q for all q, j. */
	uint32_t matches = 0;
	uint32_t fails   = 0;

	for (uint32_t q = 0; q < NUM_QPUS; q++) {
		for (uint32_t j = 0; j < ELEMS_PER_QPU; j++) {
			uint32_t got = out[q * ELEMS_PER_QPU + j];

			if (got == q) {
				matches++;
			} else {
				if (fails < 8u) {
					printk("[v3d-smoke]   mismatch q=%u j=%u "
					       "got=0x%08x expected=%u\n",
					       q, j, got, q);
				}
				fails++;
			}
		}
	}

	(void)bcm2835_v3d_kernel_free(v3d, &k);
	(void)bcm2835_v3d_free_coherent(v3d, out_handle);

	if (fails) {
		printk("[v3d-smoke] step 2 FAIL: %u mismatches out of %u\n",
		       fails, N_ELEMS);
		return -EIO;
	}

	printk("[v3d-smoke] step 2: verified %u/%u words match qpu_id\n",
	       matches, N_ELEMS);
	return 0;
}

/* APPLICATION-priority SYS_INIT runs AFTER POST_KERNEL drivers and
 * BEFORE main(). Useful as an "I got this far" marker if main() ever
 * stops printing again.
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

	if (step1_coherent_roundtrip(v3d) != 0) {
		return -1;
	}
	if (step2_qpu_write(v3d) != 0) {
		return -1;
	}

	printk("[v3d-smoke] PASS\n");
	return 0;
}
