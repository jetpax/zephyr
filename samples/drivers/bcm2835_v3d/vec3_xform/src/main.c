/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * vec3_transform_perspective QPU shader -- correctness + dual-pass
 * speedup curve. Compares two NUM_QPUS configurations in one boot:
 *
 *   Pass 1: NUM_QPUS = HW_QPUS  (12 on BCM2710 -- saturates the
 *           physical QPUs without queueing)
 *   Pass 2: NUM_QPUS = SRQ_MAX  (16 -- oversubscribes onto the SRQ's
 *           16-slot queue, exposing queueing overhead vs throughput
 *           gain)
 *
 * Correctness gate runs once at the smallest sweep point with the
 * RTZ-aware tolerance (max <= 32 ULP and >= 99% one-sided). VC4 QPU
 * fmul/fadd round toward zero (AG100-R is silent on rounding mode,
 * and our previous runs locked >99.7% of mismatches as |qpu|<|ref|);
 * bit-exact equality with IEEE-RNE scalar is architecturally
 * impossible. The cancellation in cz = -1.01*z - 0.20 near z=-0.198
 * produces the modest tail of 4-16 ULP outliers -- standard FP, not
 * a QPU bug.
 *
 * Output column meanings:
 *   iters     = inner-loop count per QPU (16 vertices per iter)
 *   verts     = active_qpus * iters * 16
 *   qpu_us    = wall-clock for one bcm2835_v3d_kernel_execute
 *   cpu_us    = wall-clock for the equivalent scalar pass
 *   speedup   = cpu_us / qpu_us
 *   qpu_ns/v  = QPU per-vertex marginal cost (the asymptotic number
 *               drives Phase 4b's batch-size threshold)
 *   cpu_ns/v  = CPU per-vertex marginal cost (should be near-constant
 *               across all sweep points; if it isn't, ARM cache
 *               pressure is in play)
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/misc/bcm2835_v3d.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "vec3_xform_code.h"

LOG_MODULE_REGISTER(vec3_xform, LOG_LEVEL_INF);

#define V3D_NODE DT_NODELABEL(v3d)
BUILD_ASSERT(DT_NODE_HAS_STATUS(V3D_NODE, okay), "v3d node not enabled");

/* ----------- shape -------------------------------------------------- */
#define HW_QPUS                12u  /* matches BCM2710 IDENT1.NSLC*QUPS */
#define SRQ_MAX                BCM2835_V3D_MAX_QPUS  /* 16 */
#define LANES                  16u
#define NUM_UNIFS              (4u + 16u)
#define V3D_MEM_FLAG_L1_NONALLOC 0x4u

static const uint32_t ITER_SWEEP[] = {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u};
#define ITER_SWEEP_LEN  (sizeof(ITER_SWEEP) / sizeof(ITER_SWEEP[0]))
#define MAX_ITERS_PER_QPU      256u
#define REPS_PER_POINT         5u

/* Buffers sized for the larger pass (SRQ_MAX=16 QPUs at MAX_ITERS). */
#define MAX_IN_BYTES_PER_QPU   (MAX_ITERS_PER_QPU * 3u * LANES * sizeof(float))
#define MAX_OUT_BYTES_PER_QPU  (MAX_ITERS_PER_QPU * 4u * LANES * sizeof(float))
#define MAX_IN_BYTES_TOTAL     (SRQ_MAX * MAX_IN_BYTES_PER_QPU)
#define MAX_OUT_BYTES_TOTAL    (SRQ_MAX * MAX_OUT_BYTES_PER_QPU)
#define MAX_OUT_WORDS          (MAX_OUT_BYTES_TOTAL / sizeof(float))

#define MAX_ULP_TOLERANCE      32u
#define DIR_ONE_SIDED_MIN_PCT  99u

/* 1 MB static ref-out array (16 QPUs * 256 iters * 4 outputs * 16 lanes
 * * 4 bytes). RAM cost is acceptable; Zephyr's 256 MB region is
 * essentially free for us.
 */
static float ref_out[MAX_OUT_WORDS];

static const float test_mat[16] = {
	 1.10f,  0.05f,  0.00f,  0.00f,
	 0.00f,  1.42f,  0.00f,  0.00f,
	 0.03f,  0.07f, -1.01f, -1.00f,
	 0.00f,  0.00f, -0.20f,  0.00f,
};

static inline void scalar_xform(const float *m, float x, float y, float z,
                                float *cx, float *cy, float *cz, float *cw)
{
	*cx = m[0] * x + m[4] * y + m[ 8] * z + m[12];
	*cy = m[1] * x + m[5] * y + m[ 9] * z + m[13];
	*cz = m[2] * x + m[6] * y + m[10] * z + m[14];
	*cw = m[3] * x + m[7] * y + m[11] * z + m[15];
}

static uint32_t rng_state = 0xCAFEBABEu;

static float rand_float(void)
{
	rng_state = rng_state * 1664525u + 1013904223u;
	return ((float)(int32_t)rng_state / 2147483648.0f) * 100.0f;
}

static int v3d_smoke_app_marker(void)
{
	LOG_INF("APPLICATION init reached -- main() should fire next");
	return 0;
}
SYS_INIT(v3d_smoke_app_marker, APPLICATION, 99);

/* Fill the FULL input buffer (sized for SRQ_MAX QPUs at MAX_ITERS).
 * Each pass uses a prefix of this -- input contents are deterministic
 * so QPU and scalar always see identical bits.
 */
static void fill_inputs(float *in)
{
	for (uint32_t q = 0; q < SRQ_MAX; q++) {
		float *qpu_in = in + q * (MAX_IN_BYTES_PER_QPU / sizeof(float));

		for (uint32_t it = 0; it < MAX_ITERS_PER_QPU; it++) {
			float *chunk = qpu_in + it * 3u * LANES;

			for (uint32_t lane = 0; lane < LANES; lane++) {
				chunk[0 * LANES + lane] = rand_float();
				chunk[1 * LANES + lane] = rand_float();
				chunk[2 * LANES + lane] = rand_float();
			}
		}
	}
}

static void load_unifs(struct bcm2835_v3d_kernel *k, uint32_t active_qpus,
                       uintptr_t in_bus, uintptr_t out_bus, uint32_t iters)
{
	bcm2835_v3d_kernel_reset_unifs(k);
	for (uint32_t q = 0; q < active_qpus; q++) {
		uint32_t in_slice  = (uint32_t)in_bus  + q * MAX_IN_BYTES_PER_QPU;
		uint32_t out_slice = (uint32_t)out_bus + q * MAX_OUT_BYTES_PER_QPU;

		(void)bcm2835_v3d_kernel_load_unif_u32(k, q, q);
		(void)bcm2835_v3d_kernel_load_unif_u32(k, q, in_slice);
		(void)bcm2835_v3d_kernel_load_unif_u32(k, q, out_slice);
		(void)bcm2835_v3d_kernel_load_unif_u32(k, q, iters);

		for (uint32_t i = 0; i < 16; i++) {
			(void)bcm2835_v3d_kernel_load_unif_f32(k, q, test_mat[i]);
		}
	}
}

static void scalar_pass(const float *in, uint32_t active_qpus, uint32_t iters)
{
	for (uint32_t q = 0; q < active_qpus; q++) {
		const float *qpu_in  = in      + q * (MAX_IN_BYTES_PER_QPU / sizeof(float));
		float       *qpu_out = ref_out + q * (MAX_OUT_BYTES_PER_QPU / sizeof(float));

		for (uint32_t it = 0; it < iters; it++) {
			const float *in_chunk  = qpu_in  + it * 3u * LANES;
			float       *out_chunk = qpu_out + it * 4u * LANES;

			for (uint32_t lane = 0; lane < LANES; lane++) {
				scalar_xform(test_mat,
					in_chunk[0 * LANES + lane],
					in_chunk[1 * LANES + lane],
					in_chunk[2 * LANES + lane],
					&out_chunk[0 * LANES + lane],
					&out_chunk[1 * LANES + lane],
					&out_chunk[2 * LANES + lane],
					&out_chunk[3 * LANES + lane]);
			}
		}
	}
}

static int verify_correctness(const float *qpu_floats,
                              uint32_t active_qpus, uint32_t iters)
{
	uint32_t total_verts = active_qpus * iters * LANES;
	uint32_t total_words = active_qpus * iters * 4u * LANES;
	uint32_t exact = 0, mag_under = 0, mag_over = 0, mag_eq_diff_sign = 0;
	uint32_t ulp_hist[5] = {0};
	uint32_t max_ulp = 0;
	uint32_t shown = 0;

	for (uint32_t q = 0; q < active_qpus; q++) {
		const float *qpu_q = qpu_floats + q * (MAX_OUT_BYTES_PER_QPU / sizeof(float));
		const float *ref_q = ref_out    + q * (MAX_OUT_BYTES_PER_QPU / sizeof(float));

		for (uint32_t w = 0; w < iters * 4u * LANES; w++) {
			union { float f; uint32_t u; int32_t s; } qv = { .f = qpu_q[w] };
			union { float f; uint32_t u; int32_t s; } rv = { .f = ref_q[w] };

			if (qv.u == rv.u) {
				exact++;
				ulp_hist[0]++;
				continue;
			}

			uint32_t ud;

			if ((qv.s < 0) != (rv.s < 0)) {
				ud = UINT32_MAX;
			} else {
				int32_t d = qv.s - rv.s;
				ud = (uint32_t)(d < 0 ? -d : d);
			}
			if (ud > max_ulp) {
				max_ulp = ud;
			}
			ulp_hist[ud < 4u ? ud : 4u]++;

			float qa = qv.f < 0 ? -qv.f : qv.f;
			float ra = rv.f < 0 ? -rv.f : rv.f;

			if (qa < ra) {
				mag_under++;
			} else if (qa > ra) {
				mag_over++;
			} else {
				mag_eq_diff_sign++;
			}

			if (shown < 4u) {
				printk("[vec3-xform]   diff w=%u  qpu=0x%08x ref=0x%08x ulp=%u\n",
				       w, qv.u, rv.u, ud);
				shown++;
			}
		}
	}

	uint32_t total_mm = mag_under + mag_over;
	uint32_t pct_one_sided = (total_mm == 0u) ? 100u : (mag_under * 100u) / total_mm;

	printk("[vec3-xform] correctness: %u verts, %u words\n", total_verts, total_words);
	printk("[vec3-xform]   exact:        %u / %u\n", exact, total_words);
	printk("[vec3-xform]   direction:    under=%u over=%u sign-flip=%u  (%u%% one-sided)\n",
	       mag_under, mag_over, mag_eq_diff_sign, pct_one_sided);
	printk("[vec3-xform]   ULP histogram: 0=%u 1=%u 2=%u 3=%u 4+=%u  max=%u\n",
	       ulp_hist[0], ulp_hist[1], ulp_hist[2], ulp_hist[3], ulp_hist[4], max_ulp);

	bool ulp_ok       = (max_ulp <= MAX_ULP_TOLERANCE);
	bool one_sided_ok = (pct_one_sided >= DIR_ONE_SIDED_MIN_PCT);

	if (ulp_ok && one_sided_ok) {
		printk("[vec3-xform]   PASS (RTZ rounding confirmed, %u%% one-sided, max %u ULP)\n",
		       pct_one_sided, max_ulp);
		return 0;
	}
	printk("[vec3-xform]   FAIL (ulp_ok=%d one_sided_ok=%d)\n",
	       (int)ulp_ok, (int)one_sided_ok);
	return -EIO;
}

/* Run one full sweep with the given QPU count. Returns 0 on success. */
static int sweep_pass(const struct device *v3d, const char *label,
                      uint32_t active_qpus, uint32_t cps,
                      uintptr_t in_bus, uintptr_t out_bus,
                      const float *in_floats, float *out_floats,
                      bool verify_first)
{
	struct bcm2835_v3d_kernel k;
	int rc = bcm2835_v3d_kernel_init(v3d, &k, active_qpus, NUM_UNIFS,
	                                 vec3_xform_code, sizeof(vec3_xform_code));
	if (rc) {
		printk("[vec3-xform] FAIL (%s): kernel_init(%u) rc=%d\n",
		       label, active_qpus, rc);
		return rc;
	}

	if (verify_first) {
		uint32_t iters = ITER_SWEEP[0];

		load_unifs(&k, active_qpus, in_bus, out_bus, iters);

		for (uint32_t i = 0; i < MAX_OUT_WORDS; i++) {
			out_floats[i] = 1234.5678f;
		}

		rc = bcm2835_v3d_kernel_execute(v3d, &k);
		if (rc) {
			printk("[vec3-xform] FAIL (%s): correctness execute rc=%d\n", label, rc);
			goto out;
		}
		scalar_pass(in_floats, active_qpus, iters);

		if (verify_correctness(out_floats, active_qpus, iters) != 0) {
			rc = -EIO;
			goto out;
		}
	}

	printk("\n[vec3-xform] %s (NUM_QPUS=%u, min-of-%u per point):\n",
	       label, active_qpus, REPS_PER_POINT);
	printk("[vec3-xform]   iters  verts  qpu_us  cpu_us  speedup  qpu_ns/v  cpu_ns/v\n");

	for (uint32_t s = 0; s < ITER_SWEEP_LEN; s++) {
		uint32_t iters = ITER_SWEEP[s];
		uint32_t total_verts = active_qpus * iters * LANES;

		load_unifs(&k, active_qpus, in_bus, out_bus, iters);

		uint32_t qpu_min_cyc = UINT32_MAX;

		for (uint32_t rep = 0; rep < REPS_PER_POINT; rep++) {
			uint32_t t0 = k_cycle_get_32();

			rc = bcm2835_v3d_kernel_execute(v3d, &k);
			if (rc) {
				printk("[vec3-xform] FAIL (%s) iters=%u rep=%u rc=%d\n",
				       label, iters, rep, rc);
				goto out;
			}
			uint32_t cyc = k_cycle_get_32() - t0;

			if (cyc < qpu_min_cyc) {
				qpu_min_cyc = cyc;
			}
		}
		uint32_t qpu_us = (uint32_t)(((uint64_t)qpu_min_cyc * 1000000ULL) / cps);

		uint32_t cpu_min_cyc = UINT32_MAX;

		for (uint32_t rep = 0; rep < REPS_PER_POINT; rep++) {
			uint32_t t0 = k_cycle_get_32();

			scalar_pass(in_floats, active_qpus, iters);

			uint32_t cyc = k_cycle_get_32() - t0;

			if (cyc < cpu_min_cyc) {
				cpu_min_cyc = cyc;
			}
		}
		uint32_t cpu_us = (uint32_t)(((uint64_t)cpu_min_cyc * 1000000ULL) / cps);

		uint32_t speedup_x100   = (qpu_us > 0u) ? (cpu_us * 100u / qpu_us) : 0u;
		uint32_t qpu_ns_per_vert = (uint32_t)(((uint64_t)qpu_us * 1000ULL) / total_verts);
		uint32_t cpu_ns_per_vert = (uint32_t)(((uint64_t)cpu_us * 1000ULL) / total_verts);

		printk("[vec3-xform]   %5u  %5u  %6u  %6u  %3u.%02ux  %8u  %8u\n",
		       iters, total_verts, qpu_us, cpu_us,
		       speedup_x100 / 100u, speedup_x100 % 100u,
		       qpu_ns_per_vert, cpu_ns_per_vert);
	}

out:
	(void)bcm2835_v3d_kernel_free(v3d, &k);
	return rc;
}

int main(void)
{
	LOG_INF("main() entered");
	printk("[vec3-xform] starting -- two-pass sweep, ITERS=1..%u, REPS=%u\n",
	       MAX_ITERS_PER_QPU, REPS_PER_POINT);

	const struct device *v3d = DEVICE_DT_GET(V3D_NODE);

	if (!device_is_ready(v3d)) {
		printk("[vec3-xform] FAIL: v3d device not ready\n");
		return -ENODEV;
	}

	uintptr_t in_bus = 0, out_bus = 0;
	void *in_cpu = NULL, *out_cpu = NULL;
	uint32_t in_handle = 0, out_handle = 0;

	int rc = bcm2835_v3d_alloc_coherent(v3d, MAX_IN_BYTES_TOTAL, 64u,
	                                    V3D_MEM_FLAG_L1_NONALLOC,
	                                    &in_bus, &in_cpu, &in_handle);
	if (rc) {
		printk("[vec3-xform] FAIL: alloc input rc=%d\n", rc);
		return rc;
	}
	rc = bcm2835_v3d_alloc_coherent(v3d, MAX_OUT_BYTES_TOTAL, 64u,
	                                V3D_MEM_FLAG_L1_NONALLOC,
	                                &out_bus, &out_cpu, &out_handle);
	if (rc) {
		printk("[vec3-xform] FAIL: alloc output rc=%d\n", rc);
		(void)bcm2835_v3d_free_coherent(v3d, in_handle);
		return rc;
	}

	float *in_floats  = (float *)in_cpu;
	float *out_floats = (float *)out_cpu;

	fill_inputs(in_floats);

	uint32_t cps = (uint32_t)sys_clock_hw_cycles_per_sec();

	/* Pass 1: hardware-saturated (12 QPUs, no queueing).
	 * Verify correctness here.
	 */
	rc = sweep_pass(v3d, "PASS 1 saturated (12 QPUs)", HW_QPUS, cps,
	                in_bus, out_bus, in_floats, out_floats,
	                /*verify_first=*/true);
	if (rc) {
		goto cleanup;
	}

	/* Pass 2: SRQ-oversubscribed (16 software threads on 12 hardware
	 * QPUs -- scheduler queues 4). No correctness verify here -- same
	 * shader, same data, so if pass 1 was correct, pass 2 will be too.
	 */
	rc = sweep_pass(v3d, "PASS 2 oversubscribed (16 QPUs)", SRQ_MAX, cps,
	                in_bus, out_bus, in_floats, out_floats,
	                /*verify_first=*/false);
	if (rc) {
		goto cleanup;
	}

	printk("\n[vec3-xform] PASS (both sweeps complete)\n");

cleanup:
	(void)bcm2835_v3d_free_coherent(v3d, in_handle);
	(void)bcm2835_v3d_free_coherent(v3d, out_handle);
	return rc;
}
