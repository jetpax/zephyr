/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_BCM2835_V3D_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_BCM2835_V3D_H_

/**
 * @file
 * @brief Broadcom VideoCore IV 3D (V3D / QPU) compute driver.
 *
 * Launches hand-written QPU kernels (assembled qasm output as a
 * @c uint32_t array) on the 12 SIMD QPUs of the BCM2835 / BCM2710 /
 * BCM2711 GPU. Bypasses the firmware mailbox tag 0x60010 and pokes
 * the V3D scheduler-request queue (SRQ) registers directly; per-kick
 * overhead is ~5-20 us vs ~1 ms for the firmware route.
 *
 * Memory layout:
 *
 *  - @ref bcm2835_v3d_kernel describes a self-contained QPU job:
 *    code (the compiled kernel), per-QPU uniforms, and the mailbox
 *    message array the V3D scheduler consumes.
 *
 *  - All four buffers live in firmware-allocated GPU-coherent memory
 *    obtained via @ref bcm2835_v3d_alloc_coherent. The driver maps
 *    the bus address back to ARM virtual via @c device_map (cache
 *    none) so ARM can fill uniforms and read results.
 *
 * Typical use:
 *
 * @code
 * const struct device *v3d = DEVICE_DT_GET(DT_NODELABEL(v3d));
 * struct bcm2835_v3d_kernel k;
 *
 * bcm2835_v3d_kernel_init(v3d, &k, NUM_QPUS, NUM_UNIFS,
 *                         qpu_code, sizeof(qpu_code));
 * for (uint32_t i = 0; i < NUM_QPUS; i++) {
 *     bcm2835_v3d_kernel_load_unif_u32(v3d, &k, i, output_bus_addr + i*stride);
 *     bcm2835_v3d_kernel_load_unif_u32(v3d, &k, i, num_iters);
 *     bcm2835_v3d_kernel_load_unif_u32(v3d, &k, i, i);
 * }
 * bcm2835_v3d_kernel_execute(v3d, &k);
 * @endcode
 */

#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum QPUs we'll ever target (the BCM2835/2710 has 12). */
#define BCM2835_V3D_MAX_QPUS 12

/**
 * @brief Handle for a single QPU kernel + its per-QPU uniforms.
 *
 * Opaque to callers -- the fields are exposed for stack allocation
 * but should not be touched directly.
 */
struct bcm2835_v3d_kernel {
	/* GPU-coherent backing buffer (bus address as returned by
	 * the firmware lock-memory tag).
	 */
	uintptr_t bus_addr;
	/* ARM-visible mapping of the same buffer. */
	void     *cpu_addr;
	uint32_t  total_size;
	uint32_t  mem_handle;   /* mbox handle for release */

	uint32_t  num_qpus;
	uint32_t  num_unifs;

	/* Cursors into cpu_addr: */
	uint32_t *code;         /* QPU instruction stream */
	uint32_t  code_size;
	uint32_t *unif;         /* [num_qpus][num_unifs] uniform values */
	uint32_t *mbox_msg;     /* [num_qpus][2] (unif_bus_addr, code_bus_addr) */
	uint32_t *cur_unif;     /* [num_qpus] per-QPU load cursor */
};

/**
 * @brief Allocate a GPU-coherent buffer via the firmware mailbox.
 *
 * Returns the bus address (what V3D sees) and, optionally, an
 * ARM-side virtual pointer mapped K_MEM_CACHE_NONE so ARM writes
 * land in DRAM without needing per-access cache maintenance.
 *
 * @param dev      v3d device.
 * @param size     Buffer size in bytes (will be rounded up to 4 KiB).
 * @param align    Alignment in bytes (typical: 16 for unif arrays,
 *                 4096 for code).
 * @param mem_flag Firmware MEM_FLAG_* bits (use 4 = L1 non-allocating
 *                 for QPU-touched memory).
 * @param[out] bus_addr    GPU bus address.
 * @param[out] cpu_addr    ARM-mapped virtual address (may be NULL).
 * @param[out] handle      Opaque handle for @ref bcm2835_v3d_free_coherent.
 *
 * @retval 0       Success.
 * @retval -EIO    Firmware rejected the allocation.
 * @retval -ENOMEM Backing-map allocation failed.
 */
int bcm2835_v3d_alloc_coherent(const struct device *dev,
                               uint32_t size, uint32_t align, uint32_t mem_flag,
                               uintptr_t *bus_addr, void **cpu_addr,
                               uint32_t *handle);

/**
 * @brief Release a coherent buffer obtained from @ref
 *        bcm2835_v3d_alloc_coherent.
 */
int bcm2835_v3d_free_coherent(const struct device *dev, uint32_t handle);

/**
 * @brief Initialise a kernel handle: allocate one coherent buffer
 *        big enough for code + uniforms + scheduler-message array,
 *        copy the kernel code in, and prepare the mailbox-message
 *        array the scheduler consumes.
 */
int bcm2835_v3d_kernel_init(const struct device *dev,
                            struct bcm2835_v3d_kernel *k,
                            uint32_t num_qpus, uint32_t num_unifs,
                            const uint32_t *code, uint32_t code_size);

/**
 * @brief Release a kernel's backing buffer.
 */
int bcm2835_v3d_kernel_free(const struct device *dev,
                            struct bcm2835_v3d_kernel *k);

/**
 * @brief Reset all per-QPU uniform load cursors back to zero -- so a
 *        second @c kernel_execute call can refill the uniforms with
 *        new values.
 */
void bcm2835_v3d_kernel_reset_unifs(struct bcm2835_v3d_kernel *k);

/**
 * @brief Append a uniform word for QPU @p qpu. Each QPU has its own
 *        cursor; calling N times fills @c num_unifs entries.
 */
int bcm2835_v3d_kernel_load_unif_u32(struct bcm2835_v3d_kernel *k,
                                     uint32_t qpu, uint32_t val);
int bcm2835_v3d_kernel_load_unif_f32(struct bcm2835_v3d_kernel *k,
                                     uint32_t qpu, float val);

/**
 * @brief Kick the kernel on all @c num_qpus QPUs and busy-wait until
 *        every QPU reports completion via SRQCS.
 */
int bcm2835_v3d_kernel_execute(const struct device *dev,
                               struct bcm2835_v3d_kernel *k);

/**
 * @brief Kick the kernel and return immediately. Pair with
 *        @ref bcm2835_v3d_kernel_wait.
 */
int bcm2835_v3d_kernel_execute_async(const struct device *dev,
                                     struct bcm2835_v3d_kernel *k);

/**
 * @brief Busy-wait for an outstanding async kick to complete.
 *
 * @param timeout_us Max microseconds to wait; 0 = forever.
 */
int bcm2835_v3d_kernel_wait(const struct device *dev,
                            struct bcm2835_v3d_kernel *k,
                            uint32_t timeout_us);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_BCM2835_V3D_H_ */
