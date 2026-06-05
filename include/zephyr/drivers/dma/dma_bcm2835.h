/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_BCM2835_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_BCM2835_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Per-channel BCM2835 DMA register snapshot.
 *
 * Captured by reading the live MMIO state of the channel — intended for
 * diagnosing wedged or stalled transfers (e.g. unexplained 128-bit
 * burst-8 lockups against the VC scanout framebuffer). The fields map
 * directly onto the engine's CS, CONBLK_AD, TI, SOURCE_AD, DEST_AD,
 * TXFR_LEN and DEBUG registers.
 */
struct dma_bcm2835_chan_state {
	uint32_t cs;          /**< Control / status (CS) */
	uint32_t conblk_ad;   /**< Active control-block bus address */
	uint32_t ti;          /**< Transfer information shadow */
	uint32_t source_ad;   /**< Current source address shadow */
	uint32_t dest_ad;     /**< Current destination address shadow */
	uint32_t txfr_len;    /**< Bytes remaining in active block */
	uint32_t debug;       /**< DEBUG: error sticky bits + outstanding writes */
};

/**
 * @brief Snapshot the BCM2835 DMA channel's live register state.
 *
 * Read-only diagnostic peek. Touches MMIO only — no driver state
 * change. Safe to call from any context, including a timed-out
 * waiter that already lost the engine.
 *
 * @param dev      DMA controller device.
 * @param channel  Channel index.
 * @param out      Snapshot destination.
 *
 * @retval 0        Snapshot written.
 * @retval -EINVAL  Channel not enabled on this controller.
 */
int dma_bcm2835_get_chan_state(const struct device *dev, uint32_t channel,
			       struct dma_bcm2835_chan_state *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_BCM2835_H_ */
