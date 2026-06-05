/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DISPLAY_BCM2835_FB_H_
#define ZEPHYR_INCLUDE_DRIVERS_DISPLAY_BCM2835_FB_H_

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Kick off a full-frame DMA blit and return immediately.
 *
 * Asynchronous variant of display_write(). Intended for double-buffered
 * renderers that want to overlap CPU rendering of frame N+1 with the
 * DMA transfer of frame N. The buffer passed in must remain valid (and
 * its contents undisturbed) until the matching bcm2835_fb_wait() call
 * observes completion -- otherwise the DMA reads stale or in-progress
 * data.
 *
 * If a previous async write is still in flight when this is called,
 * it blocks until that DMA completes (so back-to-back calls naturally
 * pipeline at the rate of the slower of render-vs-blit).
 *
 * Only the full-frame contiguous fast path is supported: @p desc must
 * describe the entire framebuffer width and the buffer must be
 * contiguous (pitch == width).
 *
 * @param dev   bcm2835_fb display device.
 * @param desc  Descriptor matching the full framebuffer geometry.
 * @param buf   Source buffer. Cache-flush is done internally before
 *              the engine reads it.
 *
 * @retval 0          Blit kicked off; caller may proceed.
 * @retval -EINVAL    Descriptor not full-frame, or device lacks DMA.
 * @retval -EAGAIN    Previous DMA timed out (channel reset).
 * @retval <0         dma_config / dma_start error.
 */
int bcm2835_fb_write_async(const struct device *dev,
			   const struct display_buffer_descriptor *desc,
			   const void *buf);

/**
 * @brief Block until the most recent async write completes.
 *
 * No-op if no async write is outstanding. Used at shutdown / on the
 * last frame to ensure the engine isn't still reading caller memory.
 *
 * @param dev      bcm2835_fb display device.
 * @param timeout  Kernel timeout.
 *
 * @retval 0          Previous write completed.
 * @retval -EAGAIN    Timeout (channel was reset; same diagnostic dump
 *                    as bcm2835_fb_write fires).
 */
int bcm2835_fb_wait(const struct device *dev, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DISPLAY_BCM2835_FB_H_ */
