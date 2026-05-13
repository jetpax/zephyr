/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WIFI_BRCMFMAC_PRIV_H_
#define ZEPHYR_DRIVERS_WIFI_BRCMFMAC_PRIV_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>

/* === SBSDIO function-1 misc-register layout =================================
 *
 * F1 exposes a 32 KiB sliding window onto the chip's AXI/SB backplane;
 * the SBADDR* registers set the window base, then accesses to func1
 * offset [0..0x7FFF] map to backplane[base + offset]. Setting bit 15
 * in the offset (SB_ACCESS_2_4B_FLAG) makes a 4-byte access at the
 * chip side, not four single-byte accesses.
 */
#define SBSDIO_FUNC1_SBADDRLOW          0x1000A
#define SBSDIO_FUNC1_SBADDRMID          0x1000B
#define SBSDIO_FUNC1_SBADDRHIGH         0x1000C
#define SBSDIO_FUNC1_CHIPCLKCSR         0x1000E
#define SBSDIO_FUNC1_SDIOPULLUP         0x1000F
#define SBSDIO_SB_OFT_ADDR_MASK         0x07FFF
#define SBSDIO_SB_ACCESS_2_4B_FLAG      0x08000
#define SBSDIO_SBWINDOW_MASK            0xFFFF8000
#define SBSDIO_SB_OFT_ADDR_LIMIT        0x8000

/* CHIPCLKCSR bits. */
#define SBSDIO_FORCE_ALP                0x01
#define SBSDIO_ALP_AVAIL_REQ            0x08
#define SBSDIO_HT_AVAIL_REQ             0x10
#define SBSDIO_FORCE_HW_CLKREQ_OFF      0x20
#define SBSDIO_ALP_AVAIL                0x40
#define SBSDIO_HT_AVAIL                 0x80
#define SBSDIO_AVBITS                   (SBSDIO_ALP_AVAIL | SBSDIO_HT_AVAIL)
#define BRCMF_INIT_CLKCTL1              (SBSDIO_FORCE_HW_CLKREQ_OFF | \
                                         SBSDIO_ALP_AVAIL_REQ)

/* Chipcommon enumeration base + chipid register layout. */
#define BRCMF_SI_ENUM_BASE              0x18000000
#define CID_ID_MASK                     0x0000FFFF
#define CID_REV_MASK                    0x000F0000
#define CID_REV_SHIFT                   16
#define CID_TYPE_MASK                   0xF0000000
#define CID_TYPE_SHIFT                  28

/* BCMA core IDs (subset; full list in Linux include/linux/bcma/bcma.h). */
#define BCMA_CORE_INTERNAL_MEM          0x80E    /* SOCRAM */
#define BCMA_CORE_80211                 0x812    /* D11 MAC */
#define BCMA_CORE_PMU                   0x827
#define BCMA_CORE_SDIO_DEV              0x829
#define BCMA_CORE_ARM_CM3               0x82A
#define BCMA_CORE_GCI                   0x840

/* SDIO core register offset within data base. */
#define SDPCMD_INTSTATUS                0x20     /* W1C ack-all = 0xFFFFFFFF */

/* BCMA wrapper-register offsets (within wrapbase). */
#define BCMA_IOCTL                      0x0408
#define BCMA_IOCTL_CLK                  0x0001
#define BCMA_IOCTL_FGC                  0x0002
#define BCMA_RESET_CTL                  0x0800
#define BCMA_RESET_CTL_RESET            0x0001

/* D11-specific IOCTL bits. */
#define D11_BCMA_IOCTL_PHYCLOCKEN       0x0004
#define D11_BCMA_IOCTL_PHYRESET         0x0008

/* SOCRAM register offsets (within SOCRAM core base). */
#define SOCRAM_BANKIDX_OFFSET           0x10
#define SOCRAM_BANKPDA_OFFSET           0x44

/* Per-CMD53 block-mode cap. 512-block CMD53 wedges this chip's SDIO
 * state machine (Wall #7 in the bring-up debug): 511 * 64 = 32704.
 */
#define BRCMFMAC_MAX_CMD53_BLOCK_BYTES  (511 * 64)

#define BRCMFMAC_MAX_CORES              12

/* === SDPCM + BCDC protocol ================================================
 *
 * Wire format for an SDIO control message:
 *   [4 B  SDPCM frame:  u16 len, u16 ~len]
 *   [8 B  SDPCM sw hdr: seq, chan, nextlen, hdrlen, flow, credit, rsv x2]
 *   [16 B BCDC/CDC hdr: cmd, outlen, inlen, flags, status]
 *   [payload (variable)]
 *   [padded to 4-byte alignment]
 */
#define BRCMFMAC_F2_FIFO_ADDR           0x8000     /* SB_ACCESS_2_4B_FLAG bit */
#define BRCMFMAC_F2_BLOCK_SIZE          512

#define BRCMFMAC_WLC_GET_VAR            262
#define BRCMFMAC_WLC_SET_VAR            263

#define BCDC_FLAG_ERROR                 0x01
#define BCDC_FLAG_SET                   0x02
#define BCDC_REQ_ID_SHIFT               16

#define SDPCM_CHAN_CTRL                 0
#define SDPCM_CHAN_EVENT                1
#define SDPCM_CHAN_DATA                 2

struct sdpcm_frame_hdr {
	uint16_t len;
	uint16_t notlen;
} __packed;

struct sdpcm_sw_hdr {
	uint8_t seq;
	uint8_t chan;
	uint8_t nextlen;
	uint8_t hdrlen;
	uint8_t flow;
	uint8_t credit;
	uint8_t reserved[2];
} __packed;

struct cdc_hdr {
	uint32_t cmd;
	uint16_t outlen;
	uint16_t inlen;
	uint32_t flags;
	uint32_t status;
} __packed;

struct bcm_core {
	uint16_t id;
	uint32_t base;
	uint32_t wrapbase;
};

struct brcmfmac_config {
	const struct device *sdhc;
	struct gpio_dt_spec reg_on;
	const char *firmware_name;
};

struct brcmfmac_data;

/* Event-frame callback (chan=1). Phase 4.4 hands the raw frame body
 * (BDC-stripped) to the handler; structured event parsing arrives
 * with wifi_mgmt in Phase 4.5.
 */
typedef void (*brcmfmac_event_cb_t)(struct brcmfmac_data *data,
				    const uint8_t *frame, uint16_t len);

/* Pending IOCTL waiter: filled by query_dcmd, completed by RX thread. */
struct brcmfmac_pending_ioctl {
	bool active;
	uint16_t reqid;
	uint8_t *out_buf;
	uint16_t out_capacity;
	uint16_t out_copied;
	int status;
	struct k_sem done;
};

struct brcmfmac_data {
	struct sd_card card;
	struct sdio_func backplane;     /* F1 */
	struct sdio_func radio;         /* F2 */

	/* Chip topology discovered by EROM scan. */
	struct bcm_core cores[BRCMFMAC_MAX_CORES];
	unsigned int num_cores;

	/* Chip identity (chipcommon[0]). */
	uint32_t chipid_reg;
	uint16_t chip_id;
	uint8_t  chip_rev;
	uint8_t  chip_type;

	/* BCDC protocol state. */
	bool f2_ready;
	uint8_t sdpcm_txseq;
	uint16_t bcdc_reqid;
	struct k_mutex bcdc_mutex;
	struct brcmfmac_pending_ioctl pending;
	brcmfmac_event_cb_t event_cb;

	/* MAC from chip OTP, read via cur_etheraddr IOCTL. */
	uint8_t chip_mac[6];

	bool probed;
};

/* === SDIO backplane primitives (brcmfmac_sdio.c) === */
int brcmfmac_sdio_set_backplane_window(struct brcmfmac_data *data, uint32_t addr);
int brcmfmac_sdio_backplane_read32(struct brcmfmac_data *data, uint32_t addr,
				   uint32_t *out);
int brcmfmac_sdio_backplane_write32(struct brcmfmac_data *data, uint32_t addr,
				    uint32_t val);
int brcmfmac_sdio_backplane_read_bytes(struct brcmfmac_data *data, uint32_t addr,
				       uint8_t *buf, uint32_t len);
int brcmfmac_sdio_backplane_write_bytes(struct brcmfmac_data *data, uint32_t addr,
					const uint8_t *buf, uint32_t len);
int brcmfmac_sdio_ramrw(struct brcmfmac_data *data, bool write,
			uint32_t chip_addr, uint8_t *buf, uint32_t size);
int brcmfmac_sdio_fw_upload(struct brcmfmac_data *data);
int brcmfmac_sdio_nvram_upload(struct brcmfmac_data *data);

/* === Chip bring-up (brcmfmac_chip.c) === */
int brcmfmac_chip_read_id(struct brcmfmac_data *data);
int brcmfmac_chip_pmu_setup(struct brcmfmac_data *data);
int brcmfmac_chip_erom_scan(struct brcmfmac_data *data);
const struct bcm_core *brcmfmac_chip_core_find(const struct brcmfmac_data *data,
					       uint16_t id);
int brcmfmac_chip_set_passive(struct brcmfmac_data *data);
int brcmfmac_chip_set_active(struct brcmfmac_data *data);

/* === BCDC protocol (brcmfmac_bcdc.c) ===
 *
 * Phase 4.3: F2 enable + polled IOCTL round-trip. Single in-flight,
 * caller blocks on the response. Phase 4.4 splits the polling out
 * into a dedicated RX thread.
 */
int brcmfmac_bcdc_init(struct brcmfmac_data *data);

/* Run a dcmd. tx_payload + tx_len go into the request; chip's response
 * payload (up to rx_capacity bytes) is copied to rx_buf. Returns bytes
 * copied or negative errno.
 */
int brcmfmac_bcdc_query_dcmd(struct brcmfmac_data *data, uint32_t cmd,
			     const uint8_t *tx_payload, uint16_t tx_len,
			     uint8_t *rx_buf, uint16_t rx_capacity);

/* IOVAR helper: WLC_GET_VAR with `name` as the var key. */
int brcmfmac_bcdc_iovar_get(struct brcmfmac_data *data, const char *name,
			    uint8_t *buf, uint16_t len);

/* Register a callback for chan=1 event frames. NULL = no callback (frames
 * are still drained from F2, just not dispatched).
 */
void brcmfmac_bcdc_set_event_cb(struct brcmfmac_data *data,
				brcmfmac_event_cb_t cb);

/* === Embedded firmware blobs (firmware/) === */
extern const unsigned char brcmfmac_fw[];
extern const unsigned int  brcmfmac_fw_len;
extern const unsigned char brcmfmac_nvram[];
extern const unsigned int  brcmfmac_nvram_len;

#endif /* ZEPHYR_DRIVERS_WIFI_BRCMFMAC_PRIV_H_ */
