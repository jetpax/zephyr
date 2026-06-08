/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom VideoCore IV 3D (V3D) compute driver -- direct V3D
 * scheduler-queue access for launching QPU kernels.
 *
 * Architecture in 3 paragraphs:
 *
 *   The BCM2835 / BCM2710 / BCM2711 SoCs contain a 12-QPU SIMD compute
 *   block (V3D) sharing DRAM with the ARM cores. Each QPU is a 16-lane
 *   32-bit SIMD core with its own register file; assembled QPU kernels
 *   (qasm output as uint32_t[]) execute on the QPUs and can read/write
 *   GPU-bus memory either via the VPM (Vertex Processor Memory)
 *   scratchpad or directly via TMU/DMA registers.
 *
 *   Two routes exist for launching QPU work from ARM: (1) firmware
 *   mailbox tag 0x60010 "execute QPU" which adds ~1 ms of round-trip
 *   per kick, or (2) direct access to the V3D scheduler-request queue
 *   (SRQ) registers, which is what this driver does. The SRQ path is
 *   ~5-20 us per kick -- usable per-frame for accelerating engine work
 *   like wipeout's vec3_transform_perspective pipeline.
 *
 *   Memory for QPU code + uniforms + I/O buffers must be GPU-coherent
 *   -- we allocate it via the firmware mailbox ALLOCATE_MEMORY /
 *   LOCK_MEMORY property tags (already in the rpi_fw driver). The
 *   firmware returns a GPU-bus address; we map that into ARM virtual
 *   memory via device_map(K_MEM_CACHE_NONE) so ARM accesses bypass
 *   cache and don't fight QPU memory traffic.
 *
 * Smoke-test scaffold: this initial commit only enables the QPU clock
 * domain and probes the V3D register block. Kernel launch logic comes
 * in the next task.
 */

#define DT_DRV_COMPAT brcm_bcm2835_v3d

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/misc/bcm2835_v3d.h>
#include <rpi_fw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#include <zephyr/sys/mem_manage.h>
#include <errno.h>

LOG_MODULE_REGISTER(bcm2835_v3d, CONFIG_BCM2835_V3D_LOG_LEVEL);

/* ------------------------------------------------------------------ *
 *  V3D register offsets (from BCM2835 ARM peripherals datasheet + the
 *  videocoreiv-qpu reference). Only the ones we touch are defined.
 * ------------------------------------------------------------------ */

#define V3D_IDENT0   0x0000  /* V3D identity register 0 -- "V3D\x02" */
#define V3D_IDENT1   0x0004
#define V3D_IDENT2   0x0008
#define V3D_L2CACTL  0x0020  /* L2 cache control */
#define V3D_SLCACTL  0x0024  /* Slice cache control */
#define V3D_SRQPC    0x0430  /* SRQ program counter (write = kick) */
#define V3D_SRQUA    0x0434  /* SRQ uniforms address */
#define V3D_SRQCS    0x043C  /* SRQ control/status */
#define V3D_DBCFG    0x0E00
#define V3D_DBQITE   0x0E2C
#define V3D_DBQITC   0x0E30

/* V3D_IDENT0 reads back as the four bytes 'V','3','D',0x02 stored in
 * little-endian order -- so a u32 read returns 0x02443356, NOT
 * 0x02334456 (that was a transposition typo in the original scaffold;
 * confirmed against real silicon during BCM2710 bring-up). When the
 * QPU clock is enabled and V3D is out of reset, IDENT0 reads this
 * magic; otherwise it reads 0xdeadbeef.
 */
#define V3D_IDENT0_MAGIC 0x02443356u

/* ------------------------------------------------------------------ *
 *  BCM2835 Power Manager (PM) + AXI Async Bridge (ASB) register map
 *
 *  On BCM2710 (Pi 3 family / Zero 2W), V3D power is NOT controlled by
 *  the firmware mailbox -- the firmware refuses to expose it. The
 *  power-up sequence is owned by ARM and goes through these two
 *  register windows, exactly as Linux's drivers/pmdomain/bcm/
 *  bcm2835-power.c does. On Pi 1 (BCM2835) the firmware brings V3D
 *  up by default which is why pigs's mailbox-only path works there
 *  and not here.
 *
 *  Addresses come from Linux's bcm2835-common.dtsi:
 *    pm:  bus 0x7e100000 / phys 0x3f100000, size 0x114
 *    asb: bus 0x7e00a000 / phys 0x3f00a000, size 0x024
 *
 *  Every PM write must include the PM_PASSWORD magic in the upper
 *  bits or the hardware silently drops it (anti-glitch protection).
 * ------------------------------------------------------------------ */

#define V3D_PM_PHYS    0x3F100000u
#define V3D_PM_SIZE    0x1000u

#define V3D_ASB_PHYS   0x3F00A000u
#define V3D_ASB_SIZE   0x100u

/* CPRMAN (clock & power management) sits 0x1000 above the PM block;
 * we want CM_V3DCTL at +0x038 to confirm the V3D clock-gate bit.
 * Per Linux's drivers/clk/bcm/clk-bcm2835.c.
 */
#define V3D_CM_PHYS    0x3F101000u
#define V3D_CM_SIZE    0x100u
#define CM_PASSWORD    0x5A000000u
#define CM_V3DCTL      0x038
#define CM_ENABLE      BIT(4)
#define CM_BUSY        BIT(7)

#define PM_PASSWORD    0x5A000000u

/* PM register offsets within the PM window. */
#define PM_GNRIC       0x000
#define PM_AUDIO       0x004
#define PM_STATUS      0x018
#define PM_PXLDO       0x060   /* Pixel pipeline LDO regulator */
#define PM_PXBG        0x064   /* Pixel bandgap reference */
#define PM_DFT         0x068
#define PM_SMPS        0x06C   /* Switched-mode power supply (main rail) */
#define PM_XOSC        0x070   /* Crystal oscillator control */
#define PM_AVS_STAT    0x080   /* Adaptive voltage scaling status */
#define PM_IMAGE       0x108
#define PM_GRAFX       0x10C
#define PM_PROC        0x110

/* PM_GRAFX bit layout (shared with PM_IMAGE / PM_PROC blocks). */
#define PM_POWUP       BIT(0)
#define PM_POWOK       BIT(1)
#define PM_ISPOW       BIT(2)
#define PM_MEMREP      BIT(3)
#define PM_MRDONE      BIT(4)
#define PM_ISFUNC      BIT(5)
#define PM_V3DRSTN     BIT(6)  /* V3D reset-not -- 1 = de-asserted */
#define PM_INRUSH_SHIFT  13
#define PM_INRUSH_MASK   (0x3u << PM_INRUSH_SHIFT)
#define PM_INRUSH_3_5MA  0
#define PM_INRUSH_20MA   3

/* ASB register offsets within the ASB window. */
#define ASB_BRDG_VERSION 0x00
#define ASB_CPR_CTRL     0x04
#define ASB_V3D_S_CTRL   0x08
#define ASB_V3D_M_CTRL   0x0C
#define ASB_AXI_BRDG_ID  0x20  /* Reads "brdg" (0x62726467) when ASB is alive */
#define ASB_REQ_STOP     BIT(0)  /* 1 = halt traffic, wait for empty */
#define ASB_ACK          BIT(1)  /* set when REQ_STOP transition done */
#define ASB_BRDG_ID_MAGIC 0x62726467u  /* Linux's BCM2835_BRDG_ID */

/* ------------------------------------------------------------------ *
 *  Firmware mailbox property-tag buffer layouts (per
 *  raspberrypi/firmware/wiki Mailbox-property-interface)
 * ------------------------------------------------------------------ */

/* MEM_FLAG_L1_NONALLOCATING: VC L1 cache bypassed; suitable for
 * buffers the ARM side will also touch. Matches what pigs uses.
 */
#define V3D_MEM_FLAG_L1_NONALLOC 0x4

/* Bus-address alias for L1-non-allocating memory. The firmware
 * returns bus addresses with this mask already applied; ARM physical
 * is bus_addr & 0x3FFFFFFF (the low 30 bits).
 */
#define V3D_BUS_TO_PHYS_MASK 0x3FFFFFFFu

/* ------------------------------------------------------------------ *
 *  Per-instance state
 * ------------------------------------------------------------------ */

struct bcm2835_v3d_config {
	uintptr_t v3d_phys;
	const struct device *fw;
};

struct bcm2835_v3d_data {
	uint8_t *v3d_regs;   /* ARM-virtual base for V3D MMIO */
	uint8_t *pm_regs;    /* PM (power manager) MMIO */
	uint8_t *asb_regs;   /* AXI Async Bridge MMIO */
	uint8_t *cm_regs;    /* CPRMAN (clock manager) MMIO */
};

/* Convenience accessors. */
static inline uint32_t v3d_read(const struct device *dev, uint32_t off)
{
	struct bcm2835_v3d_data *d = dev->data;

	return sys_read32((uintptr_t)(d->v3d_regs + off));
}

static inline void v3d_write(const struct device *dev, uint32_t off, uint32_t v)
{
	struct bcm2835_v3d_data *d = dev->data;

	sys_write32(v, (uintptr_t)(d->v3d_regs + off));
}

/* PM_PASSWORD-gated read/write. PM register writes are silently
 * dropped without the magic. Read is plain.
 */
static inline uint32_t pm_read(const struct device *dev, uint32_t off)
{
	struct bcm2835_v3d_data *d = dev->data;

	return sys_read32((uintptr_t)(d->pm_regs + off));
}

static inline void pm_write(const struct device *dev, uint32_t off, uint32_t v)
{
	struct bcm2835_v3d_data *d = dev->data;

	sys_write32(PM_PASSWORD | v, (uintptr_t)(d->pm_regs + off));
}

static inline uint32_t asb_read(const struct device *dev, uint32_t off)
{
	struct bcm2835_v3d_data *d = dev->data;

	return sys_read32((uintptr_t)(d->asb_regs + off));
}

static inline void asb_write(const struct device *dev, uint32_t off, uint32_t v)
{
	struct bcm2835_v3d_data *d = dev->data;

	/* ASB writes also need the PM_PASSWORD prefix on BCM2710. */
	sys_write32(PM_PASSWORD | v, (uintptr_t)(d->asb_regs + off));
}

static inline uint32_t cm_read(const struct device *dev, uint32_t off)
{
	struct bcm2835_v3d_data *d = dev->data;

	return sys_read32((uintptr_t)(d->cm_regs + off));
}

static inline void cm_write(const struct device *dev, uint32_t off, uint32_t v)
{
	struct bcm2835_v3d_data *d = dev->data;

	sys_write32(CM_PASSWORD | v, (uintptr_t)(d->cm_regs + off));
}

/* Enable an ASB master/slave bridge for the given reg offset. Clears
 * ASB_REQ_STOP, then polls ASB_ACK to confirm the bridge is letting
 * AXI traffic through. Ported from Linux's bcm2835_asb_control().
 */
static int asb_enable(const struct device *dev, uint32_t off)
{
	uint32_t val = asb_read(dev, off) & ~ASB_REQ_STOP;
	asb_write(dev, off, val);

	uint32_t start = k_cycle_get_32();
	uint32_t cps   = (uint32_t)sys_clock_hw_cycles_per_sec();

	while (asb_read(dev, off) & ASB_ACK) {
		if ((k_cycle_get_32() - start) > (cps / 1000)) {  /* 1 ms */
			LOG_ERR("asb_enable(+0x%x) timed out (val=0x%x)",
				off, asb_read(dev, off));
			return -ETIMEDOUT;
		}
	}
	return 0;
}

/* Diagnostic snapshot of the PM / ASB / V3D register state. Logged at
 * three points during bring-up so a remote post-mortem can tell which
 * step (if any) moved the bits. Cheap: 6 MMIO reads, one log line.
 */
static void dump_state(const struct device *dev, const char *tag)
{
	uint32_t status = pm_read(dev, PM_STATUS);
	uint32_t image  = pm_read(dev, PM_IMAGE);
	uint32_t grafx  = pm_read(dev, PM_GRAFX);
	uint32_t proc   = pm_read(dev, PM_PROC);
	uint32_t rstc   = pm_read(dev, 0x1c);  /* PM_RSTC */
	uint32_t gnric  = pm_read(dev, PM_GNRIC);
	uint32_t audio  = pm_read(dev, PM_AUDIO);
	uint32_t pxldo  = pm_read(dev, PM_PXLDO);
	uint32_t pxbg   = pm_read(dev, PM_PXBG);
	uint32_t dft    = pm_read(dev, PM_DFT);
	uint32_t smps   = pm_read(dev, PM_SMPS);
	uint32_t xosc   = pm_read(dev, PM_XOSC);
	uint32_t avs    = pm_read(dev, PM_AVS_STAT);
	uint32_t brdg   = asb_read(dev, ASB_AXI_BRDG_ID);
	uint32_t asb_m  = asb_read(dev, ASB_V3D_M_CTRL);
	uint32_t asb_s  = asb_read(dev, ASB_V3D_S_CTRL);
	uint32_t v3dctl = cm_read(dev, CM_V3DCTL);
	uint32_t ident  = v3d_read(dev, V3D_IDENT0);

	LOG_INF("[%s] PM_domains(STATUS=0x%08x IMAGE=0x%08x GRAFX=0x%08x PROC=0x%08x "
		"GNRIC=0x%08x AUDIO=0x%08x)",
		tag, status, image, grafx, proc, gnric, audio);
	LOG_INF("[%s] PM_regulators(PXLDO=0x%08x PXBG=0x%08x DFT=0x%08x SMPS=0x%08x "
		"XOSC=0x%08x AVS_STAT=0x%08x RSTC=0x%08x)",
		tag, pxldo, pxbg, dft, smps, xosc, avs, rstc);
	LOG_INF("[%s] ASB(BRDG=0x%08x M=0x%08x S=0x%08x) CM_V3DCTL=0x%08x (EN=%d BUSY=%d) "
		"V3D_IDENT0=0x%08x",
		tag, brdg, asb_m, asb_s, v3dctl, !!(v3dctl & CM_ENABLE),
		!!(v3dctl & CM_BUSY), ident);
}

/* Ported from Linux's bcm2835_power_power_on(PM_GRAFX). Brings up the
 * parent GRAFX power domain (which contains V3D + ISP + H264).
 *
 *   1. If already powered (PM_POWUP set), return early.
 *   2. Sweep PM_INRUSH 3.5mA -> 20mA looking for PM_POWOK.
 *   3. De-isolate (set PM_ISPOW), repair RAM (set PM_MEMREP, wait
 *      for PM_MRDONE), enable function (set PM_ISFUNC).
 */
static int power_on_grafx(const struct device *dev)
{
	uint32_t reg = pm_read(dev, PM_GRAFX);

	LOG_INF("power_on_grafx: initial PM_GRAFX=0x%08x "
		"(POWUP=%d POWOK=%d ISPOW=%d MRDONE=%d ISFUNC=%d V3DRSTN=%d)",
		reg,
		!!(reg & PM_POWUP), !!(reg & PM_POWOK), !!(reg & PM_ISPOW),
		!!(reg & PM_MRDONE), !!(reg & PM_ISFUNC), !!(reg & PM_V3DRSTN));

	if (reg & PM_POWUP) {
		LOG_INF("GRAFX already powered");
		return 0;
	}

	bool powok = false;

	for (uint32_t inrush = PM_INRUSH_3_5MA; inrush <= PM_INRUSH_20MA; inrush++) {
		reg = pm_read(dev, PM_GRAFX);
		reg = (reg & ~PM_INRUSH_MASK) | (inrush << PM_INRUSH_SHIFT) | PM_POWUP;
		pm_write(dev, PM_GRAFX, reg);

		/* Give the silicon a full 3 ms per inrush level -- BCM2710
		 * PM controller has been seen far slower than Linux's
		 * 3 us assumption for Pi 1 silicon.
		 */
		uint32_t start = k_cycle_get_32();
		uint32_t cps   = (uint32_t)sys_clock_hw_cycles_per_sec();
		uint32_t limit = (cps * 3) / 1000U;   /* 3 ms */

		while (!(powok = (pm_read(dev, PM_GRAFX) & PM_POWOK) != 0)) {
			if ((k_cycle_get_32() - start) > limit) {
				break;
			}
		}
		LOG_INF("inrush %u: PM_GRAFX=0x%08x powok=%d", inrush,
			pm_read(dev, PM_GRAFX), powok);
		if (powok) {
			break;
		}
	}

	if (!powok) {
		LOG_ERR("GRAFX power: POWOK never asserted (PM_GRAFX=0x%08x)",
			pm_read(dev, PM_GRAFX));
		/* Back the POWUP out so we leave the domain in a known state. */
		pm_write(dev, PM_GRAFX,
			 pm_read(dev, PM_GRAFX) & ~(PM_POWUP | PM_INRUSH_MASK));
		return -ETIMEDOUT;
	}

	/* Disable electrical isolation. */
	pm_write(dev, PM_GRAFX, pm_read(dev, PM_GRAFX) | PM_ISPOW);

	/* Repair memory + wait MRDONE (1 us in Linux). */
	pm_write(dev, PM_GRAFX, pm_read(dev, PM_GRAFX) | PM_MEMREP);
	{
		uint32_t start = k_cycle_get_32();
		uint32_t cps   = (uint32_t)sys_clock_hw_cycles_per_sec();
		uint32_t limit = cps / 1000000U;  /* 1 us */

		while (!(pm_read(dev, PM_GRAFX) & PM_MRDONE)) {
			if ((k_cycle_get_32() - start) > limit) {
				LOG_ERR("GRAFX memory repair timed out");
				pm_write(dev, PM_GRAFX,
					 pm_read(dev, PM_GRAFX) & ~PM_ISPOW);
				return -ETIMEDOUT;
			}
		}
	}

	/* Disable functional isolation. */
	pm_write(dev, PM_GRAFX, pm_read(dev, PM_GRAFX) | PM_ISFUNC);

	LOG_INF("GRAFX powered on (PM_GRAFX=0x%08x)", pm_read(dev, PM_GRAFX));
	return 0;
}

/* Ported from Linux's bcm2835_asb_power_on(PM_GRAFX, ASB_V3D_M_CTRL,
 * ASB_V3D_S_CTRL, PM_V3DRSTN). Brings up V3D as a sub-domain of GRAFX
 * by deasserting the reset and routing ASB traffic to/from V3D.
 */
static int asb_power_on_v3d(const struct device *dev)
{
	int rc;

	/* Deassert V3D reset. */
	pm_write(dev, PM_GRAFX, pm_read(dev, PM_GRAFX) | PM_V3DRSTN);
	k_busy_wait(1);

	rc = asb_enable(dev, ASB_V3D_M_CTRL);
	if (rc) {
		LOG_ERR("asb_enable(V3D_M_CTRL) failed: %d", rc);
		goto rearm_reset;
	}
	rc = asb_enable(dev, ASB_V3D_S_CTRL);
	if (rc) {
		LOG_ERR("asb_enable(V3D_S_CTRL) failed: %d", rc);
		goto rearm_reset;
	}

	LOG_INF("V3D ASB bridges enabled (PM_GRAFX=0x%08x, ASB_M=0x%08x, ASB_S=0x%08x)",
		pm_read(dev, PM_GRAFX),
		asb_read(dev, ASB_V3D_M_CTRL),
		asb_read(dev, ASB_V3D_S_CTRL));
	return 0;

rearm_reset:
	pm_write(dev, PM_GRAFX, pm_read(dev, PM_GRAFX) & ~PM_V3DRSTN);
	return rc;
}

/* ------------------------------------------------------------------ *
 *  Firmware-mailbox memory primitives (mailbox tags from rpi_fw.h)
 * ------------------------------------------------------------------ */

/* Clock IDs from the firmware mailbox spec; only V3D is used here. */
#define FW_CLOCK_ID_V3D 5u

/* Returns 0 on success. *state_out (if non-NULL) gets the response
 * state word: bit 0 = on, bit 1 = no-such-device, bit 2 = not-running.
 */
static int fw_get_clock_state(const struct device *fw, uint32_t clk_id,
                              uint32_t *state_out)
{
	uint32_t buf[2] = { clk_id, 0 };
	int rc = rpi_fw_transfer(fw, RPI_FW_TAG_GET_CLOCK_STATE, buf, sizeof(buf));

	if (rc == 0 && state_out) {
		*state_out = buf[1];
	}
	return rc;
}

static int fw_set_clock_state(const struct device *fw, uint32_t clk_id, bool on)
{
	uint32_t buf[2] = { clk_id, on ? 1u : 0u };

	return rpi_fw_transfer(fw, RPI_FW_TAG_SET_CLOCK_STATE, buf, sizeof(buf));
}

static int fw_set_qpu_enable(const struct device *fw, uint32_t enable)
{
	uint32_t buf = enable;

	return rpi_fw_transfer(fw, RPI_FW_TAG_SET_ENABLE_QPU, &buf, sizeof(buf));
}

static int fw_alloc_mem(const struct device *fw, uint32_t size, uint32_t align,
                        uint32_t flags, uint32_t *handle)
{
	uint32_t buf[3] = { size, align, flags };
	int rc = rpi_fw_transfer(fw, RPI_FW_TAG_ALLOCATE_MEMORY, buf, sizeof(buf));

	if (rc) {
		return rc;
	}
	*handle = buf[0];
	return *handle ? 0 : -ENOMEM;
}

static int fw_lock_mem(const struct device *fw, uint32_t handle, uint32_t *bus_addr)
{
	uint32_t buf = handle;
	int rc = rpi_fw_transfer(fw, RPI_FW_TAG_LOCK_MEMORY, &buf, sizeof(buf));

	if (rc) {
		return rc;
	}
	*bus_addr = buf;
	return *bus_addr ? 0 : -EIO;
}

static int fw_unlock_mem(const struct device *fw, uint32_t handle)
{
	uint32_t buf = handle;

	return rpi_fw_transfer(fw, RPI_FW_TAG_UNLOCK_MEMORY, &buf, sizeof(buf));
}

static int fw_release_mem(const struct device *fw, uint32_t handle)
{
	uint32_t buf = handle;

	return rpi_fw_transfer(fw, RPI_FW_TAG_RELEASE_MEMORY, &buf, sizeof(buf));
}

/* ------------------------------------------------------------------ *
 *  Public API (stubs land here; bring-up commits flesh them out)
 * ------------------------------------------------------------------ */

int bcm2835_v3d_alloc_coherent(const struct device *dev,
                               uint32_t size, uint32_t align, uint32_t mem_flag,
                               uintptr_t *bus_addr, void **cpu_addr,
                               uint32_t *handle)
{
	const struct bcm2835_v3d_config *cfg = dev->config;
	uint32_t h;
	uint32_t bus;
	int rc;

	rc = fw_alloc_mem(cfg->fw, size, align, mem_flag, &h);
	if (rc) {
		LOG_ERR("alloc_mem(size=%u, flags=0x%x) failed: %d", size, mem_flag, rc);
		return rc;
	}

	rc = fw_lock_mem(cfg->fw, h, &bus);
	if (rc) {
		LOG_ERR("lock_mem(handle=0x%x) failed: %d", h, rc);
		(void)fw_release_mem(cfg->fw, h);
		return rc;
	}

	if (handle) {
		*handle = h;
	}
	if (bus_addr) {
		*bus_addr = bus;
	}

	if (cpu_addr) {
		/* Map the ARM-physical alias of this bus address into ARM
		 * virtual as device memory (cache-none) so ARM reads/writes
		 * are coherent with the QPU without per-access cache flushes.
		 */
		uint8_t *va = NULL;
		uint32_t phys = bus & V3D_BUS_TO_PHYS_MASK;

		device_map((mm_reg_t *)&va, phys, size, K_MEM_CACHE_NONE);
		if (!va) {
			(void)fw_unlock_mem(cfg->fw, h);
			(void)fw_release_mem(cfg->fw, h);
			return -ENOMEM;
		}
		*cpu_addr = va;
	}

	LOG_DBG("alloc: handle=0x%x bus=0x%x size=%u", h, bus, size);
	return 0;
}

int bcm2835_v3d_free_coherent(const struct device *dev, uint32_t handle)
{
	const struct bcm2835_v3d_config *cfg = dev->config;
	int rc1 = fw_unlock_mem(cfg->fw, handle);
	int rc2 = fw_release_mem(cfg->fw, handle);

	/* Note: ARM-side virtual mapping intentionally leaked for now --
	 * Zephyr's k_mem_unmap is for k_mem_map regions, not device_map.
	 * For long-lived QPU kernels this is fine; revisit if we start
	 * allocating/freeing kernels per-frame.
	 */
	return rc1 ? rc1 : rc2;
}

/* ------------------------------------------------------------------ *
 *  Kernel / launch stubs -- implemented in the next bring-up commit
 * ------------------------------------------------------------------ */

int bcm2835_v3d_kernel_init(const struct device *dev,
                            struct bcm2835_v3d_kernel *k,
                            uint32_t num_qpus, uint32_t num_unifs,
                            const uint32_t *code, uint32_t code_size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(k);
	ARG_UNUSED(num_qpus);
	ARG_UNUSED(num_unifs);
	ARG_UNUSED(code);
	ARG_UNUSED(code_size);
	return -ENOSYS;
}

int bcm2835_v3d_kernel_free(const struct device *dev, struct bcm2835_v3d_kernel *k)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(k);
	return -ENOSYS;
}

void bcm2835_v3d_kernel_reset_unifs(struct bcm2835_v3d_kernel *k)
{
	ARG_UNUSED(k);
}

int bcm2835_v3d_kernel_load_unif_u32(struct bcm2835_v3d_kernel *k, uint32_t qpu, uint32_t val)
{
	ARG_UNUSED(k);
	ARG_UNUSED(qpu);
	ARG_UNUSED(val);
	return -ENOSYS;
}

int bcm2835_v3d_kernel_load_unif_f32(struct bcm2835_v3d_kernel *k, uint32_t qpu, float val)
{
	ARG_UNUSED(k);
	ARG_UNUSED(qpu);
	ARG_UNUSED(val);
	return -ENOSYS;
}

int bcm2835_v3d_kernel_execute(const struct device *dev, struct bcm2835_v3d_kernel *k)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(k);
	return -ENOSYS;
}

int bcm2835_v3d_kernel_execute_async(const struct device *dev, struct bcm2835_v3d_kernel *k)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(k);
	return -ENOSYS;
}

int bcm2835_v3d_kernel_wait(const struct device *dev, struct bcm2835_v3d_kernel *k,
                            uint32_t timeout_us)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(k);
	ARG_UNUSED(timeout_us);
	return -ENOSYS;
}

/* ------------------------------------------------------------------ *
 *  Probe
 * ------------------------------------------------------------------ */

static int bcm2835_v3d_init(const struct device *dev)
{
	const struct bcm2835_v3d_config *cfg = dev->config;
	struct bcm2835_v3d_data *d = dev->data;
	uint32_t ident;
	int rc;

	if (!device_is_ready(cfg->fw)) {
		LOG_ERR("rpi_fw device not ready");
		return -ENODEV;
	}

	LOG_INF("init: V3D phys=0x%lx (from DT)", (unsigned long)cfg->v3d_phys);

	/* Bring up the V3D clock via the firmware mailbox. The PM
	 * controller's POWOK signal depends on the clock being running.
	 * (Domain-state and enable-QPU mailbox tags don't work on
	 * BCM2710 -- their handlers are stubbed in this firmware --
	 * but the clock-rate path does what it says.)
	 */
	{
		uint32_t cr[2] = { 5 /* CLK_V3D */, 0 };
		rc = rpi_fw_transfer(cfg->fw, RPI_FW_TAG_GET_MAX_CLOCK_RATE, cr, sizeof(cr));
		if (rc) {
			LOG_ERR("GET_MAX_CLOCK_RATE(V3D) failed: %d", rc);
			return rc;
		}
		uint32_t v3d_max_hz = cr[1];
		LOG_INF("GET_MAX_CLOCK_RATE(V3D=5) -> %u Hz", v3d_max_hz);
		if (v3d_max_hz == 0) {
			LOG_ERR("V3D max clock reported as 0");
			return -ENOTSUP;
		}

		uint32_t sr[3] = { 5 /* CLK_V3D */, v3d_max_hz, 0 /* skip_turbo=0 */ };
		rc = rpi_fw_transfer(cfg->fw, RPI_FW_TAG_SET_CLOCK_RATE, sr, sizeof(sr));
		if (rc) {
			LOG_ERR("SET_CLOCK_RATE(V3D, %u) failed: %d", v3d_max_hz, rc);
			return rc;
		}
		LOG_INF("SET_CLOCK_RATE(V3D, %u) -> achieved %u Hz", v3d_max_hz, sr[1]);
	}

	/* Map the V3D + PM + ASB register blocks as device memory. On
	 * AArch64 Zephyr, K_MEM_CACHE_NONE resolves to MT_DEVICE_nGnRnE
	 * (see arch/arm64/core/mmu.c, K_MEM_CACHE_NONE switch case) --
	 * strict ordering, no merging, no speculation. That's what the
	 * BCM2710 PM controller's password-gated writes need.
	 */
	device_map((mm_reg_t *)&d->v3d_regs, cfg->v3d_phys, 0x1000, K_MEM_CACHE_NONE);
	device_map((mm_reg_t *)&d->pm_regs,  V3D_PM_PHYS,  V3D_PM_SIZE,  K_MEM_CACHE_NONE);
	device_map((mm_reg_t *)&d->asb_regs, V3D_ASB_PHYS, V3D_ASB_SIZE, K_MEM_CACHE_NONE);
	device_map((mm_reg_t *)&d->cm_regs,  V3D_CM_PHYS,  V3D_CM_SIZE,  K_MEM_CACHE_NONE);
	if (!d->v3d_regs || !d->pm_regs || !d->asb_regs || !d->cm_regs) {
		LOG_ERR("device_map failed: v3d=%p pm=%p asb=%p cm=%p",
			d->v3d_regs, d->pm_regs, d->asb_regs, d->cm_regs);
		return -ENOMEM;
	}
	LOG_INF("device_map ok: v3d=%p pm=%p asb=%p cm=%p",
		d->v3d_regs, d->pm_regs, d->asb_regs, d->cm_regs);

	/* Snapshot pre-anything register state. The ASB BRDG ID read is
	 * Linux's sanity check (bcm2835_power_probe in
	 * drivers/pmdomain/bcm/bcm2835-power.c) -- it must be 0x62726467
	 * ("brdg"). If it isn't, our ASB MMIO mapping is wrong and every
	 * "POWOK never asserts" debug round is wasted.
	 */
	dump_state(dev, "pre-clk");
	{
		uint32_t brdg = asb_read(dev, ASB_AXI_BRDG_ID);

		if (brdg != ASB_BRDG_ID_MAGIC) {
			LOG_ERR("ASB BRDG ID mismatch: got 0x%08x, expected 0x%08x "
				"(=\"brdg\"). ASB MMIO mapping or address is wrong.",
				brdg, ASB_BRDG_ID_MAGIC);
			/* Not fatal -- continue so we can see the full snapshot
			 * sequence, but the rest of bring-up is meaningless.
			 */
		} else {
			LOG_INF("ASB BRDG ID OK (0x%08x = \"brdg\")", brdg);
		}
	}

	/* Firmware boots BCM2710 with the V3D clock already enabled
	 * (GET_CLOCK_STATE returns on=1 and CM_V3DCTL.CM_ENABLE is set
	 * at probe), but the GRAFX domain off. Linux's flow has the
	 * opposite invariant: the V3D clock is enabled only inside
	 * bcm2835_asb_power_on, AFTER bcm2835_power_power_on(PM_GRAFX)
	 * succeeds. We've been violating that by leaving the clock on
	 * while attempting to ramp GRAFX -- and POWOK never asserts.
	 *
	 * Test the hypothesis: gate the V3D clock, power up GRAFX,
	 * re-enable the clock for the ASB step.
	 */
	{
		uint32_t state = 0;
		int rc2 = fw_get_clock_state(cfg->fw, FW_CLOCK_ID_V3D, &state);

		if (rc2 == 0) {
			LOG_INF("GET_CLOCK_STATE(V3D) -> 0x%08x "
				"(on=%d no-device=%d not-running=%d)",
				state, state & 1, !!(state & 2), !!(state & 4));
		}
	}

	/* Ask firmware to gate the V3D clock. */
	rc = fw_set_clock_state(cfg->fw, FW_CLOCK_ID_V3D, false);
	if (rc) {
		LOG_WRN("SET_CLOCK_STATE(V3D, off) failed: %d (continuing)", rc);
	} else {
		LOG_INF("SET_CLOCK_STATE(V3D, off) ok");
	}
	k_busy_wait(10);

	/* If firmware refused to disable the clock, force-gate it
	 * directly via the cprman register (CM_PASSWORD-protected).
	 */
	{
		uint32_t v = cm_read(dev, CM_V3DCTL);

		if (v & CM_ENABLE) {
			LOG_WRN("CM_V3DCTL still has CM_ENABLE set after firmware "
				"request (0x%08x); force-gating directly", v);
			cm_write(dev, CM_V3DCTL, v & ~CM_ENABLE);

			/* BUSY remains high until the divider completes its
			 * cycle (Linux comment). At 300 MHz a few microseconds
			 * is more than enough.
			 */
			uint32_t start = k_cycle_get_32();
			uint32_t cps   = (uint32_t)sys_clock_hw_cycles_per_sec();

			while (cm_read(dev, CM_V3DCTL) & CM_BUSY) {
				if ((k_cycle_get_32() - start) > (cps / 1000)) {
					LOG_WRN("CM_BUSY didn't clear within 1 ms");
					break;
				}
			}
		}
	}
	dump_state(dev, "clk-off");

	/* The firmware mailbox path turned out to be a no-op for V3D on
	 * BCM2710 -- the firmware silently drops the power tag. Linux's
	 * bcm2835-power.c brings V3D up by manipulating the PM + ASB
	 * registers directly; this is the port of that sequence.
	 *
	 *   1. Power on the parent GRAFX domain (PM_GRAFX register).
	 *   2. Deassert PM_V3DRSTN reset + enable V3D ASB bridges.
	 *
	 * After step 2 the V3D peripheral comes out of reset and IDENT0
	 * should read the magic.
	 */
	rc = power_on_grafx(dev);
	if (rc) {
		dump_state(dev, "powok-fail");
		return rc;
	}
	dump_state(dev, "post-grafx");

	/* Now that GRAFX is up, re-enable the V3D clock for the ASB step. */
	rc = fw_set_clock_state(cfg->fw, FW_CLOCK_ID_V3D, true);
	if (rc) {
		LOG_ERR("SET_CLOCK_STATE(V3D, on) [post-grafx] failed: %d", rc);
		return rc;
	}
	LOG_INF("SET_CLOCK_STATE(V3D, on) [post-grafx] ok");

	/* If firmware didn't restore the enable bit, force it back on. */
	{
		uint32_t v = cm_read(dev, CM_V3DCTL);

		if (!(v & CM_ENABLE)) {
			LOG_WRN("CM_V3DCTL not re-enabled by firmware (0x%08x); "
				"force-enabling directly", v);
			cm_write(dev, CM_V3DCTL, v | CM_ENABLE);
		}
	}
	k_busy_wait(10);

	rc = asb_power_on_v3d(dev);
	if (rc) {
		dump_state(dev, "asb-fail");
		return rc;
	}
	dump_state(dev, "post-asb");

	/* Now that V3D is alive on the bus, enable the QPU scheduler
	 * queue via the firmware. (This tag was a no-op when V3D was
	 * dead; it should actually work now.)
	 */
	rc = fw_set_qpu_enable(cfg->fw, 1);
	if (rc) {
		LOG_ERR("SET_ENABLE_QPU(1) failed: %d", rc);
		return rc;
	}
	LOG_INF("SET_ENABLE_QPU(1) ok");

	/* Probe diagnostics: raw reads from low offsets across the V3D
	 * window. If all are 0xdeadbeef (or all 0x00000000, or all
	 * 0xffffffff), the address isn't a live peripheral; if values
	 * differ register-by-register, V3D is mapped, just powered down
	 * or different semantics than expected.
	 */
	LOG_INF("probe: +0x00=0x%08x +0x04=0x%08x +0x08=0x%08x +0x20=0x%08x +0x430=0x%08x",
		v3d_read(dev, 0x00), v3d_read(dev, 0x04), v3d_read(dev, 0x08),
		v3d_read(dev, V3D_L2CACTL), v3d_read(dev, V3D_SRQPC));

	/* Sanity: read a few other peripheral-region addresses to prove
	 * our mapping path isn't poisoning. We expect non-deadbeef values
	 * at the mailbox base (0x3f00b880) and the system timer
	 * (0x3f003000), both known-good in this Zephyr config.
	 */
	{
		mm_reg_t mbox_va = 0, st_va = 0;

		device_map(&mbox_va, 0x3f00b880, 0x40, K_MEM_CACHE_NONE);
		device_map(&st_va,   0x3f003000, 0x40, K_MEM_CACHE_NONE);

		LOG_INF("known-good probe: mbox@0x3f00b880 first u32=0x%08x; "
			"sys_timer@0x3f003000 first u32=0x%08x",
			mbox_va ? sys_read32(mbox_va) : 0xBAD,
			st_va   ? sys_read32(st_va)   : 0xBAD);
	}

	/* Write/read sanity check on V3D_L2CACTL. The L2 cache control
	 * register has writable bits (bit 2 = clear L2 cache); on a real
	 * V3D, writing 0x4 and reading back should give us something
	 * non-deadbeef, even if the bit auto-clears.
	 */
	v3d_write(dev, V3D_L2CACTL, 0x4);
	uint32_t l2_after = v3d_read(dev, V3D_L2CACTL);

	LOG_INF("write/read L2CACTL: wrote 0x4, read 0x%08x", l2_after);

	/* Verify V3D came up: V3D_IDENT0 should read "V3D\x02"
	 * (= 0x02334456, low byte = 'V').
	 */
	ident = v3d_read(dev, V3D_IDENT0);
	if (ident != V3D_IDENT0_MAGIC) {
		LOG_ERR("V3D_IDENT0 = 0x%08x (expected 0x%08x); QPU power-on may have failed",
			ident, V3D_IDENT0_MAGIC);
		return -EIO;
	}

	LOG_INF("V3D up: IDENT0=0x%08x IDENT1=0x%08x IDENT2=0x%08x",
		ident, v3d_read(dev, V3D_IDENT1), v3d_read(dev, V3D_IDENT2));

	return 0;
}

/* ------------------------------------------------------------------ *
 *  Instance plumbing
 * ------------------------------------------------------------------ */

#define BCM2835_V3D_INIT(n)                                                                        \
	static const struct bcm2835_v3d_config bcm2835_v3d_config_##n = {                          \
		.v3d_phys = DT_INST_REG_ADDR(n),                                                   \
		.fw       = DEVICE_DT_GET(DT_INST_PHANDLE(n, firmware)),                           \
	};                                                                                         \
	static struct bcm2835_v3d_data bcm2835_v3d_data_##n;                                        \
	DEVICE_DT_INST_DEFINE(n, bcm2835_v3d_init, NULL,                                            \
			      &bcm2835_v3d_data_##n, &bcm2835_v3d_config_##n,                      \
			      POST_KERNEL, CONFIG_BCM2835_V3D_INIT_PRIORITY,                        \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(BCM2835_V3D_INIT)
