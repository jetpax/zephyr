/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner H616/H618 CCU reset lines. The CCU keeps each block's
 * reset in the same BGR register as its bus gate (reset at bit 16 + n,
 * active low: bit set = block out of reset). Line map mirrors Linux
 * drivers/clk/sunxi-ng/ccu-sun50i-h616.c.
 *
 * Contract with the sibling clock driver: a block's bring-up (reset
 * deassert, then gate on) is a single consumer's init sequence; the
 * two drivers do not serialize their read-modify-writes against each
 * other, so concurrent bring-up of two blocks sharing one BGR register
 * from different threads is not supported.
 */

#define DT_DRV_COMPAT allwinner_sun50i_h616_ccu_reset

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/reset/sun50i-h616-ccu.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct ccu_reset_line {
	uint16_t reg;
	uint8_t bit;
};

#define CCU_RST(_id, _reg, _bit) [_id] = { .reg = _reg, .bit = _bit }

static const struct ccu_reset_line ccu_resets[] = {
	CCU_RST(RST_MBUS,		0x540, 30),
	CCU_RST(RST_BUS_DE,		0x60c, 16),
	CCU_RST(RST_BUS_DEINTERLACE,	0x62c, 16),
	CCU_RST(RST_BUS_GPU,		0x67c, 16),
	CCU_RST(RST_BUS_CE,		0x68c, 16),
	CCU_RST(RST_BUS_VE,		0x69c, 16),
	CCU_RST(RST_BUS_DMA,		0x70c, 16),
	CCU_RST(RST_BUS_HSTIMER,	0x73c, 16),
	CCU_RST(RST_BUS_DBG,		0x78c, 16),
	CCU_RST(RST_BUS_PSI,		0x79c, 16),
	CCU_RST(RST_BUS_PWM,		0x7ac, 16),
	CCU_RST(RST_BUS_IOMMU,		0x7bc, 16),
	CCU_RST(RST_BUS_DRAM,		0x80c, 16),
	CCU_RST(RST_BUS_NAND,		0x82c, 16),
	CCU_RST(RST_BUS_MMC0,		0x84c, 16),
	CCU_RST(RST_BUS_MMC1,		0x84c, 17),
	CCU_RST(RST_BUS_MMC2,		0x84c, 18),
	CCU_RST(RST_BUS_UART0,		0x90c, 16),
	CCU_RST(RST_BUS_UART1,		0x90c, 17),
	CCU_RST(RST_BUS_UART2,		0x90c, 18),
	CCU_RST(RST_BUS_UART3,		0x90c, 19),
	CCU_RST(RST_BUS_UART4,		0x90c, 20),
	CCU_RST(RST_BUS_UART5,		0x90c, 21),
	CCU_RST(RST_BUS_I2C0,		0x91c, 16),
	CCU_RST(RST_BUS_I2C1,		0x91c, 17),
	CCU_RST(RST_BUS_I2C2,		0x91c, 18),
	CCU_RST(RST_BUS_I2C3,		0x91c, 19),
	CCU_RST(RST_BUS_I2C4,		0x91c, 20),
	CCU_RST(RST_BUS_SPI0,		0x96c, 16),
	CCU_RST(RST_BUS_SPI1,		0x96c, 17),
	CCU_RST(RST_BUS_EMAC0,		0x97c, 16),
	CCU_RST(RST_BUS_EMAC1,		0x97c, 17),
	CCU_RST(RST_BUS_TS,		0x9bc, 16),
	CCU_RST(RST_BUS_THS,		0x9fc, 16),
	CCU_RST(RST_BUS_SPDIF,		0xa2c, 16),
	CCU_RST(RST_BUS_DMIC,		0xa4c, 16),
	CCU_RST(RST_BUS_AUDIO_CODEC,	0xa5c, 16),
	CCU_RST(RST_BUS_AUDIO_HUB,	0xa6c, 16),
	CCU_RST(RST_USB_PHY0,		0xa70, 30),
	CCU_RST(RST_USB_PHY1,		0xa74, 30),
	CCU_RST(RST_USB_PHY2,		0xa78, 30),
	CCU_RST(RST_USB_PHY3,		0xa7c, 30),
	CCU_RST(RST_BUS_OHCI0,		0xa8c, 16),
	CCU_RST(RST_BUS_OHCI1,		0xa8c, 17),
	CCU_RST(RST_BUS_OHCI2,		0xa8c, 18),
	CCU_RST(RST_BUS_OHCI3,		0xa8c, 19),
	CCU_RST(RST_BUS_EHCI0,		0xa8c, 20),
	CCU_RST(RST_BUS_EHCI1,		0xa8c, 21),
	CCU_RST(RST_BUS_EHCI2,		0xa8c, 22),
	CCU_RST(RST_BUS_EHCI3,		0xa8c, 23),
	CCU_RST(RST_BUS_OTG,		0xa8c, 24),
	CCU_RST(RST_BUS_KEYADC,		0xa9c, 16),
	CCU_RST(RST_BUS_HDMI,		0xb1c, 16),
	CCU_RST(RST_BUS_HDMI_SUB,	0xb1c, 17),
	CCU_RST(RST_BUS_TCON_TOP,	0xb5c, 16),
	CCU_RST(RST_BUS_TCON_TV0,	0xb9c, 16),
	CCU_RST(RST_BUS_TCON_TV1,	0xb9c, 17),
	CCU_RST(RST_BUS_TVE_TOP,	0xbbc, 16),
	CCU_RST(RST_BUS_TVE0,		0xbbc, 17),
	CCU_RST(RST_BUS_HDCP,		0xc4c, 16),
};

struct ccu_reset_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
};

static int ccu_reset_line(const struct device *dev, uint32_t id, bool assert)
{
	struct ccu_reset_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	const struct ccu_reset_line *line;
	k_spinlock_key_t key;
	uint32_t val;

	if (id >= ARRAY_SIZE(ccu_resets) || ccu_resets[id].reg == 0) {
		return -EINVAL;
	}
	line = &ccu_resets[id];

	key = k_spin_lock(&data->lock);
	val = sys_read32(base + line->reg);
	if (assert) {
		val &= ~BIT(line->bit);
	} else {
		val |= BIT(line->bit);
	}
	sys_write32(val, base + line->reg);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ccu_reset_status(const struct device *dev, uint32_t id,
			    uint8_t *status)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);

	if (id >= ARRAY_SIZE(ccu_resets) || ccu_resets[id].reg == 0) {
		return -EINVAL;
	}
	*status = (sys_read32(base + ccu_resets[id].reg) &
		   BIT(ccu_resets[id].bit)) ? 0 : 1;
	return 0;
}

static int ccu_reset_assert(const struct device *dev, uint32_t id)
{
	return ccu_reset_line(dev, id, true);
}

static int ccu_reset_deassert(const struct device *dev, uint32_t id)
{
	return ccu_reset_line(dev, id, false);
}

static int ccu_reset_toggle(const struct device *dev, uint32_t id)
{
	int ret = ccu_reset_assert(dev, id);

	if (ret == 0) {
		ret = ccu_reset_deassert(dev, id);
	}
	return ret;
}

static int ccu_reset_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(reset, ccu_reset_api) = {
	.status = ccu_reset_status,
	.line_assert = ccu_reset_assert,
	.line_deassert = ccu_reset_deassert,
	.line_toggle = ccu_reset_toggle,
};

#define CCU_RESET_INIT(inst)						\
	static struct ccu_reset_data ccu_reset_data_##inst;		\
	static const struct {						\
		DEVICE_MMIO_ROM;					\
	} ccu_reset_config_##inst = {					\
		DEVICE_MMIO_ROM_INIT(DT_INST_PARENT(inst)),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, ccu_reset_init, NULL,		\
			      &ccu_reset_data_##inst,			\
			      &ccu_reset_config_##inst, PRE_KERNEL_1,	\
			      CONFIG_RESET_INIT_PRIORITY,		\
			      &ccu_reset_api);

DT_INST_FOREACH_STATUS_OKAY(CCU_RESET_INIT)
