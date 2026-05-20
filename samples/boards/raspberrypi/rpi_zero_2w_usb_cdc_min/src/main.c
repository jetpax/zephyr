/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal CDC ACM demo for rpi_zero_2w.
 *
 * The USB stack initializes itself at SYS_INIT (we set
 * CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=y) and the
 * `zephyr,cdc-acm-uart` DT node is bound by the class driver to look
 * like a regular UART. The app code is just a write loop: it pushes
 * a counted "hello USB" line every 500 ms via the standard UART
 * poll-out API. The CDC class translates that into bulk-IN packets
 * on the chip's endpoint 1 IN; the host PC sees them on its
 * /dev/cu.usbmodem* (macOS) / /dev/ttyACM0 (Linux) / COMn (Windows).
 *
 * For register-level diagnostics of the DWC2 controller, see the
 * sibling `rpi_zero_2w_dwc2_probe` sample.
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_cdc_min, LOG_LEVEL_INF);

static const struct device *const cdc_dev =
	DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

int main(void)
{
	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM device not ready -- driver bind failed");
		return -1;
	}

	LOG_INF("CDC ACM device ready: %s", cdc_dev->name);

	for (unsigned int n = 0; ; n++) {
		char buf[64];
		int len = snprintf(buf, sizeof(buf),
				   "hello USB #%u\r\n", n);

		for (int i = 0; i < len; i++) {
			uart_poll_out(cdc_dev, buf[i]);
		}
		k_msleep(500);
	}

	return 0;
}
