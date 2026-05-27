/*
 * Copyright (c) 2026 Jonathan Elliot Peace <jep@alphabetiq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * PiZZa -- Zephyr on the Raspberry Pi Zero 2 W.
 * Prints a boot banner on the console and registers a `pizza` shell
 * command set on the shell UART (USB CDC ACM by default).
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/usb/usbd.h>
#include <sample_usbd.h>
#include <stdio.h>
#include <string.h>

#define PIZZA_VERSION "v0.1-preview"

/*
 * Two-line header lines reused everywhere a "hello" is needed.
 *   - Bold green title.
 *   - One-line help nudge.
 * ASCII art was dropped because of font-kerning rendering quirks (logo
 * cells aren't strict monospace in some host terminals); the neofetch
 * `pizza` command keeps the peace logo where the layout is fixed by
 * explicit cursor positioning.
 */
#define BANNER_TITLE \
	"\x1b[1;32mPiZZa " PIZZA_VERSION " -- Zephyr on the Raspberry Pi Zero 2 W\x1b[0m\r\n"
#define BANNER_HELP "Type 'help' for more information.\r\n"

/* Boot-time and shell-command form: no screen clear, no fake prompt. */
static const char banner[] = "\r\n" BANNER_TITLE BANNER_HELP;

/*
 * CDC-connect form: clear screen, title + help, then a "uart:~$ "
 * placeholder in bold green that matches the Zephyr shell's real
 * prompt colour (SHELL_INFO -> SHELL_VT100_COLOR_GREEN). The first
 * keystroke from the host hands control back to the real shell --
 * which immediately re-renders ITS prompt in the same colour at the
 * same position, so the seam is invisible.
 *
 * Why fake: the shell's TX path can only be safely driven from the
 * shell's own thread or from a shell-command callback context. Calls
 * from the system workqueue block forever (K_FOREVER lock + flush on
 * a TX queue we just hammered with uart_poll_out).
 */
static const char banner_cdc[] =
	"\x1b[2J\x1b[H" BANNER_TITLE BANNER_HELP "\x1b[1;32muart:~$ \x1b[0m";

static int cmd_pizza_welcome(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	shell_fprintf(sh, SHELL_NORMAL, "%s", banner);
	return 0;
}

/*
 * USBD message callback. Fires on host CDC ACM state changes. Two
 * triggers for "host opened the port", debounced so we don't double-fire:
 *
 *   1. USBD_MSG_CDC_ACM_CONTROL_LINE_STATE with DTR=1
 *        - Linux/Windows hosts that send SET_CONTROL_LINE_STATE.
 *   2. USBD_MSG_CDC_ACM_LINE_CODING (any tcsetattr / baud-rate set)
 *        - macOS, where SET_CONTROL_LINE_STATE is often skipped but
 *          SET_LINE_CODING is always sent on open.
 *
 * The work item runs at +300ms so the banner doesn't race the host's
 * read pipeline. The banner is written byte-by-byte via uart_poll_out
 * directly to the CDC UART -- NOT through the shell, because
 * shell_fprintf called from a non-shell thread blocks in this driver's
 * TX path.
 */
static void welcome_write_banner(void)
{
	const struct device *cdc = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (!device_is_ready(cdc)) {
		return;
	}
	for (const char *p = banner_cdc; *p; p++) {
		uart_poll_out(cdc, (uint8_t)*p);
	}
}

static void welcome_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	static int64_t last_fire_ms;
	int64_t now = k_uptime_get();
	if (now - last_fire_ms < 500) {
		printk("[pizza] welcome debounced (%lld ms since last)\n",
		       now - last_fire_ms);
		return;
	}
	last_fire_ms = now;

	printk("[pizza] writing banner to CDC...\n");
	welcome_write_banner();
	printk("[pizza] banner write complete\n");
}
static K_WORK_DELAYABLE_DEFINE(welcome_work, welcome_work_fn);

static void pizza_usbd_msg_cb(struct usbd_context *const ctx,
			      const struct usbd_msg *const msg)
{
	ARG_UNUSED(ctx);

	if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;
		(void)uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		printk("[pizza] CDC line-state: DTR=%u\n", dtr);
		if (dtr) {
			k_work_reschedule(&welcome_work, K_MSEC(300));
		}
	} else if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		printk("[pizza] CDC line-coding (host opened port)\n");
		k_work_reschedule(&welcome_work, K_MSEC(300));
	}
}

int main(void)
{
	/* First print: mini-UART console (board logs path). */
	printk("%s", banner);

	/* Bring up USBD with a msg_cb so we react to CDC DTR. */
	struct usbd_context *usbd = sample_usbd_init_device(pizza_usbd_msg_cb);
	if (usbd == NULL) {
		printk("[pizza] sample_usbd_init_device failed\n");
		return 0;
	}
	if (!usbd_can_detect_vbus(usbd)) {
		int err = usbd_enable(usbd);
		if (err) {
			printk("[pizza] usbd_enable failed: %d\n", err);
			return 0;
		}
	}
	return 0;
}

/* ----------------------------------------------------------------- *
 *  `pizza` shell command set
 * ----------------------------------------------------------------- */

/*
 * Peace-symbol ASCII art -- repurposed from
 * pizza-fs/lib/sys/utils.py::neofetch() (jetpax = pax = peace).
 * Rendered in 256-colour purple bold so it pops in tio / xterm.js.
 */
#define NF_LOGO_WIDTH  29
static const char nf_logo[] =
"\r\n\x1b[38;5;135;1m"
"        -+#%@@@%#+-      \r\n"
"      %@@@@@@@@@@@@@%    \r\n"
"    =@@@%* -@@@- *%@@@=  \r\n"
"   *@@@     @@@     @@@% \r\n"
"  +@@%      @@@      %@@+\r\n"
"  @@@     .#@@@#.     @@@\r\n"
"  @@@    @@@@@@@@@    @@@\r\n"
"  *@@# .@@* @@@ *@@. #@@*\r\n"
"   #@@@@@   @@@   @@@@@# \r\n"
"    *@@@@_ _@@@_ _@@@@*  \r\n"
"      #@@@@@@@@@@@@@#    \r\n"
"        *+%@@@@@%+*      \r\n"
"\r\n\x1b[0;37m";

static const char nf_color_bar[] =
"\x1b[40m   \x1b[41m   \x1b[42m   \x1b[43m   "
"\x1b[44m   \x1b[45m   \x1b[46m   \x1b[47m   \x1b[0m";

static int cmd_pizza_about(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	/* Logo. Cursor ends 13 rows below where it started. */
	shell_fprintf(sh, SHELL_NORMAL, "%s", nf_logo);

	/* Snap back to top-right of the logo for the info column. */
	shell_fprintf(sh, SHELL_NORMAL, "\x1b[13A\x1b[%dC", NF_LOGO_WIDTH);

	/*
	 * Blank line in the title slot -- the CDC-connect banner already
	 * carries "PiZZa <ver> -- Zephyr on the Raspberry Pi Zero 2 W"
	 * up top, so we don't repeat it here.
	 */
	shell_fprintf(sh, SHELL_NORMAL, "\r\n\x1b[%dC", NF_LOGO_WIDTH);

#define IL(label, value) shell_fprintf(sh, SHELL_NORMAL, \
	"\x1b[1;31m%-9s\x1b[0;37m: %s\r\n\x1b[%dC", (label), (value), NF_LOGO_WIDTH)
	IL("SoC",     "Broadcom BCM2710 (Cortex-A53 quad, ARMv8-A AArch64)");
	IL("Wi-Fi",   "CYW43439 SDIO (brcmfmac driver)");
	IL("Console", "USB CDC ACM (you're here)");
	IL("Logs",    "PL011 / mini-UART on GPIO 14/15");
	IL("Project", "https://github.com/jetpax/PiZZa");
	IL("RFC",     "https://github.com/zephyrproject-rtos/zephyr/issues/109880");
#undef IL

	/* Blank line + ANSI colour bar (still aligned right of the logo). */
	shell_fprintf(sh, SHELL_NORMAL, "\r\n\x1b[%dC%s\r\n", NF_LOGO_WIDTH, nf_color_bar);

	/* Push cursor below the logo so the next prompt doesn't land mid-logo. */
	shell_fprintf(sh, SHELL_NORMAL, "\r\n\r\n\r\n");
	return 0;
}

static int cmd_pizza_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	uint8_t id[16];
	ssize_t n = hwinfo_get_device_id(id, sizeof(id));
	if (n > 0) {
		shell_fprintf(sh, SHELL_NORMAL, "  Board serial   : 0x");
		for (ssize_t i = 0; i < n; i++) {
			shell_fprintf(sh, SHELL_NORMAL, "%02x", id[i]);
		}
		shell_print(sh, "");
	} else {
		shell_print(sh, "  Board serial   : (hwinfo unavailable)");
	}

	const struct device *thermal = DEVICE_DT_GET_ANY(raspberrypi_bcm2835_vc_thermal);
	if (thermal && device_is_ready(thermal)) {
		struct sensor_value t;
		if (sensor_sample_fetch(thermal) == 0 &&
		    sensor_channel_get(thermal, SENSOR_CHAN_DIE_TEMP, &t) == 0) {
			shell_print(sh, "  Die temp       : %d.%02d C", t.val1,
				    t.val2 < 0 ? -t.val2 / 10000 : t.val2 / 10000);
		} else {
			shell_print(sh, "  Die temp       : (sensor fetch failed)");
		}
	} else {
		shell_print(sh, "  Die temp       : (sensor not ready)");
	}

	shell_print(sh, "  Uptime         : %u ms", k_uptime_get_32());
	return 0;
}

static int cmd_pizza_wifi(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	struct net_if *iface = net_if_get_first_wifi();
	if (!iface) {
		shell_warn(sh, "no Wi-Fi interface");
		return -ENODEV;
	}

	struct wifi_iface_status status = { 0 };
	int rc = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface,
			  &status, sizeof(status));
	if (rc < 0) {
		shell_warn(sh, "wifi status query failed (%d)", rc);
		return rc;
	}

	const char *state = (status.state >= WIFI_STATE_DISCONNECTED &&
			     status.state <= WIFI_STATE_COMPLETED) ?
				    wifi_state_txt(status.state) : "?";
	shell_print(sh, "  State          : %s", state);
	if (status.state >= WIFI_STATE_ASSOCIATED) {
		shell_print(sh, "  SSID           : %.*s",
			    status.ssid_len, status.ssid);
		shell_print(sh, "  BSSID          : %02x:%02x:%02x:%02x:%02x:%02x",
			    status.bssid[0], status.bssid[1], status.bssid[2],
			    status.bssid[3], status.bssid[4], status.bssid[5]);
		shell_print(sh, "  Channel        : %u", status.channel);
		shell_print(sh, "  RSSI           : %d dBm", status.rssi);
		shell_print(sh, "  Link mode      : %s",
			    wifi_link_mode_txt(status.link_mode));
		shell_print(sh, "  Security       : %s",
			    wifi_security_txt(status.security));
	} else {
		shell_print(sh, "  (not associated -- use `wifi connect -s <ssid> -p <psk> -k 1`)");
	}
	return 0;
}

static int cmd_pizza_temp(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	const struct device *thermal = DEVICE_DT_GET_ANY(raspberrypi_bcm2835_vc_thermal);
	if (!thermal || !device_is_ready(thermal)) {
		shell_warn(sh, "vc-thermal sensor not ready");
		return -ENODEV;
	}
	struct sensor_value t;
	int rc = sensor_sample_fetch(thermal);
	if (rc < 0) {
		shell_warn(sh, "fetch failed (%d)", rc);
		return rc;
	}
	rc = sensor_channel_get(thermal, SENSOR_CHAN_DIE_TEMP, &t);
	if (rc < 0) {
		shell_warn(sh, "get failed (%d)", rc);
		return rc;
	}
	shell_print(sh, "Die temperature: %d.%02d C",
		    t.val1, t.val2 < 0 ? -t.val2 / 10000 : t.val2 / 10000);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pizza,
	SHELL_CMD(about, NULL, "About PiZZa", cmd_pizza_about),
	SHELL_CMD(info,  NULL, "Board info (serial, temp, uptime)", cmd_pizza_info),
	SHELL_CMD(wifi,  NULL, "Wi-Fi association status", cmd_pizza_wifi),
	SHELL_CMD(temp,  NULL, "VideoCore die temperature", cmd_pizza_temp),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pizza, &sub_pizza,
		   "PiZZa device summary (about / info / wifi / temp)",
		   cmd_pizza_about);

/* Top-level command used by main() to print the banner on USB-CDC
 * DTR rising edge. shell_execute_cmd can target a top-level command;
 * keeping it separate from `pizza` keeps the subcommand list tidy. */
SHELL_CMD_REGISTER(welcome, NULL, "Print PiZZa banner",
		   cmd_pizza_welcome);
