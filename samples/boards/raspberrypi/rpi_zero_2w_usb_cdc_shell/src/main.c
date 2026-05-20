/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr shell over USB CDC ACM on rpi_zero_2w.
 *
 * Nothing for main() to do -- the shell subsystem starts its own
 * thread at SYS_INIT and binds to the UART named in the DT chosen
 * `zephyr,shell-uart` (set to the CDC ACM instance in app.overlay).
 * Host opens /dev/cu.usbmodem* and gets a `uart:~$ ` prompt.
 *
 * Try:
 *   uart:~$ help
 *   uart:~$ kernel threads
 *   uart:~$ device list
 *   uart:~$ hwinfo devid       # the 64-bit Pi OTP serial
 *   uart:~$ log enable inf udc_dwc2  # live-flip log levels
 */

int main(void)
{
	return 0;
}
