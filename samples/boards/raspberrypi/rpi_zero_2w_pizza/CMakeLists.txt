# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rpi_zero_2w_pizza)

# Pulls in samples/subsys/usb/common/sample_usbd_init.c and its header
# so we can get the usbd_context handle (auto-init in
# CDC_ACM_SERIAL_INITIALIZE_AT_BOOT keeps it static; we need it to
# register a USBD message callback for DTR rising edge -> banner reprint).
include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

target_sources(app PRIVATE src/main.c)
