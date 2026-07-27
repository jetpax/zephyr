# Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hdmi_pattern)

target_sources(app PRIVATE src/main.c)
