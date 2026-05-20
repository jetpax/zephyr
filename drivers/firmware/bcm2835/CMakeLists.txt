# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_BCM2835_MBOX     bcm2835_mbox.c)
zephyr_library_sources_ifdef(CONFIG_BCM2835_FIRMWARE bcm2835_property.c)
zephyr_library_include_directories(.)
