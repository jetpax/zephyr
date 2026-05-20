# SPDX-License-Identifier: Apache-2.0

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_ARM_SCMI scmi)
add_subdirectory_ifdef(CONFIG_BCM2835_MBOX bcm2835)
add_subdirectory_ifdef(CONFIG_QEMU_FWCFG qemu_fwcfg)
add_subdirectory_ifdef(CONFIG_TISCI tisci)
# zephyr-keep-sorted-stop
