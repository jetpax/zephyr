.. SPDX-License-Identifier: Apache-2.0

.. zephyr:board:: fxh618_d4

FX-H618-D4 V10 TV Box
#####################

Overview
********

The FX-H618-D4 V10 is a generic Allwinner H618 TV box fitted with 4 GiB
of DRAM and a microSD slot. The H618 SoC is a quad-core ARM Cortex-A53.
The serial console is UART0, exposed on TX/RX test pads next to the SoC.

Hardware
********

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

The H618 SoC needs DRAM and PLL initialization before Zephyr can run.
The U-Boot SPL performs this. Build U-Boot as for the Orange Pi Zero 2W
(see :zephyr:board:`opi_zero2w`), using a 4 GiB H618 box defconfig.

FEL loop (no SD card writes)
============================

Hold the FEL button (or boot with no SD card) so the BROM enters FEL
mode on the USB OTG port, then:

.. code-block:: console

   sunxi-fel spl u-boot-spl.bin
   sunxi-fel write 0x40080000 zephyr.bin
   sunxi-fel exe 0x40080000

SD card boot
============

Write ``u-boot-sunxi-with-spl.bin`` at an 8 KiB offset, copy
``zephyr.bin`` to a FAT partition, then from the U-Boot prompt:

.. code-block:: console

   => fatload mmc 0:1 0x40080000 zephyr.bin
   => go 0x40080000
