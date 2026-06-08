/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Precompiled VC4 QPU instruction blob for the qpu_write smoke test.
 *
 * Source: rpi_os/labs/qpu_write/qpu_code.qasm (Justin Yang's Stanford
 * CS140E lab tree). Assembled with vc4asm to a uint32_t[] of QPU
 * machine code; this file is a verbatim copy of qpu_code.c from that
 * tree with the array renamed `qpu_write_code` to avoid colliding
 * with the public symbol space.
 */

#include "qpu_write_code.h"

__attribute__((aligned(8)))
uint32_t qpu_write_code[50] = {
	0x15827d80, 0x10020027,
	0x15827d80, 0x10020067,
	0x15827d80, 0x100200a7,
	0x15067d80, 0x100208a7,
	0x00401a00, 0xe0020827,
	0x11082dc0, 0xd00208e7,
	0x0c9e70c0, 0x10021c67,
	0x150a7d80, 0x10020c27,
	0x150a7d80, 0x10020c27,
	0x150a7d80, 0x10020c27,
	0x150a7d80, 0x10020c27,
	0x82104000, 0xe0020827,
	0x119c77c0, 0xd0020867,
	0x0c9e7040, 0x10021c67,
	0x0d9c15c0, 0xd00228a7,
	0x119c85c0, 0xd0020867,
	0x0c027c40, 0x10021ca7,
	0x159f2fc0, 0x100009e7,
	0xffffff70, 0xf01809e7,
	0x009e7000, 0x100009e7,
	0x009e7000, 0x100009e7,
	0x009e7000, 0x100009e7,
	0x009e7000, 0x300009e7,
	0x009e7000, 0x100009e7,
	0x009e7000, 0x100009e7
};
