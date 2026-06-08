/*
 * Copyright (c) 2026 jetpax
 * SPDX-License-Identifier: Apache-2.0
 *
 * QPU shader for the qpu_write smoke test. Each of N invocations is
 * given (output_bus_addr, inner_iters, qpu_id) as uniforms and writes
 * qpu_id to its slice of the output array.
 *
 * Assembled with vc4asm from rpi_os/labs/qpu_write/qpu_code.qasm.
 * Bundled here as a precompiled blob to keep the Zephyr build
 * toolchain-independent of vc4asm.
 */

#ifndef QPU_WRITE_CODE_H_
#define QPU_WRITE_CODE_H_

#include <stdint.h>

extern uint32_t qpu_write_code[50];

#endif /* QPU_WRITE_CODE_H_ */
