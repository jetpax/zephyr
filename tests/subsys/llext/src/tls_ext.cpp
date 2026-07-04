/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * C++ extension exercising AArch64 local-exec TLS relocations.
 *
 * Accessing the thread-local `errno` makes the compiler emit
 * R_AARCH64_TLSLE_ADD_TPREL_HI12 / _LO12_NC relocations against the TLS symbol.
 * An extension has no TLS block of its own: it shares the loading thread's
 * block, so a local-exec access here aliases the loader thread's TLS slot at the
 * same thread-pointer-relative offset.
 *
 * For that aliasing to land on the loader's `errno`, the extension must define
 * `errno` at the same offset the loader places it. The block below mirrors what
 * the on-target build links in via a generated tls-syms.S: it defines `errno`
 * in .tbss after a leading slot, so its st_value (0x8) plus the variant-I TCB
 * (0x10) yields the loader's errno thread-pointer offset (0x18 in this test
 * configuration). If the loader's TLS layout ever changes, the round-trip check
 * in test_llext.c fails loudly rather than silently touching the wrong slot.
 *
 * Compiling as C++ (not C) reproduces the weak-undefined _ZTH5errno TLS-init
 * thunk the C++ front-end emits; the build drops it with -fno-extern-tls-init
 * (see CMakeLists.txt), matching the on-target toolchain flags. Without that,
 * llext cannot resolve the thunk and the load fails with -ENODATA.
 */

#include <errno.h>
#include <zephyr/llext/symbol.h>

asm(".section .tbss,\"awT\",%nobits\n"
    ".balign 8\n"
    ".zero 8\n"			/* leading slot (loader's z_tls_current) */
    ".globl errno\n"
    ".type errno, %tls_object\n"
    "errno:\n"
    ".zero 4\n"
    ".previous\n");

/*
 * A zero-initialised global forces a non-empty .bss. Together with the .tbss
 * above, the extension then has two SHT_NOBITS sections; without the SHF_TLS
 * skip in llext_map_sections() the load is rejected with "Multiple SHT_NOBITS",
 * the exact on-target failure this patch removes. test_entry() touches it so it
 * survives --gc-sections.
 */
static volatile int tls_bss_canary;

extern "C" void test_entry(void)
{
	tls_bss_canary = errno;
	errno = errno + 1;
}
EXPORT_SYMBOL(test_entry);
