# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if(CONFIG_SLOW_FLASH_DATA)
  if(COMPILER STREQUAL gcc)
    zephyr_compile_options(-mslow-flash-data -fno-jump-tables)
  elseif(COMPILER STREQUAL iar)
    zephyr_compile_options(--no_literal_pool --no_switch_tables)
  endif()
endif()

if(CONFIG_CPU_AARCH32_ARMV6)
  # ARM1176 / ARMv6KZ does NOT support true unaligned access -- a misaligned
  # LDR returns the aligned word rotated by (addr & 3) * 8 bits instead of
  # faulting. GCC optimises byte-shift patterns like ELM FatFs's ld_32
  # (ptr[3]<<24 | ptr[2]<<16 | ptr[1]<<8 | ptr[0]) into a single LDR,
  # which then silently returns a rotated word when the pointer isn't
  # 4-byte aligned. Symptom: FATFS read of MBR partition LBA at byte 454
  # returned 0x20020800 instead of 0x00000800. Force GCC to emit byte
  # loads. Mirrors what the kernel does for ARMv6 (CONFIG_ARM=y +
  # !CONFIG_ARCH_ARMV7 in Linux's arch/arm/Makefile).
  zephyr_compile_options(-mno-unaligned-access)
endif()

zephyr_library_sources(
  fatal.c
  nmi.c
  nmi_on_reset.S
)

zephyr_library_sources_ifdef(CONFIG_CPP __aeabi_atexit.c)
zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)
zephyr_library_sources_ifdef(CONFIG_THREAD_LOCAL_STORAGE tls.c)
zephyr_library_sources_ifdef(CONFIG_USERSPACE userspace.S)
zephyr_library_sources_ifdef(CONFIG_ARM_ZIMAGE_HEADER header.S)
zephyr_library_sources_ifdef(CONFIG_LLEXT elf.c)
zephyr_library_sources_ifdef(CONFIG_GDBSTUB gdbstub.c)
zephyr_library_sources_ifdef(CONFIG_ARCH_STACKWALK stacktrace.c)

zephyr_cc_option_ifdef(CONFIG_ARCH_STACKWALK -funwind-tables)

add_subdirectory_ifdef(CONFIG_CPU_CORTEX_M cortex_m)
add_subdirectory_ifdef(CONFIG_CPU_CORTEX_M_HAS_CMSE cortex_m/cmse)
add_subdirectory_ifdef(CONFIG_ARM_SECURE_FIRMWARE cortex_m/tz)
add_subdirectory_ifdef(CONFIG_ARM_NONSECURE_FIRMWARE cortex_m/tz)

add_subdirectory_ifdef(CONFIG_ARM_MPU mpu)
add_subdirectory_ifdef(CONFIG_ARM_AARCH32_MMU mmu)

add_subdirectory_ifdef(CONFIG_CPU_AARCH32_ARM9 arm9)
add_subdirectory_ifdef(CONFIG_CPU_AARCH32_ARMV6 armv6)
add_subdirectory_ifdef(CONFIG_CPU_AARCH32_CORTEX_R cortex_a_r)
add_subdirectory_ifdef(CONFIG_CPU_AARCH32_CORTEX_A cortex_a_r)

if(CONFIG_ARM_ZIMAGE_HEADER)
zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors zimage_header.ld)
zephyr_linker_sources(ROM_START SORT_KEY 0x1vectors vector_table.ld)
zephyr_linker_sources(ROM_START SORT_KEY 0x2vectors cortex_m/vector_table_pad.ld)
else()
zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors vector_table.ld)
zephyr_linker_sources(ROM_START SORT_KEY 0x1vectors cortex_m/vector_table_pad.ld)
endif()

if(CONFIG_GEN_SW_ISR_TABLE)
  if(CONFIG_DYNAMIC_INTERRUPTS)
    zephyr_linker_sources(RWDATA swi_tables.ld)
  else()
    zephyr_linker_sources(RODATA swi_tables.ld)
  endif()
endif()
