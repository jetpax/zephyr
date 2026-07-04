# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_FPU_SHARING)
  # soc_irq.c's arch_spin_relax drains the FPU-flush IPI, which needs
  # arch_float_disable() from the kernel-internal kernel_arch_interface.h.
  zephyr_include_directories(${ZEPHYR_BASE}/kernel/include)
endif()

zephyr_sources_ifdef(CONFIG_ARM_MMU mmu_regions.c)
zephyr_sources(soc_irq.c)
zephyr_sources_ifdef(CONFIG_REBOOT reboot.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
