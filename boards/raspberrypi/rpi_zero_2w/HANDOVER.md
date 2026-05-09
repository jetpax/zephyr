# rpi_zero_2w port -- handover

Where this board sits as of May 8 2026, what's working, what's left,
and the things you'd otherwise have to rediscover. Written for a
fresh developer (or fresh Claude session) picking the work up cold.

## TL;DR status

```
Pi VPU firmware -> armstub8.bin -> Zephyr at EL2 -> EL1 drop
  -> BCM2836 ARM-local intc + BCM2835 ARMC peripheral intc cascade
  -> ARM generic timer (virtual) IRQ on the L1 intc
  -> mini-UART (GPIO 14/15) for console
  -> 256 MiB DRAM at sram0 = 0x200000..0x10200000
  -> MicroPython 1.28 REPL with 64 MiB MP heap
```

End-to-end working. Validated under `qemu-system-aarch64 -machine
raspi3b` and on a real Pi Zero 2 W via tio @ 115200 8N1.

## Repository layout

- **Branch:** `rpi-zero-2w-port` on `jetpax/zephyr` (fork of
  `zephyrproject-rtos/zephyr`).
- **Pin:** branched off upstream `603fa4818e8` (~v4.4.0-1634).
- **Tag:** `rpi_zero_2w-first-mp` marks the first hardware MP boot.
- **Release:** https://github.com/jetpax/zephyr/releases/tag/rpi_zero_2w-first-mp
  with the MP `zephyr.bin` attached (~300 KB).
- **Three upstream PRs** carry the drive-by fixes that aren't
  rpi_zero_2w-specific:
  - https://github.com/zephyrproject-rtos/zephyr/pull/108775 -- timer
    Kconfig (allow ARM_CUSTOM_INTERRUPT_CONTROLLER alongside GIC)
  - https://github.com/zephyrproject-rtos/zephyr/pull/108776 -- UART
    poll_in API contract fix
  - https://github.com/zephyrproject-rtos/zephyr/pull/108777 -- UART
    IER read-modify-write fix

If those PRs land upstream, three commits on this branch become
redundant and can be rebased away.

## Build & run

### Sample (no MicroPython)

```sh
cd ~/zephyrproject
source ~/.zephyr-venv/bin/activate
ZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
CROSS_COMPILE=$HOME/zephyr-sdk/aarch64-zephyr-elf/bin/aarch64-zephyr-elf- \
west build -p always -b rpi_zero_2w samples/synchronization \
  -d build/rpi_zero_2w-sync
```

### MicroPython REPL

```sh
ZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
CROSS_COMPILE=$HOME/zephyr-sdk/aarch64-zephyr-elf/bin/aarch64-zephyr-elf- \
west build -p always -b rpi_zero_2w \
  ~/github/micropython/ports/zephyr \
  -d build/mpy-rpi_zero_2w
```

The MP build needs a board overlay that lives in the MicroPython
tree, not here:
`~/github/micropython/ports/zephyr/boards/rpi_zero_2w.conf`.
Contents (see "Gotchas" below for rationale):

```
CONFIG_NETWORKING=n
CONFIG_WATCHDOG=n
CONFIG_TEST_RANDOM_GENERATOR=n
CONFIG_TOOLCHAIN_CROSS_COMPILE_SUPPORTS_THREAD_LOCAL_STORAGE=y
CONFIG_THREAD_LOCAL_STORAGE=y
CONFIG_MICROPY_HEAP_SIZE=67108864
```

That file is currently a local addition to the user's micropython
clone -- not pushed to a fork yet. If you want it durable, fork
`micropython/micropython` and push.

### QEMU

```sh
qemu-system-aarch64 -machine raspi3b \
  -kernel build/.../zephyr/zephyr.bin \
  -serial null -serial stdio -display none
```

`-serial null -serial stdio` puts the second serial (mini-UART /
uart1, our console) on stdio. The first serial (PL011 / uart0) is
discarded. QEMU's stdin handling sometimes drops the first few chars
of pasted lines; on real hardware via tio this doesn't happen.

### Real hardware

```sh
boards/raspberrypi/rpi_zero_2w/support/install-to-sdcard.sh \
  /Volumes/bootfs build/.../zephyr/zephyr.bin
```

(or `make-sdcard-image.py` to build a self-contained .img). Then:

```sh
tio /dev/cu.usbserial-XX
```

Wiring: GPIO 14 (TX, header pin 8) -> adapter RX. GPIO 15 (RX, pin
10) -> adapter TX. GND (pin 6/9/14/etc) -> adapter GND. 115200 8N1.

The board doc (`doc/index.rst`) covers config.txt and SD recipes.

## Mental model

### BCM2710 != BCM2711

Pi Zero 2 W silicon is **BCM2710A1** (a re-marked BCM2837 in the
RP3A0-AU SiP), same family as the Pi 3. Pi 4's BCM2711 is a
substantially different chip, despite Zephyr's existing `rpi_4b`
board sharing some driver names. Differences that matter:

- **No GIC.** BCM2711 has a GIC-400 at 0xff841000. BCM2710 has a
  pair: BCM2836 ARM-local intc at 0x40000000 (per-core timers,
  mailboxes, PMU, GPU cascade) + BCM2835 ARMC peripheral intc at
  0x3f00b200 (64 GPU IRQs + 8 basic). There is **no GIC-emulation
  armstub** for BCM2710 -- `armstub8-gic.bin` is BCM2711-only.
- **Peripheral base.** BCM2711 high-peri base 0xfe000000 vs BCM2710
  legacy 0x3f000000.
- **Timer crystal.** BCM2711 = 54 MHz; BCM2710 = 19.2 MHz. We use
  `CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME=y` so the actual
  value is read from CNTFRQ_EL0 at boot.
- **Mini-UART core clock.** BCM2711 = 500 MHz default; BCM2710 with
  `core_freq=250` in config.txt = 250 MHz. We declare 250 in DT.
- **GPIO pull-control.** BCM2711 has a dedicated PUP_PDN_CNTRL_REG0
  at offset 0xe4 (32-bit, two bits per pin). BCM2710 uses the legacy
  GPPUD/GPPUDCLK0/GPPUDCLK1 sequence at 0x94/0x98/0x9c. The Zephyr
  `brcm,bcm2711-gpio` driver pokes 0xe4 on init -- works on Pi 4,
  silently misroutes on Pi Zero 2 W. We left gpio0/gpio1 in
  `status = "disabled"` to avoid this; see "Open work" below.

We **reuse** the BCM2711 drivers for the parts of the BCM283x family
that *are* register-compatible (mini-UART, PL011, pinctrl), with
upstream-quality bug fixes in the UART driver (the 3 PRs above).

### Interrupt cascade

```
                  +-----------------------+
   ARM core nIRQ <-- L1 intc @ 0x40000000 |
                  | (BCM2836 ARM-local)   |
                  | sources: timer 0..3,  |
                  |  mailbox 0..3, GPU,   |
                  |  PMU                  |
                  +-----------+-----------+
                              |
                              | (cascaded via "GPU" line, source 8)
                              |
                  +-----------v-----------+
                  | ARMC intc @ 0x3f00b200 |
                  | (BCM2835 peripheral) |
                  | banks: basic 0..7,   |
                  |  PEND1 0..31,        |
                  |  PEND2 0..31         |
                  +-----------------------+
```

Zephyr IRQ-number layout (matches Linux's `MAKE_HWIRQ(bank, n)` for
the ARMC half, plus +32 for the L1 reservation):

```
0..9     L1 intc per-core sources (timer 0..3, mbox 0..3, GPU=8, PMU=9)
32..39   ARMC basic bank (IRQ_BASIC_PENDING bits 0..7)
64..95   ARMC PEND1 (GPU IRQs 0..31)
96..127  ARMC PEND2 (GPU IRQs 32..63)
```

L1 IRQ #8 (the GPU cascade) is **never returned by
`z_soc_irq_get_active`** -- if only the GPU bit is set, the
function walks down through ARMC and returns the actual peripheral
IRQ number. ISRs are therefore registered at 32+ slots, not at 8.

The cascade walk in `soc/brcm/bcm2710/soc_irq.c` is a near
line-for-line port of Linux's `irq-bcm2835.c::get_next_armctrl_hwirq`,
including the three quirks documented at the top of that file
(shortcut bits in bank 0, can't-mask-cascade-bits, can't-mask-shortcut
in bank 0).

### Mask-around-ISR pattern

The arm64 ISR wrapper (`arch/arm64/core/isr_wrapper.S`) does
`daifclr` to unmask IRQs globally around the ISR call, so nested
handlers can run. With a GIC, ack-on-read prevents the same source
from re-entering. The BCM intc pair has no ack -- a level-triggered
source stays asserted until the peripheral clears it. Naive use
re-fires the same IRQ before the ISR has run, infinite recursion.

The fix in `soc_irq.c::z_soc_irq_get_active` masks the firing source
at the SoC intc on its way out; `z_soc_irq_eoi` re-enables on the
way back. This brackets the ISR call with a per-source mask, the way
GIC's running-priority register would for free. Single-source-at-a-
time, so no state tracking beyond the controllers themselves.

If you ever extend this for nested-IRQ semantics (multiple sources
firing during one ISR), revisit this. For now it's correct and
trivially small.

## Gotchas (the things that ate hours)

1. **EL2->EL1 drop** is *already handled* by Zephyr's
   `arch/arm64/core/reset.S`. It detects current EL via the
   `switch_el` macro and routes to `z_arm64_el2_init` + eret to EL1
   when needed. Pi armstub leaves you at EL2 NS; this Just Works
   without per-board code. (Earlier research suggested otherwise; in
   practice rpi_4b proves it.)

2. **TLS via cross-compile.** MicroPython's `mp_active_ctx` is
   `__thread`-qualified. Without
   `CONFIG_THREAD_LOCAL_STORAGE=y`, accessing it dereferences
   `TPIDR_EL0+0x10` which is uninitialised; data abort at boot. The
   cross-compile toolchain variant doesn't auto-advertise TLS
   support, so the rpi_zero_2w MP `boards.conf` force-sets both
   `CONFIG_TOOLCHAIN_CROSS_COMPILE_SUPPORTS_THREAD_LOCAL_STORAGE=y`
   and `CONFIG_THREAD_LOCAL_STORAGE=y`. Switching to the SDK
   toolchain variant avoids this.

3. **MICROPY_HEAP_SIZE default is 48 KiB.** Pi Zero 2 W has 512 MiB
   DRAM. The board.conf bumps it to 64 MiB; tune as you like.

4. **Linker base = 0x200000, NOT 0x80000.** Pi firmware's default
   `kernel8.img` load address is 0x80000. We deliberately use
   0x200000 (matches the upstream rpi_4b convention) so that QEMU
   `-kernel zephyr.bin` keeps working (QEMU loads at the binary's
   linker VMA). On real hardware, this requires
   `kernel_address=0x200000` in config.txt. The board doc has the
   full required config.txt.

5. **`enable_uart=1` is not enough.** It's *supposed* to lock
   `core_freq=250` automatically but doesn't reliably (raspberrypi/
   linux issue 4123). Always set `core_freq=250` explicitly --
   without it, the mini-UART baud divisor is wrong and you get
   gibberish. Documented in board doc.

6. **GPIO is left disabled.** As above -- the bcm2711-gpio driver
   pokes BCM2711-only registers. Either fix that driver to fall
   back to GPPUD/GPPUDCLK on `compatible = "brcm,bcm2710-gpio"`, or
   write a parallel `bcm2710-gpio.c`. `machine.Pin()` from MP is
   unavailable until then.

7. **QEMU stdin sometimes eats the first few chars of pasted
   commands.** Annoying for testing, harmless on real hardware via
   tio.

## SDIO bring-up status (in flight as of 2026-05-09)

Significant progress since the original handover -- a polled
SDHCI driver, GPIO + pinctrl wiring, board-DTS hookup, and CMD0
working end-to-end on real silicon. **But the chip stays silent
on every SDIO command we've tried** (CMD52 read CCCR, CMD52
IO_RESET, CMD5 inquiry). The controller transmits, eventually
fires CMD_TIMEOUT, but the chip itself never sends a response.

**Strongest remaining hypothesis:** GPCLK2 (LPO input on GPIO 43)
isn't actually generating 32.768 kHz. We pinctrl-route the pin to
ALT0, but Zephyr has no BCM2835 clock-manager driver to program
`CM_GP2CTL` / `CM_GP2DIV` -- so the pin is outputting whatever the
Pi VPU firmware left, which may not be a clean LPO. The CYW43439's
PMU likely needs LPO to advance from cold-boot to "ready for SDIO".

**Next move:** add a minimal CM_GP2 setup before WL_REG_ON-high.
CM base 0x3F101000, CM_GP2CTL at 0x80, CM_GP2DIV at 0x84, every
write needs PASSWD = 0x5A in bits 31:24. With 19.2 MHz oscillator
divided by 586 we get 32764 Hz -- close enough. ~30 lines of
direct register pokes; can be a small SoC helper or inline in
`sdhc_bcm2835_init`.

The full debug log -- what worked, what didn't, scaffolding
diagnostics still in tree, secondary hypotheses -- is captured
in the auto-memory at
`~/.claude/projects/-Users-jep-github-SS/memory/project_rpi_zero_2w_sdio_debug.md`.

## Open work, ranked

### Easy and useful

- **Land the 3 upstream PRs.** They're already submitted. If
  maintainers ask for changes, address them. When merged, rebase
  this branch to drop the equivalent local commits.
- **MP fork on github.** `gh repo fork micropython/micropython`
  then push the `boards/rpi_zero_2w.conf` so it's not just a local
  file. Symmetrical with the zephyr fork.
- **GPIO driver.** Either patch `drivers/gpio/gpio_bcm2711.c` to
  use the legacy GPPUD sequence when DT compatible matches a
  BCM2710-family string, or add `gpio_bcm2710.c` as a sibling.
  Linux's `pinctrl-bcm2835.c` is the reference; lots of clearly-
  documented register pokes. ~200 lines.

### Medium

- **SMP.** The L1 intc has per-core enable registers; my soc_irq.c
  hardcodes `CORE_ID = 0`. Bringing up cores 1-3 needs:
  - secondary-core boot via the spin-table at 0xd8..0xf0 (Pi
    armstub parks them there).
  - per-CPU `z_soc_irq_*` (current ones write to core-0 regs only).
  - mailbox IPI driver wired to L1 mailbox IRQs 4..7.
- **PL011 UART validation.** The DTS has uart0/2/3/4/5 nodes set
  `disabled`; the binding works, but the driver path hasn't been
  exercised on this SoC. May need similar IER-style fixes. The
  PL011 driver is in `drivers/serial/uart_pl011.c`.

### Harder

- **CYW43439 Wi-Fi/Bluetooth.** Pi Zero 2 W's wireless lives on
  SDIO. Zephyr's BCM SDHCI driver is missing entirely, and the
  user already has CYW43439 + WHD-via-SPI working on RP2350 (in a
  different branch -- see `rp2350-psram-bringup` in the same fork).
  The path is:
  1. Write a Zephyr SDIO host driver for the BCM2710 EMMC2/SDHOST
     controller. (Linux: `sdhci-iproc.c` + `bcm2835_mmc.c` are
     references.)
  2. Adapt the AIROC WHD HAL bus shim to use SDIO.
  3. Pull in the existing CYW43439 Zephyr driver.
  This is months of work and is the gating piece for anything
  network-based on this board.
- **Frozen `_boot.py`.** MP currently boots straight to a bare
  REPL. If you want frozen modules baked in (typical embedded MP
  workflow), the path is via MP's manifest mechanism in
  `ports/zephyr/boards/manifest.py`. Not blocking but useful.

## File map

### In this tree

```
boards/raspberrypi/rpi_zero_2w/
  rpi_zero_2w.dts            board DTS (cpu, console, sram chosen)
  rpi_zero_2w_defconfig      board defconfig (VA bits, serial)
  rpi_zero_2w.yaml           board metadata (arch, toolchains)
  Kconfig.rpi_zero_2w        BOARD_RPI_ZERO_2W -> SOC_BCM2710
  board.yml                  vendor/socs metadata
  doc/index.rst              user-facing board doc
  HANDOVER.md                THIS FILE
  support/
    install-to-sdcard.sh     drop zephyr.bin onto an imaged RPi OS card
    make-sdcard-image.py     build a self-contained sdcard.img

soc/brcm/bcm2710/
  Kconfig                    select ARM64, CPU_CORTEX_A53, ARM_CUSTOM_INTC
  Kconfig.soc, Kconfig.defconfig, soc.yml, soc.h
  CMakeLists.txt
  mmu_regions.c              MMU regions for L1 intc + ARMC
  pinctrl_soc.h              reuses bcm2711 pinctrl dt-bindings
  soc_irq.c                  the BCM intc dispatch (~200 lines)

dts/arm64/broadcom/bcm2710.dtsi    BCM2710 device tree
dts/bindings/interrupt-controller/
  brcm,bcm2836-l1-intc.yaml        L1 intc binding
  brcm,bcm2835-armctrl-ic.yaml     ARMC binding

include/zephyr/dt-bindings/interrupt-controller/
  bcm2836-l1.h                     L1 IRQ source enum
  bcm2835-armctrl.h                ARMC IRQ flat-numbering

drivers/serial/uart_bcm2711.c      poll_in + IER fixes (subject of
                                   2 of the 3 upstream PRs)
drivers/timer/Kconfig.arm_arch     dep relax (1st upstream PR)
```

### Outside this tree

- `~/github/micropython/ports/zephyr/boards/rpi_zero_2w.conf` --
  MP board overlay (NOT in any git repo yet).
- `~/zephyrproject/build/...` -- west build dirs.
- `~/.cache/zephyr-rpi-zero-2w/` -- Pi firmware blob cache (used by
  `make-sdcard-image.py`).

## How to keep going

1. Pull this branch: `git -C zephyr fetch jetpax && git checkout
   rpi-zero-2w-port`.
2. Run `samples/synchronization` first to confirm the toolchain
   path. Confirms scheduler + timer-IRQ pipeline.
3. Then MP: build, flash, see the REPL.
4. Pick an item from "Open work, ranked" above. The GPIO driver is
   the smallest meaningful next step; SDIO is the big door it opens
   into.

Good luck.
