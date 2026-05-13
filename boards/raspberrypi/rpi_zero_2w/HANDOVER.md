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

6. **GPIO is left disabled.** The pinctrl side is fixed (the
   `legacy-pull-control` DT property selects the BCM2710 GPPUD /
   GPPUDCLK sequence in `pinctrl_bcm2711.c`), so muxing and pulls
   work. The bcm2711-gpio output-control side still pokes the
   modern register layout though; until that's similarly extended,
   `machine.Pin()` from MP for output-driving is unavailable.

7. **QEMU stdin sometimes eats the first few chars of pasted
   commands.** Annoying for testing, harmless on real hardware via
   tio.

## SDIO bring-up status (as of 2026-05-12, end of day)

**Two walls cracked since the 2026-05-11 snapshot below. One wall
still open. Read this section first; the older snapshot is preserved
for archaeology but its "wall" is no longer the wall.**

### Cracked wall #1: pin contention on GPFSEL4/5 (2026-05-12 AM)

Pi VPU/firmware boots with **GPIO 48..53 at ALT3** (Arasan path used
by Pi firmware for boot-time SD slot access). On Pi 3 / Pi Zero 2 W
the Arasan controller's **RX input mux inside the SoC latches to the
48..53 pad set first** when both 34..39 and 48..53 are at ALT3 — the
TX correctly drives both pad sets, the chip on pin 35 sees clean CMD
frames and responds, but the controller's RX listens on pin 49 (empty
SD slot, pulled high) and never sees the response. CMD_TIMEOUT every
time.

**Fix:** `drivers/sdhc/sdhc_bcm2835.c::sdhc_bcm2835_init` muxes pins
48..53 → ALT0 *before* `pinctrl_apply_state` runs on 34..39. Linux/
Pi-firmware boot path implicitly does this; circle's
`ether4330.c::sdioinit` makes it explicit. Single ~15-line change.

### Cracked wall #2: pinctrl pull-up not sticking on DAT0..3 (2026-05-12 PM)

After wall #1 was lifted, CMD52 worked but the first 4-bit CMD53
fired DATA_CRC (`int_status=0x00208000`, `pstate=0x01ff0202`).
Confirmed via MP REPL test (saved at
`~/zephyrproject/tools/sdio-4bit-pullup-repro.py`): manually re-doing
the GPPUD pull-up dance on pins 36..39 right before a 4-bit CMD53
made it succeed with clean CCCR bytes `32 02 02 02 00 00 00 42`. So
the DT `bias-pull-up` on the emmc_gpio34 group was requesting
pull-up, but `pinctrl_bcm2711.c::bcm2711_pinctrl_set_pull_legacy`
wasn't actually latching it on this silicon.

**Fix:** in `drivers/pinctrl/pinctrl_bcm2711.c`, two deltas vs.
circle's `gpiopull` (`addon/wlan/p9arch.cpp:60-76`):
- Delay 1 µs → 5 µs (circle's own comment: "1 us should be enough,
  but to be sure"; our 1 µs `bcm2835_st_busy_wait_us` can return
  after a 0..1 µs wait depending on 1 MHz tick phase).
- Re-add the `GPPUD = 0` write before the strobe clear (matches
  circle and the REPL workaround; the previous comment in the file
  saying this re-latches pull-off was a wrong diagnosis carried
  forward).

Verified by re-running the REPL test on the post-fix build *without*
manually forcing GPPUD: 4-bit CMD53 succeeds. Pinctrl pull-up is now
sticking through to MP REPL time.

### Open wall #3: boot-time 4-bit CMD53 (func1 backplane chipid) fires DATA_CRC

The bring-up shim at `SYS_INIT(APPLICATION, 99)` in
`~/github/micropython/ports/zephyr/src/bcm43430_bringup.c` fails on
its first CMD53 — a 4-byte read of the chipid via the func1 backplane
window — with `int_status=0x00208000`, `pstate=0x01ff0202`. Every
CMD52 (CCCR reads, IO_ENABLE, block size, CHIPCLKCSR force-ALP,
SBADDR window writes) succeeds; only the first data-phase command
fails.

### What's confirmed working at REPL after pinctrl fix

- 4-bit CMD53 read of 8 bytes from func 0 offset 0 (CCCR header)
  succeeds *without* the GPPUD pull-up dance — i.e., pinctrl's
  bias-pull-up on DAT0..3 is now sticking through to REPL time.
  Verified by `~/zephyrproject/tools/sdio-check-pinctrl-pullup.py`.
- The pinctrl change in `drivers/pinctrl/pinctrl_bcm2711.c`
  (`bcm2835_st_busy_wait_us(5)` and an explicit `GPPUD = 0` write
  before the strobe clear) is the load-bearing pinctrl fix.

### Status of the func1 backplane CMD53 from REPL

Unknown. A REPL script that does (RESET_DATA pulse → redo three CMD52
SBADDR writes → CMD53 func1 backplane chipid read) returned a clean
chipid `0x1541a9a6 id=43430 rev=1 type=1` (BCM43430A1) on one run
during this session; subsequent runs of the same script failed with
DATA_CRC, but the user's review identified a bug in that script.
Rewrite it carefully and re-test before drawing any conclusions about
the func1 backplane path.

### What was attempted to migrate the workaround into the driver/shim, and broke things

Each of these passed `west build` but caused regressions. **All
reverted; the current driver and shim are clean.**

- **RESET_DATA pulse at end of `sdhc_bcm2835_init`** (matching
  circle's emmc.c::emmcinit lines 295-301). Init runs with clock
  gated, so the polling `soft_reset` helper timed out (RESET_DATA
  doesn't self-clear without a clock). Switched to circle's
  force-write pattern (write CTL1 = RESET_DATA, busy_wait, write
  CTL1 = 0) — boot then completed but the failing CMD53 was
  unchanged.
- **RESET_DATA at end of `sdhc_bcm2835_set_io` when `clock != 0`**.
  Boot failed identically AND the REPL workaround stopped working.
  The multiple RESET_DATA pulses during `sd_init`'s several set_io
  calls apparently destabilise something the REPL pulse can't recover.
- **RESET_DATA inside `sdhc_bcm2835_request` before every data-bearing
  command**. Same regression pattern: boot fails, REPL workaround
  stops working.
- **RESET_DATA pulse in the bring-up shim itself just before the
  chipid CMD53**. Pulse fired correctly (boot log shows
  `RESET_DATA pulse before chipid CMD53: ctl1 now 0x000e0407`).
  Next CMD53 still fired DATA_CRC. So a RESET_DATA pulse
  immediately before the CMD53 is *not* sufficient by itself —
  the REPL workaround is doing something more than just that.

### Empirical facts the next attempt should anchor on

- Function-0 CMD53 (CCCR read, 8 bytes from func 0 offset 0) works
  at REPL after only pinctrl-applied pull-up — no extra GPPUD dance
  needed. So the host's 4-bit data path is healthy for at least some
  chip-side reads.
- Function-1 backplane CMD53 (chipid, 4 bytes at offset 0x8000 with
  the `SB_ACCESS_2_4B_FLAG` bit set) is the one that fails at boot.
- All CMD52 transactions succeed throughout, including the SBADDR
  window writes — chip echoes the data back correctly in R5.
- CHIPCLKCSR reads `0x68` (ALP_AVAIL + ALP_AVAIL_REQ + HT_AVAIL_REQ)
  right before the failing CMD53. ALP is up; HT is not.
- The REPL workaround works reliably with the current build — the
  earlier confusion in this session about "REPL test failing too"
  came after speculative driver changes that have been reverted.

### Suggested directions for the next session

1. **Read the REPL workaround script carefully and identify which
   step is doing the work.** The current best understanding:
   `RESET_DATA + redo SBADDR window writes + CMD53` works at REPL,
   but `RESET_DATA + CMD53` (without redoing SBADDR) does not.
   That points at the backplane window state being lost or corrupted
   in a way the SBADDR writes restore. Worth a focused test:
   from REPL, RESET_DATA + CMD53 alone (no SBADDR writes), vs.
   re-issue SBADDR + CMD53 (no RESET_DATA). One of those should
   localise the variable.
2. **Compare brcmfmac/sdio.c's exact backplane-read sequence** in
   `~/github/linux/drivers/net/wireless/broadcom/brcm80211/brcmfmac/bcmsdh.c::brcmf_sdiod_readl`
   and friends. Look for anything between "function 1 enabled" and
   "first 4-byte backplane read" that the shim doesn't replicate.
3. **Don't add driver-side pulses speculatively** — every attempt
   this session regressed the REPL workaround. Whatever the fix is,
   it's not "pulse RESET_DATA somewhere extra".

### Disproven hypotheses from the 2026-05-11 session (don't re-test)

These were eliminated *before* the pin-contention fix and stayed
eliminated after. Preserved so future sessions skip them.

- **"GPCLK2 LPO isn't being generated."** VPU firmware leaves
  `CM_GP2CTL=0x291`, `CM_GP2DIV=0x00249f00` (MASH-1, exact 32 768
  Hz). Re-programming from Zephyr is counterproductive — the 30-50 µs
  LPO outage during KILL+restart disturbs the chip's PMU. **Leave
  VPU's GPCLK2 alone.**
- **"WL_REG_ON cycling wakes the chip."** Tested 20 ms / 100 ms /
  500 ms LOW with various HIGH-settle times up to 4 s. No change.
  VPU drives it HIGH before kernel runs; toggling makes things worse.
  **Inherit VPU state.**
- **"Ncr_min=5 lower bound."** Chip drives at Ncr=5; SDHCI spec wants
  Ncr ≥ 8. Disproven empirically.
- **"BCM2711 low-bus-clock hang."** `sdhci-iproc.c` documents that
  bug at 100 kHz × 500 MHz core_freq on Pi 4. Different SoC; doesn't
  apply here.
- **"Auto-clock-gating drops SDCLK during response window."** SDCLK
  runs continuously, verified by sampling pin 34.
- **"AxPROT / transaction-attribute filtering."** Userspace writes
  to FORCE_EVENT_ERROR registers ARE processed; ruled out.
- **"VPU has the SDHCI peripheral in a partial-power state."**
  Mailbox GET_POWER_STATE confirms device 0 (SD Card) is on. Required
  a `MBOX_SCRATCH` MMU region at 0x0F000000 + CONFIG_MAX_XLAT_TABLES
  bump from 8 to 12 to land the mailbox buffer in uncached DRAM
  addressable from MP REPL.
- **"VPU has clk_emmc gated or set to wrong rate."** Mailbox
  GET_CLOCK_STATE: on at 200 MHz. SET_CLOCK_STATE no-op.
- **"Kernel-context wall (need real ISR / spinlock / DSB)."** Was the
  prior session's leading hypothesis. Disproven by 2026-05-12: the
  actual wall was pin contention (above). Polled bare-metal flow works
  fine once the pads are routed correctly.

The auto-memory at
`~/.claude/projects/-Users-jep-github-SS/memory/project_rpi_zero_2w_sdio_debug.md`
has the longer investigation log.

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

- **CYW43436 (BCM43430A1) Wi-Fi/Bluetooth.** Pi Zero 2 W's wireless
  lives on SDIO. Polled `brcm,bcm2835-sdhci` driver at
  `drivers/sdhc/sdhc_bcm2835.c` enumerates the chip cleanly (`sd_init`
  succeeds: num_io=2, rca=0x0001, ocr=0xa0ffff00, 4-bit @ 25 MHz,
  CCCR diagnostics all read clean). The remaining open issue is the
  first 4-bit CMD53 — see "SDIO bring-up status (as of 2026-05-12,
  end of day)" above. Once that's cracked the next steps are firmware
  upload (`brcmfmac43430-sdio.{bin,txt,clm_blob}` via CMD53 block
  writes through the func-1 backplane window) and a brcmfmac-style
  WLAN driver. Linux reference at
  `~/github/linux/drivers/net/wireless/broadcom/brcm80211/brcmfmac/`.
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
