# Linux SDIO bring-up traces

These are ftrace dumps captured from a patched Linux kernel
(`raspberrypi/linux rpi-6.12.y` at 6.12.87) on the same Pi Zero 2 W
hardware, while it successfully enumerates the CYW43436 wireless chip
over SDIO. Used as the byte-for-byte reference for the Zephyr SDHCI
driver in `drivers/sdhc/sdhc_bcm2835.c`.

The patched source tree lives at `~/rpi-linux` on the Ubuntu VM
(`claude@192.168.1.21`). Rebuild and re-instrument from there if more
trace points are needed.

## Files

### `linux-bcm2835-mmc-writel-only.txt` (4.1 MB)

First pass. `trace_printk` only in `bcm2835_mmc_writel` and
`mmc_raw_writel`. Captures every MMIO write the driver makes to the
SDHCI block at `0x3F300000` from cold boot through brcmfmac
enumeration. Use this when you only need to confirm the register write
sequence.

Output format:
```
TASK-PID  CPU  flags  TIMESTAMP: function_name: WR offset=value from=N clk=hz
```

### `linux-bcm2835-mmc-deep.txt` (4.8 MB)

Second pass. Adds `trace_printk` to:

- `bcm2835_mmc_readl` (every MMIO read)
- `bcm2835_mmc_irq` entry
- `bcm2835_mmc_cmd_irq` entry (with intmask)
- `bcm2835_mmc_finish_command` entry (with opcode)
- `bcm2835_mmc_reset` entry (with reset mask)
- `bcm2835_mmc_set_clock` entry (with target clock)
- `bcm2835_mmc_set_ios` entry (with ios state)
- `bcm2835_mmc_tasklet_finish` entry
- `bcm2835_mmc_send_command` entry (with opcode/arg/flags)

Use this when you need the full function-call sequence and read values
around a specific event.

## Key timing markers (deep trace)

```
t=2.889015  first reset (probe begin)
t=2.914131  iter1 CMD52 sdio_reset READ  -> CMD_TIMEOUT
t=2.919061  iter1 CMD52 sdio_reset WRITE -> CMD_TIMEOUT
            (1.4 s gap, no register activity at all)
t=4.327493  iter2 set_ios (mmc_power_up)
t=4.328867  iter2 CMD0  -> CMD_COMPLETE (no response expected)
t=4.331663  iter2 CMD8  -> CMD_TIMEOUT  (SDIO ignores, expected)
t=4.332062  iter2 CMD5 inquiry -> CMD_COMPLETE, resp=0x20FFFF00 ✓
t=4.332399  iter2 CMD5 OCR     -> CMD_COMPLETE, resp=0xA0FFFF00 ✓
t=4.332847  iter2 CMD3 (RCA assign) -> CMD_COMPLETE
t=4.333180  iter2 CMD7 (select)     -> CMD_COMPLETE
t=4.333518+ CCCR enumeration via CMD52 reads
```

## What these traces prove (and don't)

The Zephyr driver issues IDENTICAL register writes to what Linux does
in the iter2 CMD5 success path — same offsets, same values, same order.
Yet CMD5 from Zephyr (or from Linux `/dev/mem` userspace) times out
because the controller's RX state machine fails to engage on the
chip's electrically-present response. The discriminator between
"works" and "doesn't work" is not anywhere observable at the SDHCI
register level — it's in the kernel-driver runtime context (real
registered IRQ handler that the ARMC intc unmasks, spinlock-held MMIO
within the request lifetime, tasklet-deferred state-machine reset).

See `../HANDOVER.md` SDIO bring-up section for the full diagnosis
and `~/.claude/projects/-Users-jep-github-SS/memory/project_rpi_zero_2w_sdio_debug.md`
for the longer chronological investigation log.
