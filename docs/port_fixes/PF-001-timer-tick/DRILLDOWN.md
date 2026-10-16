# PF-001 drill-down: what's still open

The fix works for Wi-Fi, but parts of the explanation are **inferred** from
evidence rather than observed directly, and the fault may have affected more
than Wi-Fi. Close these before calling PF-001 done, and before the hotspot
runs (F1.0 part 2).

Status: ⬜ open · 🟡 in progress · ✅ closed. Record each result here, with
the date and the evidence.

| ID | Question | Why it matters | Status |
|---|---|---|---|
| D1 | What value does `WATCHDOG_TICK` actually hold at boot, before our write? | Confirms the cause directly instead of inferring it from the rate | ⬜ |
| D2 | Does the TIMER now run at exactly 1 MHz on every boot path, independent of Wi-Fi? | Proves the fix in general, not just "DHCP works" | ⬜ |
| D3 | Which boot paths leave the tick programmed: reflash (BOOTSEL/UF2), debug-probe (SWD) load, `picotool` reboot, power-on? | Explains why the bug hid, and which test setups were affected | ⬜ |
| D4 | Were the port's published TIMER-based figures taken on an affected boot? | `PORT_RP2040.md` latency, lock wait/hold and SMP scaling numbers would be 43× off | ⬜ |
| D5 | Is the omission inherited from upstream µT-Kernel, or introduced in this port? | Decides whether to report it upstream | ⬜ |
| D6 | Does any other subsystem misbehave on a power-on boot without the fix (USB console, future IMU/I2C, ultrasonic timing)? | Team-wide impact | ⬜ |
| D7 | Does the reset-value explanation hold (CYCLES reset value → divide-by-512)? | The 23.5 ms/s rate matches it, but this hasn't been checked against the datasheet | ⬜ |

## D1. Read the register at boot

**Experiment:** in `startup_clock()`, save `in_w(WATCHDOG_TICK)` to a static
variable *before* the new write, and print it from the app once the console is
up. Do this on a power-on boot and on a reflash boot.

- **Expected if the explanation holds:** power-on shows `CYCLES` ≠ 12 (and
  `ENABLE` set, giving the 512 behaviour). Reflash shows `ENABLE | 12` =
  `0x20C`.
- This is a temporary build; don't commit the print.

## D2. Measure the TIMER against the kernel clock

**Experiment:** a small test task that, every 10 s, prints the change in
`time_us_64()` alongside the change in kernel time (`tk_get_tim`). Run it in
the **base profile** (`make SMP=1 -j8`, no Wi-Fi), with and without the fix,
from a power-on boot.

- **Pass:** the ratio is 1000 µs per kernel ms (±0.1%).
- **Before the fix, from power-on:** ≈ 23.4 µs per ms.
- This also shows whether non-Wi-Fi builds were affected.

## D3. Boot-path matrix

Fill in with the D2 build, **without** the fix:

| Boot path | TIMER rate | Tick programmed? |
|---|---|---|
| Power-on (unplug/replug) | | |
| Reflash via BOOTSEL + UF2 drag | | |
| Debug probe (SWD) load and reset | | |
| `picotool reboot` / reset button (RUN pin), if available | | |

## D4. Re-check the port's published figures

`docs/PORT_RP2040.md` ("Measured latency", "Lock behaviour under load", "SMP
scaling") reports µs figures taken with `knl_arch_time_us()` / the TIMER.

- Find how the validation runs were started (the validation history phase
  tags). If they were always run straight after a debug-probe load or reflash,
  the figures are valid. Confirm that with D3.
- Re-run one benchmark (e.g. SMP local wake latency) **from power-on, with the
  fix**, and compare it with the published 16 / 16 / 43 µs.

## D5. Upstream check

The file header credits TRON Forum (2022/11). Tried on 2026-10-16:
`tron-forum/mtkernel_3` (master) and `tron-forum/mtk3_bsp` (master) have no
`pico_rp2040/cpu_clock.c`, and `tron-forum/mtk3_bsp2` (main) wasn't found at
that path. Still to do: find the original RP2040 BSP source this port was
imported from (validation phase 0, `phase0-baseline`), and check whether it
programs the tick.

## D6. Team impact sweep

Ask each buddy whether their code reads the TIMER (`time_us_64()`,
`0x40054000`) or the SDK timing helpers, e.g. for ultrasonic echo timing
(Buddy 5) or IMU timestamps (Buddy 4). Any such code was 43× wrong on
power-on before this fix.

## D7. Datasheet check

Confirm the `TICK` register's reset values (`ENABLE`, `CYCLES`) and what a
`CYCLES` value of 0 does, in the RP2040 datasheet (watchdog section). Then
update [`README.md`](README.md)'s "Cause" if the wording needs to change.
