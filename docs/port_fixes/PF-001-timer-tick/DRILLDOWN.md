# PF-001 drill-down: what's still open

The fix works for Wi-Fi, but parts of the explanation are **inferred** from
evidence rather than observed directly, and the fault may have affected more
than Wi-Fi. Close these before calling PF-001 done, and before the hotspot
runs (F1.0 part 2).

Status: ⬜ open · 🟡 in progress · ✅ closed. Record each result here, with
the date and the evidence.

| ID | Question | Why it matters | Status |
|---|---|---|---|
| D1 | What value does `WATCHDOG_TICK` actually hold at boot, before our write? | Confirms the cause directly instead of inferring it from the rate | ✅ power-on 6/6: CYCLES = 0; reflash: CYCLES = 12 |
| D2 | Does the TIMER now run at exactly 1 MHz on every boot path, independent of Wi-Fi? | Proves the fix in general, not just "DHCP works" | ✅ fix: 1.000 MHz (−1 ppm) on power-on and reflash; no-fix power-on: 23.4375 kHz, reproduced without Wi-Fi |
| D3 | Which boot paths leave the tick programmed: reflash (BOOTSEL/UF2), debug-probe (SWD) load, `picotool` reboot, power-on? | Explains why the bug hid, and which test setups were affected | ✅ power-on and **reset button** clear the tick (affected); UF2 reflash keeps it (hides the bug). SWD not testable (no probe) |
| D4 | Were the port's published TIMER-based figures taken on an affected boot? | `PORT_RP2040.md` latency, lock wait/hold and SMP scaling numbers would be 43× off | ✅ assessed: unitless/count results unaffected; absolute µs figures consistent with a correct timer (UART-bound cross-check), but not re-provable here |
| D5 | Is the omission inherited from upstream µT-Kernel, or introduced in this port? | Decides whether to report it upstream | ✅ inherited from TRON Forum's Pico BSP, where it was latent; became live in `sirfonzie/mtk3smp-rp2040`, which added TIMER readers |
| D6 | Does any other subsystem misbehave on a power-on boot without the fix (USB console, future IMU/I2C, ultrasonic timing)? | Team-wide impact | 🟡 no affected code exists yet (teammate branches are empty); ultrasonic echo timing is the planned high-risk user; team notice drafted |
| D7 | Does the reset-value explanation hold (CYCLES reset value → divide-by-512)? | The 23.5 ms/s rate matches it, but this hasn't been checked against the datasheet | ✅ reset CYCLES = 0 (header, D1), and D2 measured 23.4375 µs/ms = exactly 12 MHz ÷ 512 |

## D1. Read the register at boot

**Experiment:** in `startup_clock()`, save `in_w(WATCHDOG_TICK)` *before* the
new write, and print it from the app once the console is up. Do this on a
power-on boot and on a reflash boot.

**How it's built (2026-10-16, temporary, marked `TEMP PF-001 D1`; removed after the drill-down, never committed):**
- A static variable can't hold it: `startup_clock()` runs from
  `knl_startup_hw()` (`reset_hdl.c:55`) **before** `.data`/`.bss` are
  initialised (`reset_hdl.c:71-84`). `.noinit` doesn't help either, because
  the linker script puts it inside `.bss`
  (`etc/linker/pico_rp2040/tkernel_map.ld:62-66`), so it's cleared too.
- So the value is parked in **WATCHDOG `SCRATCH0`** (`0x4005800C`), a
  hardware register that RAM initialisation doesn't touch. The SDK and boot
  ROM only use `SCRATCH4-7`.
- `app_main.c` prints three `[pf001]` lines after the banner:
  1. `tick_at_boot` (the saved value) and `tick_now`, decoded into
     `enable` / `cycles` / `running`;
  2. `wd_reason` (watchdog REASON: `timer`, `force`) and `chip_reset`
     (`por` = power-on, `run` = RUN pin, `psm_restart` = debugger). These
     identify the boot path, so they also feed D3;
  3. how many µs the TIMER advanced over a `tk_dly_tsk(1000)`: should be
     ~1 000 000 with the fix (a partial D2 check).
- Build: DNS profile, `mtk3pico_smp1_usb_cdc_wifi_dns.uf2`.

**Expected if the explanation holds:**

| Boot | `tick_at_boot` | Boot-path flags |
|---|---|---|
| Power-on | `0x200` (enable=1, cycles=0) | `por=1`, `wd_reason=0` |
| Right after a UF2 reflash | `0x20c` or `0x60c` (cycles=12, maybe running=1) | `wd_reason` ≠ 0 (the boot ROM reboots via the watchdog) |

**Results:**

Raw capture: [`logs/D1-2026-10-16.log`](logs/D1-2026-10-16.log) (11
power-on boots, 21:53-21:55).

**Power-on boots (all 11 were power cycles):**

| Boot | tick_at_boot | wd_reason | chip_reset | timer µs / 1000 ms |
|---|---|---|---|---|
| 1 | **`0x600`** cycles=0 | `0x0` | `0x100` (por=1) | 1 005 049 |
| 2 | (partial)* | `0x0` | `0x100` | 1 005 082 |
| 3 | (timer only)* | - | - | 1 005 075 |
| 4 | **`0x600`** cycles=0 | `0x0` | `0x100` | 1 005 047 |
| 5 | **`0x600`** cycles=0 | `0x0` | `0x100` | 1 005 015 |
| 6 | (partial)* | `0x0` | `0x100` | 1 005 068 |
| 7 | **`0x600`** cycles=0 | `0x0` | `0x100` | 1 005 032 |
| 8 | (timer only)* | - | - | 1 005 066 |
| 9 | (timer only)* | - | - | 1 005 030 |
| 10 | **`0x600`** cycles=0 | `0x0` | `0x100` | 1 005 021 |
| 11 | **`0x600`** cycles=0 | `0x0` | `0x100` | 1 005 028 |

\* Earlier lines printed before the serial terminal reattached after USB
re-enumeration. It's a capture gap, not a firmware result. Where
`tick_now` was captured (all full boots, plus boot 2), it reads `0x60c`.

**Reflash reference** (earlier capture, 2026-10-16 21:48:34, first boot
straight after a UF2 reflash):
```
[pf001] tick_at_boot=0x60c (enable=1 cycles=12 running=1)  tick_now=0x60c (enable=1 cycles=12 running=1)
[pf001] wd_reason=0x1 (timer=1 force=0)  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[pf001] timer advanced 1004956 us over a 1000 ms kernel delay
```

**Conclusion (2026-10-16): ✅ D1 closed. The cause is confirmed directly.**
- **Power-on: 6 / 6 full captures read `tick_at_boot = 0x600`** (enabled,
  running, **CYCLES = 0**), exactly the reset state in the SDK header (`0x200`,
  plus the read-only RUNNING bit). Nothing before our code programs it. All
  8 boots with a `wd_reason` line read `0x0`.
- **Reflash:** CYCLES was already **12**, with `wd_reason = timer`. The boot ROM
  programmed the tick, then rebooted into our app through a watchdog reset,
  which preserves it. That is why the bug only appeared after a power cycle.
- **The fix applies every time:** `tick_now = 0x60c` on every boot where it was
  captured.
- **Telling boot paths apart:** use `wd_reason` (`0` on power-on, `timer`
  after a reflash), not `por`. `chip_reset` reads `por=1` on both, because
  HAD_POR is sticky until the next chip-level reset, and a watchdog reset
  isn't one. This carries over to D3.
- **Timer rate with the fix:** 11 / 11 boots read 1 005 015-1 005 082 µs per
  `tk_dly_tsk(1000)` (spread 67 µs). The +0.5% is the kernel delay, not the
  timer: µT-Kernel adds one `TIMER_PERIOD` (10 ms) to every timeout
  (`kernel/tkernel/timer.c:125-126`), so the real delay is 1000-1010 ms
  depending on tick phase. A tight ~1005 ms cluster is exactly that. A
  precise rate check needs a longer window against the kernel clock (D2).

## D2. Measure the TIMER against the kernel clock

**Experiment:** a small test task that, every 10 s, prints the change in
`time_us_64()` alongside the change in kernel time (`tk_get_tim`). Run it in
the **base profile** (`make SMP=1 -j8`, no Wi-Fi), with and without the fix,
from a power-on boot.

- **Pass:** the ratio is 1000 µs per kernel ms (±0.1%).
- **Before the fix, from power-on:** ≈ 23.4 µs per ms.
- This also shows whether non-Wi-Fi builds were affected.

**How it's built (2026-10-16, temporary, marked `TEMP PF-001 D2`; removed after the drill-down, never committed, test images deleted):**
- Profile: `make SMP=1 CONSOLE=usb_cdc -j8`. No Wi-Fi, no lwIP, no CYW43, so
  nothing but the kernel and USB console touch the TIMER. (USB-CDC rather
  than the UART default, so the same COM-port setup works.)
- `cpu_clock.c` has a compile-time switch `PF001_APPLY_FIX` (default `1`).
  Two images were built from the same tree:

  | Image (in `build_make/`) | Fix |
  |---|---|
  | `pf001_d2_fix.uf2` | compiled in |
  | `pf001_d2_nofix.uf2` | compiled **out** (the pre-fix port) |

  `startup_clock()` records which one is running in WATCHDOG `SCRATCH1`,
  so the console says so. The source is left at `1`.
- Measurement: after the D1 lines, `app_main.c` reads the TIMER
  (`TIMERAWL`) and the kernel's operating time (`tk_get_otm`, SysTick-driven
  ms) every 10 s for 60 s, and prints the per-window and cumulative ratio in
  **µs per kernel ms**. The kernel clock has 10 ms resolution, so the
  cumulative figure is good to ~0.02% after 60 s.
- Third reference: the terminal's own timestamps. 6 × 10 s lines should be
  ~10.0 s apart in the log too. That checks SysTick itself against the PC
  clock.
- Output:
  ```
  [pf001] D2 1/6: window kernel=10010 ms timer=10010000 us (1000.000 us/ms)  cumulative 10010 ms -> 1000.000 us/ms
  ...
  [pf001] D2 done: fix compiled IN, tick cycles=12
  ```

**Results (2026-10-16)**, raw capture:
[`logs/D2-2026-10-16.log`](logs/D2-2026-10-16.log).

| Run | Image | Boot path | `tick_at_boot` / `wd_reason` | 1 s check | Window 1 | Windows 2-6 | Cumulative (60 s) | `D2 done` |
|---|---|---|---|---|---|---|---|---|
| R1 | fix | power cycle | `0x600` (cycles 0) / `0x0` | 1 004 901 µs | 999.741 | 999.992-1000.007 | **999.956 µs/ms** | fix IN, cycles=12 |
| R2 | fix | reflash | `0x60c` (cycles 12) / `0x1` | 1 004 810 µs | 999.740 | 999.992-1000.007 | **999.956 µs/ms** | fix IN, cycles=12 |
| R3 | nofix | power cycle | `0x600` (cycles 0) / `0x0` | **23 554 µs** | 23.431 | 23.437 | **23.436 µs/ms** | fix OUT, **cycles=0** |
| R4 | nofix | reflash | `0x60c` (cycles 12) / `0x1` | 1 004 812 µs | 999.741 | 999.991-1000.008 | **999.956 µs/ms** | fix OUT, cycles=12 |

**Conclusion: ✅ D2 closed.**
- **The fix gives an exact 1 MHz timer on both boot paths.** Over windows
  2-6 of R1 (power-on, fix), the TIMER advanced 50 049 951 µs against
  50 050 ms of kernel time: **−1 ppm**, which is crystal tolerance, far inside
  the ±0.1% pass mark. R1, R2 and R4 agree to within a few µs per window.
- **The fault reproduces with no Wi-Fi at all (R3).** The no-fix image on a
  power-on reads **23.4375 µs/ms**, exactly 12 MHz ÷ 512 (234 609 µs per
  10 010 ms window = 23.4375 × 10 010). That settles D7: CYCLES = 0 behaves
  as a 512-cycle tick. So **every build profile of the port was affected on
  power-on**, not just Wi-Fi builds.
- **R4 shows exactly how the bug hid.** The *same no-fix image* runs at a
  correct 1 MHz when started by a reflash (the boot ROM left CYCLES = 12), and
  at 43× slow when started by a power cycle (R3). Nothing in the firmware
  differs between those two runs.
- **The cumulative 999.956 isn't a timer error.** Window 1 is short by
  ~2.6 ms of TIMER time in every run (R3's 56 µs short = 2.4 ms × 0.0234 is
  the same offset at the slow rate). The window's start reads kernel time
  first and the TIMER a few ms later, after the previous `tm_printf` returns.
  That's a measurement-start artefact; windows 2-6 have none.
- **Windows are 10 010 ms, not 10 000:** µT-Kernel adds one `TIMER_PERIOD`
  to every timeout (`kernel/tkernel/timer.c:125-126`), as already seen in D1.
- **SysTick against the PC clock:** in R1, the log timestamps for lines
  2→6 span 50.035 s against 50.050 s of kernel time (−0.03%, within USB
  timestamp jitter). The kernel tick is sound; only the TIMER was wrong.
- **USB console under the fault (R3):** TinyUSB's millisecond clock reads the
  TIMER (`usb_tinyusb_glue.c:11-15`), so it ran 43× slow in R3, yet the
  console worked normally for the whole run. No visible USB symptom (D6
  note).

## D3. Boot-path matrix

Fill in with the D2 build, **without** the fix:

| Boot path | TIMER rate | Tick programmed? | `wd_reason` | Source |
|---|---|---|---|---|
| Power-on (unplug/replug) | **23.4375 µs/ms (43× slow)** | **No** (CYCLES = 0) | `0x0` | D2 R3, D1 (6/6) |
| Reflash via BOOTSEL + UF2 drag | 1.000 MHz | Yes, by the boot ROM (CYCLES = 12) | `0x1` (timer) | D2 R4, D1 reflash reference |
| **Reset button (RUN pin)** | 1.000 MHz with the fix | **No** (CYCLES = 0 at boot, `chip_reset` run=1) | `0x0` | D3 log, 2/2 boots |
| Debug probe (SWD) load and reset | - | - | - | ⏭ not testable: no probe available |
| `picotool reboot` | - | - | - | ⏭ not tested |

**Results (2026-10-16)**, raw capture:
[`logs/D3-2026-10-16.log`](logs/D3-2026-10-16.log). Fix image
(`pf001_d2_fix.uf2`), reset button pressed twice.

| Boot | `tick_at_boot` | `wd_reason` | `chip_reset` | Windows 2-6 | `D2 done` |
|---|---|---|---|---|---|
| 1 (22:18:31) | **`0x600`** (cycles 0) | `0x0` | `0x00010000` (por=0 **run=1**) | 999.992-1000.007 | fix IN, cycles=12 |
| 2 (22:19:40) | **`0x600`** (cycles 0) | `0x0` | `0x00010000` (por=0 **run=1**) | 999.992-1000.007 | fix IN, cycles=12 |

**Conclusion: ✅ D3 closed for every boot path available here.**
- **The reset button is an affected path, like a power cycle.** A RUN-pin
  reset clears the watchdog tick back to CYCLES = 0. So on the pre-fix port,
  **pressing the reset button would also have made the TIMER 43× slow.** That
  matters for the car: a reset on the course would have broken Wi-Fi/DHCP the
  same way a power cycle did.
- With the fix, both reset-button boots run at an exact 1 MHz (same
  windows as D2 R1/R2).
- **`chip_reset` now tells all three paths apart:**

  | Boot path | `chip_reset` | `wd_reason` |
  |---|---|---|
  | Power-on | `por=1` | `0x0` |
  | Reset button | `por=0 run=1` | `0x0` |
  | UF2 reflash (boot ROM watchdog reboot) | `por=1` (sticky from the last power-on) | `0x1` (timer) |

- **Only the boot ROM's watchdog reboot (the UF2 path) preserves the tick.**
  Every chip-level reset (power-on, RUN pin) clears it.
- **SWD not tested:** no debug probe. That's relevant to D4: if the port's
  validation runs were loaded and started over SWD, whether the tick was
  set depends on how the probe resets the chip. Leave that question to D4.

## D4. Re-check the port's published figures

`docs/PORT_RP2040.md` ("Measured latency", "Lock behaviour under load", "SMP
scaling") reports µs figures taken with `knl_arch_time_us()` / the TIMER.

- Find how the validation runs were started (the validation history phase
  tags). If they were always run straight after a debug-probe load or reflash,
  the figures are valid. Confirm that with D3.
- Re-run one benchmark (e.g. SMP local wake latency) **from power-on, with the
  fix**, and compare it with the published 16 / 16 / 43 µs.

**Findings (2026-10-16):**

1. **The original harnesses and logs aren't available.** `PORT_RP2040.md:17-19`
   says the phase harnesses and per-phase hardware logs "are not part of this
   release". This repo has **no git tags** (history starts at `75eeb0f First
   Commit`, 2026-09-14), so the `phase*` tags in the validation history live in
   the upstream `mtk3smp-rp2040` project. The pointer to
   `README.md#performance-numbers` is also **dead**: this repo's `README.md` is
   the Docker setup guide and has no such section. So a like-for-like re-run
   (the plan above) isn't possible from here.

2. **How the runs were most likely started.** The only documented flash method
   is BOOTSEL + UF2 drag (`GETTING_STARTED.md` §5: "no separate flash tool and
   no debug probe required"). Per D3, the first boot after a UF2 flash comes
   from the boot ROM's watchdog reboot, which **keeps** the tick at CYCLES = 12.
   So any result captured on the first boot after flashing had a correct TIMER.
   A result captured after a replug, power-on or reset-button press would read
   **42.7× too small**. The qualified console was UART (via a separate adapter),
   which doesn't need a replug to attach, so first-boot capture is the likely
   pattern. That's likely, not proven.

3. **Which published results the timer could affect at all:**

   | Result in `PORT_RP2040.md` | Depends on TIMER? | Verdict |
   |---|---|---|
   | Phase pass counts (58/58, 11/11, 71/71, 65/65...) | No | Unaffected |
   | Lock contentions (23 470), audits, lock op counts, zero failures | No (counts) | Unaffected |
   | SMP scaling **1.799× / 44.4% less time** | Ratio of two TIMER durations | Unaffected, *if* both runs were on the same boot type (a ratio cancels the scale) |
   | Golden checksums | No | Unaffected |
   | Latency 16/16/43, 17/17/24, 2/3/31 µs | **Yes** | At risk: see 4 |
   | Max lock wait 36 µs / hold 85 µs | **Yes** | At risk: see 4 |
   | Scaling medians 1 383 657 / 768 751 µs | **Yes** (absolute) | At risk; the ratio isn't |
   | USB print storm 475 151 µs | **Yes** | Cross-checked in 4 |

4. **Plausibility cross-check (no harness needed).** If a figure came from an
   affected boot, the true value is 42.7× larger:
   - **USB print storm, 475 151 µs:** 128 framed `tm_printf` calls "including
     the synchronous UART mirror" at 115 200 baud (≈ 11.5 bytes/ms). At an
     estimated 30-60 characters per frame, that's 3.8-7.7 kB, i.e. **0.33-0.67 s
     of UART time**, and the published 0.475 s falls squarely inside it. Read
     as an affected boot, the true time would be ~20 s, which is ~40× slower
     than the UART forces. **This figure was measured with a correct
     timer.** Put the other way round: 0.475 s of UART time is 5 472 bytes, or
     **~43 characters per frame**, a sensible framed line. For the figure to
     come from an affected boot (true time ~20.3 s), every frame would need
     to be ~1 800 characters long.
   - **Single-core local wake 2 / 3 / 31 µs:** at 125 MHz, 2-3 µs = 250-375
     cycles, a sensible wake + dispatch on a Cortex-M0+. Read as affected, the
     minimum would be ~85 µs = ~10 700 cycles, implausibly slow for this path.
   - **SMP local / cross-core wake 16-17 µs** (~2000 cycles with the big
     kernel lock and an inter-core interrupt): plausible. Read as affected,
     ~700 µs, which isn't.

**Conclusion: ✅ D4 assessed (closed with residual risk).**
- Every count, pass/fail and ratio result in `PORT_RP2040.md` is unaffected
  by PF-001.
- The absolute µs figures are **consistent with a correct timer**. The one
  figure with an independent physical bound (the UART-limited print storm)
  confirms it, and the latency figures are only plausible at the correct
  scale. Nothing suggests any of them came from an affected boot.
- **Residual risk:** they can't be *proven* without the upstream harnesses and
  logs. From now on, with the fix in place, every boot path gives a correct
  TIMER, so future measurements no longer depend on how the board was started.
- **Doc follow-ups (not done, they're `PORT_RP2040.md` edits):** fix or remove
  the dead `README.md#performance-numbers` link, and add a note that the µs
  figures predate PF-001 and assume a first-boot-after-UF2 capture. The note
  already added under "Validation history" covers the second point.

## D5. Upstream check

**Where this repo's port comes from** (per the upstream README):

```
tron-forum/mtk3_bsp  branch pico_rp2040, tag v1.00.00.B5-pico_rp2040, commit 15ed232 (2023-05-24)
        │  single-core µT-Kernel 3.0 BSP for Raspberry Pi Pico, by TRON Forum
        ▼
sirfonzie/mtk3smp-rp2040  main ("Initial public release", 2026-08-30)
        │  dual-core SMP port + lwIP/CYW43 Wi-Fi + TinyUSB console
        ▼
this repo  (75eeb0f First Commit, 2026-09-14)
```

(The first attempt on 2026-10-16 looked only at `tron-forum/mtk3_bsp` master,
`mtk3_bsp2` and `mtkernel_3`. None of those has any RP2040 or Pico file, in any
branch or release tag 3.00.06-3.00.08 / BSP2 1.00.00-1.00.04. The Pico BSP
lives only on `mtk3_bsp`'s `pico_rp2040` branch.)

**Findings (2026-10-16):**

1. **The omission is inherited from TRON Forum, unchanged.**
   `kernel/sysdepend/pico_rp2040/cpu_clock.c` is byte-identical (ignoring line
   endings) across all three:
   [TRON Forum `v1.00.00.B5-pico_rp2040`](https://github.com/tron-forum/mtk3_bsp/blob/v1.00.00.B5-pico_rp2040/kernel/sysdepend/pico_rp2040/cpu_clock.c)
   = [`sirfonzie/mtk3smp-rp2040` main](https://github.com/sirfonzie/mtk3smp-rp2040/blob/main/kernel/sysdepend/pico_rp2040/cpu_clock.c)
   = this repo before PF-001. None of them programs the watchdog tick. The
   only diff against this repo is the PF-001 fix itself.

2. **In TRON Forum's original BSP the bug was latent.** None of its 41
   RP2040/Pico `.c`/`.h` files reads the TIMER: the TIMER registers appear only
   as `#define`s in `sysdef.h`. The kernel tick is SysTick, and the BSP's
   physical timers (`lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c`) use the
   PWM blocks. Nothing there could observe a slow TIMER.

3. **It became a live bug in the SMP port**, which added the first TIMER
   readers. All of these are new in `sirfonzie/mtk3smp-rp2040`:
   - `knl_arch_time_us()`: `kernel/sysdepend/cpu/rp2040/smp_atomic.c:92-97`
     (lock and time-handler statistics);
   - TinyUSB's millisecond clock: `lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c:11-15`;
   - `time_us_64()` for lwIP and CYW43: `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c:93-112`,
     `lib/libnet/lwip/lwip_utk.c`.

   Those modules were written assuming SDK behaviour (the SDK's
   `clocks_init()` starts the tick). The port keeps TRON Forum's clock init,
   which doesn't, and nothing reconciled the two.

**Conclusion: ✅ D5 closed.**
- **Report to `sirfonzie/mtk3smp-rp2040`: yes.** The bug is live there in every
  profile on power-on or reset-button boots (D2, D3). Their qualification
  workflow (first boot after a UF2 flash) is exactly the path that hides it.
  The issue should include the D2 numbers (23.4375 µs/ms vs 1.000 MHz), the
  D1 register values and the one-line fix.
- **Report to `tron-forum/mtk3_bsp`: optional, low priority.** It's harmless in
  their tree as shipped, but anyone who adds TIMER-based code (or the Pico SDK's
  timer helpers) on top of their BSP will hit it. A short note suggesting
  `startup_clock()` start the tick, as the SDK does, would prevent that.
- This doesn't change our fix. It confirms that the right place for it is the
  inherited `startup_clock()`, not the SMP or Wi-Fi code.

## D6. Team impact sweep

Ask each buddy whether their code reads the TIMER (`time_us_64()`,
`0x40054000`) or the SDK timing helpers, e.g. for ultrasonic echo timing
(Buddy 5) or IMU timestamps (Buddy 4). Any such code was 43× wrong on
power-on before this fix.

**Findings (2026-10-16):**

1. **No teammate code exists yet.** After `git fetch`, the remote branches
   `imu_task`, `line_task`, `motor_task` and `obstacle_task` are all still at
   `75eeb0f First Commit`, and their `app_program/*_task/` folders are 3-line
   stubs. Nothing to test today. This check is about what they're *going* to
   write.

2. **Complete list of TIMER readers in the tree today**, from a full scan of
   `kernel/ lib/ device/ app_program/ include/`. It matches the
   [`README.md`](README.md) table, with nothing new:
   - kernel: `knl_arch_time_us()` (`smp_atomic.c`), used by `smp_lock.c`
     (lock wait/hold stats) and `time_calls.c` (`callback_max_us`);
   - USB: `tusb_time_millis_api()` (`usb_tinyusb_glue.c`);
   - Wi-Fi: `time_us_64()` / `busy_wait_us()` (`cyw43_utk_compat.c`), used by
     the CYW43 driver, lwIP's `sys_now()` and random seed, and `cyw43_utk.c`'s
     timeouts;
   - the temporary PF-001 test code in `app_main.c`.

   The inherited RP2040 drivers (`device/i2c`, `device/adc`, `device/ser`)
   don't read the TIMER.

3. **Telemetry timestamps are safe.** The MQTT `up` field uses
   `tk_get_otm()` (`MQTT_SCHEMA.md:115`), the SysTick kernel clock, which was
   never affected.

4. **There's no shared µs-time API outside Wi-Fi builds.** `time_us_64()` is
   compiled only with `WIFI=cyw43` (`pico_rp2040.mk:388`, `WIFI_LOCAL_SRCS`),
   and `knl_arch_time_us()` is kernel-internal. A teammate who needs µs
   timing in a non-Wi-Fi build will likely read `0x40054028` directly, and
   that is correct **only with PF-001 in place**.

5. **Planned uses, from the Week 6 design report:**

   | Buddy / task | Planned timing | Uses the TIMER? | Pre-fix effect on power-on | Risk |
   |---|---|---|---|---|
   | 5: `scan_task` / ECHO_ISR | "ECHO_ISR timestamps the rising and falling edges of the HC-SR04 echo" | **Yes, almost certainly** (µs pulse widths) | Every distance reads **42.7× too short**: an obstacle at 1 m (5.8 ms echo) would read ~2.3 cm, so constant false STOP_SCAN | 🔴 High |
   | 2: `motion_task` / ENC_ISR | Counts per 10 ms tick; stall timeout `[t_stall]` | Only if speed is measured as the time between edges | Speed 42.7× too high; stall never detected | 🟠 If edge-timing is used |
   | 4: `imu_task` | 10 ms cyclic sampling; complementary-filter dt | Only if dt comes from timestamps rather than the fixed 10 ms period | Filter dt 42.7× too small, so pitch and hump height wrong | 🟠 If timestamps are used |
   | 3: `ir_sense_task` / `barcode_task` | Bar widths counted in 1 ms samples | No, if counted in samples | — | 🟢 Low |
   | All: NFR-11/18 and WCET measurements (feedback F4) | "Log consecutive sampling and task-start times" | **Yes**, if logged in software | Measured intervals and WCETs 42.7× too small, i.e. falsely passing | 🔴 High (for evidence) |
   | 1: `comms_task` | `up` = `tk_get_otm()`; lwIP/CYW43 timers | Yes, via lwIP/CYW43 | DHCP never completes (this incident) | ✅ Fixed by PF-001 |

**Conclusion: 🟡 closed for the current code. Open until each buddy
confirms.**
- Nothing in the tree outside the known list is affected, and the known list
  is fixed by PF-001.
- The fix must reach every teammate before they write timing code, otherwise
  their tests will pass after a reflash and fail after a power cycle or reset,
  exactly as Wi-Fi did.
- Recommendations (not done; team decisions):
  1. **Merge PF-001 to `master` first**, so every task branch inherits it.
  2. **Add one shared µs-time helper** available in every profile (e.g.
     `time_us_32()` reading `TIMERAWL`, in `app_program/common/` or `lib/`), so
     nobody hand-rolls register reads or depends on the Wi-Fi-only
     `time_us_64()`.
  3. **Add a boot self-check:** at startup, compare the TIMER against the
     kernel clock over ~100 ms and print a loud warning if it's off by more
     than 1%. That stops PF-001 from silently coming back (e.g. a future clock
     change that drops the line). It costs ~100 ms of boot time.
  4. **Measure NFR-11 / NFR-18 / WCET with a GPIO toggle and logic analyser**
     where possible (the design already plans this for NFR-18). It doesn't
     depend on any on-chip clock.

**Team notice (draft, for Buddy 1 to send):**

> **Heads-up: timer bug found and fixed in the RTOS port (PF-001)**
>
> On every power-on or reset-button boot, the RP2040's microsecond timer
> (TIMER block / `time_us_64()` / register `0x40054028`) was running **43×
> slow**. After a UF2 reflash it was fine, which is why it hid. The kernel
> tick (`tk_dly_tsk`, cyclic handlers, `tk_get_tim/otm`) was never affected.
>
> **Does it affect you?** Only if your code measures time with the TIMER:
> e.g. ultrasonic echo pulse width (Buddy 5), time between encoder edges
> (Buddy 2), or IMU dt from timestamps (Buddy 4). Counting samples or
> ticks is fine.
>
> **What to do:**
> 1. Pull the fix once it's on `master` (one line in
>    `kernel/sysdepend/pico_rp2040/cpu_clock.c`).
> 2. If you use the TIMER, test from a **power cycle or reset button**, not
>    just right after flashing.
> 3. Reply if your code reads the TIMER, so I can add it to the impact list.
>
> Details: `docs/port_fixes/PF-001-timer-tick/README.md`

## D7. Datasheet check

Confirm the `TICK` register's reset values (`ENABLE`, `CYCLES`) and what a
`CYCLES` value of 0 does, in the RP2040 datasheet (watchdog section). Then
update [`README.md`](README.md)'s "Cause" if the wording needs to change.

**Result (2026-10-16), from the Pico SDK 2.2.0 register header
`hardware_regs/include/hardware/regs/watchdog.h` (generated from the
datasheet):**
- `WATCHDOG_TICK_RESET = 0x200`: `ENABLE_RESET = 1`, `CYCLES_RESET = 0x000`.
  **So at power-on the tick generator is enabled with CYCLES = 0.**
- Bit layout: `CYCLES` 8:0, `ENABLE` 9, `RUNNING` 10, `COUNT` 19:11 (read-only).
- The SDK's `tick_start()` (`hardware_ticks/ticks.c`) notes that on RP2040
  this tick "also provides a tick reference to the timer and SysTick". Our
  kernel's SysTick evidently runs from the processor clock instead, which is
  why scheduling was unaffected.
- D1 observed CYCLES = 0 with RUNNING = 1 at power-on. **D2 R3 then measured
  23.4375 µs per kernel ms over 60 s, which is exactly 12 MHz ÷ 512.** So
  CYCLES = 0 behaves as a 512-cycle tick (9-bit counter wrap). ✅ Closed.