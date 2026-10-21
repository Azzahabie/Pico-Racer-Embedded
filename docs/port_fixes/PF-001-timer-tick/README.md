# PF-001: Microsecond timer runs ~43× slow after a power-on

- **Status:** 🟡 Fix committed (`993ebc7`, branch `buddy_1`) and verified for Wi-Fi/DHCP. Cause
  confirmed at register level (D1), fix measured exact (D2), boot paths mapped (D3), published figures assessed (D4), origin traced (D5). D6 pending team replies
  ([`DRILLDOWN.md`](DRILLDOWN.md)).
- **Found:** 2026-10-16, by Buddy 1 during Week 6 feedback step F1.0 (Wi-Fi
  baseline).
- **File changed:** `kernel/sysdepend/pico_rp2040/cpu_clock.c`
  (`startup_clock()`).
- **Docs:** [`INVESTIGATION.md`](INVESTIGATION.md) ·
  [`FIX.md`](FIX.md) · [`DRILLDOWN.md`](DRILLDOWN.md). The raw console logs
  are in [`logs/F1log-2026-10-16.md`](logs/F1log-2026-10-16.md).

## Symptom

After **unplugging and replugging** the Pico W, Wi-Fi associated
(`[wifi] link UP`), but DHCP never completed. No `ip=` line appeared, not even
the 30 s timeout's `ip=0.0.0.0`, and ping failed, though the router listed the
Pico. After a **reflash**, the same firmware worked every time. The kernel,
console and LED looked completely normal throughout.

## Cause

The RP2040's TIMER block (the free-running microsecond counter behind
`time_us_64()`) advances once per "tick" from the **watchdog tick
generator**. That generator divides `clk_ref` (the 12 MHz crystal) and must be
programmed with a divisor of 12 to give 1 µs ticks. **The port never programs
it.**

- After a reflash, the boot ROM has already programmed it, and the reboot into
  the application doesn't reset it, so the timer is correct.
- After a power-on **or a reset-button (RUN pin) press** (D3), it keeps its
  reset value (CYCLES = 0). The timer was
  measured at **23.4375 µs per real ms (~43× slow), exactly 12 MHz ÷ 512**
  (D2). That is one tick per 512 `clk_ref` cycles instead of 12.
- **Confirmed directly (D1):** the register read at boot, before our fix
  writes it, held `0x600` (enabled, running, **CYCLES = 0**) on 6 / 6
  power-on boots, and `0x60c` (**CYCLES = 12**) after a reflash, where the watchdog reason
  showed the boot ROM's watchdog reboot. See [`DRILLDOWN.md`](DRILLDOWN.md)
  D1.

Kernel scheduling uses SysTick, not the TIMER, so tasks, delays and the
monitor were unaffected. That's why the fault was invisible except through
code that reads the TIMER.

## Fix

One register write in `startup_clock()`, right after `clk_ref` is switched to
the crystal: enable the tick generator with a divisor of `XOSC_MHz` (12). This
is what the Pico SDK's `clocks_init()` does. Details in [`FIX.md`](FIX.md).

## Who is affected

Everything that reads the TIMER (`0x40054000`) ran 43× slow on any power-on
boot before the fix. That applies to **every build profile**, not just Wi-Fi:
D2 reproduced it in the base profile with no Wi-Fi code at all.

| User | Where | Effect before the fix |
|---|---|---|
| lwIP clock `sys_now()` | `lib/libnet/lwip/lwip_utk.c:15-18` | All DHCP, ACD, DNS, ARP and TCP timers 43× late, so DHCP never completed |
| lwIP random seed | `lwip_utk.c:20-28` | Less entropy (minor) |
| CYW43 driver clock | `lib/libwifi/sysdepend/pico_rp2040/cyw43_configport.h:48-49` | Driver timeouts and delays 43× long |
| `time_us_64()`, `busy_wait_us()` | `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c:93-112` | Busy-waits 43× long |
| Wi-Fi join/DHCP/DNS timeouts | `cyw43_utk.c` (`CYW43_*_TIMEOUT_US`) | A 30 s timeout took ~21 min |
| TinyUSB millisecond clock | `lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c:11-15` | USB stack timeouts 43× long (no symptom seen) |
| Kernel lock statistics `knl_arch_time_us()` | `kernel/sysdepend/cpu/rp2040/smp_atomic.c:92-97`, used by `kernel/tkernel/smp_lock.c` | Lock wait/hold figures 43× too small |
| Time-handler diagnostics | `kernel/tkernel/time_calls.c:175-181` | `callback_max_us` 43× too small |

**Not affected:** task scheduling, `tk_dly_tsk`, cyclic handlers and alarms
(SysTick, `TIMER_PERIOD`).

**Open point:** the port's published measurements (latency, lock wait/hold,
SMP scaling in `PORT_RP2040.md`) are all TIMER-based. They are correct only if
each run started after a reflash or debugger load, not after a power-on. See
[`DRILLDOWN.md`](DRILLDOWN.md) D4.

## Verification so far

| Power-cycle boots | Got an IP | Timer rate |
|---|---|---|
| Before the fix (boots 2, 3, 5, 6, 8) | **0 / 5** | ~70 ms per 3 s |
| After the fix (boots 10-16) | **7 / 7** | ~3020 ms per 3 s |

The full table and boot notes are in
[`logs/F1.0-results-2026-10-16.md`](logs/F1.0-results-2026-10-16.md).

TIMER rate against the kernel clock, 60 s per run, base profile without
Wi-Fi ([`DRILLDOWN.md`](DRILLDOWN.md) D2):

| Image | Power cycle | Reflash |
|---|---|---|
| Without the fix | **23.436 µs/ms** (fault) | 999.956 µs/ms (hidden by the boot ROM) |
| With the fix | 999.956 µs/ms | 999.956 µs/ms |

(999.956 is a 2.6 ms measurement-start offset in the first window. The
steady-state windows read 1.000 MHz, −1 ppm.)
