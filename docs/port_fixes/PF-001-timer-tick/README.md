# PF-001: Microsecond timer runs ~43× slow after a power-on

- **Status:** 🟡 Fix applied (uncommitted) and verified for Wi-Fi/DHCP. Drill-down
  open ([`DRILLDOWN.md`](DRILLDOWN.md)).
- **Found:** 2026-10-16, by Buddy 1 during Week 6 feedback step F1.0 (Wi-Fi
  baseline).
- **File changed:** `kernel/sysdepend/pico_rp2040/cpu_clock.c`
  (`startup_clock()`).
- **Docs:** [`INVESTIGATION.md`](INVESTIGATION.md) ·
  [`FIX.md`](FIX.md) · [`DRILLDOWN.md`](DRILLDOWN.md). The raw console logs
  are in [`../../comms/resolve_feedback/F1log.md`](../../comms/resolve_feedback/F1log.md).

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
- After a power-on, it keeps its reset value. The timer was measured running at
  **23.5 ms per real second (~43× slow)**, consistent with one tick per 512
  `clk_ref` cycles instead of 12.

Kernel scheduling uses SysTick, not the TIMER, so tasks, delays and the
monitor were unaffected. That's why the fault was invisible except through
code that reads the TIMER.

## Fix

One register write in `startup_clock()`, right after `clk_ref` is switched to
the crystal: enable the tick generator with a divisor of `XOSC_MHz` (12). This
is what the Pico SDK's `clocks_init()` does. Details in [`FIX.md`](FIX.md).

## Who is affected

Everything that reads the TIMER (`0x40054000`) ran 43× slow on any power-on
boot before the fix:

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
[`../../comms/resolve_feedback/results.md`](../../comms/resolve_feedback/results.md) F1.0.
