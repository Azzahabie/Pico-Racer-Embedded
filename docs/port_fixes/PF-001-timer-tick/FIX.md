# PF-001 fix: program the watchdog tick generator

## The change

`kernel/sysdepend/pico_rp2040/cpu_clock.c`. This is the complete diff against
commit `f1cb60f`:

```diff
@@ -22,6 +22,13 @@
 #include <tk/tkernel.h>
 #include "sysdepend.h"
 
+/* Watchdog tick generator (RP2040 datasheet 4.7, WATCHDOG TICK register) */
+#ifndef WATCHDOG_TICK
+#define WATCHDOG_BASE		0x40058000
+#define WATCHDOG_TICK		(WATCHDOG_BASE+0x2C)
+#define WATCHDOG_TICK_ENABLE	(1<<9)
+#endif
+
@@ -104,6 +111,14 @@ EXPORT void startup_clock(ATR clkatr)
 
 	out_w(CLK_REF+CLK_x_DIV, div);
 
+	/* Microsecond tick for the TIMER block (time_us_64(), lwIP, CYW43).
+	 * The tick generator divides clk_ref (= XOSC) down to 1 MHz.  The boot
+	 * ROM programs it on the BOOTSEL path and a reboot into the application
+	 * does not reset it, but after a power-on it keeps its reset value and
+	 * the timer runs ~43x slow -- so set it here, as the SDK's clocks_init()
+	 * does. */
+	out_w(WATCHDOG_TICK, WATCHDOG_TICK_ENABLE | XOSC_MHz);
+
 	/* CLK SYS settings */
```

`startup_clock()` is called once at boot from
`kernel/sysdepend/pico_rp2040/hw_setting.c:207`.

## Why it's correct

- **Register:** WATCHDOG block at `0x40058000`, `TICK` at offset `0x2C`.
  Bit 9 `ENABLE` starts the generator. Bits 8:0 `CYCLES` set how many
  `clk_ref` cycles make one tick.
- **Value:** `clk_ref` is switched to the crystal immediately before this
  line (`cpu_clock.c`, CLK_REF settings), and the crystal is `XOSC_MHz`
  (12) MHz. So `CYCLES = 12` gives exactly one tick per µs, the rate the TIMER
  and every caller of `time_us_64()` assume.
- **Placement:** after `clk_ref` is stable on the crystal, so the divisor
  applies to the right source. It sits before any TIMER user runs
  (`startup_clock()` runs in hardware setup, before the kernel and tasks
  start).
- **Precedent:** the Pico SDK does the same in `clocks_init()` (watchdog tick
  start with `XOSC_MHZ`). Our port doesn't use the SDK's clock init, which is
  how the step was lost.
- **Idempotent:** writing the same value the boot ROM already set (the reflash
  path) changes nothing.
- **Guarded defines:** `#ifndef WATCHDOG_TICK` keeps the file compiling if a
  shared header defines these later.

## How to verify

Use a Wi-Fi DHCP build with the F1 diagnostic line (or any code that prints
`time_us_64()` alongside the console timestamp):
1. Flash, then **unplug and replug** (not reflash).
2. Pass: `t=` in `[wifi] waiting for lease` rises ~3000 ms per 3 s line, and
   `[wifi] ip=` appears.
3. Fail (the old behaviour): `t=` rises ~70 ms per line, and DHCP never
   completes.

Results so far: 7/7 power-cycle boots pass ([`README.md`](README.md)). A
profile-independent check is planned in [`DRILLDOWN.md`](DRILLDOWN.md) D2.

## How to revert

Remove the two hunks above (or `git checkout -- kernel/sysdepend/pico_rp2040/cpu_clock.c`
before it's committed). Reverting brings back the power-on fault.

## Status and ownership

- Uncommitted on branch `buddy_1` as of 2026-10-16.
- It's a **shared kernel-port change**: agree it with the team before it lands
  on `master`. It changes the timing seen by Wi-Fi, USB and the kernel's lock
  statistics on every power-on boot.
