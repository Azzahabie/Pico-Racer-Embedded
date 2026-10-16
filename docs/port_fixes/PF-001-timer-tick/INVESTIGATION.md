# PF-001 investigation: how the slow timer was found

All boots on router `Pico2_IoT`, profile
`SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1`
(`mtk3pico_smp1_usb_cdc_wifi_dns.uf2`). The boot numbers refer to
[`F1log.md`](../../comms/resolve_feedback/F1log.md) and the F1.0 table in
[`results.md`](../../comms/resolve_feedback/results.md).

## Timeline

### 1. Symptom: link UP, no IP

Task F1.0 (20-boot Wi-Fi baseline) started. Boots showed
`[wifi] link UP rssi=… mac=…`, then nothing: no `[wifi] ip=` line after
3 minutes, and ping from the laptop failed. The router listed the Pico. The
firmware and config were unchanged since M6 (commit `42722af`), when DHCP
worked.

### 2. First diagnostic: DHCP state line

To see what DHCP was doing, a console line was added (DNS profile only):
`[wifi] waiting for lease: state= tries= t= addr= gw= dns= polls=`. It prints
every 3 s while linked without a lease. It needed one new field,
`dhcp_state`, in `T_CYW43_UTK_STATUS`.

The first boot with it succeeded:
`state=8` (lwIP `DHCP_STATE_CHECKING`, i.e. ACD, Address Conflict Detection)
for ~9 s, then `ip=192.168.50.8`.

### 3. Hypothesis A: IP clash on `.8` ❌ ruled out

The Pico was given `192.168.50.8`, which is also the broker/laptop address in
`config/broker_config.h:37`. If the laptop held `.8`, ACD would decline the
lease in a loop. The user reserved addresses in the router.
**Failures continued**, so this wasn't the cause. (The `.8` overlap is still a
separate config issue for MQTT builds.)

### 4. Pattern: reflash works, power cycle fails

The diagnostic was removed, and boots 1-6 were logged:

| Boot | How started | Got IP |
|---|---|---|
| 1, 4 | right after a **reflash** | ✅ |
| 2, 3, 5, 6 | **power cycle** (quick, 30 s, 1 min off) | ❌ |

Off-time didn't matter; reflash vs. power cycle did. A second clue: with the
radio task running, the firmware *must* print `ip=0.0.0.0` 30 s after link
UP (`cyw43_utk.c` DHCP timeout). Total silence meant either the radio task
had stopped, or its 30 s wasn't 30 s.

### 5. Hypothesis B: radio task hung ❌ ruled out

On failing boots, `[monitor]` kept printing and the LED kept blinking, so the
kernel and console were alive. The diagnostic was re-added with `polls=`
(radio loop count), `stage=` (last step entered in the loop) and `rd=`
(status reads that caught the radio mid-update).

Boot 8 (power cycle) showed `polls` rising steadily and `stage=7`
(sleeping): **the radio task was alive.**

### 6. Root cause: the timer is slow ✅

Boot 8 also showed DHCP stuck at `state=1` (REQUESTING), `tries=1`, while `t=`
(milliseconds since link UP, from `time_us_64()`) did this:

| Log time (real) | `t=` |
|---|---|
| 20:45:29.0 | 55 ms |
| 20:45:32.0 | 126 ms |
| 20:45:35.1 | 196 ms |
| … every 3 s | +70.5 ms |

**70.5 ms per 3 s = 23.5 ms per second: the TIMER was ~43× slow.** On the
successful reflash boot 7, `t=` went 1739 → 4759 → 7779 ms, i.e. a correct
3020 ms per 3 s.

That explains every symptom:
- lwIP's `sys_now()` is built on the TIMER, so its DHCP retry (2 s) took
  ~85 s, ACD's ~9 s took ~6 min, and our 30 s timeout took ~21 min. Hence no
  progress and no timeout line.
- Association doesn't need those timers, so the router saw the Pico.
- The kernel tick uses SysTick, so everything else looked healthy.

Why the TIMER was slow: its tick comes from the watchdog tick generator. A
search of the repo found **no code that programs it**. `cpu_clock.c`
configures the crystal, PLLs and `clk_ref`/`clk_sys`, but not the tick. The
measured rate matches 12 MHz ÷ 512 = 23 437 ticks/s. Why a reflash hides the
fault: the boot ROM programs the tick, and the reboot into the application
doesn't reset it. (Inferred at this point. **Later confirmed directly in
[`DRILLDOWN.md`](DRILLDOWN.md) D1:** at boot, the register held CYCLES = 0
after a power-on and CYCLES = 12 after a reflash.)

### 7. Fix and verification

The tick was programmed in `startup_clock()` (see [`FIX.md`](FIX.md)):
- Boot 9 (reflash with the fix): ✅ `ip=` in ~9 s.
- Boots 10-16 (power cycles): ✅ 7/7, `t=` +3020 ms per 3 s line, `ip=` 15-18 s
  from power-on.
- Boot 12 started with `state=1 tries=2` (a REQUEST retry was needed), and the
  retry fired and succeeded. Before the fix, boot 8 sat at `tries=1` forever.

## Side findings (not part of this fix)

- **ACD costs ~9 s per lease** (`state=8`). It's lwIP's default
  (`LWIP_DHCP_DOES_ACD_CHECK`). It matters for NFR-09 (15 s recovery) and is
  tracked in the Wi-Fi work (F1.2), not here.
- **The radio poll runs every ~20 ms, not 2 ms.** `polls` rises ~50 per
  second because `tk_dly_tsk(CYW43_POLL_MS = 2)` is rounded up by the 10 ms
  kernel tick. That's relevant to Week 6 feedback F4 (1 ms kernel tick).
