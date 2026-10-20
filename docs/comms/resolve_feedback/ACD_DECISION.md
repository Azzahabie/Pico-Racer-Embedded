# Decision: turn off DHCP address-conflict detection (ACD)?

**Status: ⏸ DEFERRED (20 Oct 2026).** ACD stays **on** for now: functionality
first, optimisation near the end. Option A is no longer the lead because the
Pico has to be robust; the lead is **C2** (§7). Raised 19 Oct 2026 from the
F1.4 / T2 measurements. Part of F1 (Week 6 feedback), and it feeds NFR-09.

> **When we circle back, start at §7** (20 Oct findings: option C is possible
> without patching the SDK, the NFR-09 fit, and the app-layer levers). Then
> pick an option in §4 and do the steps in §6. The raw evidence is in
> [`results.md`](results.md) (F1.0 "Lease time", F1.4, F1.5 T2 / T6).

---

## 1. The problem in one paragraph

Every time the car gets an IP address, lwIP first checks that nobody else on
the network already uses it: **ACD** (Address Conflict Detection, RFC 5227).
The check is a fixed series of ARP probes and announcements, and it costs
**6-9 s on every lease**: at boot, after every rejoin, and after every AP drop.
That is now the single largest piece of our reconnect time, and it pushes
recovery right up to **NFR-09's 15 s** communication-recovery target.

## 2. Where the time goes (measured, hotspot `Test1`)

| Step | Measured | Source |
|---|---|---|
| Notice the AP is gone (radio firmware, beacon loss) | ≈ 8 s | stopwatch, T2 |
| Back-off before rejoin | 3 s (fixed) | `CYW43_RETRY_BACKOFF_US` |
| Wait for the next retry while the AP is still off | 0-4 s (retry cycle ≈ 4 s, `join-failed(NONET)` after ~0.94 s) | F1.4 log |
| Join (link up) | 3.0-3.2 s | F1.4 log, every attempt |
| **Lease (WAIT_LEASE → ONLINE)** | **7.0-10.0 s**, of which ACD is 6-9 s | F1.4 log (7.78, 7.88, 7.04, 6.96, 7.34, 9.96 s) |
| Stale-session retry at boot (phone rejects first join) | +6.4 s per boot | T6 + F1.4 log |

**Why ACD is 6-9 s** (lwIP `core/ipv4/acd.c`, `include/lwip/prot/acd.h`, pinned
lwIP `77dcd25`). The address is only handed to DHCP (`ACD_IP_OK`) after the
**second announcement**:

```
PROBE_WAIT   random 0-1 s
probe 1 → wait random 1-2 s → probe 2 → wait random 1-2 s → probe 3
ANNOUNCE_WAIT 2 s → announce 1 → ANNOUNCE_INTERVAL 2 s → announce 2 → ACD_IP_OK
total: 0-1 + 2×(1-2) + 2 + 2 = 6-9 s
```

The DHCP exchange itself (DISCOVER/OFFER/REQUEST/ACK) is the remainder, about
1 s. The `waiting for lease` lines confirm it: the client sits in
`state=8` (`DHCP_STATE_CHECKING`, i.e. ACD) for almost the whole wait.
The REBOOTING path after a drop runs ACD too: an ACK in REQUESTING **or**
REBOOTING calls `dhcp_check()` (lwIP `core/ipv4/dhcp.c:1835-1840`). That's why
reconnect leases are just as slow as boot leases.

## 3. What it means for NFR-09 (15 s)

| Scenario | Today (ACD on) | ACD off (estimate) |
|---|---|---|
| AP comes back → ONLINE | 0-4 + 3.2 + 7-10 = **10-17 s** (measured: 10.2-13.2 s in F1.4 drop 2) | 0-4 + 3.2 + ~1 = **4-8 s** |
| Brief AP glitch → ONLINE (includes 8 s detection) | 8 + 3 + 3.2 + 7-10 = **21-24 s** | 8 + 3 + 3.2 + ~1 = **~15 s** |
| Power-on → ONLINE (hotspot, stale session) | **18.7-21.8 s** | ~12 s |

Telemetry also needs MQTT to (re)connect after ONLINE, which is on top of
these numbers. MQTT reconnect isn't built yet (M9 / FR-25). With ACD on, the
first row is already at the 15 s limit **before** MQTT, so NFR-09 can't be met
reliably. With ACD off there's ~7 s of margin.

## 4. Options

### A. Turn ACD off (recommended)
One line in `lib/libnet/lwip/include/lwipopts.h`:
```c
#define LWIP_DHCP_DOES_ACD_CHECK 0
```
`LWIP_AUTOIP` is already 0 (`lwipopts.h:33`), so `LWIP_ACD` then becomes 0 as
well (`opt.h`: `LWIP_ACD = LWIP_AUTOIP || LWIP_DHCP_DOES_ACD_CHECK`), and the
ACD code compiles out.
- ✅ Saves 6-9 s on **every** lease (boot, rejoin, drop).
- ✅ Simplest. No SDK patching, no new logic.
- ❌ If another device already uses the address the DHCP server hands us, we
  won't notice. Both devices then fight over it and the car's traffic (MQTT)
  breaks intermittently. With ACD, lwIP would DECLINE that address and ask for
  another.
- **How likely is that?** Low on a normal DHCP network: it needs a device
  with a manually set IP inside the DHCP pool, or a misbehaving DHCP server.
  The demo network is unknown (F1.8: not asked), which is the residual risk.

### B. Keep ACD, accept the time
- ✅ No conflict risk.
- ❌ NFR-09 is met only in the best case, and not at all once MQTT reconnect
  is added. The report would have to restate NFR-09 or log it as a known miss.

### C. Keep ACD but make it faster
- The timings (`PROBE_*`, `ANNOUNCE_*`) are plain `#define`s in the Pico SDK's
  lwIP (`include/lwip/prot/acd.h`), **not** overridable from `lwipopts.h`.
  Changing them means patching the SDK inside the Docker image.
- ❌ Fragile (lost on an SDK bump) and non-standard (RFC 5227 timings).
  Not recommended.

### D. Keep ACD at boot, skip it on reconnect
- That would mean ACD only when the address is new, not when re-confirming the
  same lease after a drop. lwIP has no option for this, so it needs a patch to
  `dhcp.c`. ❌ Same problems as C.

### E. Static IP (`WIFI_STATIC=1`), no DHCP at all
- ❌ The demo-day network is supplied externally (AGENTS.md §4), so we can't
  reserve an address. Not viable.

## 5. Related levers (separate decisions, same goal)

These are not part of this decision, but they attack the same NFR-09 budget:
1. **Faster first retry after `join-failed(FAIL)`.** The failed join already
   cleared the phone's stale session (T6), so the 3 s back-off before the
   second try is mostly wasted. A shorter back-off for the *first* failure
   (e.g. 0.5 s) would save ~2.5 s per boot. Small change in
   `update_net_state()`.
2. **Faster link-loss detection (the ≈ 8 s).** It's decided inside the CYW43
   firmware (beacon loss). Broadcom firmware usually has a beacon-timeout
   setting (`bcn_timeo` iovar), but **this is unverified for the CYW43439**.
   Investigate before relying on it.

## 6. Steps once decided

**If A (off):**
1. In `lib/libnet/lwip/include/lwipopts.h`, add
   `#define LWIP_DHCP_DOES_ACD_CHECK 0` with a comment pointing here. Keep
   `LWIP_ALTCP` and the other gates untouched (AGENTS.md §7).
2. Build the DNS and MQTT profiles through Docker (commands in
   [`Actions.md`](Actions.md) F1.0 / F1.3). Check for 0 warnings.
3. On hardware, hotspot: one boot plus one hotspot off/on. Record the
   `WAIT_LEASE → ONLINE` time from the `state=` lines' `t=` values.
   **Expect ≈ 1 s instead of 7-10 s.**
4. Record it in `results.md` (new subsection under F1.5, or T2/T6 re-runs),
   and set this page's status to ✅ DECIDED with the date.
5. Update F1.0's "Lease time" note in `results.md`, which currently says
   "Decide in F1.2 / after F1.5".
6. Report (F1.9): mention the trade-off under NFR-09 / A1.

**If B (keep):** set status to ✅ DECIDED (keep), and in F1.9 restate NFR-09
or record it as a known miss with the §3 numbers.

## 7. Findings, 20 Oct 2026 (option C explored, then deferred)

**Option C doesn't need the SDK patched.** The makefile lists lwIP sources by
name (`build_make/pico_rp2040.mk:405-413`, including `core/ipv4/acd.c`), so a
modified copy of `acd.c` can live in the repo and be swapped in there. Guard
it with a `$(error)` that fires if the SDK's `acd.c` changes. Pico SDK 2.2.0
pins lwIP `77dcd25`, the version read here.

**lwIP binds 2 s later than RFC 5227 requires.** RFC §2.3: "The host may begin
legitimately using the IP address immediately after sending the first of the
two ARP Announcements". lwIP calls `ACD_IP_OK` only after the second
(`acd.c:304-314`). That last 2 s detects nothing.

**The constants can't simply be lowered.** They are whole seconds used with
`%` (`acd.c:95-100`), so `PROBE_WAIT 0` or `PROBE_MAX == PROBE_MIN` divides by
zero. A real speed-up has to change the code (100 ms ticks).

**Option A also loses ongoing detection** (RFC §2.4: defending the address
after bind). C keeps it.

| | C1: bind after the first announcement | C2: C1 + shorter windows |
|---|---|---|
| Change | `ACD_IP_OK` after announcement 1; announcement 2 sent asynchronously | Also: probe wait 0-0.2 s, 3 probes 0.2-0.4 s apart, announce wait 0.5 s |
| ACD time | 6-9 s → 4-7 s | 6-9 s → ~0.9-1.5 s (estimate) |
| RFC 5227 | compliant | non-standard timing (§1.1 says the constants are fixed; §2.2 says mixed timings cause no problems and §2.4 catches late conflicts) |

C2's floor is about 1 s in total, because a Wi-Fi client in power save only
gets broadcast ARP at DTIM beacons (typically 100-300 ms). That is reasoning,
not a measurement. **Validate with a conflict test:** a laptop with a static
IP equal to the Pico's next lease must make the Pico decline and lease another
address, on both stock lwIP and C2.

**NFR-09 fit from today's data (ACD on).** The exact NFR-09 wording is not in
this repo (`docs/embedded .md` is missing), so the result depends on where the
15 s starts:

| Measured | Result | Within 15 s? |
|---|---|---|
| AP back → ONLINE (F1.4 drop 2) | 10.2-14.2 s | yes, narrowly |
| `link-lost` → ONLINE, brief drop (F1.4 drop 1) | 14.1 s | yes, by 0.9 s |
| `link DOWN` → `ip=` (T2, older build) | 15.1 / 21.1 s | no |
| AP vanishes → ONLINE (+ ≈ 8 s firmware detection) | ≈ 22 s | no |
| Power-on → ONLINE, hotspot stale session (T6) | 18.7-21.8 s | no, if NFR-09 covers boot |

None of these include MQTT reconnect, which is not built (M9 / FR-25).

**App-layer levers (no lwIP change).** ACD and the 8 s firmware detection
can't be reached from the application. What can:
1. `comms_task` polls the status seqlock every ~100 ms, not
   `WIFI_IDLE_MS` 3000 ms (`app_main.c:255`): up to 3 s saved.
2. Reconnect MQTT on the ONLINE edge with no delay, broker as an IP (no DNS):
   part of M9.
3. `-DCYW43_RETRY_BACKOFF_US=1000000ULL` (the `#ifndef` at `cyw43_utk.c:55`):
   ~2 s off every brief drop and every T6 stale-session boot.
4. Buffer telemetry while offline and set the Wi-Fi-loss fault bit at once.
   This doesn't shorten recovery.

Brief drop, `link-lost` → ONLINE: today 13-19 s, app levers only 11-14 s,
app levers + C2 ~6.5-7 s (MQTT not included). **Recommendation when we
circle back: C2 plus levers 1-4.**

---

## Decision log

| Date | Decision | By | Notes |
|---|---|---|---|
| 2026-10-19 | Raised, recommendation A | Buddy 1 + Claude | from F1.4 / T2 / T6 measurements |
| 2026-10-20 | **Deferred**, ACD stays on; lead option now C2 | Buddy 1 + Claude | A rejected for robustness; functionality first, optimise near the end. Findings in §7 |
