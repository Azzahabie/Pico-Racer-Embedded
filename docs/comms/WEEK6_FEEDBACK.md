# Week 6 feedback: what it means for Buddy 1 (WiFi + Telemetry)

Source: [`reportFeedback.md`](reportFeedback.md) (instructor feedback, IS-24) on
[`Week6Report.md`](Week6Report.md). The feedback covers the whole team. This page
keeps only the points that touch `comms_task` / `cyw43_utk_task`, says how each
one applies to this subsystem, and turns it into steps.

The feedback ends with: *"The design is mature; now prove its assumptions on
hardware as early as possible rather than refining the documentation further."*
That applies here too. Most of the steps below are tests or code, not more prose.

> **Jargon.** Acronyms are spelled out the first time they appear. Every term
> (kernel objects, MQTT, Wi-Fi, report IDs) is defined in
> [`GLOSSARY.md`](GLOSSARY.md). The IDs used on this page:
>
> | ID | Means |
> |---|---|
> | **F1-F6** | The six feedback points on this page |
> | **FR-nn / NFR-nn** | Functional / Non-Functional Requirement in the report (what the car must do / how well) |
> | **A1, A6** | Report Section 7 assumptions owned by Buddy 1 (A1 = demo Wi-Fi network works, A6 = one app task is enough) |
> | **M7, M8** | Milestones: M7 telemetry framework, M8 command channel |
> | **Key Risk n** | The numbered "validate early" list at the end of the feedback |

---

## 1. At a glance

| # | Feedback point (paraphrased) | Ownership | Priority |
|---|---|---|---|
| F1 | ~70% association success (A1) → add a DHCP (Dynamic Host Configuration Protocol, how the car gets an IP address) retry and a timeout in WAITING FOR WI-FI; mission must start without Wi-Fi | **Mine** | 🔴 High (also Key Risk 5) |
| F2 | Document MQTT (the publish/subscribe messaging protocol) topics, payloads, command set and validation rules; define map versioning so a retained `…/map` (one the broker keeps and re-sends) isn't re-applied mid-mission | **Mine** | 🔴 High |
| F3 | Size the message queues `tele_mbfid`, `tele_pub_mbfid`, `cmd_mbx` from worst-case rates (M7); "command buffer full" = rejected command, not the Fault state | **Mine** (`bar_mbf` is Buddy 3's) | 🟠 Medium |
| F4 | Show NFR-11 (1 ms ± 0.5 ms) and NFR-18 (20 ms stop) hold **with Wi-Fi active on core 0** | **Shared**: Buddy 2/3/4 measure, I supply the Wi-Fi load | 🟠 Medium (Key Risk 4) |
| F5 | Pick an end-of-course mechanism (Assumption 1); it affects the MQTT commands | **Shared**: team decision, I implement it if it's a command | 🟡 Depends on team |
| F6 | GP28 (pin 28) reads VBAT/2 (half the battery voltage) only if the back-of-board jumper is soldered | **Touches me**: battery telemetry goes out through `comms_task` | 🟢 Low |

Points that are **not** mine: the HC-SR04 (ultrasonic distance sensor) at 3.3 V,
the GPIO (General-Purpose Input/Output pin) callback dispatcher,
the barcode/junction layout, hump-height slip, `bar_mbf` overflow and the motor
watchdog. I only need to know about them.

---

## 2. Each point in detail

### F1. Wi-Fi/DHCP reliability, and running without Wi-Fi 🔴

**What the feedback says.** With roughly 70% association success (A1), add a DHCP
retry and a timeout in WAITING FOR WI-FI, and make sure the mission can start
without Wi-Fi.

**How it concerns me.** This is the weakest point in my part of the report, and
it shows up in the code as well as the design:

- **Background.** Joining Wi-Fi takes two steps. *Association* is the handshake
  with the AP (Access Point, i.e. the router or hotspot), logged as "link UP".
  After that, *DHCP* asks the AP for an IP address (a *lease*). A1's failures
  were in both steps.
- **The driver gives up on DHCP after one try.** In
  `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c:468-472`, once
  `CYW43_DHCP_TIMEOUT_US` (30 s, line 46) passes without a lease, it sets
  `dhcp_result = ERR_TIMEOUT` and **latches** `dhcp_complete = 1` (a latch is a flag set once and never
  cleared). Nothing starts
  DHCP again. This matches the A1 symptom "certain startups no DHCP passed to
  Pico": that boot never gets an address.
- **Statechart 6.1 has a dead end.** BOOT → WAITING FOR WI-FI happens "on the
  first DHCP lease". If the first lease never comes, the car stays in BOOT for
  good, with no timeout and no retry edge.
- **"Mission can start without Wi-Fi" is implied but never stated.** Section 4
  says "Losing WiFi does not stop the mission", which covers *losing* it. But
  `cyw43_utk_task` reports the connection state (the CONN bit) on `nav_flg`
  (`nav_ctrl_task`'s event flag), and nothing says Ready doesn't wait
  for it. FR-02 only lists device init and calibration, so a reader can't be sure.

**Recommended steps.**
1. Add a retry path: when DHCP times out (or the link stays down past a bound),
   drop the attempt (`dhcp_release_and_stop` / leave and rejoin the AP), wait a
   short back-off, and try again. Clear the latch, or replace it with a state
   that can go back. Bound the attempts so the radio poll stays cheap. Also log
   the attempt count (e.g. `[wifi] dhcp retry 2/∞`) so A1 failures show up in
   evidence.
2. Consider cutting the 30 s timeout. A lease on a working AP normally arrives in
   a few seconds, so 10 s with retries recovers faster than one 30 s wait.
3. Update Statechart 6.1: add a **timeout → retry** edge out of BOOT / WAITING
   FOR WI-FI, and drop "never returns to BOOT" wording that only held because of
   the latch.
4. State in Section 5 / FR-02 that **Ready → Running does not depend on CONN**,
   and test it: boot with the AP switched off, calibrate, press start, and the
   car must drive. Turn the AP on mid-run and telemetry must appear within
   NFR-09's 15 s (the report's communication-recovery target).
5. Re-run A1 with the retry in place: **≥ 20 cold boots** on the Pico2_IoT router
   and on a phone hotspot, recording association, lease time and retries. Swap
   "~70%" for a measured figure. Get the demo-day AP details from the instructor
   as early as possible: it must be 2.4 GHz (the only band the Pico W radio
   supports), WPA2 security, and must not require PMF (Protected Management
   Frames, a WPA3 feature that is the suspected cause of the failed home-router
   join).

### F2. Document the MQTT interface, and fix map versioning 🔴

**What the feedback says.** Document the MQTT topics, payloads, command set and
validation rules, and define how map versioning stops a retained `…/map` being
re-applied mid-mission.

**How it concerns me.** Most of this already exists in
[`MQTT_SCHEMA.md`](MQTT_SCHEMA.md): the topics; the four record types `S`
(State, every 1 s), `E` (Event, when something happens), `H` (Heartbeat) and `D`
(Diagnostic snapshot); the commands (§8); and validation (§8.3-8.4). But **none
of it was in the submitted report**, so the
marker couldn't see it. There are also two real gaps:

- **The command set contradicts itself across the report.** Section 5 says Ready
  → Running can come from "an MQTT start command" and Paused from "an MQTT pause
  command". Section 9 says `comms_task` accepts only `snap`, `ping` and `map`.
  Section 9 "changes expected" lists "whether any remote mission command reaches
  nav_ctrl_task" as open. All three can't be true.
- **Map versioning only says "idempotent"** (applying it twice has the same
  effect as once). Each table the laptop sends carries a version `mv`, and the
  car reports the version it is running as `mpv`. Re-applying the *same* `mv` is
  harmless. But after a reconnect, the broker re-sends whatever table it has
  retained, and nothing says what happens if that table is **older** than the one
  the car runs (`mv < mpv`), or if a table swap lands while `barcode_task` is
  mid-decode.

**Recommended steps.**
1. Settle the command set with the team (and nav_ctrl_task's owner), then write
   it down once: `snap`, `ping`, `map`, plus `start` / `pause` / `resume` /
   `stop` **only if** the team keeps them. Give each command its arguments,
   accepted states, the reply it produces, and its reject reason.
2. Adopt and document an explicit map rule. Suggested:
   - **Apply only if `mv > mpv`.** `mv == mpv` is a no-op (a retained replay
     after reconnect). `mv < mpv` is rejected and counted, so a stale retained
     table can't roll the car back.
   - **After a reboot,** `mpv` starts at the compiled-in default (version 0), so
     the retained table applies once. That's intended.
   - **Mid-mission,** `barcode_task` swaps the table only between barcodes (as
     Section 4 already says). Decide with the team whether a remap during
     Running is allowed at all, or deferred until Ready/Paused.
   - The `mpv` echo stays the delivery proof.
3. Add a short "MQTT interface" appendix or table to the team report: topics with
   QoS (Quality of Service: 0 = fire and forget, 1 = acknowledged) and retain, one example line per record, the command table, and the rejection
   rules. Link `MQTT_SCHEMA.md` for detail instead of pasting all 400 lines.
4. Freeze the open schema items in `MQTT_SCHEMA.md` §12 (action-code meanings and
   the default table with Buddy 3).

### F3. Size the queues, and don't Fault on a full command buffer 🟠

**What the feedback says.** Size `tele_mbfid`, `tele_pub_mbfid`, `bar_mbf` and
`cmd_mbx` from worst-case rates (M7), define `bar_mbf`'s overflow policy, and
treat "command buffer full" as a rejected command rather than a system Fault.

**How it concerns me.** These are kernel queues between tasks: a *message
buffer* (`_mbf`) copies records in order, a *mailbox* (`_mbx`) passes messages
in order. Three of the four are mine. The report only says they exist, with no
sizes, and Section 9 defers the `tele_mbfid` size to M7.
The "command buffer full → Fault" rule is in Section 5's Fault list. Remote
commands come in through my path (`cmd_mbx` → `comms_task` → `mission_cmd_mbf`),
so a slow or flooding laptop could fault the car. That's the wrong behaviour for
a non-safety channel.

**Recommended steps.**
1. Size each buffer from **worst-case rate × longest time it can go undrained**,
   plus margin:

   The *overflow policy* is what happens when a queue is full.

   | Buffer | Producers → consumer | Inputs to collect | Overflow policy |
   |---|---|---|---|
   | `tele_mbfid` | 5 sensing tasks → `comms_task` | each producer's peak event rate (e.g. barcode burst, scan sweep records); `comms_task`'s longest gap (≤ 1 s cadence + worst map apply, from A6) | drop newest, count → `drp=` (already designed) |
   | `tele_pub_mbfid` | `comms_task` → `cyw43_utk_task` | records per 1 s tick (S + E burst + H + D); radio drain rate; worst stall during RETRY BACKOFF (2-5 s) | drop and count; while not STREAMING, keep only the latest S |
   | `cmd_mbx` | `cyw43_utk_task` → `comms_task` | realistic laptop command rate; one retained `map` after each resubscribe | reject newest, count, log one console line |

   Write the record sizes into `app_program/common/messages.h` (M7) so sizes come
   from `sizeof` instead of guesses.
2. Replace "full command buffer → Fault" with: **`comms_task` sends with
   `TMO_POL` (non-blocking: try once and return at once); if it's full, the command is rejected, counted and reported** (in
   the next `S`/`D` record or a reply line), and the mission state doesn't
   change. Agree this with nav_ctrl_task's owner and remove it from the Section 5
   Fault list.
3. Prove it: flood `…/cmd` from the laptop (e.g. 100 commands back-to-back) and
   show no Fault, a growing reject count, and an unbroken 1 s `S` cadence. This
   also closes A6.

### F4. Core 1 timing with Wi-Fi active 🟠

**What the feedback says.** Measure core 1 WCET (Worst-Case Execution Time, the
longest one run of a task can take) and show NFR-11 (line sensors sampled every
1 ms ± 0.5 ms) and NFR-18 (motors told to stop within 20 ms of an urgent-stop
request) hold with Wi-Fi active on core 0. It also notes the 1 ms cyclic handler
(the kernel timer that wakes a periodic task) needs a 1 ms kernel tick (the
RTOS's base timer interval; the template default is 10 ms, and nothing can be
timed more finely than one tick).

**How it concerns me.** The RP2040 has two cores. The design keeps the radio on
core 0 and the control tasks on core 1. The whole timing argument rests on "the radio is on core
0, so it never preempts (interrupts) core 1". That only holds as far as shared
resources allow, such as interrupt load, shared-bus contention and any lock or
critical section the radio driver (CYW43) or network stack (lwIP) takes. Buddies 2-4 do the measuring, but **I own the
"Wi-Fi active" condition** they measure under. NFR-10 also makes me responsible
for "without causing motion_task to miss its control deadline".

**Recommended steps.**
1. Provide a **worst-case Wi-Fi load build**: full chain with MQTT, streaming
   `S` every 1 s plus an `E` burst, a periodic `snap`/`map`, and one forced
   disconnect/reconnect during the test (RETRY BACKOFF is the busiest path).
2. Agree a GPIO-toggle convention with Buddies 2/3/4: each task flips a spare pin
   at the start and end of its work. Then one logic-analyser capture (a bench
   tool that records pins over time) shows the 1 ms / 10 ms ticks and the stop
   path, with the radio running.
3. Check the kernel tick setting with the team (1 ms needed). It also sets the
   resolution of my 2 ms radio poll and the 2-5 s back-off timer.
4. Measure my own NFR-10 side (status published at least once a second): the
   `S`-line interval jitter (how much the 1 s spacing varies) on the laptop, and
   CPU load (share of time the core is busy) on core 0 with the radio streaming.

### F5. End-of-course mechanism (Assumption 1) 🟡

**What the feedback says.** Assumption 1 is open, so Finished has no trigger.
Pick one mechanism now, because it affects `barcode_task`, `nav_ctrl_task` and
the MQTT commands.

**How it concerns me.** One of the three options is "an operator stop command",
which would be a new MQTT command on my side. The other two (end barcode,
distance) still need a "Finished" event in telemetry, plus the final highest hump
and distance.

**Recommended steps.**
1. Push the team to decide this week. Whichever option wins, add a `Finished`
   mission-state value and a final summary record (highest hump, distance,
   elapsed time) to the schema.
2. If it's an operator command, define it in F2's command table (accepted only in
   Running, with a reply).

### F6. GP28 battery sense jumper 🟢

**What the feedback says.** GP28 reads VBAT/2 only if the jumper on the back of
the Robo Pico is soldered. It isn't board-assigned by default.

**How it concerns me.** The report says battery voltage is "reported through
`comms_task`". If the jumper is open, the field will publish garbage.

**Recommended steps.** Check the jumper (a solder bridge on the back of the
board) with Buddy 3, who samples GP28 through the ADC (Analogue-to-Digital
Converter). If it's
open, either solder it or drop the battery field from the schema and say so.
Don't publish a value nobody has validated.

---

## 3. Suggested order of work

Hardware proof first, as the feedback asks:

1. **F1 code + test:** DHCP retry in `cyw43_utk.c`, then the 20-boot A1 re-test
   and the "start with AP off" test. *(Key Risk 5)*
2. **F2 decisions:** command set and map rule agreed with the team (short
   meeting), then the `mv > mpv` check goes into the M8 command code.
3. **F3 in M7:** record layouts in `messages.h`, buffer sizes from the table
   above, the `TMO_POL` reject path, then the command-flood test.
4. **F4:** hand the Wi-Fi load build to Buddies 2/3/4 and join the
   logic-analyser session.
5. **F5 / F6:** follow up on the team decision and the jumper check.
6. **Report update last:** Statechart 6.1 retry edge, MQTT interface appendix,
   A1 measured figure, and the Fault-list change. Small and evidence-driven, as
   the feedback asks.

## 4. Done when

- [ ] A1 has a measured success rate over ≥ 20 boots, with DHCP retry in place
- [ ] The car calibrates and drives with the AP off, and telemetry recovers within 15 s when the AP returns
- [ ] One command table (including any mission commands) agrees across Sections 5, 9 and `MQTT_SCHEMA.md`
- [ ] The map rule (`mv > mpv`, between-barcode swap) is documented and tested by replaying a stale retained map
- [ ] `tele_mbfid` / `tele_pub_mbfid` / `cmd_mbx` sizes are justified from rates, and a command flood causes no Fault
- [ ] NFR-11 / NFR-18 captures were taken with the Wi-Fi load build running
