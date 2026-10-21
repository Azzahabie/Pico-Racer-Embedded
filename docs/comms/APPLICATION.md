# APPLICATION — the layer on top

Buddy 1's **application-layer** work: the tasks that live in `app_program/`, what
they consume, and how they hand work down to the network layer.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Status:** seeded 2026-09-26; **updated from Tribunal 1 (2026-10-19)** —
  §3, §4, §6, §7, §8 and §9 now include Change Requirement 1 (minimum-bit data,
  binary MQTT, shared fault bitmask). The code starts at M7.

> **Updated from Tribunal 1 (2026-10-19).** The Week 8 requirement change in §8
> has arrived: **Change Requirement 1, minimum-bit data representation.** For
> this layer it means packed records in `messages.h`, packed binary MQTT
> payloads with a laptop decoder ([`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) schema v2),
> and setting the **Wi-Fi-loss** bit in the shared fault bitmask. The analysis,
> the gap check against the submitted report, and the ordered roadmap are kept
> in Buddy 1's working notes.
- **Companions:** [`NETWORKING.md`](NETWORKING.md) covers everything *below* this
  layer (transport, protocol, config, build); [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md)
  covers the application layer's two cross-task seams and the options for each;
  [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) is the subsystem design
  this roadmap is built to — tasks, queues, topics, wire format, test gates.
  General repo map: [`ARCHITECTURE.md`](../ARCHITECTURE.md).

---

## 1. What lives here

`app_program/` is the **application**: one folder per team member's subsystem
task, plus a shared contract file.

| Path | Owner | State |
|---|---|---|
| `app_program/app_main.c` | base repo | the producer/consumer demo + the Wi-Fi status reporter task |
| `app_program/demo_tasks.c` | base repo | the demo tasks (blink, monitor, …) |
| `app_program/led_test/` | base repo | GPIO loopback test |
| `app_program/motor_task/` | Buddy 2 | **empty stub** |
| `app_program/line_task/` | Buddy 3 | **empty stub** |
| `app_program/imu_task/` | Buddy 4 | **empty stub** |
| `app_program/obstacle_task/` | Buddy 5 | **empty stub** |
| **`app_program/comms_task/`** | **Buddy 1** | **empty stub — this is ours** |
| `app_program/common/messages.h` | shared | **empty** — the integration contract |

> **What it means:** the stubs are placeholders deliberately shipped so each
> person has a home for their code. They are *compiled* even while empty (every
> `.c` under `app_program/` joins the build), so an empty file is fine but a
> broken one stops the whole build.

---

## 2. The boundary — how this layer talks to the network

**The rule:** the network layer (lwIP) is `NO_SYS 1` — single-threaded and not
thread-safe. Only the radio-owner task may touch it. So `comms_task` **cannot**
call MQTT functions directly.

**The pattern:** hand work across with a kernel object, then the radio task
drains it.

```
   comms_task (our task)                     radio-owner task
   ────────────────────                     ────────────────
   gather telemetry from other buddies
   build the message
   tk_snd_mbf(...)  ──── message buffer ───▶  drains the buffer
                                              lwip_utk_mqtt_publish(...)
```

> **What it means:** a classic producer/consumer queue — and this repo already
> demonstrates it. `app_program/app_main.c` has a producer task and a consumer
> task passing records through a message buffer, guarded by a mutex and a
> semaphore. Our telemetry task is another producer; the network task is the
> consumer.

**The "Communication API" deliverable** is exactly this interface: the message
structs and kernel-object IDs in `app_program/common/messages.h`, plus the MQTT
topics/payloads.

> **In depth:** [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md) works this boundary out
> in full — the kernel-object primer, the three ways the buddies could feed
> `comms_task`, the three ways the one-shot MQTT module could grow into a
> persistent client, and the recommendation. Read it before writing `comms_task`.

---

## 3. Interfaces — what we consume, and from whom

| Source | Data we publish |
|---|---|
| Buddy 2 — Motion Control | straight/left/right state, distance moved |
| Buddy 3 — Line + Barcode | barcode read, decoded action |
| Buddy 4 — IMU + Terrain | hump count, hump height |
| Buddy 5 — Obstacle Scan | obstacle width, bypass direction |
| Vehicle Controller | mission state / coordination |

And **outbound to Buddy 3:** the live letter→action remap command.

*Added from Tribunal 1 (2026-10-19):*

| Shared object | Owner | What this layer does with it |
|---|---|---|
| Shared fault bitmask (µT-Kernel event flag) | Buddy 5, `nav_ctrl_task` | sets/clears the **Wi-Fi-loss** bit; copies all five fault bits into each `S` record |
| Packed record layouts in `messages.h` | Buddy 1 | defines them, with the pack/unpack helpers every producer calls |
| Buddy 2's ~54-bit motion record (8 B on the queue) | Buddy 2 | receives it on `tele_mbf` and folds it into `S` (format to reconcile) |
| Buddy 3's 2-bit action and 5-bit letter index | Buddy 3 | carries them in `S`, `E` and the 7-byte `map` |

> **Grading note:** "clean interfaces and teamwork" is an explicit assessment
> criterion, so `common/messages.h` is a shared file — route changes through a
> pull request so nobody's build breaks silently (the file says so itself).

---

## 4. Roadmap — M7 to M10

### M7 — Telemetry framework
- [ ] Define the event-log envelope: timestamped, ordered, reconstructable as a
      map of the run ("moved forward 20 cm, then turned right").
- [ ] Publish it on a regular cadence.
- [ ] Collect from the four data sources in §3.
- [ ] *(Tribunal 1)* Build a **JSON baseline first** and record its RAM and
      message sizes, then switch to the **packed binary** records — Week 13
      needs the before-and-after comparison.
- [ ] *(Tribunal 1)* Write the **laptop decoder** (Python) for schema v2.
- [ ] *(Tribunal 1)* Set the **Wi-Fi-loss fault bit** from the connection state
      and publish the fault bitmask in every `S` record.

### M8 — Command channel
- [ ] Subscribe to a command topic and dispatch by message type.
- [ ] **Snapshot:** on request, reply with current state at that instant
      (separate from the regular stream).
- [ ] **Live barcode remap:** the command carries the letter→action mapping and
      updates a runtime table (Buddy 3 reads it). **Must not require
      recompiling** — this is the hard demo-day requirement.
- [ ] *(Tribunal 1)* Commands are binary: `cmd` is 1 byte, `map` is 7 bytes
      (4-bit version + 26 × 2-bit actions). The laptop publisher packs them.

### M9 — Robustness
- [ ] Reconnect / recover on Wi-Fi drop.
- [ ] Heartbeat topic so liveness is visible.
- [ ] Must survive **all 3 demo attempts** — explicitly graded.

### M10 — Documentation
- [ ] Topics, payload formats, broker setup walkthrough. This doubles as the
      integration contract for the other four buddies.
- [ ] *(Tribunal 1)* Week 13 evidence: data-representation table, RAM
      comparison, message-size comparison, correctness and timing on the packed
      build.

---

## 5. Deliverables (from the briefing)

- [ ] Communication API
- [ ] Telemetry framework
- [ ] MQTT documentation
- [ ] Telemetry demonstration
- [ ] Continuous telemetry: movement, hump count + height, barcode + decoded
      action, current motion state (straight/left/right)
- [ ] Two-way: subscribe to commands, not just publish
- [ ] On-request snapshot of current state
- [ ] Live barcode mapping push (cannot be hardcoded/recompiled)
- [ ] Configurable broker target (dev = own laptop, demo day = instructor's)

---

## 6. Hard constraints for this layer

- [ ] **Never call networking directly** — hand work to the radio-owner task
      through a kernel object (`tk_snd_mbf`).
- [ ] **`tm_printf` is atomic per *call*, not per line** — compose with
      `tm_sprintf`, emit with one `tm_printf`.
- [ ] **Keep network work off the motion critical path** — the run must complete
      autonomously in under 5 minutes.
- [ ] **Config values, not constants** — the broker and network targets must stay
      in `config/*.h`.
- [ ] **Commit regularly** — GitHub history is tracked; don't dump work late.
- [ ] *(Tribunal 1)* **Minimum bits, masks and shifts.** Every field uses only
      the bits its range needs; no C bit-fields on anything that leaves the chip.
- [ ] *(Tribunal 1)* **No text on MQTT.** Debug text goes to the console only.
- [ ] *(Tribunal 1)* **No race on shared packed data.** One writer per packed
      word; cross-core bits through a kernel event flag.

---

## 7. Risks (ranked)

1. **Single-threaded lwIP** — design the hand-off (§2) before writing telemetry
   code. *(Transport itself is no longer a risk: M4, M5 and M6 are all proven.)*
2. **Telemetry must not block the car** — a 5-minute autonomous run with a
   penalty for being slow.
3. **"Explain every line, including AI-generated code."** Don't paste a generated
   MQTT client wholesale — port one you understand and can defend.
4. **Live barcode remap** — the single requirement that cannot be hardcoded; if
   it is compiled in, the demo fails.
5. *(Tribunal 1)* **Demo-day readability** — the instructor's generic client now
   shows hex, so the decoder must be ready before demo day.
6. *(Tribunal 1)* **`messages.h` drift** — a packed layout has no names to catch
   a mismatch; change `messages.h` and `MQTT_SCHEMA.md` in one PR (NFR-17).

---

## 8. Grading reminders

- [x] **Week 8:** a requirements change will be issued — must actually be
      implemented. **Issued: Change Requirement 1, minimum-bit data
      representation (Tribunal 1, 2026-10-19).** Week 13 must include the
      data-representation table, RAM and message-size comparisons, and
      correctness/performance evidence.
- [ ] **Week 13:** another requirements change — design/discussion only.
- [ ] Demo day: **3 attempts, only the latest counts**.
- [ ] Peer evaluation adjusts the group grade per person — carrying a weak
      teammate can move you up a band.

---

## 9. Open next actions

- [ ] Write the **topic/payload schema** (`docs/comms/MQTT_SCHEMA.md`) — the contract
      blocking all four teammates. The topics, the event catalogue and the wire
      format are already designed in
      [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §4, §8 and §9; the
      schema document is that design written for the other buddies rather than
      for you. (The wire format was revised to **JSON outbound** on 2026-09-28,
      then to **packed binary, schema v2, on 2026-10-19 (Tribunal 1)** —
      see §8 there.) **Rewritten for v2:**
      [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) — its §12 lists the fields still
      proposed, including every field width to agree with the other buddies.
- [ ] Reconcile the **task count, names and priorities** with the team document
      before writing `comms_task` — the team doc and the design disagree (see
      [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md); its §7
      decision log is the blank to fill).
- [ ] Define the kernel-object hand-off between `comms_task` and the radio task.
- [ ] (Optional, low priority) Reconcile `GETTING_STARTED.md` §6b task table with
      the actual five tasks — `led_test` is missing from the doc.

---

## 10. Don't redo — already true

- Don't clone `sirfonzie/mtk3smp-rp2040` separately — this repo already *is* that
  base kernel plus team scaffolding.
- Don't look for CMake — it is a plain GNU make build.
- Don't rebuild the Wi-Fi layer — scan, join, DHCP, DNS, UDP, TCP and **MQTT** all
  already work (see `NETWORKING.md`).
- Don't expect the Wi-Fi driver in `app_program/comms_task/` — the real code lives
  in `lib/libwifi/` and `lib/libnet/`; this folder is the *application* task.