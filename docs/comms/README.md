# WiFi + Telemetry (Buddy 1) — start here

The one page to read for the **current state** of Buddy 1's subsystem. Every
other document in this folder is *detail behind a link* — open one only when you
need it.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Last updated:** 2026-10-19 — **updated from Tribunal 1** (binary MQTT, shared
  fault bitmask: §1, §3, §4, §6, §7); §3 pointer to F1 / ACD; the rest is as of
  2026-10-10
- **Tasks:** `comms_task` — the application task (in `app_program/comms_task/`,
  an empty stub); `cyw43_utk_task` — the radio owner, library code that already
  exists (`lib/libwifi/`)
- **One line:** turn car events into MQTT telemetry, and apply the laptop's live
  barcode-remap and `snap` commands — without ever touching lwIP directly.

> **Updated from Tribunal 1 (2026-10-19) — Change Requirement 1.** Every data
> field now uses the minimum bits for its range; **MQTT payloads are packed
> binary, not JSON** ([`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) schema v2, with a
> laptop decoder); fault conditions are bits in **one shared fault bitmask**
> (a kernel event flag owned by Buddy 5), and Buddy 1 sets the **Wi-Fi-loss**
> bit. What it changes for this subsystem, whether the submitted report covered
> it, and the ordered roadmap: **[`Tribunal/README.md`](Tribunal/README.md)**.
> The submitted report itself is the frozen baseline
> [`Tribunal/completed_in_trib.md`](Tribunal/completed_in_trib.md).

---

## If you think in threads and queues

`comms_task` is a new **backend service** that turns car events into MQTT
messages. It talks to the network only through a queue, because the library
underneath is single-threaded and exactly one task may touch it. Everything else
— the queues, the state machine, the retry/backoff, the wire format — is what
you'd build for any shared, non-reentrant resource with many producers.

| Embedded term | Software-engineering equivalent |
|---|---|
| Task | Thread |
| Priority / `itskpri` | Scheduling priority — **lower number = higher priority**, inverted from `nice` |
| CPU affinity (`TP_PRC1`) | `taskset` / thread affinity — the task may only run on core 0 |
| Message buffer (`tk_snd_mbf` / `tk_rcv_mbf`) | A bounded, thread-safe queue / channel (MPSC: many producers, one consumer) |
| `TMO_POL` | Non-blocking `try_send` / `offer()` — fails immediately if full |
| `TMO_FEVR` | Blocking `put()` — waits forever |
| Seqlock | Optimistic read: read a version counter, copy, re-check, retry — lock-free, like an MVCC snapshot read |
| Latch (`dhcp_complete`) | A one-way flag / `AtomicBoolean` — set once, never reset |

(This is the useful half of the retired `WIFI_SUBSYSTEM_SUMMARY.md`; the rest of
that file restated [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md).)

---

## 1. Status at a glance

| Phase | Milestones | State | Detail |
|---|---|---|---|
| Radio / transport | **M0–M6** | ✅ **proven on hardware** | [`NETWORKING.md`](NETWORKING.md) §5 |
| Application: telemetry framework | **M7** | 🔜 **next** | [`APPLICATION.md`](APPLICATION.md) §4 |
| Application: command channel | M8 | ⏳ not started | [`APPLICATION.md`](APPLICATION.md) §4 |
| Application: robustness | M9 | ⏳ not started | [`APPLICATION.md`](APPLICATION.md) §4 |
| Application: documentation | M10 | 🟡 schema rewritten as binary v2 (2026-10-19, Tribunal 1); widths still proposed (§12) | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) |
| Tribunal 1 evidence (Week 13) | — | 🟡 race test measured; size table calculated; RAM / message-size comparisons not yet | [`Tribunal/README.md`](Tribunal/README.md) §5 |

> **What it means:** the hard, risky half is already done — the Wi-Fi/MQTT
> transport runs on the board and is captured as evidence. What remains is the
> *application* layer (`comms_task`), which the M7–M10 roadmap covers.

---

## 2. What works today (the proof)

All of M2–M6 were verified on hardware (see [`NETWORKING.md`](NETWORKING.md) §5):

| # | Milestone | Evidence line |
|---|---|---|
| M2 | Wi-Fi association | `[wifi] link UP  rssi=-37  mac=…` |
| M3 | DHCP + DNS | `[wifi] ip=192.168.50.7  gw=192.168.50.1  dns=8.8.8.8` |
| M4 | UDP echo | `sent=64 recv=64 matched=64/64` |
| M5 | TCP echo | `tcp: 1 sessions, 3072 bytes` |
| M6 | **MQTT connect + publish** | `[mqtt] 192.168.50.8:1883 … result=0 published=1` |

---

## 3. Current focus

> **Right now (19 Oct 2026): Week 6 feedback F1, Wi-Fi reliability.** F1.0-F1.4
> are done: the car now rejoins and re-leases on its own and logs every
> connection state change. **Next session, start at
> [`resolve_feedback/ACD_DECISION.md`](resolve_feedback/ACD_DECISION.md)**, an
> open decision on turning off DHCP address-conflict detection (6-9 s per
> lease, NFR-09). Then continue F1.5 in
> [`resolve_feedback/Actions.md`](resolve_feedback/Actions.md). M7 below
> follows after F1.
>
> **Updated from Tribunal 1 (2026-10-19):** after F1, the order is the
> Tribunal roadmap in [`Tribunal/README.md`](Tribunal/README.md) §4 — agree the
> field widths, write `messages.h`, M7 with a JSON baseline, then binary with
> the decoder, then the Wi-Fi-loss fault bit. **Week 10 target: binary
> telemetry reaching the laptop.**

### M7 (telemetry framework)

**Goal:** a timestamped, ordered event log published on a cadence, collected from
the four buddies (motion, line/barcode, IMU, obstacle) plus the vehicle
controller. See [`APPLICATION.md`](APPLICATION.md) §4.

**Next concrete step:** define the `comms_task` → `cyw43_utk_task` hand-off (a
kernel message buffer, `tk_snd_mbf`) before writing any telemetry code.

**What's blocking it:**
1. The task-layout / priority / affinity decision is still open — decision log in
   [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) §7.
2. The wire schema is drafted but not frozen — open fields in
   [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §12. *Updated from Tribunal 1:* it is now
   binary v2, and every field width must be agreed with its producer before
   `messages.h` is frozen (the team's readiness step 1).
3. The team doc's task table still lists **two** app tasks while this subsystem
   runs **one** (see §5 below).

---

## 4. Open decisions (one place)

| Decision | Where | Recommendation |
|---|---|---|
| One app task or two | [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) §7 row 1 | **A** (one `comms_task`); split deferred |
| `comms_task` priority + core | §7 rows 5, 7 | placeholder **8**; unpinned |
| Team doc: add a core/pin column | §7 row 6, paste-ready in §8 | yes |
| Wire schema fields 1–14 (binary v2) | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §12 | see §12; items 10–14 added at Tribunal 1 |
| Fault-bit positions and who clears each bit | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §4.1 | requirement order; agree with Buddy 5 (Tribunal 1) |
| Buddy 2's 54-bit motion record vs. `EVT_MOTION` | [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §4.1 | reconcile with Buddy 2 (Tribunal 1) |
| What each 2-bit barcode action means | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §12 item 7 | agree with Buddy 3 (Tribunal 1) |
| Task count / names | [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md) §7 | A + 1 |

---

## 5. Known inconsistencies & follow-ups

- **Team doc vs this subsystem (open).** `docs/embedded .md` still lists three
  tasks (`cyw43_utk_task`, `wifi_task`, `comms_task`) and gives `cyw43_utk_task`
  priority "Low (20)". Reality: **two** tasks, radio fixed at **4**. Your own
  §6.1 and assumption **A6** already say "one application task" — the shared task
  table just hasn't been merged. Prepared edits live in
  [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) §2, §8.
- **Statechart image.** `docs/embedded .md` §6.1's `image3` needs regenerating
  from [`STATECHART1_DIAGRAM_SPEC.md`](STATECHART1_DIAGRAM_SPEC.md) §10.
- **Code not yet written.** `app_program/comms_task/` is an empty stub (renamed
  from `wifi_task/` on 2026-10-10). It is compiled, so keep it valid.

---

## 6. Where to read what (the map)

**In this folder (`docs/comms/`):**

| Document | Read it for |
|---|---|
| [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) | The **master design** — tasks, queues, topics, wire format, test gates. The whole picture. |
| [`GLOSSARY.md`](GLOSSARY.md) | The **shared vocabulary** — every embedded term in plain language (one copy for the whole set). |
| [`NETWORKING.md`](NETWORKING.md) | Everything **below** the app: Wi-Fi ladder, config files, MQTT implementation, the M0–M6 evidence. |
| [`APPLICATION.md`](APPLICATION.md) | The **M7–M10 roadmap**, the briefing deliverables, and the open next actions. |
| [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md) | The **cross-task seams** — kernel-object primer and the A / A+1 decisions. |
| [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) | The **wire contract** (topics + packed binary payloads, schema v2 since Tribunal 1). Hand this to a teammate. |
| [`Tribunal/README.md`](Tribunal/README.md) | **Tribunal 1**: the new requirement, its impact on this subsystem, the gap check against the submitted report, the roadmap, and the race-test evidence. |
| [`Tribunal/completed_in_trib.md`](Tribunal/completed_in_trib.md) | The **submitted** tribunal report — frozen baseline, do not edit. |
| [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) | The **task count / priority / affinity** decision record + the prepared team-doc edits. |
| [`STATECHART1_DIAGRAM_SPEC.md`](STATECHART1_DIAGRAM_SPEC.md) | The spec to **regenerate the statechart** diagram. |
| [`WEEK6_FEEDBACK.md`](WEEK6_FEEDBACK.md) | The **Week 6 instructor feedback**, narrowed to this subsystem, with next steps. |
| [`resolve_feedback/Actions.md`](resolve_feedback/Actions.md) · [`results.md`](resolve_feedback/results.md) | The **step-by-step fix plan** for that feedback, and the log of what each step produced. |

**Outside this folder (`docs/`):**

| Document | Read it for |
|---|---|
| [`../embedded .md`](../embedded%20.md) | The **graded team design doc** — §6.1 statechart, §7 assumptions. |
| [`../ARCHITECTURE.md`](../ARCHITECTURE.md) | Repo map and glossary. |
| [`../GETTING_STARTED.md`](../GETTING_STARTED.md) | Build and flash, step by step. |
| [`../PORT_RP2040.md`](../PORT_RP2040.md) | What the µT-Kernel port has validated vs. still unproven. |

---

## 7. Where the code lives

| Path | What |
|---|---|
| `app_program/comms_task/` | The application task (**stub** — M7 onwards). |
| `app_program/common/messages.h` | The shared integration contract (**empty** — fill with the packed records, fault-bit names and pack/unpack helpers; Tribunal 1). |
| `app_program/race_test/` | Tribunal 1 two-core packed-word race test (measured; off unless `TM_RACE_TEST` is 1). |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | The radio driver (exists). |
| `lib/libnet/lwip/` | lwIP + the MQTT client (**single-threaded** — only the radio task may touch it). |

> **Read this first, then stop here** unless a task sends you into one of the
> detail documents above.