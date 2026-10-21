# WiFi + Telemetry (Buddy 1) — start here

The one page to read for Buddy 1's subsystem: what it does, what other tasks
need from it, and where the contracts live. Every other document in this folder
is *detail behind a link* — open one only when you need it.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Last updated:** 2026-10-21 — working notes, test logs and reports moved out
  of the repo; this folder now holds only the reference and interface docs
- **Tasks:** `comms_task` — the application task (in `app_program/comms_task/`,
  an empty stub); `cyw43_utk_task` — the radio owner, library code that already
  exists (`lib/libwifi/`)
- **One line:** turn car events into MQTT telemetry, and apply the laptop's live
  barcode-remap and `snap` commands — without ever touching lwIP directly.

> **Since Tribunal 1 (2026-10-19) — Change Requirement 1.** Every data field
> uses the minimum bits for its range; **MQTT payloads are packed binary, not
> JSON** ([`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) schema v2); fault conditions are
> bits in **one shared fault bitmask** (a kernel event flag owned by Buddy 5),
> and Buddy 1 sets the **Wi-Fi-loss** bit. The submitted report is the frozen
> baseline [`Tribunal/completed_in_trib.md`](Tribunal/completed_in_trib.md).

> **Not in this folder:** Buddy 1's progress tracking, hardware test logs,
> feedback write-ups and study notes are kept in a separate private repo.
> Anything another subsystem needs to build against stays here.

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

---

## 1. Status at a glance

| Phase | Milestones | State | Detail |
|---|---|---|---|
| Radio / transport | **M0–M6** | ✅ **proven on hardware** | [`NETWORKING.md`](NETWORKING.md) §5 |
| Wi-Fi reliability (Week 6 feedback F1) | — | 🟡 in progress — auto-rejoin, re-lease and state-change logging done; recovery tests T1, T2 passed 20 Oct; DHCP conflict-detection (ACD) tuning deferred to the end, ACD stays on | — |
| Application: telemetry framework | **M7** | 🔜 **next after F1** | [`APPLICATION.md`](APPLICATION.md) §4 |
| Application: command channel | M8 | ⏳ not started | [`APPLICATION.md`](APPLICATION.md) §4 |
| Application: robustness | M9 | ⏳ not started | [`APPLICATION.md`](APPLICATION.md) §4 |
| Application: documentation | M10 | 🟡 schema is binary v2; field widths still proposed (§12) | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) |

**Week 10 target:** binary telemetry reaching the laptop.

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

## 3. What other subsystems need from Buddy 1

| You are… | You need | Where |
|---|---|---|
| Any producer (Buddies 2–5) | Your telemetry record's fields, widths and packing | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §4, §12 |
| Any producer | How to hand a record to `comms_task` (queue, timeout, what happens when full) | [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md), [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §4 |
| Buddy 5 (`nav_ctrl_task`) | Fault-bit positions and who clears each bit | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §4.1 |
| Buddy 3 (line / barcode) | The barcode remap command and action codes | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §8, §12 item 7 |
| The laptop side | Topics, payload layouts, the decoder | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) |

---

## 4. Open decisions that need another buddy

| Decision | Where | Recommendation |
|---|---|---|
| One app task or two | [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) §7 row 1 | **A** (one `comms_task`); split deferred |
| `comms_task` priority + core | §7 rows 5, 7 | placeholder **8**; unpinned |
| Team doc: add a core/pin column | §7 row 6, paste-ready in §8 | yes |
| Wire schema fields 1–14 (binary v2) | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §12 | every width agreed with its producer before `messages.h` is frozen |
| Fault-bit positions and who clears each bit | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §4.1 | requirement order; agree with Buddy 5 |
| Buddy 2's 54-bit motion record vs. `EVT_MOTION` | [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §4.1 | reconcile with Buddy 2 |
| What each 2-bit barcode action means | [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §12 item 7 | agree with Buddy 3 |
| Task count / names | [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md) §7 | A + 1 |

---

## 5. Known inconsistencies

- **Team doc vs this subsystem (open).** `docs/embedded .md` still lists three
  tasks (`cyw43_utk_task`, `wifi_task`, `comms_task`) and gives `cyw43_utk_task`
  priority "Low (20)". Reality: **two** tasks, radio fixed at **4**. The shared
  task table just hasn't been merged. Prepared edits live in
  [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) §2, §8.
- **Statechart image.** `docs/embedded .md` §6.1's `image3` needs regenerating
  from [`STATECHART1_DIAGRAM_SPEC.md`](STATECHART1_DIAGRAM_SPEC.md) §10.
- **Code not yet written.** `app_program/comms_task/` is an empty stub. It is
  compiled, so keep it valid.

---

## 6. Where to read what (the map)

**In this folder (`docs/comms/`):**

| Document | Read it for |
|---|---|
| [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) | The **master design** — tasks, queues, topics, wire format, test gates. |
| [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) | The **wire contract** (topics + packed binary payloads, schema v2). Hand this to a teammate. |
| [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md) | The **cross-task seams** — kernel-object primer and the A / A+1 decisions. |
| [`NETWORKING.md`](NETWORKING.md) | Everything **below** the app: Wi-Fi ladder, config files, MQTT implementation, the M0–M6 evidence. |
| [`APPLICATION.md`](APPLICATION.md) | The **M7–M10 roadmap** and the briefing deliverables. |
| [`GLOSSARY.md`](GLOSSARY.md) | The **shared vocabulary** — every embedded term in plain language. |
| [`TASK_LAYOUT_RECONCILIATION.md`](TASK_LAYOUT_RECONCILIATION.md) | The **task count / priority / affinity** decision record + the prepared team-doc edits. |
| [`STATECHART1_DIAGRAM_SPEC.md`](STATECHART1_DIAGRAM_SPEC.md) | The spec to **regenerate the statechart** diagram. |
| [`Tribunal/completed_in_trib.md`](Tribunal/completed_in_trib.md) | The **submitted** Tribunal 1 report — frozen baseline, do not edit. |
| [`Tribunal/packed_size.py`](Tribunal/packed_size.py) | Generates the **data-representation table** (field ranges → bits → bytes). |

**Outside this folder (`docs/`):**

| Document | Read it for |
|---|---|
| [`../embedded .md`](../embedded%20.md) | The **graded team design doc** — §6.1 statechart, §7 assumptions. |
| [`../ARCHITECTURE.md`](../ARCHITECTURE.md) | Repo map and glossary. |
| [`../GETTING_STARTED.md`](../GETTING_STARTED.md) | Build and flash, step by step. |
| [`../PORT_RP2040.md`](../PORT_RP2040.md) | What the µT-Kernel port has validated vs. still unproven. |
| [`../port_fixes/`](../port_fixes/README.md) | Kernel-port bugs found and fixed (PF-001: timer tick). |

---

## 7. Where the code lives

| Path | What |
|---|---|
| `app_program/comms_task/` | The application task (**stub** — M7 onwards). |
| `app_program/common/messages.h` | The shared integration contract (**empty** — fill with the packed records, fault-bit names and pack/unpack helpers). |
| `app_program/race_test/` | Tribunal 1 two-core packed-word race test (off unless `TM_RACE_TEST` is 1). |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | The radio driver (exists). |
| `lib/libnet/lwip/` | lwIP + the MQTT client (**single-threaded** — only the radio task may touch it). |

> **Read this first, then stop here** unless a task sends you into one of the
> detail documents above.
