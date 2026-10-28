# Task Layout — decision record

The three artifacts that once described Buddy 1's subsystem differently
(`docs/embedded .md`, this design, and the firmware) now agree. This is the
**record** of the decisions, plus the one edit still owed to the team document.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Status:** decided 2026-10-10 (§4). Not an open debate.
- **Scope:** the names, count, priorities and **core affinity** of the tasks.
  Not the message contract ([`MQTT_SCHEMA.md`](MQTT_SCHEMA.md)) and not the kernel
  interface ([`RTOS_INTERFACES.md`](RTOS_INTERFACES.md)).
- **Companions:** [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §3 (the
  design's task table), [`../embedded .md`](../embedded%20.md) §3 (the team's).

---

## 1. What this records

Three sources disagreed on how many tasks Buddy 1 owns and at what priority. The
code is the tie-breaker, because it is the only source actually running.

## 2. The disagreement → resolution

| Role | Team doc (`embedded .md`) | Design (& this record) | Firmware today |
|---|---|---|---|
| Radio / MQTT client owner | `cyw43_utk_task` — "Low (20)" *(wrong)* | `cyw43_utk_task` — **4, fixed** | **4**, `TP_PRC1` — `cyw43_utk_kernel.c:17`, `:36` |
| Outbound telemetry | `comms_task` — (21) | **`comms_task`** — **16** (Week 6 report) | **16**, `TP_PRC1` — `comms_task.c` (H heartbeat only so far) |
| Inbound commands | `wifi_task` — (19) | **merged into `comms_task`** | does not exist yet |
| Existing reporter | not listed | keep for development, **off for demo day** (decision 8) | `wifi_status_task` **17**, `TP_PRC1`, gated by `TM_WIFI_REPORT` — `app_main.c` |

**Resolution (Option A).** Buddy 1 runs **one** application task, **`comms_task`**
(renamed from `wifi_task`), which both prepares outbound telemetry/heartbeat and
validates + dispatches inbound `snap`/`map`. The radio task `cyw43_utk_task` is
unchanged library code at **4**, pinned to core 0. The split into two tasks is
kept as a documented follow-up (§5).

**Why A, in one line:** fewer lines to explain in the demo (graded), one app-task
priority to choose, and the split's only real benefit — isolating inbound
validation from the 1 s heartbeat — guards a hazard the design already removes by
mandating a non-blocking, single-fragment inbound path. The kernel-object pools
(RAM is not the constraint; `CNF_MAX_MBFID`/`MBXID` are, `config/config.h:45-51`)
argue against adding seams.

## 3. The code is the tie-breaker

The running firmware already has a priority band; new app tasks are chosen
against it (smaller number = higher priority):

| Priority | Task | Where |
|---|---|---|
| 3 | USB console service | `usb_console.c:40` |
| **4** | `cyw43_utk_task` (radio, core 0) | `cyw43_utk_kernel.c:17` |
| 10 | `blink_task` — only with `TM_BLINK_LED=1`, off by default (GP16 is the IMU's SDA) | `app_main.c`, `demo_tasks.h` |
| **16** | `comms_task` | `comms_task.c` |
| 17 | `wifi_status_task` — development reporter, off for demo day | `app_main.c` |

*Updated 2026-10-21:* the kernel-port demo tasks (producer 5, consumer 6,
monitor 7) and `led_test` (9, drove GP10, the right motor input) were removed
from `app_main.c`; see the git history for them.

Legal range `1 … CNF_MAX_TSKPRI` = 32 (`config/config.h:39`).

### 3.1 One numbering, two queues — affinity, not priority, is what protects teammates

There is **one priority numbering** (1–32): every task has a single number.
*Corrected 2026-10-21:* µT-Kernel under SMP keeps **one ready queue shared by
both cores** (`knl_ready_queue`, `kernel/tkernel/ready_queue.h:51`), not one per
core as this section first said. On every reschedule the kernel walks that queue
in priority order and gives each core the highest-priority ready task **allowed
to run there** (`knl_ready_queue_assign`, `ready_queue_smp.c`). **Affinity
decides where a task may run**: a task pinned to core 1 can only be displaced by
a higher-priority task that may also run on core 1, and an unpinned task can
land on either core — which is why every task is now pinned (Buddy 2's request).

| Core | Tasks |
|---|---|
| **core 0** | USB console (3), `cyw43_utk_task` (radio, **4**, `TP_PRC1`), `comms_task` (**16**, `TP_PRC1`, core proposed — agreeing with Buddy 2), `wifi_status_task` (17, `TP_PRC1`, development only), blink (10, `TP_PRC1`, off by default) |
| **core 1** | the latency-sensitive sensing/control tasks (other buddies) |
| **either (unpinned)** | none |

> **Rule of thumb:** before asking "does X outrank Y?", ask **"same core?"** If
> not, the numbers are irrelevant — affinity already settled it.

The radio is pinned to core 0, so a task pinned to core 1 is **never** preempted
by it. On core 0 the interference is bounded (16 µs avg / 43 µs max
wake-and-dispatch, ≤85 µs kernel-lock hold, `PORT_RP2040.md:54`, `:45`).

**The team doc's defect:** its task table gives a single flat priority column
(≈10–21) with **no core/pin column**, so it implies "motion (11) outranks
`cyw43` (20)" — false on hardware. **Fix:** give teammates what they intend
through affinity — pin latency-sensitive sensing/control tasks to core 1 — rather
than by fighting the radio's fixed number.

## 4. Decision log

| # | Question | Decision |
|---|---|---|
| 1 | One app task (`comms_task`, A) or two (A + `wifi_task`, B)? | **A** — one `comms_task`; B kept as a follow-up (§5) |
| 2 | Source of truth for the task table? | [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §3 |
| 3 | `cyw43_utk_task` priority | **4** — fixed, library, core 0 |
| 4 | Keep + rename `wifi_status_task` → `wifi_report_task`? | keep: **yes** (design §16 item 7); the rename is not done yet. Superseded in part by decision 8 |
| 5 | `comms_task` priority | **16** — set by the Week 6 report (one below `nav_ctrl_task`, 15); the provisional 8 is withdrawn |
| 6 | Add a **core / pin** column to the team doc? | **yes** (§6) |
| 7 | Core for the latency-sensitive tasks? | **core 1** (§3.1) |
| 8 | `wifi_status_task` on demo day? | **Off.** *Taken 2026-10-21.* It is a development console that came with the kernel port in the first commit (`75eeb0f`), not part of the team design, and the Week 6 task table does not list it. It stays in the source for development and testing — every `[wifi]`/`[mqtt]` console line, and with them the F1 and MQTT hardware evidence, comes from it — and is built only while `TM_WIFI_REPORT` is 1 (the default, `app_main.c`). **The demo-day build sets `TM_WIFI_REPORT` to 0.** Meanwhile it runs at priority **17**, below `comms_task`, pinned to core 0 |

## 5. Build guidance carried forward

Build A so a later pivot to B is a **wrapper change**, not a rewrite — the
mailboxes, records, wire format and schema are identical in A and B:

1. **One task entry, two functions** — `comms_run()` (drain events → state table →
   publish) and `wifi_run()` (read inbound → validate → dispatch). Share nothing
   but the kernel-object IDs.
2. **No shared file-scope locals** between the two paths.
3. **Access the latest-state table only through a small API** (`state_write` /
   `state_read_copy`) with a mutex **from day one, even uncontended** — under B,
   `comms_task` writes the table and the inbound path reads it.
4. **Treat the snapshot as "publish a state line on demand"**, serviced by a flag
   the state owner picks up — so who owns the table stops mattering.

**Avoid:** a monolithic `while(1)` interleaving both paths with shared locals.

## 6. Prepared edit for the team doc

`docs/embedded .md` §3 still lists three tasks. Send the team:

1. **Delete the `wifi_task` row**; merge its job into `comms_task`.
2. **`cyw43_utk_task` row:** priority `Low (20)` → **`4` — fixed, library, core 0**.
3. **Leftover refs** (`:147`, `:151`, `:152`): "from `wifi_task`" → "from `comms_task`".
4. **Add a `Core / pin` column** and this legend:

> Priorities order tasks **within the same core only** (smaller = higher). "Core"
> is the SMP affinity; core 0 also runs the USB console and the radio, so keep
> latency-sensitive tasks off it. `cyw43_utk_task` is **4, fixed by the library and
> pinned to core 0** — not a tunable app priority.

## 7. Don't redo

- Don't re-argue the transport — M4, M5 and M6 are proven.
- Don't move `cyw43_utk_task` out of `lib/libwifi/` into `app_program/`.
- Don't widen the priority-range debate — `1…32` is the whole story.
- Don't let the team doc's task table keep saying "three tasks".

## 8. Glossary

> **Moved.** Every term is collected in the shared [`GLOSSARY.md`](GLOSSARY.md).