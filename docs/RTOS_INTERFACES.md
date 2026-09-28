# RTOS_INTERFACES — how the application layer talks to the kernel

The design questions behind `wifi_task`: **which RTOS object carries data in each
direction, and what the other four subsystem tasks have to write.** This is the
answer to "how does my layer interact with the RTOS, and how do I design to work
with the other buddies".

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Status:** design settled 2026-09-27 — both decisions in §7 are **taken**
  (**A + 1**). The subsystem design that follows from them is
  [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md).
- **Scope:** the *shape* of the application layer. Everything **below** it
  (transport, protocol, config, build) is in [`NETWORKING.md`](NETWORKING.md);
  the app-layer roadmap and deliverables are in [`APPLICATION.md`](APPLICATION.md);
  the general repo map is [`ARCHITECTURE.md`](ARCHITECTURE.md).
- **Written for a reader with no embedded background** — every RTOS term is
  glossed where it first appears.

---

## 1. The one rule that shapes everything

`lib/libnet/lwip/include/lwipopts.h:14` says:

```c
#define NO_SYS  1
```

lwIP (the Pico's TCP/IP library) was built **without an operating system**. It
has no locks and no threads of its own. It is safe to call from *exactly one*
task — the **radio-owner task** — and from nowhere else.

That task already exists. It is `cyw43_utk_task`
(`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c:496`), created by
`cyw43_utk_start()` (`.../cyw43_utk_kernel.c`), pinned to processor 1, at
priority 4 with a 6 KB stack (`cyw43_utk_kernel.c:17`, `:18`, `:36`). Its loop
(`cyw43_utk.c:554`) wakes every 2 ms, runs the radio and every lwIP client, and
is the *only* place `lwip_utk_mqtt_poll()` is called (`cyw43_utk.c:599`).

> **Why "silent" is the important word.** Calling lwIP from a second task does
> not raise an error. It corrupts lwIP's internal buffers, and you see it later
> as a hang or a garbage packet — the kind of bug that costs a demo attempt.
> This is the same class of problem as reading a hash map from two threads
> without a lock: no exception, just wrong data at random.

So `wifi_task` **may never call an MQTT or TCP function**. It cannot even call
`lwip_utk_mqtt_publish()` if that function talks to lwIP directly. That single
constraint produces the whole design below.

---

## 2. The two seams

Both directions of data cross a **kernel object** — never a direct call:

```
   the four buddies + Vehicle Controller
             │  (inbound seam — §4)
             ▼
        wifi_task          build the event / the snapshot
             │  (outbound seam — §5)
             ▼
      cyw43_utk_task ──▶ Wi-Fi ──▶ broker
      (radio-owner task — the only lwIP caller)
```

Read as a software engineer: `wifi_task` is a service in front of a
single-threaded dependency. It accepts work over a queue, and a dedicated worker
thread owns the dependency. You do not let every feature handler open its own
database connection — same idea.

> **Naming note.** The base repo already has a task called `wifi_status_task`
> living inside `app_program/app_main.c:266`. It only *reads* the radio's status
> snapshot (`cyw43_utk_get_status()`, `app_main.c:292`) and prints it — a
> read-only monitor. Our new `app_program/wifi_task/` is a **different thing**: it
> is the task that *produces* outbound traffic. Expect to rename or fold the old
> one during M7 so the two are not confused.

---

## 3. Kernel-object primer

µT-Kernel gives you three primitives that matter here. All three are already
used in this repo, so there is a working example to copy.

| Object | Plain-language meaning | API | Already used at |
|---|---|---|---|
| **Message buffer** | A thread-safe queue of variable-length byte messages. Any task may `send`; the owner `receive`s. This is a "concurrent queue of structs". | `tk_snd_mbf` / `tk_rcv_mbf` | `app_main.c:140` (send), `:163` (receive) |
| **Mutex** | A lock. One holder at a time. | `tk_loc_mtx` / `tk_unl_mtx` | `app_main.c` sequence-counter guard |
| **Event flag** | A set of bits a task can wait on — "wake me when something changed". | `tk_wai_flg` / `tk_set_flg` | `app_main.c` monitor task |

**The trap to avoid: a plain global variable.** Under SMP both cores run at once,
so a writer on core 1 and a reader on core 2 can interleave mid-struct and the
reader sees a value that never existed. That is exactly the failure lwIP's
`NO_SYS` warning is about, except in your own code. If two tasks share memory,
it goes behind a mutex or through a message buffer.

**The rule of thumb for the seams:**

- **Message buffer** when the *events* matter — you want "A then B then C",
  ordered, and the receiver may be slower than the sender.
- **Mutex + shared struct** when only the *current value* matters and you want to
  read the whole thing cheaply at one instant.

---

## 4. Inbound seam — how the four buddies hand data to `wifi_task`

This is the decision that affects your teammates the most, because it is the
shape they have to code against.

### Option A — one event mailbox (recommended)

Every producing task posts a small record into a single shared message buffer.
`wifi_task` owns the receiving end and nobody else reads it.

```c
/* app_program/common/messages.h — the shared contract */
typedef enum {
    EVT_MOTION = 1,     /* Buddy 2 */
    EVT_LINE,           /* Buddy 3 */
    EVT_BARCODE,        /* Buddy 3 */
    EVT_IMU,            /* Buddy 4 */
    EVT_OBSTACLE,       /* Buddy 5 */
    EVT_MISSION,        /* Vehicle Controller */
} T_EVT_KIND;

typedef struct {
    UW   t_ms;          /* monotonic milliseconds, stamped by the producer */
    UINT kind;          /* one of T_EVT_KIND */
    INT  a, b;          /* two numbers; meaning depends on kind (see below) */
} T_TELEMETRY_EVENT;

extern ID tele_mbfid;   /* created once in usermain() */
```

A buddy's whole job, inside its own task:

```c
/* say, inside motor_task.c */
T_TELEMETRY_EVENT e = { .t_ms = now_ms(), .kind = EVT_MOTION,
                        .a = MOTION_LEFT, .b = 0 };
tk_snd_mbf(tele_mbfid, &e, sizeof e, 10);   /* 10 ms cap, see below */
```

And `wifi_task`'s loop:

```c
for (;;) {
    /* 1. drain everything the buddies posted since last time round */
    while (tk_rcv_mbf(tele_mbfid, &e, TMO_POL) > 0) {
        fold_into_latest_state(&e);      /* wifi_task's own private table */
        append_to_event_log(&e);         /* ordered, for the run map       */
    }

    /* 2. publish on the cadence */
    if (++ticks >= PUBLISH_TICKS) {
        ticks = 0;
        build_payload(&log, &state);                /* one tm_sprintf      */
        lwip_utk_mqtt_publish(TOPIC_TELEMETRY, payload, len);  /* enqueues  */
    }

    tk_dly_tsk(TELEMETRY_TICK_MS);
}
```

**Why the 10 ms send timeout matters.** The producer must never be able to stall
the car. If the queue is momentarily full, `tk_snd_mbf` returns an error after
10 ms and the buddy counts a dropped event and carries on. A telemetry gap is
acceptable; a stalled motor task is not. (The existing demo uses a 1000 ms
timeout for the same call — deliberately relaxed because the demo has no motion
deadline. Ours must be much shorter.)

| | |
|---|---|
| **Pros** | Ordered — the log reconstructs the run ("moved forward 20 cm, *then* turned right"), which is exactly the M7 deliverable. No shared memory between buddies, so no locks to forget. `wifi_task` is the *only* writer of the latest-state table, so the M8 snapshot needs no lock at all. Backpressure is explicit and countable. Matches the repo's existing producer/consumer demo, so it is a pattern you can point at and explain. |
| **Cons** | Every buddy must learn the record and add a send call (about three lines each). One fixed record size for everyone, and each queued record costs RAM. |
| **What a teammate has to get right** | Build the record, send it, do not block on the send. That is the entire contract. |

### Option B — one shared state struct behind a mutex

```c
typedef struct {
    INT motion; UW distance_cm;      /* Buddy 2 */
    INT barcode, action;             /* Buddy 3 */
    UW humps, hump_mm;               /* Buddy 4 */
    UW obstacle_px; INT bypass;      /* Buddy 5 */
    INT mission; UW update_count;    /* Vehicle Controller */
} T_VEHICLE_STATE;

/* each buddy, each update */
tk_loc_mtx(state_mtxid, TMO_FEVR);
state.humps++;
state.update_count++;
tk_unl_mtx(state_mtxid);
```

| | |
|---|---|
| **Pros** | "Current state at this instant" is trivial — the M8 snapshot is just a copy of the struct. No per-event format to agree on; each buddy only adds fields. |
| **Cons** | It loses *history*. You know the car is turning left now; you do not know it turned left *after* moving forward 20 cm — which is the "reconstructable map of the run" the deliverable asks for. Every access must remember to unlock, and one missed unlock on an update path **stalls the whole car**. Readers and writers contend on every sample. |

### Option C — both

Mailbox for the ordered log **and** a mutex-guarded struct for instant state,
kept consistent by `wifi_task`. Most complete, most code, and two separate things
teammates can get wrong. Worth it only if Option A's snapshot turns out awkward.

### The reverse direction — M8's live barcode remap

M8 has to push a new letter→action table **out** to Buddy 3's task. Same
question, pointed the other way, and the answer is symmetric:

- **Under Option A:** a second message buffer owned by `line_task`
  (`tele_cmd_mbfid`). `wifi_task` posts the new table; `line_task` drains it.
  Clean, and the change is an event (`EVT_CMD_REMAP`) in the same vocabulary.
- **Under Option B:** `line_task` reads the shared struct — but then it needs an
  **event flag** to know the table changed, so you end up introducing a second
  primitive anyway. Option A is simpler here.

---

## 5. Outbound seam — what happens to the MQTT module

### What is actually wrong with it today

`lib/libnet/lwip/lwip_utk_mqtt.c` is a **qualification program**, not a client.
It was built to prove the MQTT path end to end (M6) and it does that well, but:

| Line | What it does | Why it blocks M7 |
|---|---|---|
| `:40` | `#define UTK_MQTT_TOPIC "robocar/test/mqtt"` | the topic is compiled in — the caller cannot choose |
| `:41` | `#define UTK_MQTT_PAYLOAD "hello-from-pico"` | the payload is a constant, not data |
| `:202` | one `mqtt_publish()` call | there is no second publish |
| `:63` `:66` | `finish()` sets `complete = 1` | terminal state |
| `:163` | `if (mqtt_status.complete) return;` | after the first message, `poll` does nothing, forever |

So the module connects once, sends one fixed message, and stops. There is no way
to choose a topic, send a second message, or receive one.

### Option 1 — generalize it in place (recommended)

Keep the shape that already works — one client, one
`lwip_utk_mqtt_poll(netif)` entry point called only by the radio task — and
replace the one-shot logic with a persistent state machine:

```
DISCONNECTED ──▶ CONNECTING ──▶ CONNECTED ──▶ drain outbound queue ──▶ publish
                                    ▲                                    │
                                    └────────── reconnect ◀──────────────┘
```

**The load-bearing subtlety.** `publish()` is called from `wifi_task`, which may
not touch lwIP. So it must **enqueue**, and the radio task does the real call:

```c
/* Callable from ANY task.  It only posts to a message buffer — no lwIP. */
ER lwip_utk_mqtt_publish(const char *topic, const void *payload, UW len);

/* Radio task ONLY.  The actual mqtt_publish() happens in here. */
void lwip_utk_mqtt_poll(struct netif *netif);
```

The same is true in reverse for receiving: the broker→Pico callback runs
**inside the radio task**, so it must copy the topic and payload into a mailbox
for `wifi_task` to drain. It must not parse or dispatch there — the radio task
has a 2 ms budget and may not block.

| | |
|---|---|
| **Pros** | One connection, one module, code you already understand and can defend line by line. The `finish()` dead-end is removed by the *same* change M9 (reconnect + heartbeat) needs anyway — so this is not extra work, it is the work. Reuses the proven CONNACK / publish-callback path. |
| **Cons** | You are editing proven code. Mitigation: the M6 evidence is already captured in `NETWORKING.md` §8, and the change stays gated on `WIFI_MQTT`. |

### Option 2 — a new module beside it

Leave `lwip_utk_mqtt.c` untouched as M6 evidence; add `lwip_utk_telemetry.c`
with its own client and its own poll call.

| | |
|---|---|
| **Pros** | Zero risk to the M6 proof; the old module stays re-runnable as a boot self-test. |
| **Cons** | Two MQTT clients means **two TCP connections and two client IDs to the same broker**, competing for the tiny lwIP pools on one RP2040. Those pools are already tight: `MEMP_NUM_SYS_TIMEOUT` was raised to 12 specifically *because* the MQTT client needs one extra slot (`lwipopts.h:56`), and exhausting it trips `lwip_utk_assert`, which **halts the board silently**. You would also end up disabling one client in the telemetry build — which is Option 1 with extra steps. |

### Option 3 — defer

Design the app layer against a placeholder `telemetry_publish()` and change
`lib/` later.

| | |
|---|---|
| **Pros** | Unblocks the shared contract (`messages.h` + schema), which is what the teammates actually need. |
| **Cons** | Nothing is verifiable on hardware until the lib work lands, and the app layer's central claim — "it never blocks and never touches lwIP" — is only really proven when it is wired. |

### Open implementation detail

Where the outbound queue is *created* is not settled: either boot-time in
`cyw43_utk_start()` (it exists before any task runs), or lazily on the radio
task's first poll (then `publish()` before that must fail cleanly). The
principle to hold either way: **kernel objects that cross app tasks are created
in `usermain()` with their IDs in `messages.h`; a queue internal to the lib is
created by the lib and reached only through `publish()`.**

---

## 6. Recommended combination — A + 1

Both seams become **the same shape**: a kernel message buffer crossing into the
task that owns the resource.

```
buddies ──[mbf]──▶ wifi_task ──[mbf]──▶ cyw43_utk_task ──▶ broker
```

That symmetry is the argument. One pattern, three places, one explanation you
can give in the demo: *"work is handed to the owner of the resource over a queue;
the owner is the only one who touches it."* Option B + Option 2 gives you four
different mechanisms and twice the lwIP footprint.

---

## 7. Decision log

| # | Decision | Choice | Date |
|---|---|---|---|
| 1 | Inbound seam: §4 Option **A / B / C** | **A** — one event mailbox | 2026-09-27 |
| 2 | Outbound seam: §5 Option **1 / 2 / 3** | **1** — generalize in place | 2026-09-27 |

**Why A + 1.** Both seams become the same shape — a kernel message buffer
crossing into the task that owns the resource — so there is one pattern to
explain in the demo instead of four mechanisms. A keeps the ordered event log
that M7's "reconstructable map of the run" needs, and it makes `wifi_task` the
only writer of the latest-state table, so the M8 snapshot needs no lock. `1`
removes the `complete` dead-end that M9's reconnect-and-heartbeat work has to
remove anyway, so it is not extra work — it is the work. The two rejected
options are not wrong, only more expensive: **B** loses event ordering, and
**2** would put two MQTT clients and two TCP connections on lwIP pools that are
already tight enough that `MEMP_NUM_SYS_TIMEOUT` had to be raised to 12.

The design built on this pair is [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md).

---

## 8. Proposed contract (for the teammates to review, not yet written)

The actual `app_program/common/messages.h` and `docs/MQTT_SCHEMA.md` come **after**
§7 is decided. This is the shape they will take under the recommended A + 1.

### Event catalogue

Two fields `a` and `b` keep the record small and fixed-size; the meaning comes
from `kind`. Split an event into two only if it genuinely carries three values.

| `kind` | Producer | `a` | `b` |
|---|---|---|---|
| `EVT_MOTION` | Buddy 2 | motion state (straight / left / right) | distance moved since last event (cm) |
| `EVT_LINE` | Buddy 3 | line state (on / lost / crossing) | deviation |
| `EVT_BARCODE` | Buddy 3 | decoded letter | decoded action code |
| `EVT_IMU` | Buddy 4 | hump count | hump height (mm) |
| `EVT_OBSTACLE` | Buddy 5 | obstacle width | bypass direction |
| `EVT_MISSION` | Vehicle Controller | mission state | 0 |

> **Proposal only.** The field meanings are yours and the teammates' to fix —
> the value of the table is that it makes the *shape* (one small fixed record,
> meaning by `kind`) concrete enough to argue about.

### Topics

Rooted under one namespace so the instructor's broker on demo day is clean:

| Topic | Direction | Contents |
|---|---|---|
| `robocar/<id>/telemetry` | Pico → broker | the event log / periodic state |
| `robocar/<id>/state` | Pico → broker | the on-request M8 snapshot reply |
| `robocar/<id>/cmd` | broker → Pico | command channel (subscribe) |
| `robocar/<id>/heartbeat` | Pico → broker | M9 liveness |

`<id>` comes from `UTK_MQTT_CLIENT_ID` in `config/broker_config.h`, so the topic
stays configuration rather than a compiled-in constant.

---

## 9. Anything to verify before building

| Claim | How to check |
|---|---|
| lwIP's bundled MQTT client supports subscribing (`mqtt_subscribe`) and a single inbound-message callback. **Not yet verified** — the lwIP source lives inside the container, not on the host. | `docker compose run --rm build bash -lc "grep -n 'mqtt_subscribe\|mqtt_set_inpub_callback' \$PICO_SDK_PATH/lib/lwip/src/include/lwip/apps/mqtt.h"` |
| A telemetry build should not also run the UDP/TCP echo phases. | Already an open item in `NETWORKING.md` §11 — `WIFI_MQTT` currently requires `WIFI_TCP`. Relax the gate to DHCP before M7. |
| The old `wifi_status_task` (`app_main.c:266`) versus the new `wifi_task` — one reporter or two? | Fold the status printing into `wifi_task` during M7, or keep the reporter read-only and rename it. |

---

## 10. Don't redo

- Don't call any `lwip_*` function from `wifi_task` — §1.
- Don't share a plain global between two tasks — §3.
- Don't re-argue the transport: M4, M5 and M6 are proven (`NETWORKING.md` §5).
- Don't put the real networking code in `app_program/wifi_task/` — it lives in
  `lib/libwifi/` and `lib/libnet/`. This folder is only the application task.
- Don't commit `config/*.h` — the populated files are gitignored on purpose.

---

## 11. Glossary

| Term | Plain meaning |
|---|---|
| **Task** | The RTOS equivalent of a thread. |
| **SMP** | Two CPU cores running tasks at once, sharing memory. |
| **Message buffer** (`tk_snd_mbf`) | A thread-safe queue of structs between tasks. |
| **Mutex** | A lock; one holder at a time. |
| **Event flag** | A set of bits a task waits on — "wake me when this changed". |
| **`TMO_POL`** | Timeout value meaning "don't block, just check and return now". |
| **`TMO_FEVR`** | Timeout value meaning "block forever". |
| **Radio-owner task** | The single task allowed to call lwIP: `cyw43_utk_task`. |
| **Seam** | The point where two layers hand data across — here, always a kernel object. |
| **QoS 0** | MQTT "at most once" — fire and forget, no acknowledgement. |
| **Snapshot** | The current state at one instant, as opposed to the history of events. |