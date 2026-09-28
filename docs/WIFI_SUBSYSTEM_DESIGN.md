# WIFI_SUBSYSTEM_DESIGN — Buddy 1's subsystem, end to end

The design for the **WiFi + Telemetry** subsystem: the tasks, the kernel objects
that carry data between them, the MQTT session that results, and the plan for
proving each piece on hardware.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Status:** design, revised **2026-09-28** — the wire format flipped to JSON
  (§8). The two seam decisions were taken 2026-09-27 (`RTOS_INTERFACES.md` §7 →
  **A + 1**); the choices still open are listed in §16.
- **Scope:** the whole subsystem as one picture — radio, protocol, application
  task, and the contract the other four buddies code against.
- **Companions:** [`NETWORKING.md`](NETWORKING.md) is the reference for
  everything *below* the application (transport, protocol, config, build, and the
  captured M0–M6 evidence) — this document does not repeat it;
  [`RTOS_INTERFACES.md`](RTOS_INTERFACES.md) argues the two seams in depth and
  holds the decision log this one builds on; [`APPLICATION.md`](APPLICATION.md)
  holds the M7–M10 roadmap and the briefing's deliverables;
  [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) is the flat wire contract derived from §4,
  §8 and §9 — the document to hand to a teammate or the instructor;
  [`ARCHITECTURE.md`](ARCHITECTURE.md) is the general repo map.
- **Written for a reader with no embedded background** — every new term is
  glossed where it first appears, and §19 collects them.

---

## 1. Where the subsystem starts and ends

The subsystem is already half built, and the half that exists is the harder
half. Being precise about the boundary is what keeps this document from
proposing work that is done.

| Layer | State | Owner document |
|---|---|---|
| Radio bring-up, join, DHCP, DNS | **proven on hardware** (M2, M3) | `NETWORKING.md` §5 |
| UDP and TCP transports | **proven on hardware** (M4, M5) | `NETWORKING.md` §5 |
| MQTT connect + one publish | **proven on hardware** (M6) | `NETWORKING.md` §6 |
| MQTT as a *persistent* client (many publishes, subscribe, reconnect) | **not written** | **this document** |
| Telemetry framework over MQTT | **not written** | **this document** |
| Command channel, snapshot, live barcode remap | **not written** | **this document** |
| The task that produces traffic (`app_program/wifi_task/`) | **empty stub** | **this document** |
| The shared contract (`app_program/common/messages.h`) | **empty** | **this document** |

> **What it means:** M2–M5 changed no source at all — they were "run the ladder
> and verify it". M6 added a module that connects once, publishes one fixed
> message, and stops. Everything this document designs sits **above** that: it is
> the difference between *proving the protocol works* and *having a telemetry
> system*. That distinction is the whole reason the M6 module is described in
> §6 as a qualification program rather than a client.

---

## 2. The one rule, and the shape it forces

`lib/libnet/lwip/include/lwipopts.h:14` says:

```c
#define NO_SYS                          1
```

lwIP — the Pico's TCP/IP library — is built **without an operating system**: no
locks, no threads, no re-entrancy. It is safe to call from *exactly one* task:
the **radio-owner task**, `cyw43_utk_task`
(`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c:496`). That task exists before
the application does — it is created and started at kernel device init
(`kernel/sysdepend/pico_rp2040/devinit.c:107`), pinned to processor 1, at
priority 4, with a 6 KB stack and `TA_RNG1`
(`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c:17`, `:18`, `:28`, `:33`,
`:36`). Its loop polls the radio every 2 ms (`cyw43_utk.c:41`, `:661`) and is
the only caller of `lwip_utk_mqtt_poll()` (`cyw43_utk.c:599`).

Violate that rule and you get no error message. You get corrupted lwIP state,
which surfaces much later as a hang or a garbage packet. Failures in this stack
are **silent** — the port's assertion handler halts the board with interrupts
masked and no output at all (`lib/libnet/lwip/lwip_utk.c:33-37`).

That single constraint produces the entire design: **every unit of work crosses
a kernel object.** Nothing in this subsystem calls anything directly across a
task boundary.

```
   the four buddies + Vehicle Controller
             │   Seam 1 — one event mailbox (§5)
             ▼
        wifi_task                    ← the task this document designs
             │   Seam 2 — one publish mailbox (§6)
             ▼
      cyw43_utk_task ──── Wi-Fi ────▶ broker
      (radio-owner; the only lwIP caller)
             ▲
             │   Seam 3 — one inbound mailbox (§9)
             └── commands from the broker
```

> **What it means:** read as a software engineer, `wifi_task` is a service in
> front of a single-threaded dependency. It accepts work over a queue; a
> dedicated worker thread owns the dependency. You do not let every feature
> handler open its own database connection — same idea, and the phrase to use in
> the demo is *"work is handed to the owner of the resource over a queue; the
> owner is the only one who touches it."*

### 2.1 Who owns what — and no, this is not a conflict with the RTOS

Three different things get conflated when this question comes up, so name them
separately.

| Name | What it actually is | Relationship to the RTOS |
|---|---|---|
| `cyw43_utk_task` | An ordinary RTOS task, created with `tk_cre_tsk` | **Obeys every rule**: it is scheduled, it is preempted, and it blocks on `tk_dly_tsk` like any other task |
| lwIP | A TCP/IP **library** with no internal locking | Not a scheduler participant at all — a library cannot "disobey" an RTOS |
| the CYW43439 | The Wi-Fi **chip**, driven by the code in `cyw43_utk.c` | Runs on the radio task's stack, inside the radio task's time slice |

So *"lwIP does not obey RTOS rules"* is the wrong frame. It is not breaking a
rule — it is a library that **requires** one: that exactly one thread call it.
That is the same situation as a non-thread-safe collection in any other language,
and the same answer applies: the fix is not a lock, it is that only the owning
thread touches it.

**Why a mutex would not fix it.** The kernel does have a mutex (`tk_loc_mtx`,
used at `app_main.c:130`), so the temptation is real. But `NO_SYS 1` means lwIP
has no OS integration at all, and its entry points are re-entered from
*callbacks that lwIP itself invokes* — from inside `sys_check_timeouts()` and
from the netif input path, i.e. from **within** the one owning call. A lock
around the outer calls would not serialize that inner re-entry, which is where
the corruption actually happens. The design answer is "one caller", not "a
locked caller".

**And the scheduler is already on your side.** Under SMP, who can disturb whom is
decided by affinity, not by priority alone:

| Processor | Tasks that run there |
|---|---|
| 1 (physical core 0) | USB console service (priority 3), the radio task (4, pinned `TP_PRC1` — `cyw43_utk_kernel.c:36`), the demo producer (5, pinned `TP_PRC1` — `app_main.c:450`) |
| 2 (physical core 1) | the demo consumer (6, pinned `TP_PRC2` — `app_main.c:462`) |
| either | the unpinned tasks: monitor (7), `wifi_status_task` (8), `led_test` (9), blink (10) |

Because the radio task is **pinned** (`TA_ASSPRC`, `cyw43_utk_kernel.c:28`,
`:36`), it can only ever run on processor 1. A task running on processor 2 is
therefore **never** preempted by it — not for 2 ms, not for 2 µs. On processor 1
the interference is bounded and small: the radio task blocks for 2 ms between
polls (`cyw43_utk.c:661`), and the port's own measurements put a local
wake-and-dispatch at 16 µs average / 43 µs maximum, with a maximum kernel-lock
hold of 85 µs (`PORT_RP2040.md:54`, `:45`). Tens of microseconds every 2 ms is
not a conflict; that is a driver task doing its job, and it is the normal
arrangement — the service that owns a piece of hardware runs *above* the
application that uses it.

> **The caveat that makes affinity a design decision.** An **unpinned** task can
> be placed on processor 1 by the scheduler, and then it does share that core
> with the radio and the USB console. That is the argument behind §16 decision 8:
> pinning the CPU-heavy application tasks to processor 2 keeps the radio's core
> free of application work. Motor control belongs there — which is a decision for
> Buddy 2, not for this subsystem.

The failure that *is* real is a different one: **a second task calling lwIP.** It
produces no scheduling symptom at all — no deadlock, no error, no priority
inversion — just corrupted buffers that surface later as a hang or a garbage
packet. That is precisely what the mailboxes in §3.2 exist to prevent.

---

## 3. Tasks and kernel objects

### 3.1 The two tasks

| Task | Entry | Priority | Stack | Affinity | Created by |
|---|---|---|---|---|---|
| `cyw43_utk_task` — exists | `cyw43_utk.c:496` | 4 | 6 KB | `TP_PRC1` (SMP) | kernel device init, `devinit.c:107` |
| `wifi_task` — **new** | `app_program/wifi_task/wifi_task.c` | 8 (proposed) | 4 KB | unpinned (proposed) | `usermain()`, `app_main.c` |

Priority: in µT-Kernel a **smaller number is a higher priority**. The radio task
sits at 4, the console at 3 (`cyw43_utk_kernel.c:14` explains that ordering —
the USB console must drain first). Putting `wifi_task` at 8 — the same level as
the existing read-only reporter — means it can never delay the radio or the
console, which is exactly what a best-effort telemetry producer should be.

`wifi_task` must be gated with the same `#if` its objects are (`TM_WIFI_MQTT`),
so **a non-MQTT radio build compiles and behaves exactly as it does today**. That
protects the M2–M6 evidence, which is the only hardware proof the subsystem has.

> **Naming hazard.** There is already a task called `wifi_status_task`
> (`app_program/app_main.c:266`). It is a *read-only reporter*: it calls
> `cyw43_utk_get_status()` and prints. Our new task is a *producer*. Two tasks
> with near-identical names in one file is how a demo gets mis-explained; see
> §16 decision 7 for the two ways out.

### 3.2 Kernel objects

All four are created in `usermain()` (`app_main.c:535-556`), which creates every
object **before** starting any task — so no task can ever run before its queue
exists. Only the radio task predates `usermain()`, and §6.3 handles that.

| Object | Type | Direction | Chosen by | Proposal |
|---|---|---|---|---|
| `tele_mbfid` | message buffer | buddies → `wifi_task` | §16 decision 4 | 16 records |
| `tele_pub_mbfid` | message buffer | `wifi_task` → radio task | §6.3 | 4 records × ~320 B (topic + 256-byte JSON payload, §8.3) |
| `tele_cmd_mbfid` | message buffer | `wifi_task` → `line_task` | §16 decision 4 | 4 records |
| `tele_rx_mbfid` | message buffer | radio task → `wifi_task` | §9.3 | 4 records × ~192 B |

IDs and the record types live in **`app_program/common/messages.h`** — the file
whose own comment says every buddy depends on it and that changes should go
through a pull request. For the radio→app direction the ID is looked up at send
time and a non-positive ID means "the app is not up yet, drop it".

> **What it means:** a **message buffer** (`tk_snd_mbf` / `tk_rcv_mbf`) is a
> thread-safe queue of variable-length messages: any task may send, one task
> receives. The repo already uses one (`app_main.c:140`, `:163`), so there is a
> working example to point at. It is the right primitive here because telemetry
> is about *events in order*, not about "the current value".

### 3.3 The plain-global trap

Under SMP both cores run at once. A writer on one core and a reader on the other
can interleave in the middle of a struct, and the reader sees a value that never
existed. That is the same class of bug as reading a hash map from two threads
without a lock: no exception, just wrong data at random. Every shared datum in
this design is behind a queue or owned by exactly one task.

---

## 4. Seam 1 — how the four buddies hand data to `wifi_task`

This is the decision that shapes your teammates' code, because it is the
interface *they* write against. Option **A** was chosen this session
(`RTOS_INTERFACES.md` §7): one shared mailbox, `wifi_task` the only reader.

### 4.1 The record

```c
/* app_program/common/messages.h */
typedef enum {
    EVT_MOTION = 1,   /* Buddy 2 — motion control   */
    EVT_LINE,         /* Buddy 3 — line state       */
    EVT_BARCODE,      /* Buddy 3 — barcode + action */
    EVT_IMU,          /* Buddy 4 — hump count/height*/
    EVT_OBSTACLE,     /* Buddy 5 — obstacle/bypass  */
    EVT_MISSION,      /* Vehicle Controller         */
} T_EVT_KIND;

typedef struct {
    UW   t_ms;        /* stamped by the producer, tk_get_otm() */
    UINT kind;        /* one of T_EVT_KIND                     */
    INT  a, b;        /* meaning depends on kind — see §4.4     */
} T_TELEMETRY_EVENT;

extern ID tele_mbfid;   /* created once in usermain() */
```

`t_ms` comes from `tk_get_otm()` (`include/tk/syscall.h:822`, enabled at
`config/config_func.h:126`), which returns operating time in milliseconds as a
`SYSTIM { hi, lo }` (`include/tk/typedef.h:93-96`). Use the low word; it wraps a
little under 50 days, far beyond a demo run. That keeps the app free of any Pico
SDK dependency — the SDK's own clock (`time_us_64()`, wrapped by lwIP as
`sys_now()` at `lib/libnet/lwip/lwip_utk.c:15`) stays inside the radio layer
where it belongs.

### 4.2 What a teammate has to write

Three lines, inside their own task:

```c
T_TELEMETRY_EVENT e = { .t_ms = tele_now_ms(), .kind = EVT_MOTION,
                        .a = MOTION_LEFT, .b = 0 };
tk_snd_mbf(tele_mbfid, &e, (INT)sizeof e, TMO_POL);
```

`TMO_POL` means "do not block, just check and return". A full queue returns an
error immediately; the buddy counts a dropped event and carries on.

> **Tradeoff, stated plainly — this one is worth a decision of its own (§16
> decision 5).** Blocking on the send instead (a bounded wait of 10 ms, as
> `RTOS_INTERFACES.md` §4 suggested) cannot lose an event but can delay a motion
> task by up to 10 ms. Never blocking can lose an event, but can never delay the
> car. Telemetry is graded on *not interfering* ("keep network work off the
> motion critical path"), and events arrive at a few per second against a
> 16-deep queue, so overflow means a bug rather than load. This document
> proposes **never block, and count the drops**, with the counter published in
> the periodic line (§7) so a gap is self-evident on the laptop instead of
> silently missing.

### 4.3 What `wifi_task` does with them

Two different needs, two different structures, one drain loop:

- **An ordered log** keeps the sequence of events, which is what M7's
  "reconstructable map of the run" asks for — *moved forward 20 cm, then turned
  right* cannot be recovered from a "current state" table alone.
- **A latest-state table** keeps the newest value per source, so a periodic line
  can report everything at once and M8's snapshot is a cheap copy.

Because `wifi_task` is the only reader of the mailbox and the only writer of the
table, the table needs **no lock at all** — the data is single-owner by
construction. That is the payoff for choosing A over B.

```
for (;;) {
    while (tk_rcv_mbf(tele_mbfid, &e, TMO_POL) > 0) {
        fold_into_latest_state(&e);       /* wifi_task's private table */
        append_to_log(&e);                /* ordered, bounded ring     */
    }
    if (++ticks >= PUBLISH_TICKS) { ticks = 0; publish_state_line(); }
    tk_dly_tsk(TELEMETRY_TICK_MS);
}
```

### 4.4 The event catalogue

Meaning comes from `kind`, which keeps the record small and fixed-size. This
table is the contract; it is the one thing to agree with the other four buddies
before anyone writes code.

| `kind` | Producer | `a` (unit) | `b` (unit) |
|---|---|---|---|
| `EVT_MOTION` | Buddy 2 | state: 0 stop / 1 straight / 2 left / 3 right | distance since last event (cm) |
| `EVT_LINE` | Buddy 3 | state: 0 on-line / 1 lost / 2 crossing | deviation (tenths of a sensor unit) |
| `EVT_BARCODE` | Buddy 3 | decoded letter (ASCII) | action code |
| `EVT_IMU` | Buddy 4 | hump count | hump height (mm) |
| `EVT_OBSTACLE` | Buddy 5 | obstacle width (cm) | bypass: 1 left / 2 right |
| `EVT_MISSION` | Vehicle Controller | mission state | 0 |

> **Proposal, not scripture.** The field meanings are yours and the team's to fix
> — the value of the table is that it makes the *shape* concrete enough to argue
> about. Two values per event is the constraint to defend: if a source genuinely
> needs three, split it into two events rather than growing the record.

---

## 5. Proven vs. unproven, at the seam

| Claim | Evidence |
|---|---|
| The radio task exists before any app task | `devinit.c:107` (kernel device init) |
| Exactly one caller of `lwip_utk_mqtt_poll` | repo-wide grep: `cyw43_utk.c:599` only |
| The MQTT poll is gated on DHCP completing | `cyw43_utk.c:598` |
| That gate is a **latch** — `dhcp_complete` is set to 1 and never cleared | `cyw43_utk.c:464`, `:472` |
| The status snapshot is reader-safe from any task | seqlock, 16 attempts, `cyw43_utk.c:44`, `:665-685` |
| Nothing else may write the snapshot | `radio_status` is file-static; no writer API in `cyw43_utk.h` |

**Consequence of the latch.** Once DHCP has completed, `lwip_utk_mqtt_poll()` is
entered every 2 ms **for the rest of the run**, including after the Wi-Fi link
drops. The module's own per-poll guard — netif up, link up, address non-zero
(`lib/libnet/lwip/lwip_utk_mqtt.c:170-175`) — is therefore load-bearing for
recovery, not a formality. M9 must keep that check and act on it (§10).

---

## 6. Seam 2 — turning the M6 module into a client

### 6.1 What is wrong with it today

`lib/libnet/lwip/lwip_utk_mqtt.c` is a **qualification program**. It proves the
path end to end, which is exactly what M6 needed, and it is honest about it: its
own comment at `:37-39` says the topic is deliberately a test topic because "the
topic schema and payload format are their own deliverable".

| Line | What it does | Why it blocks telemetry |
|---|---|---|
| `:40` | `#define UTK_MQTT_TOPIC "robocar/test/mqtt"` | compiled in; the caller cannot choose |
| `:41` | `#define UTK_MQTT_PAYLOAD "hello-from-pico"` | the payload is a constant, not data |
| `:160` | `if(mqtt_status.complete) return;` | after the one message, `poll` does nothing, **forever** |
| `:202` | a single `mqtt_publish()` call | there is no second publish |
| `:193-196` | `finish(ERR_OK)` then return | a terminal state, by design |
| `:35` | `UTK_MQTT_SESSION_TIMEOUT_MS 45000` | a whole-session deadline; wrong shape for a client that must live for the run |

Two further facts that matter more than they look:

- The client is created once (`mqtt_client_new()`, `:140`) and **never freed**,
  and `mqtt_disconnect` appears nowhere. So the TCP connection outlives the
  "complete" state and lwIP's keep-alive timer keeps running, serviced by
  `sys_check_timeouts()` in the radio loop (`cyw43_utk.c:560`).
- There is **no subscribe path and no inbound callback**. A repo-wide grep for
  `mqtt_subscribe` / `mqtt_set_inpub_callback` finds only doc mentions — no code.

> **A stale comment worth fixing while you are in there:** `:193-194` says "The
> connection callback already published". It does not — the publish is issued
> from `lwip_utk_mqtt_poll()` at `:202`, and the connection callback
> (`:89-103`) contains no publish at all. The behaviour is fine; the sentence
> will mislead whoever reads it as a specification.

### 6.2 The design (Option 1 — generalize in place)

Keep the shape that already works — one client, one `lwip_utk_mqtt_poll(netif)`
entry point called only by the radio task — and replace the one-shot logic with
a persistent state machine:

```
        ┌────────────┐  no address (link down / no lease)
        │ WAIT_ADDR  │◀───────────────────────────────┐
        └──────┬─────┘                                │
               │ address present                      │
               ▼                                      │
        ┌────────────┐  connect() != OK / timeout      │
        │ CONNECTING │──────────────────────────────┐  │
        └──────┬─────┘                              │  │
               │ CONNACK accepted                    ▼  │
               ▼                              ┌──────────────┐
        ┌────────────┐  out queue non-empty  │ DISCONNECTED │
        │ CONNECTED  │─────────────────────▶ │  (backoff)   │
        │ subscribe  │  publish → callback   └──────┬───────┘
        │ drain out  │◀──────────────────┐          │ backoff elapsed
        │ drain in   │                   │          └──▶ CONNECTING
        └────────────┘                   │
              └── ERR_MEM from publish retries next poll (keep `:211-214`)
```

**The load-bearing subtlety.** `lwip_utk_mqtt_publish()` must be callable from
`wifi_task`, which may not touch lwIP. So it **enqueues**, and the radio task
does the real call:

```c
/* Callable from ANY task.  Posts to a message buffer — touches no lwIP. */
ER  lwip_utk_mqtt_publish(const char *topic, const void *payload, UW len);

/* Radio task ONLY.  The real mqtt_publish() happens in here. */
void lwip_utk_mqtt_poll(struct netif *netif);
```

Symmetric rule in reverse: the inbound MQTT callback runs **inside the radio
task**, so it may only copy into a mailbox — never parse, never dispatch, never
block. The radio task has a 2 ms budget and shares its core with the USB console.

| | |
|---|---|
| **Pros** | One connection, one client, one module — code you already understand and can defend line by line. The `complete` dead-end is removed by the *same* work M9's reconnect needs, so it is not extra work; it **is** the work. Reuses the proven CONNACK and publish-callback path. |
| **Cons** | You are editing code that produced your M6 evidence. Mitigation: the evidence is captured in `NETWORKING.md` §8, the change stays gated on `WIFI_MQTT`, and the M6 console line is kept working (§6.4). |
| **Rejected** | A second module beside it (Option 2) would mean two MQTT clients, two TCP connections and two client IDs to one broker, on pools that are already tight enough that `MEMP_NUM_SYS_TIMEOUT` had to be raised to 12 (`lwipopts.h:56`) — and exhausting that pool trips an assertion that **halts the board silently**. |

### 6.3 Where the outbound queue lives — and why it cannot be in `usermain()`

`RTOS_INTERFACES.md` §5 left this open. The constraint resolves it:

- The radio task is started at **kernel device init** (`devinit.c:107`), before
  `usermain()` runs (`app_main.c:535`). So a queue created in `usermain()` does
  not exist during the radio task's first polls.
- The app-side queues (`tele_mbfid`, `tele_pub_mbfid`, `tele_cmd_mbfid`,
  `tele_rx_mbfid`) are created in `usermain()` and are guaranteed to exist before
  any app task is started — `usermain()` creates all objects, *then* starts
  tasks. That ordering is the reason Seam 1 can assume the queue exists.
- Therefore the **lib owns its own outbound queue** (created on the radio task's
  first poll, or in `cyw43_utk_start()`), and reaches it only through
  `lwip_utk_mqtt_publish()`. A publish before the queue exists must fail cleanly
  and be counted, not crash.

The rule to hold either way: **kernel objects that cross app tasks are created in
`usermain()` with their IDs in `messages.h`; a queue internal to the lib is
created by the lib and reached only through `publish()`.**

### 6.4 Sizes, and one thing not to do

Topic and payload maxima are 48 bytes today, mirrored between
`lib/libnet/lwip/include/lwip_utk_mqtt.h:15-16` and
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h:21-22`, with a compile-time check
that the two agree (`cyw43_utk.c:249-252`). A telemetry line needs more than 48,
so the **payload maximum rises (proposed: 256 bytes)** and both constants move
together — the check makes that safe. (The first draft proposed 192; the JSON
records in §8.3 and the diagnostic snapshot in §9.3 need the extra headroom.)

**Two more buffers set the real limits, and they are not ours.** Both live inside
`mqtt_client_t` (`mqtt_priv.h:66`, `:95`), which `mqtt_client_new()` allocates
from lwIP's heap:

| Buffer | Default | What it bounds |
|---|---|---|
| `MQTT_OUTPUT_RINGBUF_SIZE` | 256 (`mqtt_opts.h:56`) | **the whole outgoing publish** — topic *and* payload must fit, or `mqtt_publish` returns `ERR_MEM` |
| `MQTT_VAR_HEADER_BUFFER_LEN` | 128 (`mqtt_opts.h:64`) | **the incoming topic plus what fits of the payload**; the remainder is streamed in fragments (§9.3) |

The arithmetic for the proposed sizes: a `robocar/<id>/telemetry` topic runs about
25 bytes, so a 256-byte payload plus framing lands near 290 — comfortably inside
the proposed 512-byte ring buffer, with room to spare for a longer configurable
prefix. **Proposal: raise `MQTT_OUTPUT_RINGBUF_SIZE` to 512 and
`MQTT_VAR_HEADER_BUFFER_LEN` to 256** under the MQTT gate in `lwipopts.h`. That
costs 384 bytes of lwIP heap, and the failure mode if a limit is too small is
benign — `ERR_MEM`, which the existing code already retries on the next poll
(`lwip_utk_mqtt.c:211-214`).

Do **not** grow the mirror inside `T_CYW43_UTK_STATUS` to match. That struct is
copied whole on every `cyw43_utk_get_status()` call, seqlock and all, and is
already roughly 2.3 KB (168 fields plus the 1.7 KB UDP echo-ring mirror). Keep
`mqtt_topic` / `mqtt_payload` at 48 as a **truncated diagnostic copy of the last
publish**, which is all the `[mqtt] …` console line (`app_main.c:333`) needs,
and let the real payload live in the lib where it belongs.

> **What it means:** the snapshot is a *status* surface, not a data plane. Using
> it to carry telemetry would tax every reader with a copy of data only one task
> needs — and the snapshot has no reader-visible timestamp at all, so it could
> not express "when" anyway.

---

## 7. Publish policy — the hybrid stream

Chosen this session: **a periodic state line plus immediate edge events.** The
two answer different questions, which is why neither alone is enough.

| | Periodic state line | Edge event line |
|---|---|---|
| Answers | "what is the car doing *now*" | "what *happened*" |
| Rate | every 1000 ms (proposed) | on the event, a few per second at most |
| Failure if lost | invisible — the next one is 1 s later | a gap in the run map |
| M9 liveness | **is** the heartbeat: no line for N s means the car or the link is gone | not a liveness signal |
| M8 snapshot | shares its format | unrelated |

Both travel on **one topic**, distinguished by a record-type field (`"t"`), so the
sequence number orders *everything* and the laptop needs one subscription. Full
run of a 5-minute attempt at 1 Hz is ~300 lines — a rounding error for a link
that carries TCP echo tests today.

> **Why one topic is the recommendation.** Two topics (`…/telemetry` and
> `…/event`) sort the stream by *kind* instead of by *time*, so reassembling the
> run means merging two streams by timestamp — exactly the ordering problem
> Option A's mailbox was chosen to avoid. Counter-argument, for §16 decision 3:
> a subscriber interested only in events must filter rather than subscribe.

Edge events are **not** held until the next tick: they are enqueued immediately
and the radio task drains the queue on its next 2 ms poll. Ordering is preserved
because the queue is FIFO and both producers use it in order.

---

## 8. Wire format

**Revised 2026-09-28: the recommendation is JSON.** The first draft chose a
`key=value` log line, on the stated ground that *"the payload's only consumer is
your own subscriber."* That premise is wrong for this project. **On demo day the
reader is the instructor's MQTT client, not a script you wrote** — you do not
control which broker or which client sits on the other end. A custom line format
is only free when you also own the parser; when the other end is off-the-shelf
tooling, the format must be one that tooling already reads. Every generic MQTT
client (MQTT Explorer, `mosquitto_sub`, Node-RED, Grafana, `paho` + `json.loads`,
`jq`) renders JSON with no help at all. None of them parses `mot=2 dst=20`.

The constraints are unchanged, and the JSON form satisfies all four:

1. **No floating point.** The RP2040's cores have no FPU
   (`PORT_RP2040.md`, "Non-goals"), so every value on the wire is an integer. JSON
   does not change this: an integer is written as an integer literal, never `%f`.
2. **One format string per record type.** Still one `tm_sprintf` per record; only
   the literal text differs.
3. **Readable by generic tooling**, not merely by eye — the reader may not be you.
4. **Self-identifying**, because the M10 schema document has to be explainable
   without cross-referencing code — and JSON is self-identifying by construction.

### 8.1 The asymmetry that decides it

The two directions are not symmetric, and the whole argument turns on that:

| Direction | Who writes it | Who reads it | Cost of JSON |
|---|---|---|---|
| **Device → broker** — `telemetry`, `heartbeat`, `state` | **us**: one `tm_sprintf` | **possibly the instructor's client** | **≈ zero.** A format string. No parser, and no FPU concern: every value is an integer and the only strings are single letters, so there is nothing to escape |
| **Broker → device** — `cmd`, `map` | **us**: our own laptop publisher (§9.2) | **us**: a parser in `wifi_task` | **real.** A JSON scanner runs on the device, where every line is graded and explainability is a scored risk (§15) |

> **What it means:** JSON is *free to produce* and *costly to consume*. So the
> rule is **produce JSON, and be deliberate about what you parse.** The outbound
> stream is the one the instructor's client sees, and making it universal costs
> the device nothing. The inbound stream's publisher is your own laptop script —
> you already control both ends there — so that is a *separate* decision
> (§16 decision 17), not a reason to hold the outbound format hostage.

> **The hedge, while the inbound publisher is unconfirmed (2026-09-28).** Not
> knowing whether the instructor will ever publish does **not** argue for a
> device-side JSON scanner. He cannot publish a *valid* command in either format
> without knowing your schema — so what makes inbound work with a foreign
> publisher is `MQTT_SCHEMA.md`, not the delimiter. Keep the inbound default
> (`key=value`, `strtok`) and hand anyone who wants to publish the exact payload
> strings to copy. A JSON branch is additive and can wait until someone actually
> asks for it; commands are tiny, so it would be a small change, not a redesign.

### 8.2 The recommended records

Flat objects, fixed key order, integers and single letters only — which is what
keeps the writer to one `tm_sprintf`:

```
{"t":"S","v":1,"s":137,"up":41230,"mot":2,"dst":20,"lin":1,"dev":3,"bar":69,"act":4,"hum":3,"hgt":12,"obs":0,"byp":0,"mis":1,"mpv":3,"drp":0}
{"t":"E","v":1,"s":138,"up":41310,"k":"B","a":69,"b":4}
```

| Field | Meaning |
|---|---|
| `t` | record type: `"S"` periodic state, `"E"` edge event — the same discriminant as before, now a key instead of a leading letter |
| `v` | schema version — bumped when a field changes meaning, so an old subscriber fails loudly instead of silently |
| `s` | monotonic sequence number for the whole run, assigned by `wifi_task` |
| `up` | milliseconds since boot (`tk_get_otm()`), the producer's stamp |
| `k` | event kind, from §4.4 |
| `a`, `b` | the two values, meaning by `k` |
| `drp` | events dropped at the mailbox in the last window — makes a gap *visible* rather than mysterious |
| `mpv` | the letter→action table version the car is actually running; the laptop compares it with what it published (§9.2) |
| `mot/dst/lin/dev/bar/act/hum/hgt/obs/byp/mis` | the latest-state table, i.e. one field per source |

### 8.3 Size, and why it does not matter here

Measured, not estimated: the state line above is **141 bytes** against **101** for
the `key=value` form — about **1.4×, not the "roughly 2×"** the first draft
claimed. At 1 Hz that is 1.1 kbit/s, on a link that ran 64 × 48-byte UDP packets
stop-and-wait. The cost that actually matters is not bandwidth but the payload
ceiling and the queue record (§6.4), so §16 decision 10 moves the payload maximum
from 192 to **256** — enough for the state line *and* the §9.3 snapshot, which is
longer because it carries diagnostics.

### 8.4 Why `tm_sprintf` is still the right writer

Integer-only JSON needs no new code: the value formatter is the same integer
`%u`/`%d`, and the punctuation (`{`, `"`, `:`, `,`) is literal text in the format
string. There is no float to print (constraint 1 above), no string to escape, and
no nesting — so it is genuinely **one `tm_sprintf` per record**, exactly as
before, still composable into a **single** `tm_printf` if it is printed
(`NETWORKING.md` §7). A generated JSON library would be a new dependency and a
new thing to explain; a format string is neither.

### 8.5 The alternatives

| Format | Pro | Con |
|---|---|---|
| **JSON (recommended)** | any off-the-shelf client parses it and renders a tree; self-describing; additive fields; `jq` and MQTT Explorer work with zero setup | ~1.4× the bytes (irrelevant here); the **inbound** parse is real work, so the inbound format is decided separately (§16 decision 17) |
| `key=value` (first draft) | compact; parses with `strtok` on the device — cheapest inbound | **readable only by your own parser**; the instructor's client shows an opaque line |
| compact positional (`S 137 41230 2 20 …`) | smallest; trivially cheap on the device | position is meaning: a parser cannot tell a version skew from corruption, and the schema document becomes mandatory reading — while the instructor's client shows nothing usable at all |

Bandwidth was never the deciding factor. **Who owns the reader was.**

---

## 9. Command channel (M8) — subscribe, dispatch, and the barcode remap

### 9.1 The good news, verified in the container

Docker was up for this revision, so every check that was open in the first draft
is now closed — and the answers are better than the guesses.

The starting evidence was this project's own link map,
`build_make/mtk3pico_smp1_usb_cdc_wifi_mqtt.map`: it lists `mqtt_sub_unsub`,
`mqtt_set_inpub_callback`, `mqtt_disconnect`, `mqtt_client_free` and
`mqtt_client_is_connected` among its **discarded input sections** — present in the
compiled `mqtt.o`, thrown away by the linker only because nothing calls them.
Reading the SDK headers inside the build image turns that into facts:

| Fact | Evidence |
|---|---|
| lwIP version | **2.2.1** (`init.h:53-57`) |
| `mqtt_subscribe` exists — as a **macro**, which is why the map had no such symbol | `mqtt.h:195`: `#define mqtt_subscribe(client, topic, qos, cb, arg) mqtt_sub_unsub(client, topic, qos, cb, arg, 1)` |
| …and the sixth argument is **1 to subscribe, 0 to unsubscribe** | `mqtt.h:198` (`…, 0, cb, arg, 0`); the function adds the QoS byte when `sub != 0` (`mqtt.c:1197`) |
| the inbound hook, exact signature | `mqtt.h:188-189`: `void mqtt_set_inpub_callback(mqtt_client_t *client, mqtt_incoming_publish_cb_t pub_cb, mqtt_incoming_data_cb_t data_cb, void *arg)` |
| the two callback types | `mqtt.h:151`, `:163` — `void (*)(void *arg, const u8_t *data, u16_t len, u8_t flags)` and `void (*)(void *arg, const char *topic, u32_t tot_len)` |
| incoming QoS 1 is **acknowledged by the library**, not by us | `mqtt.c:776-783`: "Reply if QoS > 0 … Send PUBACK for QoS 1 or PUBREC for QoS 2" |
| liveness check | `mqtt.c:1480-1485`: `mqtt_client_is_connected()` returns `conn_state == MQTT_CONNECTED` |
| credentials and last will are supported by the struct | `mqtt.h:71-87`: `client_user`, `client_pass`, `will_topic`, `will_msg`, `will_qos`, `will_retain` |

> **The `LWIP_MQTT` puzzle is solved: there is no such option in lwIP 2.2.1.** It
> appears nowhere in the SDK's include tree. The client is guarded by
> `#endif /* LWIP_TCP && LWIP_CALLBACK_API */` (`mqtt.c:1487`), with
> `LWIP_CALLBACK_API` defaulting to 1 (`opt.h:1517-1519`) and `LWIP_TCP` set to
> `TM_WIFI_TCP` in this tree (`lwipopts.h:25`). So the **makefile's source list is
> the entire gate.** The earlier suggestion to "pin `LWIP_MQTT` explicitly" is
> withdrawn — and §11's gate change turns out to be mandatory, with an exact
> mechanism, rather than tidy.

### 9.2 Topics and quality of service

| Topic | Direction | QoS | Why |
|---|---|---|---|
| `robocar/<id>/telemetry` | Pico → broker | 0 | periodic + edge records (§7). Fire-and-forget is right for a stream |
| `robocar/<id>/state` | Pico → broker | 0, retained | the on-request snapshot reply, so a late subscriber still sees the last one |
| `robocar/<id>/cmd` | broker → Pico | **1** subscribe | the on-request commands (`snap`, `ping`) — QoS 1 retries within a live session, but is **not** durability across a disconnect; see the correction below |
| `robocar/<id>/map` | broker → Pico | **1** subscribe, **retained** | the live barcode table. Retained, so a reconnecting or rebooting car is handed the current table the moment it subscribes |
| `robocar/<id>/heartbeat` | Pico → broker | 0 | see §10.3 — recommended to fold into the periodic line, decision 6 |

`<id>` comes from `UTK_MQTT_CLIENT_ID` in `config/broker_config.h`, so the topic
tree stays configuration rather than a compiled-in constant. QoS 0 on publish is
what the module already does (`lwip_utk_mqtt.c:202-204`, the two literal `0`
arguments are QoS and retain) and its comment at `:111` explains why a QoS 0
publish completes from the TCP sent-callback rather than from an ack.

> **Correction — QoS 1 does not survive a disconnect here.** lwIP's client forces
> a clean session unconditionally: *"Don't complicate things, always connect
> using clean session"*, `flags |= MQTT_CONNECT_FLAG_CLEAN_SESSION;`
> (`mqtt.c:1375-1376`). A clean session means the broker **throws the session
> state away the moment we drop** — our subscriptions (hence the re-subscribe in
> §10.1) *and* any QoS 1 message it was holding for us. So QoS 1 buys delivery
> across a lost packet **while connected**, not across a reconnect. The first
> draft of this document said otherwise; that claim was wrong.
>
> What makes the barcode remap reliable is **retained delivery plus
> convergence** — not broker session durability, and not a polling loop on the
> laptop. This suits the requirement better anyway, because the demo-day broker
> must not be assumed to keep session state:
>
> 1. the laptop publishes the table **with the retain flag** to `…/map`;
> 2. the car **subscribes** on connect, and a broker delivers a retained message
>    *immediately on subscribe* — so after a reboot or a reconnect the car
>    receives the current table with nothing re-sent by hand. The retain bit
>    changes nothing on our side: lwIP's inbound handler looks only at the packet
>    type and QoS (`mqtt.c:708-784`), so it arrives through the same two
>    callbacks (§9.3);
> 3. the table carries a **version**, and `wifi_task` reports the version it is
>    actually running in every periodic line (`mpv=` in §8) — so the laptop can
>    *verify* what the car holds instead of assuming;
> 4. the remap is **idempotent** — applying the same table twice changes nothing,
>    which makes a duplicate or a retained replay harmless;
> 5. if some future broker does not honour retain, step 3 still catches it: the
>    laptop re-publishes until `mpv` matches what it sent.
>
> So a `map` that arrives twice is a no-op, and a `map` missed while the car was
> away comes back with the retained message. The version comparison is the last
> line of defence, and it needs no broker feature at all.

### 9.3 The inbound path, step by step

```
broker ──▶ radio task (inside mqtt_set_inpub_callback)
             1. topic + payload arrive, possibly in chunks
             2. reassemble into a bounded buffer (~192 B); overflow = drop + count
             3. tk_snd_mbf(tele_rx_mbfid, …)  ← copy only; never parse, never block
           ──▶ wifi_task
             4. tk_rcv_mbf, parse the textual command
             5. dispatch by type
```

> **Two rules the callbacks impose — both documented by the library, not
> inferred.** They are the kind of detail that produces working-looking code
> that corrupts at random, so they belong in the design rather than in a
> discovery during debugging.
>
> 1. **The pointers die when the callback returns.** `mqtt.h:150` — the data
>    pointer "may not be referenced after callback return"; `mqtt.h:163` — the
>    topic string likewise, and lwIP restores the byte it used to zero-terminate
>    the topic as soon as `pub_cb` returns (`mqtt.c:753-766`). So the copy into
>    `tele_rx_mbfid` happens **inside** the callbacks.
> 2. **Reassembly is mandatory, not defensive.** The client copies what fits into
>    its own receive buffer and streams the remainder in further calls
>    (`mqtt.c:896-915`); the final fragment carries `MQTT_DATA_FLAG_LAST`
>    (`mqtt.h:143-148`, `mqtt.c:774`). With a ~25-byte topic and the default
>    128-byte receive buffer (`mqtt_opts.h:64`), any command payload over ~100
>    bytes arrives in pieces — which the `map` command will be.

```c
static void inpub_cb(void *arg, const char *topic, u32_t tot_len)
{ /* reset the assembler; copy the topic now, while it is still valid */ }

static void indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{ /* append to the assembler; on MQTT_DATA_FLAG_LAST → post to tele_rx_mbfid */ }
```

`wifi_task` dispatches three command types:

| Command | Action | Evidence it must produce |
|---|---|---|
| `snap` | publish a state line plus **diagnostics** (uptime, RSSI, IP, queue depth, drop counters, MQTT verdict) on `…/state` | the laptop asks, and gets an answer within one tick |
| `map` (arrives on `…/map`, retained) | validate the letter→action table, then `tk_snd_mbf(tele_cmd_mbfid, …)` to `line_task`, and update `wifi_task`'s own copy and its `mpv` | the remap arrives **without a recompile** — the requirement that cannot be hardcoded |
| anything else | count it and print one line | an unknown command is visible, not silence |

A fourth, optional: `ping` → a single line on `…/heartbeat`, for a manual
"are you alive" that does not wait for the 1 s cadence.

> **Why the snapshot carries diagnostics and not just state.** Reported failures
> in this stack are silent (`NETWORKING.md` §7). The on-request snapshot is the
> one place where "why did it fail?" can be answered *from the laptop* without a
> debugger — so it should carry the counters, not only the sensor values.

`line_task` is not written yet (it is Buddy 3's stub), so the remap path needs
one agreement: the record shape on `tele_cmd_mbfid` and whether `line_task` calls
`tk_rcv_mbf` from its own loop (proposed) or uses an event flag. Proposed record:
a fixed 26-byte ASCII letter→action table plus a version counter, so `line_task`
can swap it in one assignment.

---

## 10. Recovery and liveness (M9)

M9 is graded as "must survive all 3 demo attempts", so it is not polish — it is
the difference between one pass and three.

### 10.1 The reconnect ladder

```
connection lost (callback status, publish error, or is_connected() false)
    → state DISCONNECTED, disconnect + free the client if needed
    → wait backoff: 2 s, 4 s, 8 s, capped at 15 s
    → CONNECTING: mqtt_client_connect() again
    → on CONNACK: re-subscribe to …/cmd   ← subscriptions do NOT survive a new session
```

Three details the library dictates, now verified rather than assumed:

- `mqtt_client_is_connected()` (`mqtt.h:186`, `mqtt.c:1480-1485`) is the cheap
  liveness check to run each poll. Do **not** rely on the module's current
  `connected` flag — nothing clears it when the session dies.
- **Subscribe only while connected.** `mqtt_sub_unsub` returns `ERR_CONN`
  immediately if the client is in the disconnected state (`mqtt.c:1202-1205`),
  and asserts `qos < 3` (`:1201`).
- **Log the subscribe verdict — it is M8's evidence.** The request callback
  reports `ERR_OK`, `ERR_TIMEOUT` (no SUBACK in time) or `ERR_ABRT` (denied)
  (`mqtt.h:172-177`). "We subscribed and the broker said yes" is a console line
  worth having on demo day.

### 10.2 Wi-Fi dropping, which is a different failure

If the link goes down, the address disappears, and the per-poll guard
(`lwip_utk_mqtt.c:170-175`) blocks every attempt. Because `dhcp_complete` is a
latch that never clears (`cyw43_utk.c:464`), the *outer* gate stays open, so the
module keeps being called every 2 ms — good, because that is what lets it notice
recovery. The ladder must therefore treat "guard blocked" as a state to leave
gracefully rather than as "nothing to do": reset to `WAIT_ADDR`, and reconnect
from scratch once an address returns.

### 10.3 Liveness

The periodic state line **is** the heartbeat: if the laptop receives nothing for
a few seconds, either the car stopped publishing or the link is down, and both
are worth knowing. That makes a separate heartbeat topic redundant — decision 6.

The stronger option is an MQTT **last will** (LWT): the broker publishes a
message on your behalf if the connection dies abruptly, so the laptop can tell
"crashed" from "network died" without waiting. lwIP's
`mqtt_connect_client_info_t` supports it, and the module's client info currently
sets only `client_id` and `keep_alive` (`lwip_utk_mqtt.c:121-124`). It is
optional, needs the broker to honour it, and is worth one sentence in the demo —
not worth blocking M7.

---

## 11. One build-gate change this design requires

Today `WIFI_MQTT=1` requires `WIFI_TCP=1` (`build_make/pico_rp2040.mk:155`), and
`WIFI_TCP` enables *two different things at once*: the TCP **stack**, and the TCP
**echo test**. MQTT needs the first and not the second.

Consequence today: every telemetry build also runs the UDP and TCP echo phases —
noisy on the console, wasteful on the radio, and `result=-3` whenever no echo
server is running (`NETWORKING.md` §11, item 1). The fix is to decouple the two,
which is a small, well-bounded change:

| Where | Today | Proposed |
|---|---|---|
| `lwipopts.h:25` | `LWIP_TCP  TM_WIFI_TCP` | `LWIP_TCP  (TM_WIFI_TCP \|\| TM_WIFI_MQTT)` |
| `lwipopts.h:45-49` | `MEM_SIZE` gated on `TM_WIFI_TCP` | same gate, plus `\|\| TM_WIFI_MQTT` |
| `lwipopts.h:71-77` | TCP pools gated on `TM_WIFI_TCP` | same gate, plus `\|\| TM_WIFI_MQTT` |
| `pico_rp2040.mk:155` | `WIFI_MQTT=1 requires WIFI_TCP=1` | `WIFI_MQTT=1 requires WIFI_DHCP=1` |

> **The mechanism, now verified — this is not cosmetic.** The MQTT client's
> translation unit is compiled out entirely when `LWIP_TCP` is 0: its guard is
> `#if LWIP_TCP && LWIP_CALLBACK_API` (`mqtt.c:1487`). So a build with
> `WIFI_TCP=0` and `WIFI_MQTT=1` still hands `apps/mqtt/mqtt.c` to the compiler —
> the makefile adds it on the MQTT gate (`pico_rp2040.mk:418-420`) — but gets an
> **empty object file**, and the link then fails on undefined references to
> `mqtt_client_new`, `mqtt_publish` and `mqtt_set_inpub_callback`. The two edits
> are one change, not two. (`LWIP_ALTCP` already follows the MQTT gate —
> `lwipopts.h:30` — so half the prerequisites are already in place.)

After it, the telemetry build string becomes the M2–M3 rungs plus MQTT — a
shorter ladder with less unrelated traffic:

```sh
make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 \
     WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 -j8
```

---

## 12. Configuration

Nothing about the deployment may be compiled in: on demo day the broker is the
instructor's.

| Value | Lives in | State |
|---|---|---|
| broker address, port | `config/broker_config.h` | exists (`broker_config.example.h:44-45`) |
| client id | same | exists — and doubles as the `<id>` in every topic |
| keep-alive | same | exists (60 s) |
| **topic prefix and map topic** | same, **proposed addition** | new — so `robocar` and the `…/map` name are not compiled in either (§9.2) |
| **username / password** | same, **proposed addition** | new — the struct supports both (`mqtt.h:71-73`); the module sets neither today (`lwip_utk_mqtt.c:121-124`) |
| publish cadence | `messages.h` or a config header | TBD — proposed 1 s, tracked as open item 4 in [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §12 |

The `*_SET` guard pattern is the enforcement: copy the `.example.h`, fill it in,
flip the flag to `1`. An unedited copy **fails at compile time**
(`broker_config.example.h:39-40`), which is deliberate — a value to flip rather
than a line to delete. Populated `config/*.h` files are gitignored, and the
`.gitignore` entry for `broker_config.h` exists because the file once showed up
as an untracked near-miss (`NETWORKING.md` §6.3, Group A).

---

## 13. Resource budget

The RP2040 has 264 KB of RAM and the TCP profile's last recorded image footprint
was `text 340505 / data 4640 / bss 52188` (`NETWORKING.md` §6.5) — about 51 KB of
`bss`. Everything below is affordable, but the numbers should be *stated* rather
than discovered:

| Consumer | Cost | Note |
|---|---|---|
| App tasks now: 6 × 4 KB stacks | 24 KB | `STACK_SZ 4096`, `app_main.c:73`; stacks are the dominant RAM cost |
| `wifi_task` stack | 4 KB | a seventh task |
| Radio task stack | 6 KB | `cyw43_utk_kernel.c:18` |
| Status snapshot | ~2.3 KB | copied per `cyw43_utk_get_status()` call; **do not grow it** (§6.4) |
| lwIP heap `MEM_SIZE` | 12 KB | `lwipopts.h:45-49` |
| lwIP pbuf pool | 8 × 1600 B = 12.8 KB | `lwipopts.h:60-61` |
| MQTT client buffers | 384 B by default, 768 B after the §6.4 proposal | inside `mqtt_client_t` (`mqtt_priv.h:66`, `:95`), from the lwIP heap |
| TCP send window + buffers | ~6 KB | `TCP_SND_BUF`/`TCP_WND` = 2 × MSS each, `lwipopts.h:75-76` |
| `MEMP_NUM_SYS_TIMEOUT` | 12 slots | raised *because* MQTT parks a cyclic timer (`lwipopts.h:51-58`) |
| Four app mailboxes | ~2.5 KB total | 16 × 12 B + 4 × 320 B + 4 × 26 B + 4 × 192 B plus per-message overhead |
| lwIP TCP PCBs | 6 | `lwipopts.h:71` — MQTT holds one, the echo test held one |

Two entries are load-bearing rather than merely interesting:

- **`MEMP_NUM_SYS_TIMEOUT = 12`.** lwIP's fixed pool of timer slots is consumed by
  DHCP, DNS, ARP and TCP, and the MQTT client parks a cyclic keep-alive timer on
  top. Running out trips `lwip_utk_assert`, which halts the board **silently**
  (`lib/libnet/lwip/lwip_utk.c:33-37`). Adding a second client, or a second
  cyclic timer, means revisiting this number.
- **`MEMP_NUM_TCP_PCB = 6`.** Each finished connection lingers in `TIME_WAIT`
  while still holding a pool entry (`lwipopts.h:67-70` explains the history). A
  reconnecting client frees and re-acquires entries, so the ladder must not
  leak clients — `mqtt_client_free()` on every disconnect, one client alive at a
  time.

---

## 14. Test plan — M7 to M10, one gate at a time

Every gate is stated as *observable evidence*, because the graded claim is not
"the code compiles" but "here is the console line and the broker log". Build
through Docker always (`arm-none-eabi-gcc` exists only in the image), and note
that **Docker's daemon was not running on this machine at the time of writing**
(§17) — nothing in this document has been built.

### M7 — telemetry framework
- [ ] Build: the §11 string. Console: `[wifi] link UP`, `[wifi] ip=…`,
      then telemetry lines beginning `{"t":"S","v":1,"s":1,"up":…`.
- [ ] **Gate:** a subscriber on `robocar/<id>/telemetry` logs 10 consecutive
      state lines whose `s` increments by exactly 1 with no gap, over ≥10 s.
- [ ] **Gate:** the two demo-critical events appear as `"t":"E"` records within
      one tick of being raised: send a synthetic `EVT_BARCODE` and see `"k":"B"`,
      and an `EVT_IMU` and see `"k":"I"`.
- [ ] **Gate:** subscribe once with a **generic** client (MQTT Explorer or
      `mosquitto_sub`) and confirm the JSON renders as a tree/object with no
      custom parser — the demo-day reader (§8.1).
- [ ] **Gate:** pull the AP for 20 s mid-run. The car keeps running, `drp` stays
      0 (nothing stalled), and after the AP returns the state lines resume.
- [ ] Regression: a TCP-only build's image size is unchanged by the §11 gate
      edit — the same check that proved M6 left M5 untouched.

### M8 — command channel
- [ ] Subscriber→publisher: publish `snap` on `…/cmd`; a `…/state` line appears
      within one tick carrying the diagnostics.
- [ ] **Gate (the hard requirement):** publish a new letter→action table on
      `…/cmd`; `line_task` (or a stand-in test task) receives it on
      `tele_cmd_mbfid`, and a barcode read afterwards decodes to the **new**
      action. Recompile: none. This is the requirement that cannot be satisfied
      by a compiled-in table.
- [ ] Gate: a malformed command produces one visible line and changes nothing.

### M9 — robustness
- [ ] Kill the broker mid-run: the ladder backs off, logs each attempt, and
      reconnects when the broker returns — including re-subscribing, proven by
      a `snap` command working *after* the reconnect.
- [ ] Kill and restart the Wi-Fi AP: the same, from `WAIT_ADDR`.
- [ ] **Gate:** three consecutive full runs with no manual intervention — M9 is
      graded on surviving all 3 demo attempts.
- [ ] Gate: total run time still under the 5-minute budget, with telemetry
      enabled, measured on the console timestamps.

### M10 — documentation
- [ ] `docs/MQTT_SCHEMA.md`: topics, the `"S"`/`"E"` JSON records (§8.2), the
      event catalogue as a table a teammate can implement from without reading C.
      **First draft written 2026-09-28**; its §12 lists the fields still proposed
      (the mission letter, the heartbeat/snapshot key sets, the cadence).
- [ ] Broker setup walkthrough (the `amqtt` + `paho-mqtt` recipe, including the
      `python.exe`-not-`amqtt.exe` firewall trap in `NETWORKING.md` §4).
- [ ] A "third party" test: hand the schema to a teammate and have them write a
      subscriber **without** looking at the firmware — and subscribe once with a
      **generic** client (MQTT Explorer or `mosquitto_sub`) to confirm the JSON
      renders for a reader who never saw this repo (§8.1).

> **What it means:** M7's third gate is the one people skip and the one that
> catches the most. Every failure mode in this stack is silent
> (`NETWORKING.md` §7), so "it worked while everything was healthy" is not
> evidence — an interrupted run that recovers *is*.

---

## 15. Risks, ranked

1. **The single-threaded rule.** Calling lwIP from `wifi_task` compiles fine and
   corrupts state at random. The queues in §3 are the mitigation, and the
   discipline to keep is that no `lwip_*` symbol ever appears in
   `app_program/`.
2. **Editing proven code.** Option 1 modifies the module that produced the M6
   evidence. Mitigated by the gate staying on `WIFI_MQTT`, the evidence being
   captured in `NETWORKING.md` §8, and the M6 console line being preserved.
3. **Telemetry delaying the car.** Every producer send is non-blocking (§4.2),
   `wifi_task` runs at a lower priority than the radio and console (§3.1), and
   payload formatting is integer-only.
4. **"Explain every line."** The M6 module is only 224 lines and is already
   yours; growing it in place keeps the code explainable. Do not paste in a
   generated MQTT client.
5. **The live barcode remap.** The single requirement that cannot be
   hardcoded, and the one that cannot lean on the broker's memory: with a clean
   session forced (§9.2), a plain command published while the car is away is
   simply gone. Hence retain plus a reported table version. It is also the only
   reason the inbound path (subscribe) exists, so build it early in M8 rather
   than last.
6. **Resource exhaustion → silent halt.** `MEMP_NUM_SYS_TIMEOUT` and
   `MEMP_NUM_TCP_PCB` (§13) fail by halting the board with no message.
7. **Demo-day network.** The robot must join the instructor's AP, and this Pico W
   has already refused a WPA2/WPA3 mixed network and seen a hotspot's DHCP stop
   (`NETWORKING.md` §4). Confirm 2.4 GHz WPA2 without required WPA3/PMF **before**
   demo day — no amount of firmware robustness survives it.

---

## 16. Decision log

**Taken** (recorded this session):

| # | Decision | Choice | Date |
|---|---|---|---|
| 1 | Inbound seam (`RTOS_INTERFACES.md` §4) | **A** — one event mailbox | 2026-09-27 |
| 2 | Outbound seam (`RTOS_INTERFACES.md` §5) | **1** — generalize in place | 2026-09-27 |
| 3 | Publish model | **hybrid** — periodic state + edge events | 2026-09-27 |

**Open — yours to take.** Each is bounded, and the design above states a
recommendation for each so a decision is a yes/no rather than new analysis.

| # | Question | Recommendation | Alternatives |
|---|---|---|---|
| 4 | Mailbox granularity: one `EVT_*` mailbox for all four buddies, or one per buddy? | one — one contract, one drain loop | per-buddy queues isolate a noisy producer |
| 5 | Producer send: never block (+ drop counter) or a bounded block? | never block, count drops (§4.2) | 10 ms bound, as `RTOS_INTERFACES.md` §4 |
| 6 | Heartbeat: fold into the periodic line, or its own topic? | **own topic** — changed 2026-09-27: M9's requirement says "**heartbeat topic** so liveness is visible" in writing, and a topic name is a string literal, not code. Fold was the earlier recommendation | fold into the periodic line; cheaper, and the absence of a line is still the signal |
| 7 | The reporter: keep `wifi_status_task` as-is, rename it, or fold it into `wifi_task`? | keep it as a read-only reporter and **rename** it `wifi_report_task` | fold — one task, but it then prints radio internals it does not own |
| 8 | `wifi_task` affinity: unpinned, or pinned to processor 2? | unpinned; priority 8 already keeps it out of the radio's way | `TP_PRC2` keeps it off the radio+USB core entirely |
| 9 | **Outbound** wire format: JSON, key=value, or compact positional? | **changed 2026-09-28: JSON (§8).** The reader may be the instructor's client, and every generic MQTT tool renders JSON with no help; producing it costs one `tm_sprintf`. The earlier `key=value` recommendation assumed you own the reader, which you do not | key=value saves ~35% of the bytes; positional saves more, but neither is readable by the instructor's client |
| 10 | Payload maximum: 256 bytes, or larger? | **changed 2026-09-28: 256** (was 192), **plus** raising the client's ring buffer to 512 and its receive buffer to 256 (§6.4) — the JSON state line is 141 B and the diagnostic snapshot is longer (§8.3) | smaller forces shorter lines and breaks the snapshot; larger costs RAM in the lib's queues |
| 11 | QoS on the wire: QoS 0 everywhere, or QoS 1 for the remap? | **changed 2026-09-27: QoS 0 for telemetry *and* for `…/cmd`; QoS 1 on the `…/map` *subscription* only.** lwIP never retransmits a QoS 1 publish — the request just expires with `ERR_TIMEOUT` (`mqtt.c:390-413`) — so QoS 1 outbound buys detection, not delivery. QoS 1 *inbound* is acknowledged by the library (`mqtt.c:776-783`), so subscribing at QoS 1 costs no application code | QoS 1 outbound on `cmd`; costs a request slot and a callback for a signal the design does not act on |
| 12 | Decouple the TCP stack from the TCP echo test (§11)? | yes — it removes the noise and the `-3` lines | leave as-is; telemetry builds stay noisy |
| 13 | Remap transport: a **retained `…/map` topic** (converges on subscribe), or plain commands on `…/cmd` re-sent by the laptop? | retained `…/map` — a rebooting car is handed the current table for free (§9.2) | one command topic is marginally simpler to explain; costs a re-publish loop on the laptop and has no answer for a car that missed the message |
| 14 | Reconnect rate: the 2/4/8/15 s ladder, a fixed interval, or "attempt on every poll"? | **fixed interval of 2–5 s** (a counter and a timestamp, ~6 lines). The ladder's *curve* is optional; a **rate limit is not** — the radio poll runs every 2 ms, each attempt takes one of only **6** TCP PCBs (`mqtt.c:1395`, `lwipopts.h:71`), and connections that were established then closed hold a pool entry for 2MSL (`lwipopts.h:67-70`), so an unbounded retry rate is what can leave the client unable to reconnect at all | the full 2/4/8/15 s ladder, if the extra states are worth it to you |
| 15 | Device-side event history: keep the bounded ring, or drop it? | **drop it** — nothing on the device consumes the history. M7's "reconstructable map of the run" is reconstructed from the **published stream** on the laptop, using `s=` and `drp=` (§8); the ring only makes sense if the device must *replay what was missed while offline*, which M7 does not ask for and which fights the never-block producer rule (§4.2) | keep the ring if you want a reconnect-replay feature later |
| 16 | Inbound reassembly: full fragment assembler, or single-fragment? | **single-fragment, with a bound**: handle `MQTT_DATA_FLAG_LAST` and reject anything that arrives in more than one fragment. With a 128-byte receive buffer and a ~25-byte topic, one fragment carries ~95 bytes — so the `map` payload must fit that, enforced by a static assert. Cheaper, and the failure is a rejected command rather than a truncated table | full assembler (as §9.3 describes) if `map` may ever exceed ~95 bytes |
| 17 | **Inbound** (`cmd`, `map`) wire format: `key=value`, or a flat JSON subset? | **`key=value`** — you author the publisher (§9.2), so the reader here *is* yours, and `strtok` is far cheaper to defend line by line than a device-side JSON scanner (§8.1). Consistency with the outbound JSON is cosmetic; the cost is real. **Open on 2026-09-28: the inbound publisher is unconfirmed** — do not flip this on "maybe"; see the hedge in §8.1 | flat JSON inbound, so the whole system speaks one format — at the cost of a scanner in `wifi_task` and more code to explain. Choose this only if the instructor may publish commands himself |

**Reviewed 2026-09-28** — the wire-format question was reopened because the reader
on demo day is the instructor's client, not your own script: rows 9 and 10 changed,
and row 17 (the *inbound* format) is new. (`§8`'s original reason for rejecting
JSON — "the payload's only consumer is your own subscriber" — was the wrong
premise, and §8.1 replaces it with the produce-vs-parse asymmetry.)

**Previously reviewed 2026-09-27** against a scope pressure-test: rows 6 and 11
changed their recommendation; 14–16 were added. Rows 4, 5, 7, 8, 12, 13 stand.

---

## 17. Verification status

Everything that was open in the first draft was checked against the build image
while Docker was up. Nothing below is assumed.

| Question | Verdict | Evidence |
|---|---|---|
| Does the SDK's lwIP expose a subscribe and an inbound path? | **yes** — no SDK change needed | §9.1 (`mqtt.h:188-189`, `:195`) |
| Is `mqtt_subscribe` a function or a macro? | **macro** over `mqtt_sub_unsub`; sixth arg `1` = subscribe | `mqtt.h:195` |
| Who acknowledges an incoming QoS 1 publish? | **the library**, automatically | `mqtt.c:776-783` |
| Who defines `LWIP_MQTT`? | **nobody — it does not exist in lwIP 2.2.1**; the gate is `LWIP_TCP && LWIP_CALLBACK_API` | `mqtt.c:1487`, `opt.h:1517-1519` |
| Which millisecond clock should the app stamp records with? | `tk_get_otm()` | `include/tk/syscall.h:822`, `config/config_func.h:126` |
| Does a plain command survive a reconnect? | **no** — a clean session is forced, which is why §9.2 uses retain | `mqtt.c:1375-1376` |
| How big are the client's own buffers? | 256 B out, 128 B in by default — inbound payloads fragment | `mqtt_opts.h:56`, `:64` (§6.4) |
| Can several readers copy the status snapshot safely? | yes — retry-until-clean seqlock, 16 attempts | `cyw43_utk.c:44`, `:665-685` |

Two housekeeping items remain. Neither is a design question, but both are
worth closing before the report:

| Item | Action |
|---|---|
| `cyw43_utk_get_status`'s comment calls the copy "~230-byte"; the struct is ~2.3 KB (`cyw43_utk.c:670`) | fix the number while in the file |
| `app_program/app_main.c.old` is a stray backup in the app tree — not compiled, since the extension is not `.c` | delete it, or say why it is there; it is clutter in a graded repo |

---

## 18. Don't redo

- Don't rebuild the transport. Scan, join, DHCP, DNS, UDP, TCP and a one-shot
  MQTT publish are proven on hardware (`NETWORKING.md` §5, §6).
- Don't write a second MQTT client. Option 1 was chosen so there is one client,
  one connection and one client id (§6.2).
- Don't call any `lwip_*` function from `app_program/` (§2).
- Don't put a shared variable between two tasks — use a queue (§3.3).
- Don't grow `T_CYW43_UTK_STATUS` to carry telemetry (§6.4).
- Don't invent a wire format only your own script can read — on demo day the
  reader is the instructor's client, so the outbound stream is JSON (§8.1).
- Don't hardcode the broker, the topic prefix, or the barcode mapping (§12).
- Don't commit a populated `config/*.h` (§12).
- Don't expect the radio code in `app_program/wifi_task/` — it lives in
  `lib/libwifi/` and `lib/libnet/`; that folder is the application task.
- Don't build on the host. `arm-none-eabi-gcc` exists only inside the Docker
  image.
- Don't print a line with several `tm_printf` calls — compose with `tm_sprintf`,
  then emit once (`NETWORKING.md` §7).

---

## 19. Glossary

| Term | Plain meaning |
|---|---|
| **Task** | The RTOS equivalent of a thread. |
| **Radio-owner task** | `cyw43_utk_task` — the single task allowed to call lwIP. |
| **Message buffer** (`tk_snd_mbf` / `tk_rcv_mbf`) | A thread-safe queue of variable-length records between tasks. |
| **`TMO_POL`** | Timeout value meaning "do not block; check and return now". |
| **`TMO_FEVR`** | Timeout value meaning "block forever". |
| **Seam** | A point where two layers hand data across — here, always a kernel object. |
| **Seqlock** | A lock-free way to copy a struct one task writes while another reads: check a version counter before and after, retry if it changed. |
| **Latch** | A flag set once and never cleared — e.g. `dhcp_complete`, which is why the MQTT poll runs forever after DHCP succeeds. |
| **Persistent client** | A connection kept open across the run, rather than connecting for one message. |
| **QoS 0 / QoS 1** | MQTT delivery guarantees: "at most once, fire and forget" / "at least once, acknowledged". |
| **Retain** | An MQTT flag: the broker keeps the last message on a topic and hands it to any later subscriber. |
| **Clean session** | An MQTT connect flag meaning "forget my previous session". lwIP always sets it, so subscriptions and undelivered messages are discarded the moment we disconnect. |
| **Idempotent** | Doing it twice has the same effect as doing it once — what makes a re-sent or replayed command safe. |
| **LWT / last will** | A message the broker publishes on your behalf if your connection dies abruptly. |
| **Backoff** | Waiting progressively longer between retries so a broken peer is not hammered. |
| **Reassembly** | Rebuilding a message that arrived in pieces, bounded by a fixed buffer. |
| **Payload** | The message body, as opposed to the topic (the channel name). |
| **Edge event** | A record published the moment something happens, rather than at the next tick. |
| **Snapshot** | The current state at one instant, as opposed to the history of events. |
| **FPU** | Floating-point unit — the hardware for decimal maths. The RP2040 has none, so payloads are integers. |
| **UF2** | The file format the Pico's bootloader accepts on drag-and-drop. |
| **Build profile** | The flags a build was made with, baked into the image name. |