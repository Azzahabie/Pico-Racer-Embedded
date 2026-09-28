# WIFI_SUBSYSTEM_DESIGN, translated for a software engineer

`WIFI_SUBSYSTEM_DESIGN.md` is the full spec, written for someone with zero
embedded background. This document is the opposite trade: it assumes you
already think in threads, queues, APIs and services, and just need the
embedded vocabulary mapped onto that model. Read this first, then dip into
the full doc's section numbers (given in `§N` below) for the evidence and
line-number citations.

---

## 1. The one-sentence version

**`wifi_task` is a new backend service that turns car events into MQTT
messages; it talks to the network only through a queue, because the
networking library underneath is not thread-safe and only one "worker
thread" is allowed to touch it.**

Everything else in the design — the four queues, the state machine, the
retry/backoff logic, the wire format — is what you'd build in any system
that has exactly one connection to a shared, non-reentrant resource and many
producers that want to use it.

---

## 2. The core constraint, in SE terms

lwIP (the TCP/IP stack) is built with `NO_SYS=1` — no internal locks, no
thread safety, at all (`§2`). That's not a limitation of this RTOS; it's a
design choice lwIP makes for footprint, the same way you'd choose a
single-threaded event loop (think: classic Redis, or Node's event loop, or
an unsynchronized `HashMap`) to avoid locking overhead when you control who
calls in.

The consequence is familiar from any codebase with one of those:

> **You don't put a lock around it. You give it exactly one caller.**

Here, that caller is `cyw43_utk_task` — call it **the radio-owner thread**.
It already exists (created before your code runs), it polls the radio every
2ms, and it is the *only* thread allowed to call into lwIP or the MQTT
client. Every other task — including the new `wifi_task` — is API-blocked
from touching the network directly. They can only hand it work.

This is the actor model / single-writer principle you already know:
**shared mutable state is owned by exactly one thread, and everyone else
talks to it via messages**, not by grabbing the state directly. The design
doc calls the message queues "kernel objects"; functionally they're
bounded, thread-safe **channels** (like a Go channel, a `BlockingQueue`, or
a tiny in-process Kafka partition with one consumer).

| Embedded term | SE equivalent |
|---|---|
| Task | Thread |
| `tk_cre_tsk` / task priority | Thread creation / scheduling priority (lower number = higher priority here — inverted from `nice`) |
| CPU affinity / "pinned to processor 1" | `taskset` / thread affinity — this thread can only run on core 0 |
| Message buffer (`tk_snd_mbf` / `tk_rcv_mbf`) | A bounded, thread-safe queue / channel — MPSC (many producers, single consumer) |
| `TMO_POL` | Non-blocking `try_send` / `offer()` — fails immediately if full, doesn't wait |
| `TMO_FEVR` | Blocking `put()` — waits forever |
| Seqlock | Optimistic concurrency read: read a version counter, copy the data, re-check the counter, retry if it changed mid-copy — lock-free reads, like an MVCC snapshot read |
| Latch (e.g. `dhcp_complete`) | A one-way flag / `AtomicBoolean` that's set once and never reset — like a "warmed up" flag that never goes back to cold |

---

## 3. The architecture, as a system diagram you'd actually draw

```
 4 producer threads  ──▶  [queue: tele_mbfid]  ──▶  wifi_task (consumer)
 (motion/line/imu/       "events in, ordered"        - maintains an in-memory
  obstacle tasks)                                       "latest state" table
                                                        - appends to a ring log
                                                        - formats + rate-limits
                                                              │
                                                              ▼
                                            [queue: tele_pub_mbfid]
                                            "outbox to the network thread"
                                                              │
                                                              ▼
                                              radio-owner thread (the ONLY
                                              thread allowed to touch lwIP/MQTT)
                                                              │
                                                       MQTT over Wi-Fi
                                                              │
                                                              ▼
                                                          broker (laptop)
                                                              │
                                        (commands / config push, retained)
                                                              │
                                                              ▼
                                   [queue: tele_rx_mbfid]  ◀── radio-owner thread
                                   "inbox from the network thread"
                                              │
                                              ▼
                                          wifi_task (dispatches commands)
                                              │
                                              ▼
                                   [queue: tele_cmd_mbfid] ──▶ line_task
```

If you've built a service that sits in front of a single pooled connection
(a single Redis connection multiplexer, a single gRPC channel shared by many
callers, a job that owns the one write-handle to a file), this is the same
shape: **fan-in queue → single owner → fan-out queue**, applied twice
(outbound and inbound).

`wifi_task` itself is the interesting new piece — it's a small **event
aggregator / API gateway**, not a networking component. It:

- drains an inbound event queue (`§4.3`),
- folds each event into a **latest-state table** (think: an in-memory cache
  keyed by source, like `Map<Source, LatestValue>` — no lock needed because
  exactly one thread ever writes it, the textbook single-owner argument),
- formats and pushes both a periodic snapshot and ad-hoc "edge" events
  downstream, rate-limited by *not blocking* on a full queue (drop + count,
  `§4.2`) — the same trade-off as a metrics/logging pipeline that prefers
  dropping a sample over blocking the hot path.

---

## 4. Why a queue and not just a mutex — the design's own argument (§2.1)

This is worth calling out because it's a genuinely good piece of reasoning,
and it generalizes beyond embedded work:

A mutex around lwIP's entry points wouldn't be enough, because lwIP
**re-enters itself** from its own internal callbacks (timer callbacks,
inbound packet handling) — i.e. the "second caller" can be *lwIP calling
lwIP*, from inside the same call stack, not just another external thread
racing in. A lock only serializes *external* callers; it does nothing about
re-entrancy from within the one call you already let through.

The SE analogy: this is the same reason a library that fires callbacks
*synchronously from inside your own call into it* is dangerous to wrap in a
naive lock — you can deadlock yourself, or (as here) still corrupt state,
because the "second caller" is the library, not a separate thread. The fix
isn't a smarter lock, it's **structural**: never let anything but the
designated thread cross that boundary directly, full stop. That's why the
whole design pivots on queues instead of synchronization primitives.

---

## 5. The MQTT client — a stateful client wrapper with retry/reconnect (§6, §10)

Today's MQTT code (`lwip_utk_mqtt.c`) is what you'd call a **smoke-test
script**: connect once, publish one hardcoded message, done. The design's
job is to turn it into what you'd recognize as a **long-lived client with
a reconnect loop**, e.g. like a resilient Kafka/AMQP client wrapper:

```
WAIT_ADDR → CONNECTING → CONNECTED (drain outbox, drain inbox, subscribe)
                              │
                    connection lost / error
                              ▼
                        DISCONNECTED (backoff)
                              │
                    backoff elapsed
                              ▼
                          CONNECTING  (loop)
```

Familiar patterns mapped in:

- **Backoff on reconnect** = exponential backoff with a cap, same as any
  retrying HTTP/gRPC client (`§10.1`, though the design ultimately
  recommends a simple fixed interval instead of a full exponential ladder —
  `§16` decision 14 — because the constrained resource here is a shared
  pool of only 6 TCP connection slots, not just "don't hammer the peer").
- **Re-subscribe after reconnect** = exactly like re-establishing
  subscriptions after a Kafka consumer group rebalance or a WebSocket
  reconnect — the broker throws away your subscription state on disconnect
  (`clean session`, `§9.2`), so the client must redo it, every time.
- **Liveness check** (`mqtt_client_is_connected()`) polled every cycle,
  rather than trusting a "connected" flag that nothing clears on failure —
  this is the same bug class as trusting a cached `isHealthy` boolean instead
  of an active health check.
- **Enqueue-then-flush for publish** (`§6.2`): calling code (`wifi_task`)
  never calls the network directly — it calls
  `lwip_utk_mqtt_publish()`, which just enqueues; the radio-owner thread
  is the only one that actually calls `mqtt_publish()`. This is the
  **outbox pattern** you'd use to keep a hot path from blocking on I/O:
  write the intent to a local queue, let a dedicated worker drain it.

---

## 6. Publish policy — periodic + event-driven (§7)

This is a **metrics + logging hybrid**, and if you've built observability
pipelines you've made this exact call before:

| | Periodic state line | Edge event |
|---|---|---|
| SE equivalent | A `/health` or metrics scrape, polled on an interval | A structured log line / audit event, emitted the instant something happens |
| Answers | "what's the current state" | "what just happened" |
| Loss tolerance | fine — the next scrape fixes it | not fine — it's a gap in the audit trail |
| Doubles as | the **heartbeat** — silence for N seconds means "down", same as a missed liveness probe | — |

Both go on **one MQTT topic** (one Kafka-partition-style ordered stream)
rather than two, specifically so a single sequence number totally orders
"state" and "event" records together — the same reason you'd avoid splitting
one logical event stream across two Kafka topics if a consumer needs a
single causal order without a merge step.

---

## 7. Wire format — JSON outbound, so a stranger's client can read it (§8)

```
{"t":"S","v":1,"s":137,"up":41230,"mot":2,"dst":20,"lin":1,"dev":3,"bar":69,"act":4,"hum":3,"hgt":12,"obs":0,"byp":0,"mis":1,"mpv":3,"drp":0}
{"t":"E","v":1,"s":138,"up":41310,"k":"B","a":69,"b":4}
```

The first draft picked a `key=value` `logfmt` line (the Heroku/Grafana Loki
format), reasoning that JSON's self-description roughly doubles the payload
and that "the only consumer is your own subscriber script." **That premise
broke on demo day: the consumer is the instructor's MQTT client, which you
do not control.** A `logfmt` line is free only if you also ship the parser; a
generic client shows it as an opaque string. JSON is the format off-the-shelf
tooling (MQTT Explorer, `mosquitto_sub`, Node-RED, `paho` + `json.loads`,
`jq`) already renders — and producing it costs the device nothing, because it
is one `tm_sprintf` with a different format string. No FPU is needed: every
value is an integer, so there is no float formatting, and the only strings are
single letters, so there is nothing to escape.

The design's real insight is an **asymmetry between the two directions** —
the reason this is not simply "switch to JSON everywhere":

| Direction | Who writes / who reads | JSON cost |
|---|---|---|
| Device → broker (`telemetry`, `heartbeat`, `state`) | you write, **a stranger may read** | ≈ zero — a format string |
| Broker → device (`cmd`, `map`) | **you write, you read** (your own laptop publisher) | real — a JSON *parser* on the device |

JSON is cheap to **produce** and expensive to **consume**. So the rule is
"produce JSON, and be deliberate about what you parse." The inbound side stays
`key=value` with `strtok`, because there you own both ends — that is decision
17, still an open call if you would rather have one format end to end.

`v` is a schema version field — the API-versioning field you would put in any
wire protocol so an old subscriber fails loudly on a breaking change instead
of silently misreading a shifted field. `s` (monotonic sequence number) and
`drp` (drop counter) are the same idea as a Kafka offset plus a "messages
dropped" counter — they make **gaps observable** instead of silently missing,
which matters a lot here because (per `§5`, `§14`) failures in this stack are
otherwise completely silent — no exception, no stack trace, just a hang. Treat
every counter in this design as instrumentation you would add specifically
*because* you cannot attach a debugger to a robot mid-demo.

---

## 8. Command channel and the "config push" problem (§9)

The subscribe/inbound side turns the client into something you'd recognize
as a **feature-flag or remote-config client**:

- The laptop publishes a new barcode→action table to a topic with the
  **retain** flag set. MQTT's retain is functionally a **"last known
  config" cache the broker holds for you** — any client that (re)subscribes,
  even after a reboot, immediately gets the current value without the
  publisher re-sending anything. If you've used a config service that hands
  a client its current config on connect (Consul, etcd watch-with-initial-value,
  LaunchDarkly's streaming init), this is the same primitive.
- The client reports back which config **version** it's actually running
  (`"mpv"` in the wire format). The publisher can then verify convergence by
  comparing what it sent to what's echoed back — the same reconciliation
  loop a config-management system (Kubernetes controllers, in essence) uses:
  don't assume delivery, observe the reported state and re-publish if it
  doesn't match.
- Applying the same config twice is a **no-op** (idempotent apply) — exactly
  the property you want for anything that might replay, because MQTT here
  gives you *at-least-once within a session*, not *exactly-once*.

The corrected finding worth flagging: the design's first draft assumed
QoS 1 ("at least once") would make commands durable across a disconnect. It
doesn't — lwIP forces a "clean session" on every reconnect, which is MQTT's
equivalent of a session-scoped cache that's flushed on every reconnect, so
QoS 1 only helps *while already connected*. That's precisely why retained
config-push, not "resend the command reliably," is the mechanism that
actually survives a reboot — a good example of not trusting a delivery
guarantee's marketing name without reading what it actually promises.

---

## 9. What's already decided vs. still open (§16)

Already locked in this design revision:
1. One shared inbound queue, single consumer (`wifi_task`) — not one queue
   per producer.
2. Generalize the existing MQTT smoke-test module in place, rather than
   writing a second client (avoids two TCP connections competing for an
   already-tight connection pool).
3. Hybrid publish model (periodic + edge), one topic.
4. JSON on the outbound stream, because the reader may be the instructor's
   client and not your own script (`§7` above, decided 2026-09-28).

Open decisions are all framed as **yes/no calls with a stated default**, not
open design questions — e.g. "never block on send and count drops (default)
vs. block up to 10ms," "fixed-interval reconnect (default) vs. full
exponential backoff ladder." If you want to influence this design further,
`§16`'s table is the fastest way in — it's already pre-chewed into ship/no-ship
choices.

---

## 10. What this document is not

This is a reading aid, not a replacement. For line-number citations, the
exact struct layouts, the resource budget (`§13`), and the M7–M10 hardware
test plan, go to `WIFI_SUBSYSTEM_DESIGN.md` directly — every claim there is
backed by a file:line reference into this repo or the SDK, and that's the
version to cite in review or in the M10 schema doc.
