# GLOSSARY — the shared vocabulary for the WiFi + Telemetry subsystem

One copy of every term used across the documents in `docs/comms/`, so each
document no longer repeats it. Look a term up here.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Scope:** the vocabulary shared by the whole `docs/comms/` set.
- **Companions:** every document in this folder links back here instead of
  carrying its own glossary.

---

## 1. RTOS & kernel objects

| Term | Plain meaning |
|---|---|
| **Task** | The RTOS equivalent of a thread. |
| **SMP** | Two CPU cores running tasks at once, sharing memory. |
| **Mutex** | A lock — one holder at a time. |
| **Event flag** | A set of bits a task waits on: "wake me when this changed". |
| **Radio-owner task** | `cyw43_utk_task` — the single task allowed to call lwIP/MQTT. |
| **`comms_task`** | The application task (renamed from `wifi_task`); prepares telemetry and dispatches inbound commands. Never calls lwIP. |
| **Kernel object** | A thread-safe primitive tasks share instead of a plain global (mailbox, message buffer, mutex, flag, semaphore). |
| **Message buffer** (`tk_snd_mbf` / `tk_rcv_mbf`) | A thread-safe queue of variable-length records between tasks. |
| **Seam** | A point where two layers hand data across — here, always a kernel object. |
| **`TMO_POL`** | Timeout value meaning "do not block; check and return now". |
| **`TMO_FEVR`** | Timeout value meaning "block forever". |
| **Seqlock** | A lock-free way to copy a struct one task writes while another reads: check a version counter before and after, retry if it changed. |
| **Latch** | A flag set once and never cleared — e.g. `dhcp_complete`, which is why the MQTT poll runs forever after DHCP succeeds. |

## 2. Scheduling & affinity

| Term | Plain meaning |
|---|---|
| **Priority (µT-Kernel)** | A task's scheduler rank. **Smaller number = higher priority** (the opposite of Unix `nice`), range 1–32. |
| **`itskpri`** | The initial-priority field of a task-creation structure (`T_CTSK`). |
| **`TP_PRC1` / `TP_PRC2` / affinity** | SMP pins: `TP_PRC1` = core 0, `TP_PRC2` = core 1. Priority only orders tasks on the **same** core. |

## 3. MQTT

| Term | Plain meaning |
|---|---|
| **MQTT** | A lightweight publish/subscribe messaging protocol — like an event bus over the network. |
| **Broker** | The central message server (here, `amqtt` on the laptop). |
| **Topic / payload** | The channel name and the message body. |
| **QoS 0 / QoS 1** | Delivery guarantees: "at most once, fire and forget" / "at least once, acknowledged". |
| **CONNACK** | The broker's reply accepting/rejecting a connection. |
| **Keep-alive** | A periodic "still here" ping so an idle client is not dropped. |
| **Retain** | An MQTT flag: the broker keeps the last message on a topic and hands it to any later subscriber. |
| **Clean session** | A connect flag meaning "forget my previous session"; lwIP always sets it, so subscriptions and undelivered messages are discarded the moment we disconnect. |
| **LWT / last will** | A message the broker publishes on your behalf if your connection dies abruptly. |
| **Idempotent** | Doing it twice has the same effect as once — what makes a re-sent or replayed `map` safe. |
| **Backoff** | Waiting progressively longer between retries so a broken peer is not hammered. |
| **Persistent client** | A connection kept open across the run, rather than connecting per message. |
| **Reassembly** | Rebuilding a message that arrived in pieces, bounded by a fixed buffer. |

## 4. Telemetry records & wire format

| Term | Plain meaning |
|---|---|
| **`S` / `E` line** | The two telemetry record types: periodic State line, and Edge (event) line. |
| **Edge event** | A record published the moment something happens, rather than at the next tick. |
| **Snapshot** | The current state at one instant, as opposed to the history of events. |
| **Sequence number (`s`)** | A per-run counter that makes gaps observable. |
| **Sentinel** | A reserved value meaning "no value yet" — here, `0` in the state fields. |
| **ASCII code** | The integer for a character: `'A'`=65, `'E'`=69, `'Z'`=90. |
| **Action code** | The number a barcode letter maps to (the table in [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §8.3). What a given number *means* is not yet pinned — §12 item 7. |
| **`mpv`** | The letter→action table version the car is currently running (reported so the laptop can confirm the remap took). |
| **`drp`** | Count of telemetry records dropped because the queue was full. |
| **`snap`** | Laptop command: "send me the current state now" → reply on `…/state`. |
| **Statechart** | A state-machine diagram: boxes are states, arrows are transitions. |

## 5. Build & platform

| Term | Plain meaning |
|---|---|
| **lwIP** | A small embeddable TCP/IP library (the Pico's network stack). |
| **`NO_SYS 1`** | lwIP built without OS/threading: not thread-safe — one task only. |
| **netif** | lwIP's network-interface object — the Wi-Fi adapter. |
| **altcp** | lwIP's layer letting application code (MQTT) use TCP without touching raw buffers. |
| **Poll loop** | A task that wakes periodically and does a little work each time. |
| **`sys_timeout`** | lwIP's internal alarm/timer facility. |
| **Error codes** | lwIP: `0` OK, `-1` out of memory, `-3` timeout, `-11` connection refused. |
| **Static assert** | A compiler-checked invariant (e.g. the `typedef char[±1]` trick). |
| **FPU** | Floating-point unit — the hardware for decimal maths. The RP2040 has none, so payloads are integers. |
| **UF2** | The file format the Pico's bootloader accepts on drag-and-drop. |
| **Build profile** | The flags a build was made with, baked into the image name. |
| **Source of truth** | Which document, when two disagree, is the one that must be corrected to. |