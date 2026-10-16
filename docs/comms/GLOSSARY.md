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
| **RTOS** | Real-Time Operating System — a small OS that schedules tasks with predictable timing. Here, µT-Kernel 3.0. |
| **Mailbox** (`cmd_mbx`) | A kernel object that passes messages between tasks by pointer, in order. `cmd_mbx` carries inbound MQTT commands from `cyw43_utk_task` to `comms_task`. |
| **`tele_mbfid` / `tele_pub_mbfid`** | The two outgoing telemetry message buffers: sensing tasks → `comms_task`, then `comms_task` → `cyw43_utk_task`. |
| **`nav_flg`** | `nav_ctrl_task`'s event flag. Buttons, faults, motion replies and the Wi-Fi connection state (CONN) all set bits on it. |
| **CONN** | The connection-state bit `cyw43_utk_task` reports to `comms_task` and `nav_ctrl_task`. |
| **Cyclic handler** | A kernel timer callback that fires at a fixed period and wakes a periodic task (sets its TICK bit). |
| **Kernel tick** | The RTOS's base timer interval. Nothing can be timed more finely than one tick, so a 1 ms cyclic handler needs a 1 ms tick (the template default is 10 ms). |
| **ISR** | Interrupt Service Routine — code the hardware runs immediately on an event (e.g. an encoder edge), ahead of every task. Kept as short as possible. |
| **Overflow policy** | What a queue does when it is full: drop the new record, drop the oldest, or block the sender. This design drops and counts. |

## 2. Scheduling & affinity

| Term | Plain meaning |
|---|---|
| **WCET** | Worst-Case Execution Time — the longest one run of a task or ISR can take. Needed to prove deadlines hold. |
| **Jitter** | How much a periodic event's timing varies from its nominal period (e.g. an `S` line every 1 s ± 50 ms). |
| **CPU load** | The share of time a core spends running tasks instead of idling, measured with an idle-task counter. |
| **Logic analyser** | A bench tool that records many digital pins over time. Toggling a GPIO at the start/end of an event lets you measure its timing to the microsecond. |
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
| **SUBACK** | The broker's reply confirming (or refusing) a subscription. |
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
| **`H` / `D` record** | Heartbeat record (on `…/heartbeat`, every 1 s and in reply to `ping`) and Diagnostic snapshot (on `…/state`, in reply to `snap`). |
| **`mv`** | The map version the laptop sends with a new barcode table. The car compares it with `mpv`. |
| **`mpv`** | The letter→action table version the car is currently running (reported so the laptop can confirm the remap took). |
| **`drp`** | Count of telemetry records dropped because the queue was full. |
| **`snap`** | Laptop command: "send me the current state now" → reply on `…/state`. |
| **Statechart** | A state-machine diagram: boxes are states, arrows are transitions. |

## 5. Wi-Fi & networking

| Term | Plain meaning |
|---|---|
| **AP** | Access Point — the router or hotspot the car joins. |
| **Association** | The Wi-Fi handshake that joins the car to an AP ("link UP"). It comes before getting an IP address. |
| **DHCP** | Dynamic Host Configuration Protocol — how the car asks the AP for an IP address. |
| **Lease** | The IP address DHCP hands out, valid for a limited time. "No lease" means the car joined Wi-Fi but has no address, so it can't talk to anything. |
| **IP address** | The car's network address (e.g. `192.168.50.7`). |
| **DNS** | Domain Name System — turns a hostname into an IP address. |
| **TCP / UDP** | The two transport protocols: TCP is a reliable connection (MQTT runs on it); UDP is fire-and-forget packets. |
| **2.4 GHz** | The Wi-Fi band the Pico W's CYW43 radio supports. It can't join 5 GHz networks. |
| **WPA2 / WPA3** | Wi-Fi security standards. WPA3 is newer; a "WPA2/WPA3 transition" network failed to join in A1 testing. |
| **PMF** | Protected Management Frames — a security feature WPA3 requires. Suspected cause of the failed join. |
| **RSSI** | Received Signal Strength Indicator, in dBm. Closer to 0 is stronger: −40 is excellent, −80 is weak. |
| **CYW43 / CYW43439** | The Infineon Wi-Fi chip on the Pico W, and the driver library for it. |

## 6. Hardware

| Term | Plain meaning |
|---|---|
| **GPIO** / **GPn** | General-Purpose Input/Output — a programmable pin. `GP28` is pin 28. |
| **ADC** | Analogue-to-Digital Converter — reads a voltage as a number (used for line sensors and battery sense). |
| **VBAT / VBAT/2** | The battery voltage. The board halves it through a divider so the ADC can read it safely. |
| **Jumper** | A solder bridge or link on the board that connects a feature. GP28's battery sense works only if it is closed. |
| **Robo Pico** | The Cytron carrier board the Pico W sits on (motor driver, Grove ports, buttons). |

## 7. Report & project IDs

| Term | Plain meaning |
|---|---|
| **FR-nn** | Functional Requirement — something the car must *do* (e.g. FR-25 reconnect after Wi-Fi loss). |
| **NFR-nn** | Non-Functional Requirement — how *well* it must do it: timing, reliability, accuracy (e.g. NFR-09 telemetry back within 15 s). |
| **A1 … A7** | The report's Section 7 assumptions. A1 (demo network) and A6 (one app task is enough) are Buddy 1's. |
| **M0 … M10** | This subsystem's milestones: M0–M6 radio/transport (done), M7 telemetry, M8 commands, M9 robustness, M10 docs. |
| **Key Risk n** | The numbered "validate early" list at the end of the instructor feedback. |
| **Fault (state)** | The system state where motors are braked until a reset. It is meant for real failures, not for a non-safety channel like telemetry. |

## 8. Build & platform

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