# GLOSSARY — the shared vocabulary for the WiFi + Telemetry subsystem

One copy of every term used across the documents in `docs/comms/`, so each
document no longer repeats it. Look a term up here.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Scope:** the vocabulary shared by the whole `docs/comms/` set.
- **Companions:** every document in this folder links back here instead of
  carrying its own glossary.
- **Updated from Tribunal 1 (2026-10-19):** §4 revised for the binary wire
  format, and §4.1 added for the minimum-bit terms.

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
| **Sentinel** | A reserved value meaning "no value yet" — `0` in `hgt`, `obs`, `byp`; `31` in the barcode letter index. *(Updated from Tribunal 1.)* |
| **ASCII code** | The integer for a character: `'A'`=65, `'E'`=69, `'Z'`=90. Since Tribunal 1 the wire carries a **letter index** instead (0 = `A`); the reader adds 65. |
| **Letter index** | The barcode letter as 0–25 (`A`–`Z`) in 5 bits, with 31 = none. Replaced the 7-bit ASCII code at Tribunal 1. |
| **Action code** | The number a barcode letter maps to (the table in [`MQTT_SCHEMA.md`](MQTT_SCHEMA.md) §8.3). **2 bits (0–3) since Tribunal 1**, owned by Buddy 3. What each code *means* is not yet pinned — §12 item 7. |
| **`H` / `D` record** | Heartbeat record (on `…/heartbeat`, every 1 s and in reply to `ping`) and Diagnostic snapshot (on `…/state`, in reply to `snap`). |
| **`mv`** | The map version the laptop sends with a new barcode table. The car compares it with `mpv`. |
| **`mpv`** | The letter→action table version the car is currently running (reported so the laptop can confirm the remap took). |
| **`drp`** | Count of telemetry records dropped because the **intake** queue (`tele_mbf`) was full. 4 bits, saturates at 15 (Tribunal 1). Drops at the outbound queue while disconnected are not counted; they show only on the console (`MQTT_SCHEMA.md` §3). |
| **Schema v1 / v2** | The value of `v` on the wire: v1 = JSON (2026-09-28), **v2 = packed binary (2026-10-19, Tribunal 1)**. |
| **Decoder** | The laptop script that turns a binary record back into named fields. Required since Tribunal 1, because a generic MQTT client shows binary as hex. |
| **`snap`** | Laptop command: "send me the current state now" → reply on `…/state`. |
| **Statechart** | A state-machine diagram: boxes are states, arrows are transitions. |

## 4.1 Minimum-bit data (added from Tribunal 1, 2026-10-19)

| Term | Plain meaning |
|---|---|
| **Change Requirement 1** | The Tribunal 1 requirement: store and send every field in the minimum bits for its range, put faults in one shared bitmask, use binary MQTT payloads, and keep shared packed data race-free. |
| **Packed record** | Several fields squeezed into one block of bits with no gaps, e.g. the 15-byte `S` record. |
| **Bit-field** | C syntax (`unsigned x:3;`) that packs a struct field into bits. Its order and padding are up to the compiler, so it is **not** used for anything sent off the chip. |
| **Mask and shift** | Packing by hand: `(word >> pos) & ((1u << bits) - 1)`. Same result on every compiler, so the car and the laptop agree. |
| **LSB-first / little-endian** | The bit order of every record: the first field starts at the lowest bit of the first byte. |
| **Offset encoding** | Storing a signed value as unsigned by adding a fixed offset (`dev + 128`), so no field is signed on the wire. |
| **Clamp / `ovf`** | Limiting an out-of-range value to the field's maximum instead of letting it wrap, and setting an overflow bit to say so. |
| **Shared fault bitmask** | One word where each fault (motor stall, encoder fault, IMU fault, no echo, line sensor, Wi-Fi loss) is one bit, set and cleared only by the task that detects it. Lives in a kernel event flag owned by Buddy 5; Buddy 1 sets the Wi-Fi-loss bit (bit 5), which is report-only. Bit names: `FAULT_*` in `messages.h`. |
| **Read-modify-write (RMW)** | Read a word, change some bits, write it back. Two cores doing this at once lose updates — measured at 50% in the Tribunal race test. |
| **Data-representation table** | The Week 13 deliverable listing each field's range, bit count and how it is stored. Generated by `Tribunal/packed_size.py`. |

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