# APPLICATION — the layer on top

Buddy 1's **application-layer** work: the tasks that live in `app_program/`, what
they consume, and how they hand work down to the network layer.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Status:** seeded 2026-09-26 — the milestone roadmap below is the plan; the
  code starts at M7.
- **Companion:** [`NETWORKING.md`](NETWORKING.md) covers everything *below* this
  layer (transport, protocol, config, build). General repo map:
  [`ARCHITECTURE.md`](ARCHITECTURE.md).

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
| **`app_program/wifi_task/`** | **Buddy 1** | **empty stub — this is ours** |
| `app_program/common/messages.h` | shared | **empty** — the integration contract |

> **What it means:** the stubs are placeholders deliberately shipped so each
> person has a home for their code. They are *compiled* even while empty (every
> `.c` under `app_program/` joins the build), so an empty file is fine but a
> broken one stops the whole build.

---

## 2. The boundary — how this layer talks to the network

**The rule:** the network layer (lwIP) is `NO_SYS 1` — single-threaded and not
thread-safe. Only the radio-owner task may touch it. So `wifi_task` **cannot**
call MQTT functions directly.

**The pattern:** hand work across with a kernel object, then the radio task
drains it.

```
   wifi_task (our task)                     radio-owner task
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

### M8 — Command channel
- [ ] Subscribe to a command topic and dispatch by message type.
- [ ] **Snapshot:** on request, reply with current state at that instant
      (separate from the regular stream).
- [ ] **Live barcode remap:** the command carries the letter→action mapping and
      updates a runtime table (Buddy 3 reads it). **Must not require
      recompiling** — this is the hard demo-day requirement.

### M9 — Robustness
- [ ] Reconnect / recover on Wi-Fi drop.
- [ ] Heartbeat topic so liveness is visible.
- [ ] Must survive **all 3 demo attempts** — explicitly graded.

### M10 — Documentation
- [ ] Topics, payload formats, broker setup walkthrough. This doubles as the
      integration contract for the other four buddies.

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

---

## 8. Grading reminders

- [ ] **Week 8:** a requirements change will be issued — must actually be
      implemented.
- [ ] **Week 13:** another requirements change — design/discussion only.
- [ ] Demo day: **3 attempts, only the latest counts**.
- [ ] Peer evaluation adjusts the group grade per person — carrying a weak
      teammate can move you up a band.

---

## 9. Open next actions

- [ ] Write the **topic/payload schema** (`docs/MQTT_SCHEMA.md`) — the contract
      blocking all four teammates.
- [ ] Define the kernel-object hand-off between `wifi_task` and the radio task.
- [ ] (Optional, low priority) Reconcile `GETTING_STARTED.md` §6b task table with
      the actual five tasks — `led_test` is missing from the doc.

---

## 10. Don't redo — already true

- Don't clone `sirfonzie/mtk3smp-rp2040` separately — this repo already *is* that
  base kernel plus team scaffolding.
- Don't look for CMake — it is a plain GNU make build.
- Don't rebuild the Wi-Fi layer — scan, join, DHCP, DNS, UDP, TCP and **MQTT** all
  already work (see `NETWORKING.md`).
- Don't expect the Wi-Fi driver in `app_program/wifi_task/` — the real code lives
  in `lib/libwifi/` and `lib/libnet/`; this folder is the *application* task.