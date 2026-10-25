# NETWORKING — the layers below the application

Everything about the **protocol / transport / platform layers** of Buddy 1's
subsystem: the Wi-Fi ladder, the config files, the MQTT implementation, the
constraints, and the captured evidence.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Last updated:** 2026-09-26
- **Scope:** everything *below* `app_program/`. The application layer has its own
  companion document: [`APPLICATION.md`](APPLICATION.md). The general repo map is
  in [`ARCHITECTURE.md`](../ARCHITECTURE.md).
- **Written for a reader with no embedded background** — technical terms are
  glossed in §10, and every design choice has a plain-language note.

---

## 1. Where this sits — the three layers

```
┌─────────────────────────────────────────────────────────────┐
│  Application        app_program/                            │
│  "what the car does"  comms_task (MQTT), usermain,          │
│                       + stubs for motor / line / imu / ...  │
├─────────────────────────────────────────────────────────────┤
│  Kernel (RTOS)      kernel/, lib/libtk, lib/libtm           │
│  "threads + scheduler"                                        │
├─────────────────────────────────────────────────────────────┤
│  Hardware           RP2040 (Pico W)                         │
└─────────────────────────────────────────────────────────────┘
   beside the kernel: lib/libnet (lwIP), lib/libwifi (radio glue), lib/libbsp
```

**This document covers the `lib/` side.** The working networking code — the radio
driver and the lwIP protocol clients — lives in `lib/`, **not** in
`app_program/comms_task/` (which is an empty scaffold for the *application* task).
Two reasons:

1. **MQTT is a protocol, not application logic.** It belongs beside the existing
   UDP and TCP clients (`lwip_utk_udp.c`, `lwip_utk_tcp.c`).
2. **It must run in one specific task.** lwIP here is built `NO_SYS 1` — no
   threading, not thread-safe. Every networking call must happen from the single
   "radio-owner" task. A file under `app_program/` would be a *different* thread
   and therefore the wrong execution context.

> **What it means:** same split as a web app — the HTTP server and DB driver live
> in the dependency layers; your feature handlers live in the app layer. You do
> not reimplement TCP inside a controller.

---

## 2. The network ladder (build switches)

Networking is opt-in, one step at a time. Each switch **requires** the one below
it, enforced by hard errors in `build_make/pico_rp2040.mk`:

```
SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1
      ├─ WIFI_MQTT=1                                  (the car build)
      └─ WIFI_UDP=1 WIFI_TCP=1 [WIFI_TCPBULK=1]       (echo tests, bring-up only)
```

| Switch | What it turns on | Needs a config file? |
|---|---|---|
| `WIFI=cyw43` | the radio (bring-up) | no |
| `WIFI_JOIN=1` | associate to an AP | **`config/wifi_credentials.h`** |
| `WIFI_NETIF=1` | connect the radio to lwIP's `netif` | no |
| `WIFI_DHCP=1` | get an IP address | no |
| `WIFI_STATIC=1` | *alternative to DHCP*: fixed address | **`config/network_config.h`** |
| `WIFI_DNS=1` | resolve hostnames | no |
| `WIFI_UDP=1` | UDP echo test | **`config/udp_test_config.h`** |
| `WIFI_TCP=1` | TCP echo test | **`config/tcp_test_config.h`** |
| `WIFI_TCPBULK=1` | bulk/pipelined TCP | *(same as TCP)* |
| `WIFI_MQTT=1` | **MQTT client** (this project's goal) | **`config/broker_config.h`** |

`WIFI_DHCP=1` and `WIFI_STATIC=1` are mutually exclusive. `WIFI_MQTT=1` requires
`WIFI_DNS=1` (and so `WIFI_DHCP`, `WIFI_NETIF`, `WIFI_JOIN`, `WIFI=cyw43`), **not**
the echo tests: `lwipopts.h` turns the TCP stack on for either `WIFI_TCP` or
`WIFI_MQTT` (changed 2026-10-25). The car build is therefore:

```sh
make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 \
     WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 -j8
```

Adding `WIFI_UDP=1 WIFI_TCP=1` to it still builds and brings the echo tests back,
for bring-up only. Without them the image is 3.4 KB smaller in flash and 1.9 KB
smaller in initialised RAM, and sends no echo traffic.

### Build command (always through Docker)

`arm-none-eabi-gcc` exists **only** inside the Docker image; the repo is
bind-mounted at `/workspace` so the container compiles exactly what you edit.

```sh
docker compose run --rm build bash -lc "cd build_make && make <flags> -j8"
```

### Image naming and profiles

Output is `mtk3pico_smp<SMP>_<CONSOLE><wifi-suffix>.uf2` in `build_make/`, e.g.
`mtk3pico_smp1_usb_cdc_wifi_mqtt.uf2`. The active flag set is stamped into
`build_make/.build_profile`; **objects built for a different profile are
discarded automatically**, so switching profiles does not need `make clean` (it
does force a recompile).

> **What it means:** the image name encodes the build's feature flags so you can
> never confuse two drag-and-drop files, and the profile stamp stops the build
> system from silently reusing object files compiled with different flags.

---

## 3. Local config files (gitignored — never commit)

Every phase reads a local header, copied from a `.example.h` template:

| File | Holds |
|---|---|
| `config/wifi_credentials.h` | SSID / password / auth mode |
| `config/network_config.h` | static IP (only for `WIFI_STATIC=1`) |
| `config/udp_test_config.h` | UDP echo peer address + port (7007) |
| `config/tcp_test_config.h` | TCP echo peer address + port (7008) |
| `config/broker_config.h` | **MQTT broker** address, port, client id, keep-alive |

Two rules, both enforced by the repo:

1. **A `*_SET` flag must go from `0` to `1`.** An unedited copy *fails the build
   loudly*, instead of compiling and then failing mysteriously at runtime.
2. **The populated copy is gitignored.** These files hold credentials and
   machine-specific addresses. `git check-ignore` confirms each is excluded.

> **What it means:** this is a deliberate "fail fast" design — the value to flip
> is a flag rather than a line to delete, so a careless edit cannot leave the
> guard half-removed. On demo day the broker is the instructor's, so the target
> must stay configuration and never a compiled-in constant.

---

## 4. Environment (as of 2026-09-26)

### Hardware and console

| | |
|---|---|
| Board | Raspberry Pi **Pico W** (RP2040), radio CYW43439 — **2.4 GHz only** |
| Console | **USB-CDC** (`CONSOLE=usb_cdc`), mirrors UART0; open the COM port at 115200 |
| Liveness LED | **external LED on GP16** (`BOARD_LED_PIN`); the Pico W's onboard LED is on the radio chip, not an RP2040 pin |

### Network

| | |
|---|---|
| Router | **`Pico2_IoT`**, WPA/WPA2-Personal + AES |
| LAN | `192.168.50.0/24`, gateway `192.168.50.1` |
| Laptop (echo server / broker) | `192.168.50.8` |
| Pico | DHCP lease in `192.168.50.x` (observed `.7`) |

**Networks that failed — do not use:**

| Network | Symptom |
|---|---|
| Home Wi-Fi (`WPA2-PSK[AES]+WPA3-Personal`) | never associated — suspected WPA3/PMF rejection |
| Phone hotspot (`Test1`) | associated fine, then its DHCP server stopped issuing leases (`dhcp=0 addr=0.0.0.0`) |

> **Demo-day risk:** the robot must join the instructor's AP. Confirm it is
> 2.4 GHz WPA2 **without required WPA3/PMF**, or the robot cannot connect and
> MQTT cannot run.

### Laptop-side tooling

| Tool | Detail |
|---|---|
| Broker | **`amqtt` 0.12.1**, installed into the global Python 3.13; start it with `py tools\run_broker.py` |
| Client / listener | **`paho-mqtt` 2.1.0**; `python tools\telemetry_decode.py` decodes the packed records |
| Echo server (UDP/TCP tests) | `python tools\linux_echo.py` (pure stdlib) |

⚠ **Run the broker via `python.exe`, never `amqtt.exe`.** The Windows firewall
inbound allow-rule is bound to `python.exe` (TCP + UDP, any port, Private +
Public) and this Wi-Fi is classified `Public`. `amqtt.exe` is a different
executable and would be **silently blocked over the LAN**. `python -m amqtt` is
unsupported (no `__main__`), so a small launcher shim is used:
`tools/run_broker.py` (moved there from an untracked scratch folder on
2026-10-21; the M6 log in §8 shows its old path).

---

## 5. Milestones M0–M6 — all verified on hardware

| # | Milestone | Evidence |
|---|---|---|
| M0 | Confirm the board is a Pico W | radio (CYW43439) present |
| M1 | Toolchain / flash / console | UART demo banner + `[monitor] seq=…` |
| M2 | Wi-Fi association | `[wifi] link UP  rssi=-37  mac=d8:3a:dd:ae:4b:eb` |
| M3 | Network (DHCP + DNS) | `[wifi] ip=192.168.50.7  gw=192.168.50.1  dns=8.8.8.8` |
| M4 | UDP echo | `sent=64 recv=64 matched=64/64` |
| M5 | TCP echo | host: `tcp: 1 sessions, 3072 bytes` |
| M6 | **MQTT connect + publish** | `[mqtt] 192.168.50.8:1883 … result=0 published=1` + `MQTT RECEIVED: …` |

**M2–M5 changed no source code at all** — the Wi-Fi stack already existed; those
milestones were "run the ladder and verify it". Configuration and build output
are gitignored, so `git status` was legitimately empty.

### 5.1 Recovery — the car reconnects on its own (Week 6 feedback F1)

The radio task replaces the one-shot join and the DHCP latch with a state
machine (`SCANNING → JOINING → WAIT_LEASE → ONLINE`, any failure → `BACKOFF`
3 s → rejoin; join timeout 15 s, lease timeout 20 s), in
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`. Each failure is logged
with its reason, and each rejoin also resets lwIP's DHCP back-off, which would
otherwise grow to 60 s between attempts.

Tested on hardware, 19–25 Oct 2026 (F1.5; the raw logs are in Buddy 1's
private notes repo). Times come from the Pico's own `t=` stamps: `[wifi]`
console lines can print up to 3 s late.

| Test | What was done | Result |
|---|---|---|
| T1 AP off at boot | router off, Pico powered, router on later | ✅ 18 `join-failed(NONET)` retries, one every ~4 s; joined with no reset. Router on → ONLINE 24.5–25 s, of which the router's own boot was ~11–15 s; first beacon → ONLINE 10–13 s |
| T2 AP drop while online | router or hotspot switched off, then on | ✅ 5 / 5. Link loss noticed after ≈ 8 s (hotspot) / ≈ 14–15 s (router); rejoined with no reset; first beacon → ONLINE 10–13 s |
| T3 DHCP refused | router's DHCP server off, then back on | ✅ `lease-timeout` every 20.0 s, rejoin every 26.2 s, no crash. DHCP back on → ONLINE in the lease attempt already running (≈ 2 s after the hand-noted switch-on); MQTT connected 0.2 s later |
| T4 wrong password | image built with a bad password | ✅ 15 / 15 joins `join-failed(BADAUTH)`, one every 10.2 s (rejected ≈ 7.1 s in, then 3 s back-off), no drift, no crash |
| T5 soak | left online, MQTT build | ✅ 24 min 49 s (planned 30): no link loss, one MQTT session for the whole run (broker log), no gap in `s` |
| T6 hotspot power cycles | 8 back-to-back boots on a phone hotspot | ✅ every boot reached `ip=` (before the fix: 6 / 11). One `join-failed(FAIL)` per boot: the phone rejects the stale session first |

**Throughout every test, the rest of the system kept running:** `comms_task`'s
`H`/`S` line kept its 1 s period with no step missed (T3–T5; 1501 heartbeats
exactly 1000 ms apart in T5), with the radio retrying on the same core.

**What it means for NFR-09 (recover within 15 s):**
- Met when measured from the AP being available again: first beacon → ONLINE
  is 10–13 s (join ≈ 3 s, lease 7–9 s).
- Not met end to end in every case: noticing a vanished AP takes 8–15 s, a
  router reboot adds its own boot time, and if DHCP returns with < ~8 s left in
  a lease attempt the car waits for the next rejoin (worst case ≈ 34 s,
  derived from the timings, not measured).
- Levers, deferred to the optimisation pass: disable DHCP address-conflict
  detection (≈ 5–6 s off every lease), shorten the lease timeout, and the
  radio's link-loss timeout (not yet checked for the CYW43439).

**Telemetry while offline:** every record produced before the first connect or
during an outage is dropped at the radio's 112 B outbound queue (about 3 s of
`H` + `S`), and the ~6 records that fit are delivered late on reconnect. These
drops show on the console (`[mqtt] … drop=N`), not in `S.drp`
(`MQTT_SCHEMA.md` §3, §12 item 20).

---

## 6. M6 — the MQTT implementation (protocol layer)

### 6.1 The governing constraint

lwIP is configured `NO_SYS 1`: **one execution context, no locking**. So the MQTT
module is written as a **non-blocking state machine** with exactly one entry
point, called only from the radio-owner task's polling loop:

```c
void lwip_utk_mqtt_poll(struct netif *netif);
```

> **What it means:** this is the same shape as a promise chain — you start a
> request and a callback delivers the result later. The function is re-entered
> every poll and asks "what state are we in?"; timestamps (from a millisecond
> clock) bound how long each step may take.

### 6.2 What existed vs. what M6 added

- **Existed:** scan, join, DHCP, DNS, UDP, and a raw-TCP client.
- **Added:** the MQTT protocol layer — connect, publish, and on-device status
  reporting. It rides on TCP; MQTT is to TCP roughly what WebSocket is to TCP.

### 6.3 Changes, file by file

#### Group A — build plumbing (committed as `94a7962`)

```
 .gitignore                         |  6 +++--
 build_make/pico_rp2040.mk          | 29 +++++++++++++++++++++-
 config/broker_config.example.h     | 49 ++++++++++++++++++++++++++++++++++++++
 lib/libnet/lwip/include/lwipopts.h | 17 +++++++++----
```

**`lib/libnet/lwip/include/lwipopts.h`** (lwIP's configuration)

| Change | Detail |
|---|---|
| `LWIP_ALTCP` | was a hard `0` → now `TM_WIFI_MQTT` (so `1` only in MQTT builds) |
| `MEMP_NUM_SYS_TIMEOUT` | was `8` → now `12` when MQTT is on, `8` otherwise |

> **What it means:** `LWIP_ALTCP` enables the abstraction layer lwIP's MQTT client
> is written against; without it, the client cannot compile. Gating it on the MQTT
> switch keeps the **already-proven TCP build completely unchanged**.
> `MEMP_NUM_SYS_TIMEOUT` is lwIP's fixed pool of timer slots — DHCP/DNS/ARP/TCP
> already consume nearly all 8, and MQTT needs one more for keep-alive. Running
> out trips an assertion that **halts the board silently**.

**`build_make/pico_rp2040.mk`**

| Change | Detail |
|---|---|
| New switch | `WIFI_MQTT ?= 0`; rejects anything but 0/1 |
| Dependencies | `WIFI_MQTT=1` requires `WIFI_TCP=1` **and** `config/broker_config.h` |
| Image name | `WIFI_SUFFIX := _wifi_mqtt` |
| Profile stamp | `PROFILE_ID` gains `-mqtt$(WIFI_MQTT)` |
| Sources | `apps/mqtt/mqtt.c` added to `LWIP_SRCS` when on |
| Defines | `-DTM_WIFI_MQTT=$(WIFI_MQTT)` |

**`config/broker_config.example.h`** (new) — `UTK_MQTT_BROKER_ADDRESS`,
`UTK_MQTT_BROKER_PORT` (1883), `UTK_MQTT_CLIENT_ID`, `UTK_MQTT_KEEPALIVE_S`,
guarded by `UTK_MQTT_CONFIG_SET`.

**`.gitignore`** — added `config/broker_config.h`. *(A genuine near-miss: the new
config file first appeared as untracked, so it could have been committed by
accident.)*

#### Group B — the implementation (currently uncommitted)

```
 M app_program/app_main.c                        +21
 M build_make/pico_rp2040.mk                     +3
 M lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c +58
 M lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h +31
?? lib/libnet/lwip/include/lwip_utk_mqtt.h       (new, 46 lines)
?? lib/libnet/lwip/lwip_utk_mqtt.c               (new, 224 lines)
```

**`lib/libnet/lwip/include/lwip_utk_mqtt.h`** (new) — the module's interface and
a `T_LWIP_UTK_MQTT_STATUS` struct describing *everything that happened*: which
step, which error code, how long it took, how many attempts, the broker, topic
and payload.

> **What it means:** it is the module's DTO — enough to answer "why did it fail?"
> without a debugger. `struct netif` is only forward-declared, so the header stays
> free of lwIP dependencies.

**`lib/libnet/lwip/lwip_utk_mqtt.c`** (new, 224 lines) — the state machine.

| Element | Value |
|---|---|
| Connect timeout | 15 s (no CONNACK → give up) |
| Publish timeout | 10 s |
| Session timeout | 45 s |
| Topic | `robocar/test/mqtt` (deliberately a *test* topic) |
| Payload | `hello-from-pico` |
| QoS | `0` (fire-and-forget) |

Flow: guard on having an address → allocate client + request connect
(non-blocking) → connect callback reports acceptance → publish once → publish
callback (fires when the bytes are on the wire) → mark complete.

**`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h`** — 22 MQTT fields added to the
shared status snapshot, plus `CYW43_MQTT_TOPIC_MAX` / `CYW43_MQTT_PAYLOAD_MAX`
(= 48, mirrored from the module header).

**`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`** — includes the module, adds
`update_mqtt_status()` (copies the module status into the shared snapshot), and
one call in the network task's loop, gated on `dhcp_complete`.

> **What it means:** the snapshot is how *other* tasks can see the result (the
> network task publishes it; any task may read). The mirrored size constants are
> checked by the compiler:
> ```c
> typedef char cyw43_mqtt_size_matches[
>     (LWIP_UTK_MQTT_TOPIC_MAX == CYW43_MQTT_TOPIC_MAX
>      && LWIP_UTK_MQTT_PAYLOAD_MAX == CYW43_MQTT_PAYLOAD_MAX) ? 1 : -1];
> ```
> If the two ever disagree, the build fails — instead of a message being silently
> truncated at runtime.

**`app_program/app_main.c`** — a one-shot `[mqtt] …` line on the completion edge,
printing broker, topic, payload, result code and a published flag.

> **What it means:** this closes a real gap. Join failures, TCP, and MQTT were
> previously **invisible on the device** — you could only tell from the laptop.
> Printed edge-triggered (once, not every poll) and built with a single
> `tm_printf` (that call is atomic, but a line assembled from several calls can be
> interleaved by another task).

**`build_make/pico_rp2040.mk`** (+3 lines, with the module) — adds
`lwip_utk_mqtt.o` to `LWIP_PORT_OBJS`, so the module is only linked when
`WIFI_MQTT=1`.

### 6.4 How it works, end to end

```
boot
 └─ radio task starts, scans, joins Wi-Fi
     └─ DHCP completes → IP address assigned
         ├─ MQTT poll sees an address → connect to broker 192.168.50.8:1883
         │   └─ broker replies (CONNACK) → mark "connected"
         │       └─ next poll publishes "hello-from-pico" on robocar/test/mqtt
         │           └─ TCP sends the bytes → callback marks "published"
         │               └─ next poll marks complete → console prints [mqtt] …
         └─ (the UDP/TCP echo tests also run — see §11)
```

### 6.5 Verification evidence

| Source | Line |
|---|---|
| Pico console | `[mqtt] 192.168.50.8:1883  topic=robocar/test/mqtt  payload=hello-from-pico  result=0  published=1` |
| Broker | `Connection from 192.168.50.7:62150 on listener 'default'` |
| Subscriber | `MQTT RECEIVED: robocar/test/mqtt -> b'hello-from-pico'` |

`result=0` = success (lwIP error code 0). The broker shows a connection from the
Pico's IP; the subscriber shows the exact payload arriving.

**Regression evidence:** after the plumbing change, the TCP-only profile rebuilt
**identically** (`text 340505 / data 4640 / bss 52188`), proving the `LWIP_ALTCP`
gating left the earlier milestone untouched.

---

## 7. Hard constraints and gotchas

| Constraint | Consequence if ignored |
|---|---|
| **lwIP is `NO_SYS 1`** (`lib/libnet/lwip/include/lwipopts.h`) | Calling networking from any task other than the radio-owner task causes intermittent corruption/hangs, not clean errors |
| **Failure modes are silent** | `lwip_utk_assert` halts the board with no message (`cpsid i` + `wfi`); join failures print nothing; TCP prints nothing |
| **`tm_printf` is atomic per *call*, not per line** | Compose with `tm_sprintf`, then emit with exactly one `tm_printf` |
| **Config files are gitignored** | Never commit populated `config/*.h` — credentials and addresses |
| **The ladder is chained** | Each switch requires the one below; the build hard-errors otherwise |
| **`MEMP_NUM_SYS_TIMEOUT`** | Too small → silent assert when lwIP's timer slots run out |
| **Keep network work off the motion critical path** | The run must complete autonomously in under 5 minutes |
| **Every `.c` under `app_program/` joins the build** | A non-compiling stub breaks the whole build, even if nothing calls it |

---

## 8. Evidence appendix — captured logs (2026-09-26)

```
PS C:\Users\azzah\Downloads\Pico-Racer-Embedded> py .\.qwen\tmp\run_broker.py
[2026-09-26 19:18:14,799] :: INFO - Loading plugins from config
[2026-09-26 19:18:14,864] :: INFO - Listener 'default' bind to 0.0.0.0:1883 (max_connections=0)
[2026-09-26 19:18:14,864] :: INFO - Starting session expiration monitor.
[2026-09-26 19:18:16,800] :: INFO - Listener 'default': 1/∞ connections acquired
[2026-09-26 19:18:16,800] :: INFO - Connection from 127.0.0.1:52628 on listener 'default'
[2026-09-26 19:18:16,801] :: INFO - ### 'on_broker_client_connected' EVENT FIRED ###
[2026-09-26 19:23:20,908] :: INFO - Listener 'default': 2/∞ connections acquired
[2026-09-26 19:23:20,908] :: INFO - Connection from 192.168.50.7:62150 on listener 'default'
[2026-09-26 19:23:20,909] :: INFO - ### 'on_broker_client_connected' EVENT FIRED ###

PS C:\Users\azzah\Downloads\Pico-Racer-Embedded> python .qwen\tmp\mqtt_subscribe.py
connected (rc=Success), subscribing to robocar/test/mqtt
MQTT RECEIVED: robocar/test/mqtt -> b'hello-from-pico'

Pico console (USB-CDC):
19:23:08:568 -> [wifi] link UP  rssi=-44  mac=d8:3a:dd:ae:4b:eb
19:23:13:188 -> [udp] waiting: link=1 netif_up=1 link_up=1 dhcp=0 addr=0.0.0.0
19:23:18:428 -> [udp] waiting: link=1 netif_up=1 link_up=1 dhcp=0 addr=0.0.0.0
19:23:20:748 -> [wifi] ip=192.168.50.7  gw=192.168.50.1  dns=8.8.8.8
19:23:20:748 -> [udp] session 1: echoing to 192.168.50.8:7007, 64 packets of 48 bytes
19:23:21:168 -> [mqtt] 192.168.50.8:1883  topic=robocar/test/mqtt  payload=hello-from-pico  result=0  published=1
19:23:21:368 -> [udp] 192.168.50.8:7007  sent=3 recv=0 matched=0/64  corrupt=0 retries=2 errs=0  779 ms
19:23:21:368 -> [udp] session 1 complete: result=-3
19:23:26:408 -> [udp] session 2: echoing to 192.168.50.8:7007, 64 packets of 48 bytes
19:23:27:248 -> [udp] 192.168.50.8:7007  sent=3 recv=0 matched=0/64  corrupt=0 retries=2 errs=0  780 ms
19:23:27:248 -> [udp] session 2 complete: result=-3
```

> The `[udp] … result=-3` lines are **expected, not a fault**: the echo server
> (`tools\linux_echo.py`) was not running during this capture — only the broker
> and subscriber were. `-3` is a timeout. MQTT published successfully *while* UDP
> was failing, so there is no interference between them.

---

## 9. Git state

| Item | State |
|---|---|
| `94a7962` "Add WIFI_MQTT gate and enable altcp when MQTT is on" | committed (build plumbing) |
| Six implementation paths (§6.3 Group B) | **uncommitted** — to review and commit |
| `config/broker_config.h` and the other populated configs | gitignored |

Suggested message for the implementation commit:
*"Add lwIP MQTT publish module and report it on the console"*.

---

## 10. Glossary

> **Moved.** Every term is collected in the shared [`GLOSSARY.md`](GLOSSARY.md) — one copy for the whole `docs/comms/` set.

---

## 11. Open items (protocol side)

1. ~~**Echo tests run alongside MQTT.**~~ *Closed 2026-10-25:* `WIFI_MQTT` now
   requires `WIFI_DNS`, not `WIFI_TCP`, so the car build has no echo tests (§2).
   Before this, the F1.5 soak ran 257 UDP echo sessions, all `result=-3`.
2. **Single-shot publish.** The client stays connected, but only one message is
   sent; a real cadence belongs to the telemetry framework (see `APPLICATION.md`).
3. **No authentication.** The broker allows anonymous clients. If demo day needs
   credentials, `config/broker_config.h` needs username/password fields.
4. **Wi-Fi environment.** The robot must join a 2.4 GHz WPA2 network without
   required WPA3/PMF (§4).