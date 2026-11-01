# AGENTS.md — working in Pico-Racer-Embedded

Firmware for a small autonomous car: a **µT-Kernel 3.0 RTOS port to a
Raspberry Pi Pico W (RP2040)** running on both cores (SMP). The build is a
hand-written **GNU make** system inherited from upstream µT-Kernel — there is
**no CMake anywhere**, despite the Pico SDK being a real dependency.

**Start at `docs/INTEGRATION.md`**: the baseline for the integrated car (what is
built, measured, deviated from the design, and still open). Where another doc
disagrees with it, it is current. The rest of `docs/` is subsystem detail and
history: `ARCHITECTURE.md` (repo map, plain-language overview, glossary),
`GETTING_STARTED.md`, `PORT_RP2040.md`, `BUDDY3_LINE_BARCODE.md`, `motion/`,
`imu/`, `obstacle/`, and **Buddy 1's Wi-Fi + Telemetry docs in `docs/comms/`**
(start at `docs/comms/README.md`, the progress hub).

## 1. Never build on the host — always build through Docker

`arm-none-eabi-gcc` is **not installed on developer machines**. It exists only
inside the Docker build image defined by `Dockerfile`. Every build goes
through Docker:

```sh
# On a fresh clone nothing is running yet — use this form. Compose builds the
# image automatically on first use; that first build takes several minutes
# (Ubuntu + the ARM toolchain download + a Pico SDK clone), so wait it out
# rather than assuming it has hung.
docker compose run --rm build bash -lc "cd build_make && make -j8"

# Only if a container is ALREADY up (e.g. VS Code attached via Dev Containers):
docker compose exec build bash -lc "cd build_make && make -j8"
# If that reports `service "build" is not running`, use the run form above.
```

If you see `arm-none-eabi-gcc: not found` or `g++: not found`, you have broken
this rule. It is **not** a missing dependency, and **not** a reason to install
a toolchain or rewrite build files. The toolchain is in the container by
design. The repo and your host share the same files (the repo is bind-mounted
at `/workspace`), so an edit here is exactly what the container compiles.

## 2. Build profiles

The switches chain: each level requires the one below it, enforced by explicit
`$(error ...)` checks in `build_make/pico_rp2040.mk`. After `WIFI_DNS` it
branches: MQTT (the car) or the echo tests (bring-up only):

```
SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_UDP=1 WIFI_TCP=1 WIFI_TCPBULK=1
```

| Command | Result | Pico SDK needed? |
|---|---|---|
| `make -j8` | single-core, UART console | no |
| `make SMP=1 -j8` | dual-core SMP | no |
| `make CONSOLE=usb_cdc -j8` | USB-CDC console | yes |
| `make SMP=1 WIFI=cyw43 -j8` | radio bring-up | yes |
| `... WIFI_JOIN=1` | associate to an AP — needs `config/wifi_credentials.h` | yes |
| `... WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1` | get an IP address, resolve names | yes |
| `... WIFI_UDP=1` | UDP — needs `config/udp_test_config.h` | yes |
| `... WIFI_TCP=1` | TCP — needs `config/tcp_test_config.h` | yes |
| `... WIFI_DNS=1 WIFI_MQTT=1` | **the car build**: MQTT telemetry, no echo tests — needs `config/broker_config.h` | yes |
| `... WIFI_MQTT=1 WIFI_NETLOG=1` | **debug only, never demo day**: the car build plus its whole console over UDP to the broker's host (port 31884), shown by the viewer as `serial`; image name ends `_netlog` | yes |

`WIFI_STATIC=1` is an alternative to `WIFI_DHCP=1` (mutually exclusive; both
require `WIFI_NETIF=1`).

Output is `mtk3pico_smp$(SMP)_$(CONSOLE)$(WIFI_SUFFIX).uf2` in `build_make/`
(`pico_rp2040.mk:194`). The active profile is stamped to
`build_make/.build_profile`; objects built for a different profile are
discarded automatically, so switching profiles does not need a manual clean.

## 3. Where sources live — and the directory you must never edit

Edit at the **repo root**: `app_program/`, `kernel/`, `lib/`, `device/`,
`include/`, `etc/`, `config/`, `docs/`.

**`build_make/mtkernel_3/` is generated output.** It mirrors the root tree but
contains compiled `.o`/`.d` files and aggregated rules; the only tracked files
inside it are `subdir.mk` glue. Read
`build_make/mtkernel_3/app_program/subdir.mk` and the rule is literally:

```make
mtkernel_3/app_program/%.o: ../app_program/%.c
```

So the real source is `../app_program/app_main.c` — i.e. the repo root.
Editing anything under `build_make/mtkernel_3/` is a silent no-op.

App sources are discovered by a recursive `find` over `app_program/`, so
**every `.c` there joins the build**. An empty or non-compiling stub breaks
the whole build even if nothing calls it.

## 4. Config files and secrets

- Pattern: copy `config/<name>.example.h` → `config/<name>.h`, edit it, and
  flip the matching `*_SET` flag from `0` to `1`. An unedited copy fails
  loudly at compile time.
- Populated `config/*.h` files are **gitignored**. Never commit Wi-Fi
  credentials, broker addresses or other secrets.
- Network targets must be config values, not hardcoded constants — the broker
  used on demo day is supplied externally.

## 5. IntelliSense database

`compile_commands.json` at the repo root is generated, gitignored, and records
**container paths** (`/workspace/...`, `/opt/arm-toolchain/...`), so it only
means anything to a VS Code window that is itself running inside the
container. Regenerate it whenever build flags change:

```sh
docker compose run --rm build bash -lc "cd build_make && make clean && bear --output ../compile_commands.json -- make -j8"
```

`make clean` matters here — `bear` records only compilations that actually ran,
so an up-to-date tree yields a near-empty database.

## 6. Not implemented — do not assume it exists

- **No HTTP or CoAP layer.** MQTT is implemented and verified on hardware
  (gated by `WIFI_MQTT`): a persistent client with reconnect (back-off 2/4/5 s),
  `comms_task` publishing the packed `S`/`H`/`E`/`D` records, inbound `cmd` and
  `map`, and the `tele_mbf` intake from the other tasks. The
  `docs/GETTING_STARTED.md` §7 and `docs/PORT_RP2040.md:119` notes predate MQTT.
- **Every subsystem is built and integrated** (`docs/INTEGRATION.md` §2-§5):
  motion, line + barcode, IMU, scan, nav. **Not built** (`INTEGRATION.md` §6, §8):
  an end-of-course trigger (D10: Finished only by an MQTT command), battery
  sense on GP28 (M4), the RGB LED (M5), and an `E imu` record (M6). **Junction
  turns are coded but never run on the car**: the GP7 barcode/junction sensor is
  faulty.
- The Wi-Fi driver and the MQTT client live in `lib/`, not the app:
  `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` and
  `lib/libnet/lwip/lwip_utk_mqtt.c`. App code publishes only through
  `cyw43_utk_mqtt_send()`, which queues for the radio task.
- **No demo tasks.** The producer/consumer/monitor demo and `led_test` were
  removed from `app_main.c` (2026-10-21). `blink_task` is built only with
  `TM_BLINK_LED=1`, and `wifi_status_task` (the `[wifi]`/`[mqtt]` console) only
  with `TM_WIFI_REPORT=1`, which is the default but is turned off for demo day.

## 7. Gotchas that cause real bugs

- **lwIP is single-threaded**: `NO_SYS 1` in
  `lib/libnet/lwip/include/lwipopts.h:14`. Never call lwIP from an arbitrary
  task — hand work to the processor-1 polling service through a kernel object
  (e.g. `tk_snd_mbf`).
- **`LWIP_ALTCP` follows the MQTT gate** — `#define LWIP_ALTCP TM_WIFI_MQTT`
  (`lib/libnet/lwip/include/lwipopts.h:30`): 1 in MQTT builds, 0 otherwise. Don't
  make it unconditional — the comment there notes it swaps the TCP dispatch path
  underneath the raw-TCP phases.
- **`tm_printf` is atomic per call, not per line.** Compose the message with
  `tm_sprintf`, then emit it with exactly one `tm_printf`, or concurrent tasks
  will interleave mid-line.
- **Pin every task.** The SMP kernel has one ready queue shared by both cores
  (`kernel/tkernel/ready_queue.h:51`), so an unpinned task can run on core 1 and
  interfere with the time-critical tasks there. Create tasks with `TA_ASSPRC`;
  Buddy 1's tasks use `TP_PRC1` (core 0).
- **Kernel and lwIP headers can't share a `.c` file**: `tk/syslib.h` and lwIP's
  `<stddef.h>` disagree on `size_t`. Keep kernel code (spinlocks, `tm_printf`)
  and lwIP code in separate files joined by a plain `<stdint.h>` header, as
  `cyw43_utk.c` / `cyw43_utk_kernel.c` and `tm_com.c` / `lwip_utk_netlog.c` do.

## 8. Do not commit generated files

`*.o`, `*.d`, `*.elf`, `*.uf2`, `*.map`, `compile_commands.json` and
`build_make/.build_profile` are all gitignored. Leave them that way.