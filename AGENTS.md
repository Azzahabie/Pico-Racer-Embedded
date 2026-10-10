# AGENTS.md — working in Pico-Racer-Embedded

Firmware for a small autonomous car: a **µT-Kernel 3.0 RTOS port to a
Raspberry Pi Pico W (RP2040)** running on both cores (SMP). The build is a
hand-written **GNU make** system inherited from upstream µT-Kernel — there is
**no CMake anywhere**, despite the Pico SDK being a real dependency.

`docs/` holds the reference set: `ARCHITECTURE.md` (repo map, plain-language
overview, glossary), `GETTING_STARTED.md`, `PORT_RP2040.md`, and the graded team
design doc `embedded .md`. **Buddy 1's WiFi + Telemetry subsystem docs live in
`docs/comms/`** — start at `docs/comms/README.md` (the progress hub); every other
file in that folder is detail behind a link from it.

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
`$(error ...)` checks in `build_make/pico_rp2040.mk`. Full chain:

```
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

- **No HTTP or CoAP layer.** MQTT itself is implemented and verified on hardware
  (M6, gated by `WIFI_MQTT`); what is *not* built is the application layer on top
  of it — the `comms_task` telemetry framework and command channel (M7–M10, see
  `docs/comms/APPLICATION.md`). The `docs/GETTING_STARTED.md:596` and
  `docs/PORT_RP2040.md:119` notes predate M6.
- `app_program/{motor_task,line_task,obstacle_task,imu_task,comms_task}/` are
  empty team stubs, as is `app_program/common/messages.h`.
- The working Wi-Fi driver lives in `lib/`, not the app:
  `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`. `app_program/comms_task/` is
  a separate, still-empty scaffold.

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
- **Docs drift**: `GETTING_STARTED.md` §6b lists four tasks, but
  `app_program/app_main.c` actually starts five — the extra is `led_test`.

## 8. Do not commit generated files

`*.o`, `*.d`, `*.elf`, `*.uf2`, `*.map`, `compile_commands.json` and
`build_make/.build_profile` are all gitignored. Leave them that way.