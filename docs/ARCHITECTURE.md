# Architecture & Design Foundations

Companion documents:

- [`GETTING_STARTED.md`](GETTING_STARTED.md) — build and flash, step by step
- [`PORT_RP2040.md`](PORT_RP2040.md) — what is validated vs. still unproven

> **Status note.** This file describes the design as it exists in the tree
> today(26 Sep 2026). Where the shipped docs disagree with the code, the code is treated as
> the truth, and the disagreement is called out explicitly.

---

## 1. The one-paragraph version

This repo is the firmware for **"Pico Racer"**, a school racing-car project.
The car's brain is a Raspberry Pi **Pico W** — a small microcontroller board
(the chunk of hardware; compare it to a very small computer with no operating
system pre-installed, pins instead of a screen, and kilobytes instead of
gigabytes of RAM).

Rather than write everything as one big loop, the project runs a real
**RTOS kernel** called **µT-Kernel 3.0**. An RTOS is just an operating-system
kernel specialised for small hardware: it gives you *tasks* (the equivalent of
threads), a *scheduler* (decides which task runs when), and the usual
concurrency primitives (*mutexes, semaphores, message buffers*) — the same
concepts you would meet in any server-side threading library.

The kernel was **ported** — adapted to run on this specific chip — and the
port is special because it drives **both of the chip's two CPU cores at once**.
That "use both cores" feature is called **SMP** (Symmetric Multiprocessing),
and it is the genuinely custom part of this repo.

On top of the kernel sits the racing-car application, which is still mostly
scaffolding (see [§8](#8-current-implementation-state)).

---

## 2. The three layers

Think of it as three stacked layers, like an application running on an OS
running on bare metal:

```
┌─────────────────────────────────────────────────────────────┐
│  Application        app_program/                            │
│  "what the car does"  producer/consumer demo, LED test,     │
│                       + stubs for motor / line / imu / ...  │
├─────────────────────────────────────────────────────────────┤
│  Kernel (RTOS)      kernel/, lib/libtk, lib/libtm           │
│  "threads + sched"    µT-Kernel 3.0: tasks, mutexes,        │
│                       semaphores, message buffers, SMP      │
├─────────────────────────────────────────────────────────────┤
│  Hardware           RP2040 (Pico W)                         │
│  "the metal"          two Cortex-M0+ cores, 264 KB RAM     │
└─────────────────────────────────────────────────────────────┘
```

Sitting beside the kernel are support libraries, not part of the stack above:

| Directory | Plain meaning |
|---|---|
| `lib/libbsp/` | **B**oard **S**upport **P**ackage — the pin/register names for *this* board, so the kernel is not hard-coded to one chip |
| `lib/libtk/`, `lib/libtm/` | Kernel and monitor libraries that µT-Kernel ships with |
| `lib/libnet/` | The networking layer (lwIP — a TCP/IP stack written for small devices) |
| `lib/libwifi/` | Glue between the WiFi chip's driver and this kernel |

A **BSP** is the same idea as a platform/abstraction layer in application
software: it is the one place that knows the physical details.

---

## 3. Repository map

| Path | What lives there |
|---|---|
| `app_program/` | The application. One folder per team member's subsystem task |
| `kernel/` | µT-Kernel 3.0 kernel source (upstream code) |
| `include/`, `config/` | Public headers and build-time configuration |
| `lib/` | Support libraries (see table above) |
| `build_make/` | The GNU make build system + target-specific rules |
| `device/`, `etc/` | Device drivers and misc support |
| `tools/` | Host-side helper tools (see `elf2uf2` below) |
| `docs/` | Documentation |
| `Dockerfile`, `docker-compose.yml`, `.devcontainer/` | Reproducible build environment |

---

## 4. The kernel: what is upstream and what is custom

This distinction matters, because "custom kernel" is only half true.

- **Upstream:** the kernel core is **µT-Kernel 3.0**, an existing open-source
  RTOS from the TRON project. This repo did **not** write it from scratch —
  `kernel/` is that kernel's source.
- **Derived:** the port is a fork of the community port
  `sirfonzie/mtk3smp-rp2040` (this working copy is
  `Azzahabie/Pico-Racer-Embedded`).
- **Genuinely custom:** the **dual-core SMP work on RP2040**. Per
  [`PORT_RP2040.md`](PORT_RP2040.md), this is described as *"the only SMP port
  in the family — the ESP32-C3 and ESP32-C6 ports are single-core."* The SMP
  scheduler, the global lock (a **big kernel lock** — one lock that serialises
  entry into the kernel across both cores), cross-core interrupts, and task
  affinity are the distinctive engineering.

### Plain-language glosses for the SMP vocabulary

| Term in the docs | Plain meaning |
|---|---|
| SMP | Running work on both CPU cores, sharing one memory space |
| Global ready queue | One list of "tasks that want to run", shared by both cores, instead of one list per core |
| Big kernel lock | A single lock taken on entering the kernel, so only one core is inside kernel code at a time. Simple and safe; the cost is that kernel calls from two cores can queue up |
| IPI | Inter-Processor Interrupt — one core poking the other to say "you have work" |
| Task affinity (`TA_ASSPRC`) | Pinning a task to a specific core |

That last one is used by the demo: the producer task is pinned to core 1 and
the consumer to core 2, so every message crosses a core boundary on purpose
(`app_program/app_main.c`, and [`GETTING_STARTED.md` §6b](GETTING_STARTED.md)).

---

## 5. The build system

### Plain GNU make, not CMake

This is the single most surprising thing about the repo if you come from
application development. The Raspberry Pi ecosystem normally builds with
**CMake**; this project does **not**. There is no `CMakeLists.txt` anywhere.

It uses the **plain GNU make** system inherited from upstream µT-Kernel —
the same one used for its other boards (RX, STM32). In software-engineering
terms: it is a hand-written Makefile build, not a modern framework build.

Consequence: the RP2040 bootloader wants a `.uf2` file, and the SDK normally
produces that invisibly via its CMake helpers. Here the build compiles its
**own** converter tool from `tools/elf2uf2/main.cpp` with your PC's host C++
compiler (`g++`) — that program runs on your machine, never on the Pico.

### Build profiles

The build is configured by passing **switches on the make command line**. The
image name encodes which switches were used, so you can always tell an artifact
apart at a glance.

| Switch | Default | Meaning |
|---|---|---|
| `SMP` | `0` | `0` = one core, `1` = both cores |
| `CONSOLE` | `uart` | `uart` = wired serial console; `usb_cdc` = console over the Pico's own USB port |
| `WIFI` | `none` | `cyw43` = enable the WiFi chip. **Requires `SMP=1`** |
| `WIFI_JOIN` … `WIFI_TCPBULK` | `0` | A ladder of cumulative networking features (join network → get an address → DNS → UDP → TCP → bulk TCP) |

Defined in `build_make/pico_rp2040.mk`. Each rung of the networking ladder
needs the one below it (`WIFI_TCP` requires `WIFI_UDP`, `WIFI_UDP` requires
`WIFI_DNS`, and so on).

**Artifact naming** (`pico_rp2040.mk:194`):

```
mtk3pico_smp<SMP>_<CONSOLE>[<wifi suffix>].uf2
```

So `make SMP=1 CONSOLE=usb_cdc` yields `mtk3pico_smp1_usb_cdc.uf2`, and a radio
build adds a suffix like `_wifi_udp`.

**Profile stamping:** the build writes the active profile to
`build_make/.build_profile` and automatically discards object files built for a
different profile. You do not need to `make clean` when switching profiles.

> **Why the default is single-core (UART, no WiFi):** it is the conservative
> configuration and the one with the strongest hardware test record. The
> qualified boundary is documented as **SMP + UART**, plus an optional
> **SMP + USB-CDC** console profile; everything else — notably the radio — is a
> *development* profile ([`PORT_RP2040.md`](PORT_RP2040.md)).

---

## 6. Dependencies: exactly when the Pico SDK is needed

The **Pico SDK** is Raspberry Pi's official library for these chips. This is a
common point of confusion, so here is the precise rule.

The SDK is a **required dependency for some profiles and completely absent for
others**:

| Profile | Pico SDK needed? | What it supplies |
|---|---|---|
| Single-core or SMP, `CONSOLE=uart` (**default**) | **No** | Nothing — only `arm-none-eabi-gcc` and a host `g++` |
| `CONSOLE=usb_cdc` | **Yes** | `lib/tinyusb` (the USB stack) + SDK headers |
| `WIFI=cyw43` | **Yes** | `lib/cyw43-driver` (WiFi chip driver), `lib/lwip` (TCP/IP stack), SDK hardware headers |

Enforced in `build_make/pico_rp2040.mk` (§ the `cyw43` block, ~lines 350–365):
the build **hard-errors** with a named message if the SDK is missing:

```
CYW43 driver not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules
lwIP not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules
```

### It is a dependency, not "the base"

Two distinctions worth keeping straight, because they affect troubleshooting:

1. **The project does not use the SDK's build system.** Normal Pico projects
   build with the SDK's CMake. This one ignores that entirely and compiles a
   hand-picked subset of SDK `.c` files with its own make rules.
2. **The project does not link the SDK as a runtime library.** Only selected
   source files and headers are pulled in.

The software-engineering analogy: it is as if you **vendored a third-party
library's source** into your repo and compiled it with your own build, rather
than depending on that library's own build tooling. The *foundation* of the
build is µT-Kernel's make system plus the ARM compiler; the SDK is a **required
dependency** whenever USB or WiFi is in play.

### Version pinning

The SDK is pinned to **2.2.0** (`Dockerfile`, `build_make/pico_rp2040.mk`
defaults). Only three of its five submodules are fetched — `mbedtls` and
`btstack` are skipped to save a large download, because nothing here uses them.

> **Practical consequence:** because the WiFi code is written against this
> specific SDK's driver source, bumping the SDK version is not a
> "change one number" operation. Treat it as a code change.

---

## 7. The build environment (Docker)

`Dockerfile` + `docker-compose.yml` + `.devcontainer/` exist for one reason:
**so every teammate builds byte-identical firmware.** The kernel port is
maintained by one person with thin community docs, so a build that silently
differs between a Windows machine and a Mac is a real risk. Pinning the
toolchain once removes it.

What is pinned:

| Thing | Version |
|---|---|
| `arm-none-eabi-gcc` (cross-compiler) | `13.2.rel1` |
| Pico SDK | `2.2.0` |
| SDK submodules | `tinyusb`, `cyw43-driver`, `lwip` only |

What stays **outside** the container, because USB passthrough to containers is
unreliable on Windows and macOS:

- **Flashing** — drag the `.uf2` onto the `RPI-RP2` drive that appears in
  BOOTSEL mode
- **Serial console** — a terminal program on the host (PuTTY / Tera Term /
  `screen`)
- **Debugging** over a probe, if added later

Two gotchas are documented in [`README.md`](../README.md) and worth repeating:
`devcontainer.json` must be inside a folder named exactly `.devcontainer/`, and
`compile_commands.json` (generated by the `bear` tool for editor IntelliSense)
must never be committed.

---

## 8. Current implementation state

This is the honest picture. The infrastructure is mature; the car is not yet
real.

### Working

- **The kernel port** — rigorously tested, per [`PORT_RP2040.md`](PORT_RP2040.md)
  (SMP 58/58, single-core regression 11/11, measured latencies recorded).
- **The demo application** (`app_program/app_main.c`) — a small
  producer/consumer pipeline that exercises every IPC primitive the kernel
  offers. `GETTING_STARTED.md` calls it *"meant to be read and modified, not
  just run."*
- **A GPIO loopback test** (`app_program/led_test/`) — mirrors an input pin
  onto an output pin, validating that the folder-per-task build structure works.
- **The networking paths** — UDP and TCP clients on the radio profile, TCP
  explicitly marked as *not* fully unvalidated in the README's own open items.

### Not yet written — the racing-car subsystems

The five subsystem tasks are **empty stubs**. Each contains only includes and a
`TODO` comment:

| File | Contents today |
|---|---|
| `app_program/motor_task/motor_task.c` | `/* TODO: motor_task implementation. */` |
| `app_program/line_task/line_task.c` | same |
| `app_program/obstacle_task/obstacle_task.c` | same |
| `app_program/imu_task/imu_task.c` | same |
| `app_program/wifi_task/wifi_task.c` | same |

The shared contract they will communicate through,
`app_program/common/messages.h`, is also empty — its own comment states the
intent: *"Shared message structs, mailbox/queue IDs, and pin assignments. Every
buddy's task depends on this file — route changes through a pull request so
nobody's build breaks silently."*

Each subsystem has its own **git branch** (`motor_task`, `line_task`,
`obstacle_task`, `imu_task`, `wifi_task`) — a deliberate one-branch-per-person
layout.

> **Two things to know before building on this.**
>
> 1. **The stubs are still compiled.** `build_make/mtkernel_3/app_program/subdir.mk`
>    discovers application sources with a recursive `find`, so every `.c` file
>    under `app_program/` — stubs included — joins the build. They are harmless
>    empty translation units today, but they are not inert: a stub that fails to
>    compile will break the build even though nothing calls it.
> 2. **Real WiFi code lives in `lib/`, not `app_program/wifi_task/`.**
>    The working radio driver is `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`
>    plus `lib/libnet/lwip/*`. The app-level WiFi reporter is
>    `wifi_status_task` inside `app_main.c` (compiled only when
>    `TM_WIFI_CYW43` is set). The `app_program/wifi_task/` folder is a separate,
>    still-empty team scaffold.

### Documentation drift found

`GETTING_STARTED.md` §6b documents the running application as *four tasks plus
a fifth in radio builds*. The code actually starts **five** unconditionally —
the extra one is the `led_test` task, added at priority 9 in `app_main.c` but
absent from that table. Worth reconciling if the docs are graded.

---

## 9. Verified claims

Two claims about this repo were checked against the source and corrected:

| Claim | Verdict |
|---|---|
| "A custom µT-Kernel Pico kernel" | **Half true.** The kernel is upstream **µT-Kernel 3.0**; the *custom* part is the **dual-core SMP port** to RP2040. |
| "A kernel to run an RTOS" | **Backwards.** It *is* the RTOS. The application runs on top of it. |
| "Uses the Pico 2.2 SDK as the base" | **Misleading in one direction, true in another.** The SDK is **not** the base and is **not needed at all** for the default UART build — but it **is a required dependency** for the WiFi and USB profiles this project actually uses. Pinned at SDK **2.2.0**. Also note: the board is a **Pico W (RP2040)**, not a "Pico 2" (which is a different chip, RP2350). |

---

## 10. Glossary

| Term | Plain meaning |
|---|---|
| **RP2040** | The microcontroller chip on the Pico / Pico W. Two CPU cores (Cortex-M0+). |
| **Pico W** | The specific board. Same chip as a Pico, plus a WiFi chip. |
| **Firmware** | Software that runs directly on the hardware, with no OS beneath it. |
| **RTOS** | Real-Time Operating System — a small kernel giving tasks, scheduling and synchronisation. |
| **Kernel** | The core of an OS: scheduling, synchronisation, memory. |
| **Task** | The RTOS equivalent of a thread. |
| **SMP** | Running tasks on both CPU cores, sharing memory. |
| **BSP** | Board Support Package — the layer that names this board's pins and registers. |
| **Cross-compiler** | A compiler that runs on your PC but produces code for *different* hardware (the Pico). |
| **SDK** | Software Development Kit — here, Raspberry Pi's official library for the chip. |
| **Toolchain** | The set of tools (compiler, linker, assembler) used to build firmware. |
| **UF2** | The file format the Pico's bootloader accepts. You copy it onto the board like a USB stick; the board reboots into it. |
| **BOOTSEL** | The button you hold while plugging in, to make the board accept a new `.uf2`. |
| **Bare metal** | Running with no OS at all. The kernel *is* what is running; there is nothing beneath it. |
| **lwIP / TinyUSB** | Third-party libraries for networking / USB respectively. |