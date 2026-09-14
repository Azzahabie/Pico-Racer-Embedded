# Docker dev setup for mtk3smp-rp2040

Reproducible build environment for the kernel repo, so Windows/WSL2 and Mac
teammates build against the exact same toolchain instead of whatever apt or
Homebrew happen to have installed that week.

## What's here

| File | Purpose |
|---|---|
| `Dockerfile` | Pinned build environment: ARM toolchain, Pico SDK, `bear` |
| `docker-compose.yml` | Convenience wrapper — bind-mounts the repo into the container |
| `.devcontainer/devcontainer.json` | Wires the same compose service into VS Code's Dev Containers extension |
| `.vscode/settings.json` | Fallback IntelliSense config that applies even if VS Code isn't opened via Dev Containers |

Drop all four at the root of your `mtk3smp-rp2040` clone, alongside `build_make/`.

> [!WARNING]
> `devcontainer.json` **must** be inside a folder named exactly `.devcontainer/`
> (or be a dot-file `.devcontainer.json` at the root) — nothing else. A loose
> `devcontainer.json` sitting directly at the repo root is invisible to VS
> Code: it won't error, it'll just silently offer to generate a brand new
> config from scratch instead of using yours. This is a real mistake we hit
> and lost time to — check the folder, not just the filename, if "Reopen in
> Container" ever prompts a setup wizard instead of just working.

## Why bother

This kernel port is maintained by one person, with thin community docs, so a
build that silently differs between machines is a real risk here — worse than
on a project with wide community testing. Pinning the toolchain once removes
that risk entirely for the whole team.

## Prerequisites

- Docker Desktop running (Windows/Mac), or Docker Engine (Linux)
- For VS Code: the Dev Containers extension (`ms-vscode-remote.remote-containers`)

## Usage — command line

```sh
docker compose build
docker compose run --rm build bash

cd build_make
make -j8                       # single-core, UART console — no Pico SDK needed
make SMP=1 -j8                  # dual-core SMP
make CONSOLE=usb_cdc -j8        # USB-CDC console (Pico SDK already baked in)
make SMP=1 WIFI=cyw43 -j8       # radio bring-up profile (development)
```

The resulting `.uf2` lands in `build_make/` on your host filesystem (it's a
bind mount, not container-only storage) — drag it onto the Pico in BOOTSEL
mode exactly as you would without Docker.

## Usage — VS Code

1. Open the repo folder in VS Code.
2. Command palette → **Dev Containers: Reopen in Container** — do this even if
   you already have a Docker terminal open separately. Running `docker compose
   run` in a plain terminal next to a normal (non-containerized) VS Code
   window does **not** give you IntelliSense against the container's
   toolchain — VS Code has to be running *inside* the container itself for
   the C/C++ extension to find `arm-none-eabi-gcc` at all. If IntelliSense
   logs ever show `Using "cl.exe" instead` or a Windows-style backslash path,
   that's the tell: VS Code isn't actually in the container.
3. First build takes a few minutes (base image, toolchain, Pico SDK); cached after that.
4. For working IntelliSense:
   ```sh
   cd build_make
   make clean
   bear -- make -j8
   ```
   `make clean` matters — if most `.o` files are already up to date, plain
   `make` skips recompiling them and `bear` only captures whatever actually
   ran, giving you a mostly-empty `compile_commands.json`.
5. `bear` writes `compile_commands.json` into your current directory
   (`build_make/`), not the repo root. Either copy it up:
   ```sh
   cp compile_commands.json ../compile_commands.json
   ```
   or generate it there directly: `bear --output ../compile_commands.json -- make -j8`.
6. If IntelliSense still hasn't picked it up: Command palette →
   **C/C++: Reset IntelliSense Database**, then **Developer: Reload Window**.
   VS Code caches its fallback results and won't always notice a new file on its own.

## Troubleshooting quick reference

| Symptom | Cause | Fix |
|---|---|---|
| "Reopen in Container" prompts a setup wizard instead of just working | `devcontainer.json` isn't inside `.devcontainer/` | `mkdir .devcontainer && mv devcontainer.json .devcontainer/` |
| IntelliSense log shows `Using "cl.exe" instead` | VS Code isn't actually running inside the container | Reopen in Container, don't just run a Docker terminal alongside a normal window |
| `compile_commands.json` exists but squiggles persist | Wrong location, or IntelliSense cache is stale | Copy/regenerate at the repo root, then Reset IntelliSense Database + reload |
| `compile_commands.json` only has a handful of entries | Build was already up to date when `bear` ran | `make clean` first, then `bear -- make -j8` |

## What stays outside the container

Docker Desktop doesn't reliably pass raw USB devices through to containers on
Windows or Mac, so these run on the host, not inside:

- **Flashing** — drag-and-drop the `.uf2` via Windows Explorer or Finder
- **Serial console** — PuTTY / Tera Term / `screen`, connecting to the host's
  COM port or `/dev/tty.usbserial*` / `/dev/tty.usbmodem*`
- **Future GDB/OpenOCD debugging** over a debug probe (e.g. Picoprobe), if you
  add that later — same USB passthrough limitation applies

## Pinned versions

| Component | Version | Source |
|---|---|---|
| `arm-none-eabi-gcc` | 13.2.rel1 | Kernel repo README baseline |
| Pico SDK | `2.2.0` | `GETTING_STARTED.md` § 1b |
| Pico SDK submodules | `tinyusb`, `cyw43-driver`, `lwip` only | `GETTING_STARTED.md` § 1b — `mbedtls`/`btstack` skipped |

## Keep out of git

`compile_commands.json` is machine-generated and bakes in container-specific
absolute paths (`/opt/arm-toolchain/...`) — it should never be committed.
Confirm `.gitignore` covers it, plus the usual build output:
`build_make/mtkernel_3/**/*.o`, `*.d`, `.build_profile`, `*.elf`, `*.map`, `*.uf2`.