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

## Usage — try VS Code first

1. Open the repo folder in VS Code.
2. Command palette → **Dev Containers: Reopen in Container** — do this even if
   you already have a Docker terminal open separately. Running `docker compose
   run` in a plain terminal next to a normal (non-containerized) VS Code
   window does **not** give you IntelliSense against the container's
   toolchain — VS Code has to be running *inside* the container itself for
   the C/C++ extension to find `arm-none-eabi-gcc` at all.

> [!WARNING]
> **Known issue on some Windows machines:** "Reopen in Container" can fail
> with `accessing specified distro mount service: stat
> /run/guest-services/distro-services/<distro>.sock: no such file or
> directory`. This is a Docker Desktop ↔ WSL2 integration bug, not anything
> wrong with this repo's setup — the image still builds successfully, it's
> only the extra step of attaching VS Code itself to the container that
> fails. It survives a full PC restart, so it's a persistent Docker Desktop
> setting, not a one-off glitch. First thing to check: Docker Desktop →
> Settings → Resources → WSL Integration — confirm your distro is listed
> and toggled on. If "Reopen in Container" doesn't work after that, **skip
> straight to "Usage — command line" below** — it gets you the identical
> pinned build environment without needing VS Code attached to the
> container at all. You lose in-editor IntelliSense until this is resolved,
> nothing else.

3. First build takes a few minutes (base image, toolchain, Pico SDK); cached after that.
4. For working IntelliSense, build *through* `bear` and write the database to
   the repo root — which is where both `.vscode/settings.json` and
   `devcontainer.json` look for it:
   ```sh
   cd build_make
   make clean
   bear --output ../compile_commands.json -- make -j8
   ```
   `make clean` matters — if most `.o` files are already up to date, plain
   `make` skips recompiling them and `bear` only captures whatever actually
   ran, giving you a mostly-empty `compile_commands.json`.

   The paths recorded inside it are container paths (`/workspace/...`,
   `/opt/arm-toolchain/...`), so it only means anything to a VS Code window
   that is itself running inside the container. That's the deliberate
   trade-off of this setup rather than a bug — open the folder normally and
   the C/C++ extension falls back to `cl.exe` and gets nothing useful.
5. If IntelliSense still hasn't picked it up: Command palette →
   **C/C++: Reset IntelliSense Database**, then **Developer: Reload Window**.
   VS Code caches its fallback results and won't always notice a new file on its own.

## Usage — command line (fallback, or if you just prefer it)

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
mode exactly as you would without Docker. This gets you the exact same
pinned toolchain as the VS Code path; the only thing you lose is in-editor
IntelliSense, and even that's recoverable with the `.vscode/settings.json`
fallback below if `bear` has been run at least once.

## AI coding agents — where they run

The image deliberately ships **no AI tooling**. Everyone uses whichever agent
they prefer (Claude Code, Codex, Qwen Code, Cursor, Copilot, …) on their own
host, so this repo never imposes one person's choice on anyone else.

Two things follow from that inside a Dev Container:

- The container and your host see the **same files** — `docker-compose.yml`
  bind-mounts the repo at `/workspace` — so an agent editing on the host is
  editing exactly what the container compiles. Nothing to sync, no drift.
- But the agent **cannot run `arm-none-eabi-gcc` or `make` directly**, because
  that toolchain exists only inside the container. Route every build through
  Docker instead:

  ```sh
  # VS Code is attached to the container ("Reopen in Container"):
  docker compose exec build bash -lc "cd build_make && make -j8"

  # no VS Code attached:
  docker compose run --rm build bash -lc "cd build_make && make -j8"
  ```

  Add whatever profile flags the build needs (`SMP=1`, `CONSOLE=usb_cdc`,
  `WIFI=cyw43`, …) after `make`.

Tell your agent this up front, or record it in whichever context file that
agent reads (`QWEN.md`, `CLAUDE.md`, `AGENTS.md`, …). Left to its own devices
it will run `make` on the host, find no toolchain, and set about fixing a
problem that doesn't exist.

## Troubleshooting quick reference

| Symptom | Cause | Fix |
|---|---|---|
| "Reopen in Container" prompts a setup wizard instead of just working | `devcontainer.json` isn't inside `.devcontainer/` | `mkdir .devcontainer && mv devcontainer.json .devcontainer/` |
| `accessing specified distro mount service: ... no such file or directory` | Docker Desktop ↔ WSL2 integration bug, survives a reboot | Check WSL Integration is on for your distro in Docker Desktop settings; if still broken, use the command-line fallback |
| IntelliSense log shows `Using "cl.exe" instead` | VS Code isn't actually running inside the container | Reopen in Container, don't just run a Docker terminal alongside a normal window |
| `compile_commands.json` exists but squiggles persist | Wrong location, or IntelliSense cache is stale | Copy/regenerate at the repo root, then Reset IntelliSense Database + reload |
| `compile_commands.json` only has a handful of entries | Build was already up to date when `bear` ran | `make clean` first, then `bear --output ../compile_commands.json -- make -j8` |
| Your AI agent runs `make` on the host and reports `arm-none-eabi-gcc: not found` | Agent isn't routed through the container — the toolchain isn't installed on your host | Have it build via `docker compose exec build bash -lc "cd build_make && make -j8"` |

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

## Open items

- **Confirm this is an actual git clone, not a downloaded ZIP.** A folder
  named `mtk3smp-rp2040-main` (the `-main` suffix) is what GitHub names a ZIP
  download, not `git clone` — check for a `.git` folder with `git status`.
  None of the fork/branch/PR workflow below can start until this is a real
  repository.
- `PICO_SDK_TAG` is pinned, but the kernel repo itself isn't yet — once you
  fork `mtk3smp-rp2040` into your own org (see the repo-setup discussion),
  pin the Dockerfile's `git clone` step to that fork and a specific commit
  too, not just the upstream Pico SDK.
- TCP/MQTT status on this kernel is still unconfirmed as of the README —
  DHCP/DNS/UDP echo are the only networking paths explicitly validated.
  Check `docs/PORT_RP2040.md` before building the telemetry plan on the
  assumption TCP already works.
