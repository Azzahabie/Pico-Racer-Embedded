# Port fixes: changes to the µT-Kernel RP2040 port

The RTOS port (`kernel/`, `lib/libtk/`, `include/sys/`) is meant to be treated
as a working, validated base (see [`../PORT_RP2040.md`](../PORT_RP2040.md)).
Any change to it is a big deal, however small the diff: every subsystem
depends on it, and its validation history no longer describes the code
exactly.

So each port fix gets its own folder here, with a fixed set of documents:

| File | Holds |
|---|---|
| `README.md` | One-page summary: symptom, cause, fix, status, who is affected |
| `INVESTIGATION.md` | How it was found: hypotheses tried, evidence, what was ruled out |
| `FIX.md` | The exact change, why it is correct, how to verify it, how to revert it |
| `DRILLDOWN.md` | Open questions and experiments still to run before the fix is closed |

## Index

| ID | Title | Area | Status |
|---|---|---|---|
| [PF-001](PF-001-timer-tick/README.md) | Microsecond timer runs ~43× slow after a power-on or reset-button press | `kernel/sysdepend/pico_rp2040/cpu_clock.c` | 🟡 Cause confirmed (D1), fix measured exact (D2), boot paths mapped (D3), published figures assessed (D4), origin traced (D5); D6 pending team replies |
