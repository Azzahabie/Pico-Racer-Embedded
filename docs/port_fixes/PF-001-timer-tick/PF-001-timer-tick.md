# Build (runs 1-3)

| | |
|---|---|
| Source | Upstream `sirfonzie/mtk3smp-rp2040` `main` @ `2e8e6ad`, plus the PF-001 test code (`pf001-test-instrumentation.diff`, never part of the PR) |
| Profile | Base profile, no Wi-Fi: `make SMP=1 CONSOLE=usb_cdc -j8` → `mtk3pico_smp1_usb_cdc.uf2` |
| Fix switch | `PF001_APPLY_FIX` in `kernel/sysdepend/pico_rp2040/cpu_clock.c`: `0` → `pf001_upstream_nofix.uf2`, `1` → `pf001_upstream_fix.uf2` |
| Toolchain | Docker image from this repo's `Dockerfile`: Arm GNU Toolchain 13.2.rel1, Pico SDK 2.2.0 |
| Built | 2026-10-19 (per the handoff), no warnings |

# nofix (pf001_upstream_nofix.uf2): reflash

## Setup: Serial Monitor on USB, baud rate 115200
## Reflash methodology
1. Remove USB from Pico 
2. Hold BOOTSEL on unpowered Pico
3. Insert USB into Pico
4. Copy pf001_upstream_nofix into folder that pops up

## 1st reflash
[22:16:27.399] [pf001] window 1/6: kernel=10010 ms timer=10002795 us -> 999.280 us/ms   cumulative 999.280 us/ms
[22:16:37.400] [pf001] window 2/6: kernel=10010 ms timer=10009957 us -> 999.995 us/ms   cumulative 999.637 us/ms
[22:16:47.410] [pf001] window 3/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[22:16:57.409] [pf001] window 4/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.818 us/ms
[22:17:07.437] [pf001] window 5/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.855 us/ms
[22:17:17.432] [pf001] window 6/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.879 us/ms
[22:17:17.433] [pf001] done: fix compiled OUT, tick cycles now=12
[22:17:17.435] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:17:19.455] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd reflash
[22:17:43.011] === uT-Kernel 3.0 / RP2040 demo ===
[22:17:43.012] SMP build: 2 processors
[22:17:43.014] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:17:43.015] [pf001] upstream main, fix compiled OUT
[22:17:43.023] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x3e0c (enable=1 cycles=12 running=1)
[22:17:43.024] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:17:53.020] [pf001] window 1/6: kernel=10010 ms timer=10002416 us -> 999.242 us/ms   cumulative 999.242 us/ms
[22:18:03.044] [pf001] window 2/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.622 us/ms
[22:18:13.037] [pf001] window 3/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.747 us/ms
[22:18:23.055] [pf001] window 4/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.811 us/ms
[22:18:33.064] [pf001] window 5/6: kernel=10010 ms timer=10009961 us -> 999.996 us/ms   cumulative 999.848 us/ms
[22:18:43.086] [pf001] window 6/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.874 us/ms
[22:18:43.088] [pf001] done: fix compiled OUT, tick cycles now=12
[22:18:43.089] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:18:45.102] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
### Notes
For some reason the first 3 lines appeared on the console this time, might be due to serial monitor mismatch timing or other factors. 

## 3rd reflash
[22:20:37.478] === uT-Kernel 3.0 / RP2040 demo ===
[22:20:37.479] SMP build: 2 processors
[22:20:37.481] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:20:37.482] [pf001] upstream main, fix compiled OUT
[22:20:37.492] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x560c (enable=1 cycles=12 running=1)
[22:20:37.493] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:20:47.517] [pf001] window 1/6: kernel=10010 ms timer=10002824 us -> 999.283 us/ms   cumulative 999.283 us/ms
[22:20:57.519] [pf001] window 2/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.639 us/ms
[22:21:07.542] [pf001] window 3/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.760 us/ms
[22:21:17.535] [pf001] window 4/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.819 us/ms
[22:21:27.553] [pf001] window 5/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.856 us/ms
[22:21:37.555] [pf001] window 6/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.879 us/ms
[22:21:37.557] [pf001] done: fix compiled OUT, tick cycles now=12
[22:21:37.580] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:21:39.579] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 4th reflash
[22:22:30.710] [pf001] window 1/6: kernel=10010 ms timer=10002838 us -> 999.284 us/ms   cumulative 999.284 us/ms
[22:22:40.735] [pf001] window 2/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.644 us/ms
[22:22:50.723] [pf001] window 3/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.761 us/ms
[22:23:00.749] [pf001] window 4/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.822 us/ms
[22:23:10.746] [pf001] window 5/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.856 us/ms
[22:23:20.770] [pf001] window 6/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[22:23:20.771] [pf001] done: fix compiled OUT, tick cycles now=12
[22:23:20.773] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:23:22.785] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[22:23:24.787] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc

## 5th reflash
[22:23:52.688] [pf001] window 1/6: kernel=10010 ms timer=10002785 us -> 999.279 us/ms   cumulative 999.279 us/ms
[22:24:02.693] [pf001] window 2/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.637 us/ms
[22:24:12.708] [pf001] window 3/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[22:24:22.712] [pf001] window 4/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.818 us/ms
[22:24:32.736] [pf001] window 5/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.855 us/ms
[22:24:42.730] [pf001] window 6/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.879 us/ms
[22:24:42.732] [pf001] done: fix compiled OUT, tick cycles now=12
[22:24:42.754] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:24:44.758] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[22:24:46.764] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc


# nofix (pf001_upstream_nofix.uf2): power-cycle
## Power-cycle methodology
1. unplug USB A that plugs into PC to power down pico
2. Replug USB into PC to power up pico
3. Approx 1-2 secs between plugging and unplugging USB

Attempts after first powercycle it would be sequential there is no reflash then powercycle other than the first attempt

## 1st power-cycle (after reflash)
[22:27:14.891] [pf001] window 1/6: kernel=10010 ms timer=234441 us -> 23.420 us/ms   cumulative 23.420 us/ms
[22:27:24.888] [pf001] window 2/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.429 us/ms
[22:27:34.904] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[22:27:44.902] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:27:54.918] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:28:04.932] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:28:04.934] [pf001] done: fix compiled OUT, tick cycles now=0
[22:28:04.935] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

## 2nd power-cycle
[22:29:41.774] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[22:29:51.768] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[22:30:01.789] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[22:30:11.787] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:30:21.808] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:30:31.817] [pf001] window 6/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:30:31.819] [pf001] done: fix compiled OUT, tick cycles now=0
[22:30:31.825] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:30:33.824] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 3rd power-cycle
[22:31:14.630] === uT-Kernel 3.0 / RP2040 demo ===
[22:31:14.631] SMP build: 2 processors
[22:31:14.631] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:31:14.632] [pf001] upstream main, fix compiled OUT
[22:31:14.634] [pf001] tick_at_boot=0x80e00 (enable=1 cycles=0 running=1)  tick_now=0x6de00 (enable=1 cycles=0 running=1)
[22:31:14.635] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:31:24.644] [pf001] window 1/6: kernel=10010 ms timer=234432 us -> 23.419 us/ms   cumulative 23.419 us/ms
[22:31:34.673] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.428 us/ms
[22:31:44.669] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[22:31:54.689] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:32:04.684] [pf001] window 5/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:32:14.711] [pf001] window 6/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:32:14.713] [pf001] done: fix compiled OUT, tick cycles now=0
[22:32:14.716] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:32:16.723] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 4th power-cycle
[22:32:25.177] === uT-Kernel 3.0 / RP2040 demo ===
[22:32:25.178] SMP build: 2 processors
[22:32:25.179] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:32:25.179] [pf001] upstream main, fix compiled OUT
[22:32:25.186] [pf001] tick_at_boot=0x5e00 (enable=1 cycles=0 running=1)  tick_now=0xac600 (enable=1 cycles=0 running=1)
[22:32:25.187] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:32:35.214] [pf001] window 1/6: kernel=10010 ms timer=234443 us -> 23.420 us/ms   cumulative 23.420 us/ms
[22:32:45.213] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[22:32:55.233] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[22:33:05.235] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:33:15.251] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:33:25.257] [pf001] window 6/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:33:25.258] [pf001] done: fix compiled OUT, tick cycles now=0
[22:33:25.260] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:33:27.277] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 5th power-cycle
[22:33:53.637] [pf001] window 1/6: kernel=10010 ms timer=234399 us -> 23.416 us/ms   cumulative 23.416 us/ms
[22:34:03.659] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.427 us/ms
[22:34:13.659] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.430 us/ms
[22:34:23.678] [pf001] window 4/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.432 us/ms
[22:34:33.676] [pf001] window 5/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:34:43.694] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:34:43.696] [pf001] done: fix compiled OUT, tick cycles now=0
[22:34:43.698] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:34:45.713] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# nofix (pf001_upstream_nofix.uf2): RUN/reset button on Pico maker board

## 1st reset
[22:57:27.365] [pf001] window 1/6: kernel=10010 ms timer=234441 us -> 23.420 us/ms   cumulative 23.420 us/ms
[22:57:37.364] [pf001] window 2/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.429 us/ms
[22:57:47.381] [pf001] window 3/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.431 us/ms
[22:57:57.384] [pf001] window 4/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:58:07.402] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:58:17.412] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:58:17.413] [pf001] done: fix compiled OUT, tick cycles now=0
[22:58:17.414] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:58:19.426] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd reset
[22:58:36.893] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[22:58:46.918] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[22:58:56.915] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[22:59:06.935] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[22:59:16.931] [pf001] window 5/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:59:26.954] [pf001] window 6/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.434 us/ms
[22:59:26.955] [pf001] done: fix compiled OUT, tick cycles now=0
[22:59:26.956] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:59:28.972] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 3rd reset
[23:00:34.973] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:00:44.991] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:00:54.989] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:01:05.015] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:01:15.017] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:01:25.036] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:01:25.038] [pf001] done: fix compiled OUT, tick cycles now=0
[23:01:25.039] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:01:27.050] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 4th reset
[23:01:46.906] [pf001] window 1/6: kernel=10010 ms timer=234441 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:01:56.903] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:02:06.925] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:02:16.921] [pf001] window 4/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:02:26.943] [pf001] window 5/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:02:36.944] [pf001] window 6/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:02:36.946] [pf001] done: fix compiled OUT, tick cycles now=0
[23:02:36.947] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:02:38.962] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 5th reset
[23:02:57.873] [pf001] window 1/6: kernel=10010 ms timer=234441 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:03:07.865] [pf001] window 2/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:03:17.893] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:03:27.889] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:03:37.916] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:03:47.918] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:03:47.919] [pf001] done: fix compiled OUT, tick cycles now=0
[23:03:47.920] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:03:49.931] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# fix (pf001_upstream_fix.uf2): reflash

## Setup: Serial Monitor on USB, baud rate 115200
## Reflash methodology
1. Remove USB from Pico 
2. Hold BOOTSEL on unpowered Pico
3. Insert USB into Pico
4. Copy pf001_upstream_fix into folder that pops up

## 1st reflash
[22:35:05.590] === uT-Kernel 3.0 / RP2040 demo ===
[22:35:05.591] SMP build: 2 processors
[22:35:05.593] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:35:05.594] [pf001] upstream main, fix compiled IN
[22:35:05.601] [pf001] tick_at_boot=0x2e0c (enable=1 cycles=12 running=1)  tick_now=0xe0c (enable=1 cycles=12 running=1)
[22:35:05.603] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:35:15.631] [pf001] window 1/6: kernel=10010 ms timer=10002997 us -> 999.300 us/ms   cumulative 999.300 us/ms
[22:35:25.621] [pf001] window 2/6: kernel=10010 ms timer=10009956 us -> 999.995 us/ms   cumulative 999.648 us/ms
[22:35:35.640] [pf001] window 3/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.766 us/ms
[22:35:45.638] [pf001] window 4/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.824 us/ms
[22:35:55.656] [pf001] window 5/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.859 us/ms
[22:36:05.663] [pf001] window 6/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.882 us/ms
[22:36:05.665] [pf001] done: fix compiled IN, tick cycles now=12
[22:36:05.677] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:36:07.683] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd reflash
[22:36:50.973] [pf001] window 1/6: kernel=10010 ms timer=10002844 us -> 999.285 us/ms   cumulative 999.285 us/ms
[22:37:00.993] [pf001] window 2/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.644 us/ms
[22:37:10.993] [pf001] window 3/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.761 us/ms
[22:37:21.007] [pf001] window 4/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.822 us/ms
[22:37:31.013] [pf001] window 5/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.856 us/ms
[22:37:41.032] [pf001] window 6/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[22:37:41.033] [pf001] done: fix compiled IN, tick cycles now=12
[22:37:41.034] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:37:43.044] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 3rd reflash
[22:38:04.485] === uT-Kernel 3.0 / RP2040 demo ===
[22:38:04.485] SMP build: 2 processors
[22:38:04.490] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:38:04.492] [pf001] upstream main, fix compiled IN
[22:38:04.493] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x2e0c (enable=1 cycles=12 running=1)
[22:38:04.494] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:38:14.503] [pf001] window 1/6: kernel=10010 ms timer=10002347 us -> 999.235 us/ms   cumulative 999.235 us/ms
[22:38:24.516] [pf001] window 2/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.619 us/ms
[22:38:34.527] [pf001] window 3/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.745 us/ms
[22:38:44.543] [pf001] window 4/6: kernel=10010 ms timer=10010039 us -> 1000.003 us/ms   cumulative 999.809 us/ms
[22:38:54.544] [pf001] window 5/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.847 us/ms
[22:39:04.566] [pf001] window 6/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.873 us/ms
[22:39:04.568] [pf001] done: fix compiled IN, tick cycles now=12
[22:39:04.569] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:39:06.575] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 4th reflash
[22:39:42.574] [pf001] window 2/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.644 us/ms
[22:39:52.585] [pf001] window 3/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.761 us/ms
[22:40:02.601] [pf001] window 4/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.822 us/ms
[22:40:12.597] [pf001] window 5/6: kernel=10010 ms timer=10009961 us -> 999.996 us/ms   cumulative 999.856 us/ms
[22:40:22.618] [pf001] window 6/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[22:40:22.619] [pf001] done: fix compiled IN, tick cycles now=12
[22:40:22.621] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:40:24.638] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
## 5th reflash
[22:40:52.671] === uT-Kernel 3.0 / RP2040 demo ===
[22:40:52.672] SMP build: 2 processors
[22:40:52.677] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:40:52.678] [pf001] upstream main, fix compiled IN
[22:40:52.681] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[22:40:52.682] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:41:02.681] [pf001] window 1/6: kernel=10010 ms timer=10002505 us -> 999.251 us/ms   cumulative 999.251 us/ms
[22:41:12.703] [pf001] window 2/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.627 us/ms
[22:41:22.703] [pf001] window 3/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.750 us/ms
[22:41:32.728] [pf001] window 4/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.813 us/ms
[22:41:42.728] [pf001] window 5/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.850 us/ms
[22:41:52.739] [pf001] window 6/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.875 us/ms
[22:41:52.740] [pf001] done: fix compiled IN, tick cycles now=12
[22:41:52.741] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:41:54.767] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


# fix (pf001_upstream_fix.uf2): power-cycle
## Power-cycle methodology
1. unplug USB A that plugs into PC to power down pico
2. Replug USB into PC to power up pico
3. Approx 1-2 secs between plugging and unplugging USB

Attempts after the first powercycle it would be sequential, there is no reflash then powercycle other than the first attempt

## 1st power-cycle (after reflash)
[22:42:30.033] [pf001] window 1/6: kernel=10010 ms timer=10002919 us -> 999.292 us/ms   cumulative 999.292 us/ms
[22:42:40.025] [pf001] window 2/6: kernel=10010 ms timer=10009959 us -> 999.995 us/ms   cumulative 999.644 us/ms
[22:42:50.048] [pf001] window 3/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.763 us/ms
[22:43:00.045] [pf001] window 4/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.822 us/ms
[22:43:10.072] [pf001] window 5/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.858 us/ms
[22:43:20.071] [pf001] window 6/6: kernel=10010 ms timer=10009966 us -> 999.996 us/ms   cumulative 999.881 us/ms
[22:43:20.073] [pf001] done: fix compiled IN, tick cycles now=12
[22:43:20.085] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:43:22.086] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd power-cycle
[22:43:33.352] === uT-Kernel 3.0 / RP2040 demo ===
[22:43:33.353] SMP build: 2 processors
[22:43:33.354] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[22:43:33.356] [pf001] upstream main, fix compiled IN
[22:43:33.365] [pf001] tick_at_boot=0xeb600 (enable=1 cycles=0 running=1)  tick_now=0x1e0c (enable=1 cycles=12 running=1)
[22:43:33.366] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[22:43:43.394] [pf001] window 1/6: kernel=10010 ms timer=10002912 us -> 999.291 us/ms   cumulative 999.291 us/ms
[22:43:53.394] [pf001] window 2/6: kernel=10010 ms timer=10009958 us -> 999.995 us/ms   cumulative 999.643 us/ms
[22:44:03.412] [pf001] window 3/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.763 us/ms
[22:44:13.411] [pf001] window 4/6: kernel=10010 ms timer=10009958 us -> 999.995 us/ms   cumulative 999.821 us/ms
[22:44:23.428] [pf001] window 5/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.858 us/ms
[22:44:33.423] [pf001] window 6/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.881 us/ms
[22:44:33.424] [pf001] done: fix compiled IN, tick cycles now=12
[22:44:33.446] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:44:35.444] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
## 3rd power-cycle
[22:44:58.589] [pf001] window 1/6: kernel=10010 ms timer=10002913 us -> 999.292 us/ms   cumulative 999.292 us/ms
[22:45:08.598] [pf001] window 2/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.644 us/ms
[22:45:18.613] [pf001] window 3/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.763 us/ms
[22:45:28.617] [pf001] window 4/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.822 us/ms
[22:45:38.632] [pf001] window 5/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.858 us/ms
[22:45:48.640] [pf001] window 6/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.881 us/ms
[22:45:48.641] [pf001] done: fix compiled IN, tick cycles now=12
[22:45:48.643] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:45:50.650] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
## 4th power-cycle
[22:46:10.537] [pf001] window 1/6: kernel=10010 ms timer=10002872 us -> 999.287 us/ms   cumulative 999.287 us/ms
[22:46:20.556] [pf001] window 2/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.645 us/ms
[22:46:30.557] [pf001] window 3/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.762 us/ms
[22:46:40.568] [pf001] window 4/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.822 us/ms
[22:46:50.567] [pf001] window 5/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.857 us/ms
[22:47:00.591] [pf001] window 6/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[22:47:00.593] [pf001] done: fix compiled IN, tick cycles now=12
[22:47:00.595] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:47:02.621] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
## 5th power-cycle
[22:47:33.006] [pf001] window 1/6: kernel=10010 ms timer=10002911 us -> 999.291 us/ms   cumulative 999.291 us/ms
[22:47:43.004] [pf001] window 2/6: kernel=10010 ms timer=10009960 us -> 999.996 us/ms   cumulative 999.643 us/ms
[22:47:53.025] [pf001] window 3/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.763 us/ms
[22:48:03.025] [pf001] window 4/6: kernel=10010 ms timer=10009961 us -> 999.996 us/ms   cumulative 999.821 us/ms
[22:48:13.050] [pf001] window 5/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.858 us/ms
[22:48:23.047] [pf001] window 6/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.881 us/ms
[22:48:23.051] [pf001] done: fix compiled IN, tick cycles now=12
[22:48:23.063] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:48:25.059] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[22:48:27.082] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

# fix (pf001_upstream_fix.uf2): RUN/reset button on Pico maker board

## 1st reset
[22:49:09.281] [pf001] window 1/6: kernel=10010 ms timer=10002959 us -> 999.296 us/ms   cumulative 999.296 us/ms
[22:49:19.298] [pf001] window 2/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.650 us/ms
[22:49:29.301] [pf001] window 3/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.765 us/ms
[22:49:39.314] [pf001] window 4/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.825 us/ms
[22:49:49.320] [pf001] window 5/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.859 us/ms
[22:49:59.331] [pf001] window 6/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.883 us/ms
[22:49:59.335] [pf001] done: fix compiled IN, tick cycles now=12
[22:49:59.342] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:50:01.356] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd reset
[22:50:26.047] [pf001] window 1/6: kernel=10010 ms timer=10002878 us -> 999.288 us/ms   cumulative 999.288 us/ms
[22:50:36.060] [pf001] window 2/6: kernel=10010 ms timer=10010032 us -> 1000.003 us/ms   cumulative 999.645 us/ms
[22:50:46.065] [pf001] window 3/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.762 us/ms
[22:50:56.081] [pf001] window 4/6: kernel=10010 ms timer=10010037 us -> 1000.003 us/ms   cumulative 999.822 us/ms
[22:51:06.076] [pf001] window 5/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.857 us/ms
[22:51:16.105] [pf001] window 6/6: kernel=10010 ms timer=10010039 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[22:51:16.111] [pf001] done: fix compiled IN, tick cycles now=12
[22:51:16.113] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:51:18.116] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 3rd reset
[22:51:48.085] [pf001] window 1/6: kernel=10010 ms timer=10002999 us -> 999.300 us/ms   cumulative 999.300 us/ms
[22:51:58.082] [pf001] window 2/6: kernel=10010 ms timer=10009961 us -> 999.996 us/ms   cumulative 999.648 us/ms
[22:52:08.107] [pf001] window 3/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.766 us/ms
[22:52:18.110] [pf001] window 4/6: kernel=10010 ms timer=10009961 us -> 999.996 us/ms   cumulative 999.824 us/ms
[22:52:28.122] [pf001] window 5/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.860 us/ms
[22:52:38.128] [pf001] window 6/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.882 us/ms
[22:52:38.129] [pf001] done: fix compiled IN, tick cycles now=12
[22:52:38.140] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:52:40.140] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 4th reset
[22:53:10.789] [pf001] window 1/6: kernel=10010 ms timer=10002962 us -> 999.296 us/ms   cumulative 999.296 us/ms
[22:53:20.812] [pf001] window 2/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.650 us/ms
[22:53:30.816] [pf001] window 3/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.765 us/ms
[22:53:40.832] [pf001] window 4/6: kernel=10010 ms timer=10010040 us -> 1000.003 us/ms   cumulative 999.825 us/ms
[22:53:50.832] [pf001] window 5/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.859 us/ms
[22:54:00.849] [pf001] window 6/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.883 us/ms
[22:54:00.850] [pf001] done: fix compiled IN, tick cycles now=12
[22:54:00.851] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[22:54:02.873] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

