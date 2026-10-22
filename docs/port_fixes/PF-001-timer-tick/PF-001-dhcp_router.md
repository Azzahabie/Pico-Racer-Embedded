# Used SerialMonitor on USB, buad rate 115200
## Reflash methodology
1. Remove USB from Pico 
2. Hold BOOTSEL on unpowered Pico
3. Insert USB into Pico
4. Copy pf001_run4_router_fix0 into folder that pops up

# Network details (Taken from windows PC connected to same hotspot as pico)
- Conductecd on Asus Router
- Security: WPA2/WPA3 Personal
- Encyption: AES
- 2.4Ghz Only
- Radio type: 802.11ax
- SSID: Pico2_IoT

# 1st Boot after flash pf001_run4_router_fix0.uf2
[00:56:22.678] === uT-Kernel 3.0 / RP2040 demo ===
[00:56:22.691] SMP build: 2 processors
[00:56:22.693] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:56:22.694] [pf001] upstream main, fix compiled OUT
[00:56:22.697] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[00:56:22.711] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:56:22.897] [wifi] link DOWN
[00:56:28.914] [wifi] link UP  rssi=-40  mac=d8:3a:dd:ae:4b:eb
[00:56:32.719] [pf001] window 1/6: kernel=10010 ms timer=10002834 us -> 999.284 us/ms   cumulative 999.284 us/ms
[00:56:40.950] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[00:56:42.733] [pf001] window 2/6: kernel=10010 ms timer=10009934 us -> 999.993 us/ms   cumulative 999.638 us/ms
[00:56:52.737] [pf001] window 3/6: kernel=10010 ms timer=10009975 us -> 999.997 us/ms   cumulative 999.758 us/ms
[00:57:02.755] [pf001] window 4/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[00:57:12.749] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.855 us/ms
[00:57:22.769] [pf001] window 6/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.879 us/ms
[00:57:22.770] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[00:57:22.771] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:57:24.796] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd Try Reflash
[00:57:59.964] [wifi] link UP  rssi=-33  mac=d8:3a:dd:ae:4b:eb
[00:58:00.782] [pf001] window 1/6: kernel=10010 ms timer=10002799 us -> 999.280 us/ms   cumulative 999.280 us/ms
[00:58:08.988] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[00:58:10.762] [pf001] window 2/6: kernel=10010 ms timer=10009947 us -> 999.994 us/ms   cumulative 999.637 us/ms
[00:58:20.779] [pf001] window 3/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.759 us/ms
[00:58:30.779] [pf001] window 4/6: kernel=10010 ms timer=10009966 us -> 999.996 us/ms   cumulative 999.818 us/ms
[00:58:40.803] [pf001] window 5/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.855 us/ms
[00:58:50.805] [pf001] window 6/6: kernel=10010 ms timer=10009969 us -> 999.996 us/ms   cumulative 999.879 us/ms
[00:58:50.816] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[00:58:50.817] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:58:52.838] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


## 3rd try reflash
[00:59:11.627] [wifi] link DOWN
[00:59:20.647] [wifi] link UP  rssi=-35  mac=d8:3a:dd:ae:4b:eb
[00:59:21.443] [pf001] window 1/6: kernel=10010 ms timer=10002756 us -> 999.276 us/ms   cumulative 999.276 us/ms
[00:59:26.665] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[00:59:31.460] [pf001] window 2/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.639 us/ms
[00:59:41.467] [pf001] window 3/6: kernel=10010 ms timer=10009971 us -> 999.997 us/ms   cumulative 999.758 us/ms
[00:59:51.480] [pf001] window 4/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[01:00:01.490] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.855 us/ms
[01:00:11.502] [pf001] window 6/6: kernel=10010 ms timer=10010025 us -> 1000.002 us/ms   cumulative 999.879 us/ms
[01:00:11.503] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:00:11.504] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:00:13.516] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[01:00:15.539] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[01:00:17.538] [monitor] seq=132 produced=132 consumed=132 dropped=0  made_on=prc1 seen_on=prc2
[01:00:19.565] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[01:00:21.562] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2

## 4th try reflash
[01:00:39.453] === uT-Kernel 3.0 / RP2040 demo ===
[01:00:39.453] SMP build: 2 processors
[01:00:39.454] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:00:39.455] [pf001] upstream main, fix compiled OUT
[01:00:39.464] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x460c (enable=1 cycles=12 running=1)
[01:00:39.474] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:00:39.649] [wifi] link DOWN
[01:00:48.683] [wifi] link UP  rssi=-33  mac=d8:3a:dd:ae:4b:eb
[01:00:49.490] [pf001] window 1/6: kernel=10010 ms timer=10002769 us -> 999.277 us/ms   cumulative 999.277 us/ms
[01:00:57.704] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:00:59.483] [pf001] window 2/6: kernel=10010 ms timer=10009982 us -> 999.998 us/ms   cumulative 999.637 us/ms
[01:01:09.503] [pf001] window 3/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[01:01:19.506] [pf001] window 4/6: kernel=10010 ms timer=10009971 us -> 999.997 us/ms   cumulative 999.819 us/ms
[01:01:29.523] [pf001] window 5/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.855 us/ms
[01:01:39.528] [pf001] window 6/6: kernel=10010 ms timer=10009989 us -> 999.998 us/ms   cumulative 999.879 us/ms
[01:01:39.539] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:01:39.540] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:01:41.564] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
# 5th try reflash
[01:02:31.395] === uT-Kernel 3.0 / RP2040 demo ===
[01:02:31.412] SMP build: 2 processors
[01:02:31.418] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:02:31.429] [pf001] upstream main, fix compiled OUT
[01:02:31.439] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x360c (enable=1 cycles=12 running=1)
[01:02:31.442] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:02:31.619] [wifi] link DOWN
[01:02:40.634] [wifi] link UP  rssi=-39  mac=d8:3a:dd:ae:4b:eb
[01:02:41.435] [pf001] window 1/6: kernel=10010 ms timer=10002752 us -> 999.275 us/ms   cumulative 999.275 us/ms
[01:02:46.654] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:02:51.456] [pf001] window 2/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.638 us/ms
[01:03:01.458] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.758 us/ms
[01:03:11.479] [pf001] window 4/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[01:03:21.477] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.855 us/ms
[01:03:31.498] [pf001] window 6/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.879 us/ms
[01:03:31.499] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:03:31.501] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
# 6th try reflash
[01:04:04.785] [wifi] link DOWN
[01:04:13.821] [wifi] link UP  rssi=-34  mac=d8:3a:dd:ae:4b:eb
[01:04:14.618] [pf001] window 1/6: kernel=10010 ms timer=10002812 us -> 999.281 us/ms   cumulative 999.281 us/ms
[01:04:22.836] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:04:24.616] [pf001] window 2/6: kernel=10010 ms timer=10009956 us -> 999.995 us/ms   cumulative 999.638 us/ms
[01:04:34.640] [pf001] window 3/6: kernel=10010 ms timer=10010027 us -> 1000.002 us/ms   cumulative 999.760 us/ms
[01:04:44.641] [pf001] window 4/6: kernel=10010 ms timer=10009971 us -> 999.997 us/ms   cumulative 999.819 us/ms
[01:04:54.662] [pf001] window 5/6: kernel=10010 ms timer=10010027 us -> 1000.002 us/ms   cumulative 999.856 us/ms
[01:05:04.661] [pf001] window 6/6: kernel=10010 ms timer=10009971 us -> 999.997 us/ms   cumulative 999.879 us/ms
[01:05:04.672] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:05:04.675] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:05:06.700] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
# 7th try reflash
[01:05:35.223] === uT-Kernel 3.0 / RP2040 demo ===
[01:05:35.241] SMP build: 2 processors
[01:05:35.255] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:05:35.264] [pf001] upstream main, fix compiled OUT
[01:05:35.270] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[01:05:35.280] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:05:35.435] [wifi] link DOWN
[01:05:44.456] [wifi] link UP  rssi=-40  mac=d8:3a:dd:ae:4b:eb
[01:05:45.253] [pf001] window 1/6: kernel=10010 ms timer=10002739 us -> 999.274 us/ms   cumulative 999.274 us/ms
[01:05:55.277] [pf001] window 2/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.638 us/ms
[01:05:56.495] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:06:05.280] [pf001] window 3/6: kernel=10010 ms timer=10009979 us -> 999.997 us/ms   cumulative 999.758 us/ms
[01:06:15.297] [pf001] window 4/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.819 us/ms
[01:06:25.297] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.854 us/ms
[01:06:35.318] [pf001] window 6/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.879 us/ms
[01:06:35.319] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:06:35.321] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:06:37.336] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
# 8th try reflash
[01:07:02.192] === uT-Kernel 3.0 / RP2040 demo ===
[01:07:02.214] SMP build: 2 processors
[01:07:02.233] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:07:02.249] [pf001] upstream main, fix compiled OUT
[01:07:02.259] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x2e0c (enable=1 cycles=12 running=1)
[01:07:02.275] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:07:02.410] [wifi] link DOWN
[01:07:11.427] [wifi] link UP  rssi=-35  mac=d8:3a:dd:ae:4b:eb
[01:07:12.234] [pf001] window 1/6: kernel=10010 ms timer=10002748 us -> 999.275 us/ms   cumulative 999.275 us/ms
[01:07:22.244] [pf001] window 2/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.638 us/ms
[01:07:23.471] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:07:32.248] [pf001] window 3/6: kernel=10010 ms timer=10009979 us -> 999.997 us/ms   cumulative 999.758 us/ms
[01:07:42.263] [pf001] window 4/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[01:07:52.272] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.855 us/ms
[01:08:02.286] [pf001] window 6/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.879 us/ms
[01:08:02.289] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:08:02.290] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:08:04.308] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# 9th try reflash
[01:08:30.496] === uT-Kernel 3.0 / RP2040 demo ===
[01:08:30.506] SMP build: 2 processors
[01:08:30.513] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:08:30.517] [pf001] upstream main, fix compiled OUT
[01:08:30.527] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[01:08:30.535] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:08:30.690] [wifi] link DOWN
[01:08:39.730] [wifi] link UP  rssi=-35  mac=d8:3a:dd:ae:4b:eb
[01:08:40.525] [pf001] window 1/6: kernel=10010 ms timer=10002799 us -> 999.280 us/ms   cumulative 999.280 us/ms
[01:08:48.755] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:08:50.534] [pf001] window 2/6: kernel=10010 ms timer=10009944 us -> 999.994 us/ms   cumulative 999.637 us/ms
[01:09:00.545] [pf001] window 3/6: kernel=10010 ms timer=10010040 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[01:09:10.547] [pf001] window 4/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.818 us/ms
[01:09:20.566] [pf001] window 5/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.855 us/ms
[01:09:30.566] [pf001] window 6/6: kernel=10010 ms timer=10009969 us -> 999.996 us/ms   cumulative 999.879 us/ms
[01:09:30.581] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:09:30.582] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:09:32.603] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# 10th try reflash
[01:10:00.316] === uT-Kernel 3.0 / RP2040 demo ===
[01:10:00.337] SMP build: 2 processors
[01:10:00.350] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:10:00.358] [pf001] upstream main, fix compiled OUT
[01:10:00.366] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[01:10:00.378] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:10:00.512] [wifi] link DOWN
[01:10:06.527] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[01:10:10.355] [pf001] window 1/6: kernel=10010 ms timer=10002797 us -> 999.280 us/ms   cumulative 999.280 us/ms
[01:10:18.570] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:10:20.348] [pf001] window 2/6: kernel=10010 ms timer=10009948 us -> 999.994 us/ms   cumulative 999.637 us/ms
[01:10:30.376] [pf001] window 3/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[01:10:40.372] [pf001] window 4/6: kernel=10010 ms timer=10009966 us -> 999.996 us/ms   cumulative 999.818 us/ms
[01:10:50.389] [pf001] window 5/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.855 us/ms
[01:11:00.389] [pf001] window 6/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.879 us/ms
[01:11:00.404] [pf001] done: fix compiled OUT, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:11:00.405] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:11:02.433] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# Notes for reflash
All 10 link up got a ping from PC -> pico where PC is in the same network

## Power-cycle methodology
1. unplug USB A that plugs into PC to power down pico
2. Replug USB into PC to power up pico
3. Approx 1-2 secs between plugging and unplugging USB

## 1st power-cycle (after reflash)
[01:11:29.216] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:11:31.429] [wifi] link UP  rssi=-32  mac=d8:3a:dd:ae:4b:eb
[01:11:39.229] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:11:49.237] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:11:59.252] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:12:09.251] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:12:19.270] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:12:19.271] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:12:19.272] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:12:21.289] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2

# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 2nd power-cycle
[01:12:55.449] === uT-Kernel 3.0 / RP2040 demo ===
[01:12:55.469] SMP build: 2 processors
[01:12:55.470] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:12:55.470] [pf001] upstream main, fix compiled OUT
[01:12:55.471] [pf001] tick_at_boot=0xe0e00 (enable=1 cycles=0 running=1)  tick_now=0x98e00 (enable=1 cycles=0 running=1)
[01:12:55.488] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:12:55.675] [wifi] link DOWN
[01:13:05.493] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:13:07.713] [wifi] link UP  rssi=-34  mac=d8:3a:dd:ae:4b:eb
[01:13:15.514] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:13:25.508] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:13:35.536] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:13:45.528] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:13:55.555] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:13:55.556] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:13:55.557] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:13:57.571] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2

# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 3rd power-cycle
[01:14:21.869] === uT-Kernel 3.0 / RP2040 demo ===
[01:14:21.889] SMP build: 2 processors
[01:14:21.890] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:14:21.892] [pf001] upstream main, fix compiled OUT
[01:14:21.894] [pf001] tick_at_boot=0x69e00 (enable=1 cycles=0 running=1)  tick_now=0xeae00 (enable=1 cycles=0 running=1)
[01:14:21.909] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:14:22.094] [wifi] link DOWN
[01:14:31.916] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:14:34.128] [wifi] link UP  rssi=-29  mac=d8:3a:dd:ae:4b:eb
[01:14:41.932] [pf001] window 2/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:14:51.933] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:15:01.947] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:15:11.949] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:15:21.974] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:15:21.975] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:15:21.976] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:15:23.987] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2

# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 4th power-cycle
[01:15:38.239] === uT-Kernel 3.0 / RP2040 demo ===
[01:15:38.256] SMP build: 2 processors
[01:15:38.257] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:15:38.259] [pf001] upstream main, fix compiled OUT
[01:15:38.260] [pf001] tick_at_boot=0x95e00 (enable=1 cycles=0 running=1)  tick_now=0xd2e00 (enable=1 cycles=0 running=1)
[01:15:38.275] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:15:38.460] [wifi] link DOWN
[01:15:48.277] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:15:50.505] [wifi] link UP  rssi=-28  mac=d8:3a:dd:ae:4b:eb
[01:15:58.296] [pf001] window 2/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:16:08.295] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:16:18.318] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:16:28.322] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:16:38.339] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:16:38.341] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:16:38.342] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:16:40.360] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2

# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 5th power-cycle
[01:16:54.876] [wifi] link DOWN
[01:17:04.696] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:17:06.916] [wifi] link UP  rssi=-31  mac=d8:3a:dd:ae:4b:eb
[01:17:14.718] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:17:24.716] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:17:34.728] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:17:44.735] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:17:54.749] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:17:54.750] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:17:54.752] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:17:56.775] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2

# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 6th power-cycle
[01:18:07.348] === uT-Kernel 3.0 / RP2040 demo ===
[01:18:07.369] SMP build: 2 processors
[01:18:07.370] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:18:07.372] [pf001] upstream main, fix compiled OUT
[01:18:07.374] [pf001] tick_at_boot=0xe6e00 (enable=1 cycles=0 running=1)  tick_now=0x56e00 (enable=1 cycles=0 running=1)
[01:18:07.379] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:18:07.565] [wifi] link DOWN
[01:18:17.383] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:18:19.604] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[01:18:27.400] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:18:37.406] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:18:47.428] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:18:57.426] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:19:07.446] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:19:07.448] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:19:07.448] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:19:09.462] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2

# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 7th power-cycle
[01:19:21.882] [wifi] link DOWN
[01:19:31.725] [pf001] window 1/6: kernel=10010 ms timer=234441 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:19:33.913] [wifi] link UP  rssi=-34  mac=d8:3a:dd:ae:4b:eb
[01:19:41.719] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:19:51.734] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:20:01.736] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:20:11.761] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:20:21.760] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:20:21.761] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:20:21.771] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:20:23.795] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 8th power-cycle
[01:20:42.385] [wifi] link DOWN
[01:20:52.227] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:20:54.426] [wifi] link UP  rssi=-29  mac=d8:3a:dd:ae:4b:eb
[01:21:02.222] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:21:12.245] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:21:22.250] [pf001] window 4/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:21:32.263] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:21:42.275] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:21:42.276] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:21:42.280] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:21:44.302] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time
## 9th power-cycle
[01:21:55.918] [wifi] link DOWN
[01:22:05.738] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:22:07.957] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[01:22:15.761] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:22:25.755] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:22:35.781] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:22:45.776] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:22:55.794] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:22:55.795] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:22:55.796] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:22:57.819] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time

## 10th power-cycle
[01:23:18.387] === uT-Kernel 3.0 / RP2040 demo ===
[01:23:18.390] SMP build: 2 processors
[01:23:18.392] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:23:18.399] [pf001] upstream main, fix compiled OUT
[01:23:18.402] [pf001] tick_at_boot=0xe4e00 (enable=1 cycles=0 running=1)  tick_now=0x6e00 (enable=1 cycles=0 running=1)
[01:23:18.403] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:23:18.579] [wifi] link DOWN
[01:23:28.420] [pf001] window 1/6: kernel=10010 ms timer=234442 us -> 23.420 us/ms   cumulative 23.420 us/ms
[01:23:30.625] [wifi] link UP  rssi=-28  mac=d8:3a:dd:ae:4b:eb
[01:23:38.428] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[01:23:48.441] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[01:23:58.447] [pf001] window 4/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[01:24:08.468] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:24:18.466] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[01:24:18.476] [pf001] done: fix compiled OUT, tick cycles now=0 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:24:18.478] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[01:24:20.506] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[01:24:22.499] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
# Notes
PC cannot ping Pico, router network panel detects Pico with refreshed access time


# 1st Boot after flash pf001_run4_router_fix1.uf2
[01:25:08.238] === uT-Kernel 3.0 / RP2040 demo ===
[01:25:08.240] SMP build: 2 processors
[01:25:08.242] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:25:08.247] [pf001] upstream main, fix compiled IN
[01:25:08.249] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x5e0c (enable=1 cycles=12 running=1)
[01:25:08.251] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:25:08.426] [wifi] link DOWN
[01:25:17.443] [wifi] link UP  rssi=-37  mac=d8:3a:dd:ae:4b:eb
[01:25:18.249] [pf001] window 1/6: kernel=10010 ms timer=10002460 us -> 999.246 us/ms   cumulative 999.246 us/ms
[01:25:28.270] [pf001] window 2/6: kernel=10010 ms timer=10010028 us -> 1000.002 us/ms   cumulative 999.624 us/ms
[01:25:29.487] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:25:38.270] [pf001] window 3/6: kernel=10010 ms timer=10009973 us -> 999.997 us/ms   cumulative 999.748 us/ms
[01:25:48.291] [pf001] window 4/6: kernel=10010 ms timer=10010024 us -> 1000.002 us/ms   cumulative 999.812 us/ms
[01:25:58.283] [pf001] window 5/6: kernel=10010 ms timer=10009969 us -> 999.996 us/ms   cumulative 999.849 us/ms
[01:26:08.303] [pf001] window 6/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.874 us/ms
[01:26:08.305] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:26:08.305] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:26:10.324] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd Try Reflash
[01:27:03.832] [wifi] link DOWN
[01:27:12.873] [wifi] link UP  rssi=-28  mac=d8:3a:dd:ae:4b:eb
[01:27:13.670] [pf001] window 1/6: kernel=10010 ms timer=10002910 us -> 999.291 us/ms   cumulative 999.291 us/ms
[01:27:21.894] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:27:23.672] [pf001] window 2/6: kernel=10010 ms timer=10009948 us -> 999.994 us/ms   cumulative 999.643 us/ms
[01:27:33.696] [pf001] window 3/6: kernel=10010 ms timer=10010029 us -> 1000.002 us/ms   cumulative 999.763 us/ms
[01:27:43.691] [pf001] window 4/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.821 us/ms
[01:27:53.711] [pf001] window 5/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[01:28:03.709] [pf001] window 6/6: kernel=10010 ms timer=10009974 us -> 999.997 us/ms   cumulative 999.881 us/ms
[01:28:03.724] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:28:03.724] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:28:05.752] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 3rd Try Reflash
[01:28:27.494] [wifi] link DOWN
[01:28:36.532] [wifi] link UP  rssi=-31  mac=d8:3a:dd:ae:4b:eb
[01:28:37.329] [pf001] window 1/6: kernel=10010 ms timer=10002899 us -> 999.290 us/ms   cumulative 999.290 us/ms
[01:28:45.549] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:28:47.326] [pf001] window 2/6: kernel=10010 ms timer=10009955 us -> 999.995 us/ms   cumulative 999.643 us/ms
[01:28:57.353] [pf001] window 3/6: kernel=10010 ms timer=10010029 us -> 1000.002 us/ms   cumulative 999.763 us/ms
[01:29:07.348] [pf001] window 4/6: kernel=10010 ms timer=10009972 us -> 999.997 us/ms   cumulative 999.821 us/ms
[01:29:17.363] [pf001] window 5/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.857 us/ms
[01:29:27.364] [pf001] window 6/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.880 us/ms
[01:29:27.382] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:29:27.384] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:29:29.407] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


## 4th Try Reflash
[01:29:57.669] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[01:29:58.467] [pf001] window 1/6: kernel=10010 ms timer=10002824 us -> 999.283 us/ms   cumulative 999.283 us/ms
[01:30:06.705] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:30:08.491] [pf001] window 2/6: kernel=10010 ms timer=10010017 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[01:30:18.485] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.760 us/ms
[01:30:28.507] [pf001] window 4/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[01:30:38.509] [pf001] window 5/6: kernel=10010 ms timer=10009982 us -> 999.998 us/ms   cumulative 999.856 us/ms
[01:30:48.527] [pf001] window 6/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.880 us/ms
[01:30:48.529] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:30:48.530] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:30:50.544] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


## 5th Try Reflash
[01:31:10.559] === uT-Kernel 3.0 / RP2040 demo ===
[01:31:10.576] SMP build: 2 processors
[01:31:10.597] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:31:10.614] [pf001] upstream main, fix compiled IN
[01:31:10.624] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[01:31:10.632] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:31:10.777] [wifi] link DOWN
[01:31:19.796] [wifi] link UP  rssi=-27  mac=d8:3a:dd:ae:4b:eb
[01:31:20.594] [pf001] window 1/6: kernel=10010 ms timer=10002828 us -> 999.283 us/ms   cumulative 999.283 us/ms
[01:31:28.839] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:31:30.620] [pf001] window 2/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[01:31:40.617] [pf001] window 3/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.761 us/ms
[01:31:50.637] [pf001] window 4/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[01:32:00.637] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.856 us/ms
[01:32:10.658] [pf001] window 6/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.880 us/ms
[01:32:10.659] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:32:10.660] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2


## 6th Try Reflash
[01:32:49.892] [wifi] link UP  rssi=-32  mac=d8:3a:dd:ae:4b:eb
[01:32:50.696] [pf001] window 1/6: kernel=10010 ms timer=10002889 us -> 999.289 us/ms   cumulative 999.289 us/ms
[01:32:58.905] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:33:00.692] [pf001] window 2/6: kernel=10010 ms timer=10009948 us -> 999.994 us/ms   cumulative 999.642 us/ms
[01:33:10.710] [pf001] window 3/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[01:33:20.706] [pf001] window 4/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.821 us/ms
[01:33:30.730] [pf001] window 5/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[01:33:40.734] [pf001] window 6/6: kernel=10010 ms timer=10009972 us -> 999.997 us/ms   cumulative 999.880 us/ms
[01:33:40.743] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:33:40.745] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:33:42.770] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


## 7th Try Reflash
[01:34:08.642] =12 running=1)  tick_now=0x360c (enable=1 cycles=12 running=1)
[01:34:08.650] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:34:08.816] [wifi] link DOWN
[01:34:17.857] [wifi] link UP  rssi=-34  mac=d8:3a:dd:ae:4b:eb
[01:34:18.652] [pf001] window 1/6: kernel=10010 ms timer=10002911 us -> 999.291 us/ms   cumulative 999.291 us/ms
[01:34:23.874] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:34:28.660] [pf001] window 2/6: kernel=10010 ms timer=10009945 us -> 999.994 us/ms   cumulative 999.643 us/ms
[01:34:38.672] [pf001] window 3/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.763 us/ms
[01:34:48.676] [pf001] window 4/6: kernel=10010 ms timer=10009976 us -> 999.997 us/ms   cumulative 999.821 us/ms
[01:34:58.695] [pf001] window 5/6: kernel=10010 ms timer=10010025 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[01:35:08.692] [pf001] window 6/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.881 us/ms
[01:35:08.708] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:35:08.709] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:35:10.732] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[01:35:12.738] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[01:35:14.749] [monitor] seq=132 produced=132 consumed=132 dropped=0  made_on=prc1 seen_on=prc2
[01:35:16.749] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2


## 8th Try Reflash
[01:35:38.657] === uT-Kernel 3.0 / RP2040 demo ===
[01:35:38.681] SMP build: 2 processors
[01:35:38.701] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:35:38.713] [pf001] upstream main, fix compiled IN
[01:35:38.721] [pf001] tick_at_boot=0x660c (enable=1 cycles=12 running=1)  tick_now=0x2e0c (enable=1 cycles=12 running=1)
[01:35:38.728] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:35:38.883] [wifi] link DOWN
[01:35:47.901] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[01:35:48.697] [pf001] window 1/6: kernel=10010 ms timer=10002839 us -> 999.284 us/ms   cumulative 999.284 us/ms
[01:35:53.916] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:35:58.715] [pf001] window 2/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.643 us/ms
[01:36:08.716] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.761 us/ms
[01:36:18.740] [pf001] window 4/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[01:36:28.741] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.856 us/ms
[01:36:38.757] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.881 us/ms
[01:36:38.759] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:36:38.761] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:36:40.779] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 9th Try Reflash
[01:37:04.777] === uT-Kernel 3.0 / RP2040 demo ===
[01:37:04.807] SMP build: 2 processors
[01:37:04.821] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:37:04.832] [pf001] upstream main, fix compiled IN
[01:37:04.843] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x1e0c (enable=1 cycles=12 running=1)
[01:37:04.854] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:37:04.989] [wifi] link DOWN
[01:37:14.015] [wifi] link UP  rssi=-29  mac=d8:3a:dd:ae:4b:eb
[01:37:14.810] [pf001] window 1/6: kernel=10010 ms timer=10002834 us -> 999.284 us/ms   cumulative 999.284 us/ms
[01:37:23.053] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:37:24.833] [pf001] window 2/6: kernel=10010 ms timer=10010015 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[01:37:34.828] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.761 us/ms
[01:37:44.854] [pf001] window 4/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.821 us/ms
[01:37:54.848] [pf001] window 5/6: kernel=10010 ms timer=10009981 us -> 999.998 us/ms   cumulative 999.856 us/ms
[01:38:04.869] [pf001] window 6/6: kernel=10010 ms timer=10010024 us -> 1000.002 us/ms   cumulative 999.881 us/ms
[01:38:04.870] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:38:04.871] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:38:06.894] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 10th Try Reflash
[01:38:38.194] [wifi] link UP  rssi=-33  mac=d8:3a:dd:ae:4b:eb
[01:38:38.991] [pf001] window 1/6: kernel=10010 ms timer=10002840 us -> 999.284 us/ms   cumulative 999.284 us/ms
[01:38:49.008] [pf001] window 2/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.643 us/ms
[01:38:50.227] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:38:59.012] [pf001] window 3/6: kernel=10010 ms timer=10009983 us -> 999.998 us/ms   cumulative 999.761 us/ms
[01:39:09.024] [pf001] window 4/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[01:39:19.024] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.856 us/ms
[01:39:29.053] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.881 us/ms
[01:39:29.056] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=12 wd_reason=0x1 por=1 run=0
[01:39:29.058] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:39:31.062] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# Notes
All test from reflash, PC can ping the pico

# fix (pf001_run4_router_fix1.uf2): power-cycle
## Power-cycle methodology
1. unplug USB A that plugs into PC to power down pico
2. Replug USB into PC to power up pico
3. Approx 1-2 secs between plugging and unplugging USB

Attempts after the first powercycle it would be sequential, there is no reflash then powercycle other than the first attempt

## 1st power-cycle (after reflash)
[01:40:33.074] [wifi] link UP  rssi=-29  mac=d8:3a:dd:ae:4b:eb
[01:40:33.869] [pf001] window 1/6: kernel=10010 ms timer=10002882 us -> 999.288 us/ms   cumulative 999.288 us/ms
[01:40:42.092] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:40:43.868] [pf001] window 2/6: kernel=10010 ms timer=10009974 us -> 999.997 us/ms   cumulative 999.643 us/ms
[01:40:53.892] [pf001] window 3/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.763 us/ms
[01:41:03.890] [pf001] window 4/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.821 us/ms
[01:41:13.908] [pf001] window 5/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[01:41:23.911] [pf001] window 6/6: kernel=10010 ms timer=10009976 us -> 999.997 us/ms   cumulative 999.881 us/ms
[01:41:23.921] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:41:23.922] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

## 2nd power-cycle
[01:41:51.476] [wifi] link UP  rssi=-28  mac=d8:3a:dd:ae:4b:eb
[01:41:52.270] [pf001] window 1/6: kernel=10010 ms timer=10002816 us -> 999.282 us/ms   cumulative 999.282 us/ms
[01:42:02.293] [pf001] window 2/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.642 us/ms
[01:42:03.514] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:42:12.292] [pf001] window 3/6: kernel=10010 ms timer=10009983 us -> 999.998 us/ms   cumulative 999.761 us/ms
[01:42:22.309] [pf001] window 4/6: kernel=10010 ms timer=10010012 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[01:42:32.309] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.856 us/ms
[01:42:42.330] [pf001] window 6/6: kernel=10010 ms timer=10010050 us -> 1000.004 us/ms   cumulative 999.881 us/ms
[01:42:42.331] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:42:42.331] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:42:44.352] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 3rd power-cycle
[01:42:54.103] === uT-Kernel 3.0 / RP2040 demo ===
[01:42:54.116] SMP build: 2 processors
[01:42:54.117] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:42:54.118] [pf001] upstream main, fix compiled IN
[01:42:54.119] [pf001] tick_at_boot=0x77600 (enable=1 cycles=0 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[01:42:54.136] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:42:54.321] [wifi] link DOWN
[01:43:03.339] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[01:43:04.143] [pf001] window 1/6: kernel=10010 ms timer=10002826 us -> 999.283 us/ms   cumulative 999.283 us/ms
[01:43:14.158] [pf001] window 2/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.642 us/ms
[01:43:15.378] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:43:24.155] [pf001] window 3/6: kernel=10010 ms timer=10009979 us -> 999.997 us/ms   cumulative 999.761 us/ms
[01:43:34.177] [pf001] window 4/6: kernel=10010 ms timer=10010024 us -> 1000.002 us/ms   cumulative 999.821 us/ms
[01:43:44.180] [pf001] window 5/6: kernel=10010 ms timer=10009973 us -> 999.997 us/ms   cumulative 999.856 us/ms
[01:43:54.200] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.880 us/ms
[01:43:54.201] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:43:54.201] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

## 4th power-cycle
[01:44:05.735] === uT-Kernel 3.0 / RP2040 demo ===
[01:44:05.735] SMP build: 2 processors
[01:44:05.736] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:44:05.736] [pf001] upstream main, fix compiled IN
[01:44:05.737] [pf001] tick_at_boot=0xb9600 (enable=1 cycles=0 running=1)  tick_now=0x560c (enable=1 cycles=12 running=1)
[01:44:05.738] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:44:05.924] [wifi] link DOWN
[01:44:14.942] [wifi] link UP  rssi=-42  mac=d8:3a:dd:ae:4b:eb
[01:44:15.747] [pf001] window 1/6: kernel=10010 ms timer=10002470 us -> 999.247 us/ms   cumulative 999.247 us/ms
[01:44:23.980] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:44:25.767] [pf001] window 2/6: kernel=10010 ms timer=10010024 us -> 1000.002 us/ms   cumulative 999.625 us/ms
[01:44:35.769] [pf001] window 3/6: kernel=10010 ms timer=10009973 us -> 999.997 us/ms   cumulative 999.749 us/ms
[01:44:45.785] [pf001] window 4/6: kernel=10010 ms timer=10010025 us -> 1000.002 us/ms   cumulative 999.812 us/ms
[01:44:55.787] [pf001] window 5/6: kernel=10010 ms timer=10009973 us -> 999.997 us/ms   cumulative 999.849 us/ms
[01:45:05.806] [pf001] window 6/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.875 us/ms
[01:45:05.808] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:45:05.809] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

## 5th power-cycle
[01:45:26.289] === uT-Kernel 3.0 / RP2040 demo ===
[01:45:26.291] SMP build: 2 processors
[01:45:26.292] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:45:26.294] [pf001] upstream main, fix compiled IN
[01:45:26.297] [pf001] tick_at_boot=0xaf600 (enable=1 cycles=0 running=1)  tick_now=0xe0c (enable=1 cycles=12 running=1)
[01:45:26.298] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:45:26.484] [wifi] link DOWN
[01:45:35.518] [wifi] link UP  rssi=-40  mac=d8:3a:dd:ae:4b:eb
[01:45:36.324] [pf001] window 1/6: kernel=10010 ms timer=10002983 us -> 999.299 us/ms   cumulative 999.299 us/ms
[01:45:44.542] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:45:46.316] [pf001] window 2/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.647 us/ms
[01:45:56.343] [pf001] window 3/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.765 us/ms
[01:46:06.342] [pf001] window 4/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.823 us/ms
[01:46:16.361] [pf001] window 5/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.859 us/ms
[01:46:26.366] [pf001] window 6/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.882 us/ms
[01:46:26.367] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:46:26.374] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:46:28.397] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 6th power-cycle
[01:46:41.602] === uT-Kernel 3.0 / RP2040 demo ===
[01:46:41.622] SMP build: 2 processors
[01:46:41.624] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:46:41.626] [pf001] upstream main, fix compiled IN
[01:46:41.628] [pf001] tick_at_boot=0xdc600 (enable=1 cycles=0 running=1)  tick_now=0x460c (enable=1 cycles=12 running=1)
[01:46:41.641] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:46:41.827] [wifi] link DOWN
[01:46:50.842] [wifi] link UP  rssi=-39  mac=d8:3a:dd:ae:4b:eb
[01:46:51.649] [pf001] window 1/6: kernel=10010 ms timer=10002830 us -> 999.283 us/ms   cumulative 999.283 us/ms
[01:46:59.884] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:47:01.661] [pf001] window 2/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[01:47:11.663] [pf001] window 3/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.761 us/ms
[01:47:21.683] [pf001] window 4/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[01:47:31.688] [pf001] window 5/6: kernel=10010 ms timer=10009981 us -> 999.998 us/ms   cumulative 999.856 us/ms
[01:47:41.710] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.880 us/ms
[01:47:41.711] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:47:41.713] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2


## 7th power-cycle
[01:47:59.200] [wifi] link DOWN
[01:48:08.240] [wifi] link UP  rssi=-39  mac=d8:3a:dd:ae:4b:eb
[01:48:09.047] [pf001] window 1/6: kernel=10010 ms timer=10002885 us -> 999.289 us/ms   cumulative 999.289 us/ms
[01:48:19.045] [pf001] window 2/6: kernel=10010 ms timer=10009953 us -> 999.995 us/ms   cumulative 999.642 us/ms
[01:48:20.284] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:48:29.059] [pf001] window 3/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.762 us/ms
[01:48:39.060] [pf001] window 4/6: kernel=10010 ms timer=10009968 us -> 999.996 us/ms   cumulative 999.821 us/ms
[01:48:49.083] [pf001] window 5/6: kernel=10010 ms timer=10010027 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[01:48:59.085] [pf001] window 6/6: kernel=10010 ms timer=10009971 us -> 999.997 us/ms   cumulative 999.880 us/ms
[01:48:59.094] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:48:59.095] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2


## 8th power-cycle
[01:49:16.961] === uT-Kernel 3.0 / RP2040 demo ===
[01:49:16.980] SMP build: 2 processors
[01:49:16.982] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[01:49:16.983] [pf001] upstream main, fix compiled IN
[01:49:16.984] [pf001] tick_at_boot=0xee600 (enable=1 cycles=0 running=1)  tick_now=0x260c (enable=1 cycles=12 running=1)
[01:49:16.999] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[01:49:17.185] [wifi] link DOWN
[01:49:26.201] [wifi] link UP  rssi=-47  mac=d8:3a:dd:ae:4b:eb
[01:49:27.007] [pf001] window 1/6: kernel=10010 ms timer=10002826 us -> 999.283 us/ms   cumulative 999.283 us/ms
[01:49:35.240] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:49:37.027] [pf001] window 2/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.643 us/ms
[01:49:47.019] [pf001] window 3/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.760 us/ms
[01:49:57.039] [pf001] window 4/6: kernel=10010 ms timer=10010041 us -> 1000.004 us/ms   cumulative 999.821 us/ms
[01:50:07.044] [pf001] window 5/6: kernel=10010 ms timer=10009960 us -> 999.996 us/ms   cumulative 999.856 us/ms
[01:50:17.068] [pf001] window 6/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[01:50:17.069] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:50:17.071] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

## 9th power-cycle
[01:50:34.304] [wifi] link DOWN
[01:50:40.323] [wifi] link UP  rssi=-40  mac=d8:3a:dd:ae:4b:eb
[01:50:44.137] [pf001] window 1/6: kernel=10010 ms timer=10002988 us -> 999.299 us/ms   cumulative 999.299 us/ms
[01:50:52.361] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:50:54.138] [pf001] window 2/6: kernel=10010 ms timer=10009953 us -> 999.995 us/ms   cumulative 999.647 us/ms
[01:51:04.160] [pf001] window 3/6: kernel=10010 ms timer=10010069 us -> 1000.006 us/ms   cumulative 999.767 us/ms
[01:51:14.159] [pf001] window 4/6: kernel=10010 ms timer=10009920 us -> 999.992 us/ms   cumulative 999.823 us/ms
[01:51:24.182] [pf001] window 5/6: kernel=10010 ms timer=10010029 us -> 1000.002 us/ms   cumulative 999.859 us/ms
[01:51:34.184] [pf001] window 6/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.882 us/ms
[01:51:34.194] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:51:34.197] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

## 10th power-cycle
[01:52:02.584] [wifi] link UP  rssi=-41  mac=d8:3a:dd:ae:4b:eb
[01:52:03.380] [pf001] window 1/6: kernel=10010 ms timer=10002951 us -> 999.295 us/ms   cumulative 999.295 us/ms
[01:52:13.378] [pf001] window 2/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.646 us/ms
[01:52:14.626] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[01:52:23.405] [pf001] window 3/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.765 us/ms
[01:52:33.397] [pf001] window 4/6: kernel=10010 ms timer=10009974 us -> 999.997 us/ms   cumulative 999.823 us/ms
[01:52:43.421] [pf001] window 5/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.859 us/ms
[01:52:53.423] [pf001] window 6/6: kernel=10010 ms timer=10009975 us -> 999.997 us/ms   cumulative 999.882 us/ms
[01:52:53.435] [pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
[01:52:53.437] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[01:52:55.457] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# Notes
All powercycles here PC can ping picoOKay


# Analysis (2026-10-20)

## Result
| Image | Boot path | Boots | Timer rate | `link UP` | `ip=` within 60 s | PC ping |
|---|---|---|---|---|---|---|
| router_fix0 (no fix) | reflash | 10 | ~1000 us/ms | 10/10 | 10/10 | yes |
| router_fix0 (no fix) | power-cycle | 10 | 23.437 us/ms | 10/10 | **0/10** | no |
| router_fix1 (fix) | reflash | 10 | ~1000 us/ms | 10/10 | 10/10 | yes |
| router_fix1 (fix) | power-cycle | 10 | ~1000 us/ms | 10/10 | **10/10** | yes |

Every boot is identified by its own `done` line, not by the method notes:
- no-fix power-cycle: `at boot: cycles=0 wd_reason=0x0`, `cycles now=0` (10/10)
- fix power-cycle: `at boot: cycles=0 wd_reason=0x0`, `cycles now=12` (10/10)
- all 20 reflashes: `at boot: cycles=12 wd_reason=0x1`

So all 20 power-cycle boots started on the power-on path with the tick
unprogrammed. With the fix, `startup_clock()` reprogrammed it every time, and
the TIMER ran at 1 MHz.

## Cross-examination
- **Only the fix changes.** fix0 and fix1 are the same source, profile,
  credentials and board. They differ only in `PF001_APPLY_FIX`, which adds one
  register write.
- **The router was healthy throughout.** Reflash boots got an IP immediately
  before the no-fix power-cycles (01:10) and immediately after them (01:25).
  The 0/10 is therefore not a router outage or a lease problem. The same MAC
  got the same IP, 192.168.50.8, on every successful boot.
- **The failure point matches the hypothesis.** With the slow timer:
  - Association still succeeded: `link UP` 10/10, and the router panel
    refreshed the Pico's access time.
  - DHCP did not complete, and the PC could not ping the Pico.
  
  Association does not use the TIMER. lwIP's `sys_now()` does
  (INVESTIGATION.md §6).
- **There was a retry to stretch.** Even on good boots, `ip=` came 6, 9 or 12 s
  after `link UP` (always a multiple of 3 s, the status-print period), so the
  first DHCP exchange usually needed at least one retransmit. Its 2 s retry
  becomes ~85 s at 23.4 us/ms, which is past the 60 s window. This is the same
  mechanism INVESTIGATION.md §6 recorded on our firmware.
- **The hotspot join alternation did not occur here.** All 40 boots on the
  router joined first time. The alternate-boot join failures in the hotspot
  run were specific to the phone hotspot, and they appeared with and without
  the fix. They are a separate issue (the upstream one-shot join), not
  part of PF-001.
- **Strength of the result.** 0/10 vs 10/10 on matched boots is a two-sided
  Fisher exact p ≈ 1 × 10⁻⁵. It reproduces the earlier 0/5 → 7/7 on our
  firmware, this time on unmodified upstream code.

## Limits
- "No IP" means no IP within the 60 s test window. The earlier 3-minute
  observation (INVESTIGATION.md §1) also saw none. At 43× slow, the retries
  might eventually succeed after several minutes. That is not tested here,
  and it does not matter for a board that has to be usable at boot.
- The RUN-button reset path was not repeated with Wi-Fi. Runs 1-3 showed that
  it gives the same `cycles=0` state as a power-cycle.
- One header was partly lost (fix reflash 7, `01:34:08.642`), and the last
  line of the notes ends in a stray "OKay". Both were left as captured.

## Conclusion
The hypothesis holds on upstream `sirfonzie/mtk3smp-rp2040` `main`:
- On a power-on boot the TIMER runs at 1/43 speed. The board joins Wi-Fi, but
  it never gets an IP address.
- The one-line WATCHDOG_TICK write in `startup_clock()` fixes this on every
  boot tested.
- The DHCP/lwIP stack was not changed, so this is not a TCP/IP bug.
