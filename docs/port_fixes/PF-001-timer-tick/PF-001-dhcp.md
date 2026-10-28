# Build (run 4, phone hotspot)

| | |
|---|---|
| Source | Upstream `sirfonzie/mtk3smp-rp2040` `main` @ `2e8e6ad`, plus the PF-001 test code (`pf001-test-instrumentation.diff`, never part of the PR), in the local worktree `mtk3smp-rp2040-run4` (branch `test/pf001-run4`, not pushed) |
| Profile | DHCP profile: `make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 -j8` → `mtk3pico_smp1_usb_cdc_wifi_dhcp.uf2` |
| Fix switch | `PF001_APPLY_FIX` in `kernel/sysdepend/pico_rp2040/cpu_clock.c`: `0` → `pf001_run4_dhcp_fix0.uf2`, `1` → `pf001_run4_dhcp_fix1.uf2` |
| Wi-Fi config | `config/wifi_credentials.h` (gitignored): SSID `Test1` (phone hotspot), `UTK_WIFI_AUTH = CYW43_AUTH_WPA3_WPA2_AES_PSK` |
| Toolchain | Docker image from this repo's `Dockerfile`: Arm GNU Toolchain 13.2.rel1, Pico SDK 2.2.0 |
| Built | 2026-10-19, no warnings |

# Used SerialMonitor on USB, buad rate 115200
## Reflash methodology
1. Remove USB from Pico 
2. Hold BOOTSEL on unpowered Pico
3. Insert USB into Pico
4. Copy pf001_run4_dhcp_fix0 into folder that pops up

# Network details (Taken from windows PC connected to same hotspot as pico)
- Conductecd on Android Phone Hotspot
- Security: WPA2 Personal
- Encyption: AES
- 2.4Ghz Only
- Radio type: 802.11n
- SSID: Test1

# 1st Boot after flash pf001_run4_dhcp_fix0.uf2
[23:13:09.807] === uT-Kernel 3.0 / RP2040 demo ===
[23:13:09.810] SMP build: 2 processors
[23:13:09.814] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:13:09.822] [pf001] upstream main, fix compiled OUT
[23:13:09.824] [pf001] tick_at_boot=0x460c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[23:13:09.825] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:13:10.002] [wifi] link DOWN
[23:13:16.026] [wifi] link UP  rssi=-59  mac=d8:3a:dd:ae:4b:eb
[23:13:19.848] [pf001] window 1/6: kernel=10010 ms timer=10002793 us -> 999.280 us/ms   cumulative 999.280 us/ms
[23:13:22.043] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[23:13:29.845] [pf001] window 2/6: kernel=10010 ms timer=10009947 us -> 999.994 us/ms   cumulative 999.637 us/ms
[23:13:39.869] [pf001] window 3/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[23:13:49.866] [pf001] window 4/6: kernel=10010 ms timer=10009966 us -> 999.996 us/ms   cumulative 999.818 us/ms
[23:13:59.886] [pf001] window 5/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.855 us/ms
[23:14:09.881] [pf001] window 6/6: kernel=10010 ms timer=10009968 us -> 999.996 us/ms   cumulative 999.879 us/ms
[23:14:09.883] [pf001] done: fix compiled OUT, tick cycles now=12
[23:14:09.899] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:14:11.906] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 2nd Try Reflash
[23:15:28.398] === uT-Kernel 3.0 / RP2040 demo ===
[23:15:28.399] SMP build: 2 processors
[23:15:28.401] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:15:28.410] [pf001] upstream main, fix compiled OUT
[23:15:28.411] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x4e0c (enable=1 cycles=12 running=1)
[23:15:28.412] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:15:28.597] [wifi] link DOWN
[23:15:34.610] [wifi] link UP  rssi=-50  mac=d8:3a:dd:ae:4b:eb
[23:15:38.429] [pf001] window 1/6: kernel=10010 ms timer=10002757 us -> 999.276 us/ms   cumulative 999.276 us/ms
[23:15:43.650] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[23:15:48.433] [pf001] window 2/6: kernel=10010 ms timer=10009990 us -> 999.999 us/ms   cumulative 999.637 us/ms
[23:15:58.451] [pf001] window 3/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[23:16:08.449] [pf001] window 4/6: kernel=10010 ms timer=10009961 us -> 999.996 us/ms   cumulative 999.818 us/ms
[23:16:18.472] [pf001] window 5/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.855 us/ms
[23:16:28.471] [pf001] window 6/6: kernel=10010 ms timer=10009976 us -> 999.997 us/ms   cumulative 999.879 us/ms
[23:16:28.472] [pf001] done: fix compiled OUT, tick cycles now=12
[23:16:28.487] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:16:30.499] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


## 3rd try reflash
[23:16:55.148] === uT-Kernel 3.0 / RP2040 demo ===
[23:16:55.150] SMP build: 2 processors
[23:16:55.152] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:16:55.165] [pf001] upstream main, fix compiled OUT
[23:16:55.167] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x4e0c (enable=1 cycles=12 running=1)
[23:16:55.169] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:16:55.354] [wifi] link DOWN
[23:17:05.191] [pf001] window 1/6: kernel=10010 ms timer=10002792 us -> 999.279 us/ms   cumulative 999.279 us/ms
[23:17:15.192] [pf001] window 2/6: kernel=10010 ms timer=10009940 us -> 999.994 us/ms   cumulative 999.636 us/ms
[23:17:25.213] [pf001] window 3/6: kernel=10010 ms timer=10010035 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[23:17:35.209] [pf001] window 4/6: kernel=10010 ms timer=10009969 us -> 999.996 us/ms   cumulative 999.818 us/ms
[23:17:45.226] [pf001] window 5/6: kernel=10010 ms timer=10010028 us -> 1000.002 us/ms   cumulative 999.855 us/ms
[23:17:55.229] [pf001] window 6/6: kernel=10010 ms timer=10009973 us -> 999.997 us/ms   cumulative 999.879 us/ms
[23:17:55.230] [pf001] done: fix compiled OUT, tick cycles now=12
[23:17:55.242] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:17:57.243] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[23:17:59.271] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[23:18:01.269] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
[23:18:03.283] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[23:18:05.284] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2
[23:18:07.310] [monitor] seq=143 produced=143 consumed=143 dropped=0  made_on=prc1 seen_on=prc2
[23:18:09.304] [monitor] seq=147 produced=147 consumed=147 dropped=0  made_on=prc1 seen_on=prc2
[23:18:11.328] [monitor] seq=151 produced=151 consumed=151 dropped=0  made_on=prc1 seen_on=prc2
[23:18:13.327] [monitor] seq=155 produced=155 consumed=155 dropped=0  made_on=prc1 seen_on=prc2
[23:18:15.344] [monitor] seq=159 produced=159 consumed=159 dropped=0  made_on=prc1 seen_on=prc2
[23:18:17.349] [monitor] seq=163 produced=163 consumed=163 dropped=0  made_on=prc1 seen_on=prc2
[23:18:19.369] [monitor] seq=167 produced=167 consumed=167 dropped=0  made_on=prc1 seen_on=prc2
[23:18:21.368] [monitor] seq=171 produced=171 consumed=171 dropped=0  made_on=prc1 seen_on=prc2
[23:18:23.384] [monitor] seq=175 produced=175 consumed=175 dropped=0  made_on=prc1 seen_on=prc2
[23:18:25.385] [monitor] seq=179 produced=179 consumed=179 dropped=0  made_on=prc1 seen_on=prc2
[23:18:27.409] [monitor] seq=183 produced=183 consumed=183 dropped=0  made_on=prc1 seen_on=prc2
[23:18:29.404] [monitor] seq=187 produced=187 consumed=187 dropped=0  made_on=prc1 seen_on=prc2
[23:18:31.431] [monitor] seq=191 produced=191 consumed=191 dropped=0  made_on=prc1 seen_on=prc2
[23:18:33.425] [monitor] seq=195 produced=195 consumed=195 dropped=0  made_on=prc1 seen_on=prc2
[23:18:35.441] [monitor] seq=198 produced=198 consumed=198 dropped=0  made_on=prc1 seen_on=prc2
[23:18:37.443] [monitor] seq=202 produced=202 consumed=202 dropped=0  made_on=prc1 seen_on=prc2
[23:18:39.470] [monitor] seq=206 produced=206 consumed=206 dropped=0  made_on=prc1 seen_on=prc2
[23:18:41.469] [monitor] seq=210 produced=210 consumed=210 dropped=0  made_on=prc1 seen_on=prc2
[23:18:43.488] [monitor] seq=214 produced=214 consumed=214 dropped=0  made_on=prc1 seen_on=prc2
[23:18:45.481] [monitor] seq=218 produced=218 consumed=218 dropped=0  made_on=prc1 seen_on=prc2
[23:18:47.501] [monitor] seq=222 produced=222 consumed=222 dropped=0  made_on=prc1 seen_on=prc2
[23:18:49.509] [monitor] seq=226 produced=226 consumed=226 dropped=0  made_on=prc1 seen_on=prc2
[23:18:51.522] [monitor] seq=230 produced=230 consumed=230 dropped=0  made_on=prc1 seen_on=prc2
[23:18:53.520] [monitor] seq=234 produced=234 consumed=234 dropped=0  made_on=prc1 seen_on=prc2
[23:18:55.540] [monitor] seq=238 produced=238 consumed=238 dropped=0  made_on=prc1 seen_on=prc2
[23:18:57.542] [monitor] seq=242 produced=242 consumed=242 dropped=0  made_on=prc1 seen_on=prc2
[23:18:59.566] [monitor] seq=246 produced=246 consumed=246 dropped=0  made_on=prc1 seen_on=prc2
[23:19:01.564] [monitor] seq=250 produced=250 consumed=250 dropped=0  made_on=prc1 seen_on=prc2
[23:19:03.583] [monitor] seq=254 produced=254 consumed=254 dropped=0  made_on=prc1 seen_on=prc2
[23:19:05.582] [monitor] seq=258 produced=258 consumed=258 dropped=0  made_on=prc1 seen_on=prc2
[23:19:07.605] [monitor] seq=262 produced=262 consumed=262 dropped=0  made_on=prc1 seen_on=prc2

## 4th try reflash
[23:19:35.817] [wifi] link UP  rssi=-51  mac=d8:3a:dd:ae:4b:eb
[23:19:39.634] [pf001] window 1/6: kernel=10010 ms timer=10002810 us -> 999.281 us/ms   cumulative 999.281 us/ms
[23:19:44.848] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[23:19:49.635] [pf001] window 2/6: kernel=10010 ms timer=10009955 us -> 999.995 us/ms   cumulative 999.638 us/ms
[23:19:59.651] [pf001] window 3/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.760 us/ms
[23:20:09.647] [pf001] window 4/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.819 us/ms
[23:20:19.675] [pf001] window 5/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.856 us/ms
[23:20:29.670] [pf001] window 6/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.879 us/ms
[23:20:29.671] [pf001] done: fix compiled OUT, tick cycles now=12
[23:20:29.685] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:20:31.688] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[23:20:33.713] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[23:20:35.709] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
[23:20:37.729] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[23:20:39.726] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2
[23:20:41.754] [monitor] seq=143 produced=143 consumed=143 dropped=0  made_on=prc1 seen_on=prc2
[23:20:43.748] [monitor] seq=147 produced=147 consumed=147 dropped=0  made_on=prc1 seen_on=prc2
[23:20:45.773] [monitor] seq=151 produced=151 consumed=151 dropped=0  made_on=prc1 seen_on=prc2
[23:20:47.772] [monitor] seq=155 produced=155 consumed=155 dropped=0  made_on=prc1 seen_on=prc2
[23:20:49.785] [monitor] seq=159 produced=159 consumed=159 dropped=0  made_on=prc1 seen_on=prc2
[23:20:51.792] [monitor] seq=163 produced=163 consumed=163 dropped=0  made_on=prc1 seen_on=prc2
[23:20:53.808] [monitor] seq=167 produced=167 consumed=167 dropped=0  made_on=prc1 seen_on=prc2
[23:20:55.812] [monitor] seq=171 produced=171 consumed=171 dropped=0  made_on=prc1 seen_on=prc2
[23:20:57.827] [monitor] seq=175 produced=175 consumed=175 dropped=0  made_on=prc1 seen_on=prc2

# 5th try reflash
[23:21:17.114] === uT-Kernel 3.0 / RP2040 demo ===
[23:21:17.131] SMP build: 2 processors
[23:21:17.133] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:21:17.135] [pf001] upstream main, fix compiled OUT
[23:21:17.141] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[23:21:17.152] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:21:17.339] [wifi] link DOWN
[23:21:27.160] [pf001] window 1/6: kernel=10010 ms timer=10002746 us -> 999.275 us/ms   cumulative 999.275 us/ms
[23:21:37.175] [pf001] window 2/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.638 us/ms
[23:21:47.178] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.758 us/ms
[23:21:57.195] [pf001] window 4/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[23:22:07.194] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.855 us/ms
[23:22:17.213] [pf001] window 6/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.879 us/ms
[23:22:17.215] [pf001] done: fix compiled OUT, tick cycles now=12
[23:22:17.216] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:22:19.231] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[23:22:21.239] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

# 6th try reflash
[23:27:28.684] [wifi] link UP  rssi=-40  mac=d8:3a:dd:ae:4b:eb
[23:27:32.489] [pf001] window 1/6: kernel=10010 ms timer=10002748 us -> 999.275 us/ms   cumulative 999.275 us/ms
[23:27:34.708] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[23:27:42.512] [pf001] window 2/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.638 us/ms
[23:27:52.505] [pf001] window 3/6: kernel=10010 ms timer=10009979 us -> 999.997 us/ms   cumulative 999.758 us/ms
[23:28:02.529] [pf001] window 4/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[23:28:12.523] [pf001] window 5/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.855 us/ms
[23:28:22.549] [pf001] window 6/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.879 us/ms
[23:28:22.550] [pf001] done: fix compiled OUT, tick cycles now=12
[23:28:22.553] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:28:24.563] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# 7th try reflash
[23:29:34.443] === uT-Kernel 3.0 / RP2040 demo ===
[23:29:34.461] SMP build: 2 processors
[23:29:34.463] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:29:34.468] [pf001] upstream main, fix compiled OUT
[23:29:34.476] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[23:29:34.481] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:29:34.667] [wifi] link DOWN
[23:29:40.694] [wifi] link UP  rssi=-41  mac=d8:3a:dd:ae:4b:eb
[23:29:44.489] [pf001] window 1/6: kernel=10010 ms timer=10002820 us -> 999.282 us/ms   cumulative 999.282 us/ms
[23:29:49.701] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[23:29:54.510] [pf001] window 2/6: kernel=10010 ms timer=10009947 us -> 999.994 us/ms   cumulative 999.638 us/ms
[23:30:04.500] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.758 us/ms
[23:30:14.529] [pf001] window 4/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[23:30:24.519] [pf001] window 5/6: kernel=10010 ms timer=10009981 us -> 999.998 us/ms   cumulative 999.855 us/ms
[23:30:34.543] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.879 us/ms
[23:30:34.545] [pf001] done: fix compiled OUT, tick cycles now=12
[23:30:34.547] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:30:36.561] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

# 8th try reflash
[23:39:31.899] === uT-Kernel 3.0 / RP2040 demo ===
[23:39:31.904] SMP build: 2 processors
[23:39:31.908] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:39:31.909] [pf001] upstream main, fix compiled OUT
[23:39:31.912] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[23:39:31.915] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:39:32.092] [wifi] link DOWN
[23:39:41.934] [pf001] window 1/6: kernel=10010 ms timer=10002780 us -> 999.278 us/ms   cumulative 999.278 us/ms
[23:39:51.931] [pf001] window 2/6: kernel=10010 ms timer=10009951 us -> 999.995 us/ms   cumulative 999.636 us/ms
[23:40:01.950] [pf001] window 3/6: kernel=10010 ms timer=10010029 us -> 1000.002 us/ms   cumulative 999.758 us/ms
[23:40:11.957] [pf001] window 4/6: kernel=10010 ms timer=10009973 us -> 999.997 us/ms   cumulative 999.818 us/ms
[23:40:21.974] [pf001] window 5/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.855 us/ms
[23:40:31.974] [pf001] window 6/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.878 us/ms
[23:40:31.975] [pf001] done: fix compiled OUT, tick cycles now=12
[23:40:31.988] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:40:33.995] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[23:40:36.014] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

# 9th try reflash
[23:41:15.517] === uT-Kernel 3.0 / RP2040 demo ===
[23:41:15.523] SMP build: 2 processors
[23:41:15.525] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:41:15.528] [pf001] upstream main, fix compiled OUT
[23:41:15.531] [pf001] tick_at_boot=0x3e0c (enable=1 cycles=12 running=1)  tick_now=0x3e0c (enable=1 cycles=12 running=1)
[23:41:15.533] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:41:15.709] [wifi] link DOWN
[23:41:21.731] [wifi] link UP  rssi=-52  mac=d8:3a:dd:ae:4b:eb
[23:41:25.546] [pf001] window 1/6: kernel=10010 ms timer=10002799 us -> 999.280 us/ms   cumulative 999.280 us/ms
[23:41:27.746] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[23:41:35.547] [pf001] window 2/6: kernel=10010 ms timer=10009949 us -> 999.994 us/ms   cumulative 999.637 us/ms
[23:41:45.563] [pf001] window 3/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.759 us/ms
[23:41:55.563] [pf001] window 4/6: kernel=10010 ms timer=10009965 us -> 999.996 us/ms   cumulative 999.818 us/ms
[23:42:05.586] [pf001] window 5/6: kernel=10010 ms timer=10010032 us -> 1000.003 us/ms   cumulative 999.855 us/ms
[23:42:15.585] [pf001] window 6/6: kernel=10010 ms timer=10009968 us -> 999.996 us/ms   cumulative 999.879 us/ms
[23:42:15.586] [pf001] done: fix compiled OUT, tick cycles now=12
[23:42:15.603] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:42:17.607] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[23:42:19.627] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[23:42:21.628] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
[23:42:23.648] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[23:42:25.644] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2
[23:42:27.663] [monitor] seq=143 produced=143 consumed=143 dropped=0  made_on=prc1 seen_on=prc2
[23:42:29.669] [monitor] seq=147 produced=147 consumed=147 dropped=0  made_on=prc1 seen_on=prc2
[23:42:31.686] [monitor] seq=151 produced=151 consumed=151 dropped=0  made_on=prc1 seen_on=prc2
[23:42:33.686] [monitor] seq=155 produced=155 consumed=155 dropped=0  made_on=prc1 seen_on=prc2
[23:42:35.703] [monitor] seq=159 produced=159 consumed=159 dropped=0  made_on=prc1 seen_on=prc2
[23:42:37.705] [monitor] seq=163 produced=163 consumed=163 dropped=0  made_on=prc1 seen_on=prc2
[23:42:39.724] [monitor] seq=167 produced=167 consumed=167 dropped=0  made_on=prc1 seen_on=prc2
[23:42:41.729] [monitor] seq=171 produced=171 consumed=171 dropped=0  made_on=prc1 seen_on=prc2
[23:42:43.743] [monitor] seq=175 produced=175 consumed=175 dropped=0  made_on=prc1 seen_on=prc2
[23:42:45.748] [monitor] seq=179 produced=179 consumed=179 dropped=0  made_on=prc1 seen_on=prc2
[23:42:47.763] [monitor] seq=183 produced=183 consumed=183 dropped=0  made_on=prc1 seen_on=prc2
[23:42:49.767] [monitor] seq=187 produced=187 consumed=187 dropped=0  made_on=prc1 seen_on=prc2
[23:42:51.790] [monitor] seq=191 produced=191 consumed=191 dropped=0  made_on=prc1 seen_on=prc2
[23:42:53.789] [monitor] seq=195 produced=195 consumed=195 dropped=0  made_on=prc1 seen_on=prc2
[23:42:55.805] [monitor] seq=198 produced=198 consumed=198 dropped=0  made_on=prc1 seen_on=prc2
[23:42:57.810] [monitor] seq=202 produced=202 consumed=202 dropped=0  made_on=prc1 seen_on=prc2
[23:42:59.831] [monitor] seq=206 produced=206 consumed=206 dropped=0  made_on=prc1 seen_on=prc2
[23:43:01.830] [monitor] seq=210 produced=210 consumed=210 dropped=0  made_on=prc1 seen_on=prc2
[23:43:03.844] [monitor] seq=214 produced=214 consumed=214 dropped=0  made_on=prc1 seen_on=prc2
[23:43:05.848] [monitor] seq=218 produced=218 consumed=218 dropped=0  made_on=prc1 seen_on=prc2
[23:43:07.869] [monitor] seq=222 produced=222 consumed=222 dropped=0  made_on=prc1 seen_on=prc2
[23:43:09.869] [monitor] seq=226 produced=226 consumed=226 dropped=0  made_on=prc1 seen_on=prc2
[23:43:11.884] [monitor] seq=230 produced=230 consumed=230 dropped=0  made_on=prc1 seen_on=prc2
[23:43:13.886] [monitor] seq=234 produced=234 consumed=234 dropped=0  made_on=prc1 seen_on=prc2
[23:43:15.907] [monitor] seq=238 produced=238 consumed=238 dropped=0  made_on=prc1 seen_on=prc2
[23:43:17.907] [monitor] seq=242 produced=242 consumed=242 dropped=0  made_on=prc1 seen_on=prc2
[23:43:19.923] [monitor] seq=246 produced=246 consumed=246 dropped=0  made_on=prc1 seen_on=prc2
[23:43:21.927] [monitor] seq=250 produced=250 consumed=250 dropped=0  made_on=prc1 seen_on=prc2
[23:43:23.939] [monitor] seq=254 produced=254 consumed=254 dropped=0  made_on=prc1 seen_on=prc2
[23:43:25.938] [monitor] seq=258 produced=258 consumed=258 dropped=0  made_on=prc1 seen_on=prc2
[23:43:27.961] [monitor] seq=262 produced=262 consumed=262 dropped=0  made_on=prc1 seen_on=prc2
[23:43:29.964] [monitor] seq=265 produced=265 consumed=265 dropped=0  made_on=prc1 seen_on=prc2

# 10th try reflash
[23:43:43.157] === uT-Kernel 3.0 / RP2040 demo ===
[23:43:43.168] SMP build: 2 processors
[23:43:43.169] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:43:43.170] [pf001] upstream main, fix compiled OUT
[23:43:43.171] [pf001] tick_at_boot=0x5e0c (enable=1 cycles=12 running=1)  tick_now=0x260c (enable=1 cycles=12 running=1)
[23:43:43.188] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:43:43.375] [wifi] link DOWN
[23:43:53.190] [pf001] window 1/6: kernel=10010 ms timer=10002748 us -> 999.275 us/ms   cumulative 999.275 us/ms
[23:44:03.208] [pf001] window 2/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.638 us/ms
[23:44:13.215] [pf001] window 3/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.758 us/ms
[23:44:23.226] [pf001] window 4/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.819 us/ms
[23:44:33.227] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.855 us/ms
[23:44:43.255] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.879 us/ms
[23:44:43.257] [pf001] done: fix compiled OUT, tick cycles now=12
[23:44:43.258] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[23:44:45.265] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[23:44:47.270] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[23:44:49.292] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
[23:44:51.288] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[23:44:53.309] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2
[23:44:55.311] [monitor] seq=143 produced=143 consumed=143 dropped=0  made_on=prc1 seen_on=prc2
[23:44:57.331] [monitor] seq=147 produced=147 consumed=147 dropped=0  made_on=prc1 seen_on=prc2
[23:44:59.327] [monitor] seq=151 produced=151 consumed=151 dropped=0  made_on=prc1 seen_on=prc2
[23:45:01.345] [monitor] seq=155 produced=155 consumed=155 dropped=0  made_on=prc1 seen_on=prc2
[23:45:03.351] [monitor] seq=159 produced=159 consumed=159 dropped=0  made_on=prc1 seen_on=prc2
[23:45:05.367] [monitor] seq=163 produced=163 consumed=163 dropped=0  made_on=prc1 seen_on=prc2
[23:45:07.371] [monitor] seq=167 produced=167 consumed=167 dropped=0  made_on=prc1 seen_on=prc2
[23:45:09.387] [monitor] seq=171 produced=171 consumed=171 dropped=0  made_on=prc1 seen_on=prc2
[23:45:11.388] [monitor] seq=175 produced=175 consumed=175 dropped=0  made_on=prc1 seen_on=prc2
[23:45:13.408] [monitor] seq=179 produced=179 consumed=179 dropped=0  made_on=prc1 seen_on=prc2
[23:45:15.411] [monitor] seq=183 produced=183 consumed=183 dropped=0  made_on=prc1 seen_on=prc2
[23:45:17.426] [monitor] seq=187 produced=187 consumed=187 dropped=0  made_on=prc1 seen_on=prc2
[23:45:19.431] [monitor] seq=191 produced=191 consumed=191 dropped=0  made_on=prc1 seen_on=prc2
[23:45:21.448] [monitor] seq=195 produced=195 consumed=195 dropped=0  made_on=prc1 seen_on=prc2
[23:45:23.447] [monitor] seq=199 produced=198 consumed=198 dropped=0  made_on=prc1 seen_on=prc2
[23:45:25.472] [monitor] seq=202 produced=202 consumed=202 dropped=0  made_on=prc1 seen_on=prc2
[23:45:27.469] [monitor] seq=206 produced=206 consumed=206 dropped=0  made_on=prc1 seen_on=prc2

## Power-cycle methodology
1. unplug USB A that plugs into PC to power down pico
2. Replug USB into PC to power up pico
3. Approx 1-2 secs between plugging and unplugging USB

## 1st power-cycle (after reflash)
[23:46:21.300] === uT-Kernel 3.0 / RP2040 demo ===
[23:46:21.314] SMP build: 2 processors
[23:46:21.315] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:46:21.316] [pf001] upstream main, fix compiled OUT
[23:46:21.317] [pf001] tick_at_boot=0xa0e00 (enable=1 cycles=0 running=1)  tick_now=0xba600 (enable=1 cycles=0 running=1)
[23:46:21.334] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:46:21.520] [wifi] link DOWN
[23:46:31.341] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:46:33.556] [wifi] link UP  rssi=-42  mac=d8:3a:dd:ae:4b:eb
[23:46:41.363] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:46:51.361] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:47:01.380] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:47:11.373] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:47:21.395] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:47:21.396] [pf001] done: fix compiled OUT, tick cycles now=0
[23:47:21.399] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:47:23.422] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:47:25.414] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[23:47:27.438] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2
[23:47:29.437] [monitor] seq=125 produced=125 consumed=125 dropped=0  made_on=prc1 seen_on=prc2

## 2nd power-cycle
[23:47:38.079] === uT-Kernel 3.0 / RP2040 demo ===
[23:47:38.093] SMP build: 2 processors
[23:47:38.095] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:47:38.096] [pf001] upstream main, fix compiled OUT
[23:47:38.099] [pf001] tick_at_boot=0xece00 (enable=1 cycles=0 running=1)  tick_now=0xa4e00 (enable=1 cycles=0 running=1)
[23:47:38.114] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:47:38.301] [wifi] link DOWN
[23:47:48.120] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:47:58.141] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:48:08.136] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:48:18.155] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:48:28.156] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:48:38.179] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:48:38.180] [pf001] done: fix compiled OUT, tick cycles now=0
[23:48:38.182] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:48:40.200] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:48:42.197] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[23:48:44.216] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2
[23:48:46.217] [monitor] seq=125 produced=125 consumed=125 dropped=0  made_on=prc1 seen_on=prc2

## 3rd power-cycle
[23:48:52.824] === uT-Kernel 3.0 / RP2040 demo ===
[23:48:52.825] SMP build: 2 processors
[23:48:52.825] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:48:52.835] [pf001] upstream main, fix compiled OUT
[23:48:52.836] [pf001] tick_at_boot=0xdbe00 (enable=1 cycles=0 running=1)  tick_now=0x42600 (enable=1 cycles=0 running=1)
[23:48:52.837] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:48:53.022] [wifi] link DOWN
[23:49:02.857] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:49:05.055] [wifi] link UP  rssi=-60  mac=d8:3a:dd:ae:4b:eb
[23:49:12.860] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:49:22.884] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:49:32.887] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:49:42.898] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:49:52.902] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:49:52.903] [pf001] done: fix compiled OUT, tick cycles now=0
[23:49:52.912] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:49:54.913] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:49:56.935] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[23:49:58.939] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2

## 4th power-cycle
[23:50:09.074] [wifi] link DOWN
[23:50:18.888] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:50:28.913] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:50:38.907] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:50:48.925] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:50:58.926] [pf001] window 5/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:51:08.944] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:51:08.946] [pf001] done: fix compiled OUT, tick cycles now=0
[23:51:08.946] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:51:10.971] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:51:12.965] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2

## 5th power-cycle
[23:51:22.681] [wifi] link DOWN
[23:51:32.503] [pf001] window 1/6: kernel=10010 ms timer=234439 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:51:34.726] [wifi] link UP  rssi=-50  mac=d8:3a:dd:ae:4b:eb
[23:51:42.527] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:51:52.519] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:52:02.545] [pf001] window 4/6: kernel=10010 ms timer=234611 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:52:12.541] [pf001] window 5/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:52:22.565] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:52:22.566] [pf001] done: fix compiled OUT, tick cycles now=0
[23:52:22.567] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:52:24.578] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:52:26.577] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2

## 6th power-cycle
[23:52:35.162] [wifi] link DOWN
[23:52:45.005] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:52:55.001] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:53:05.017] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:53:15.019] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:53:25.042] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:53:35.036] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:53:35.038] [pf001] done: fix compiled OUT, tick cycles now=0
[23:53:35.053] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:53:37.056] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:53:39.078] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[23:53:41.079] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2
## 7th power-cycle
[23:53:50.749] [wifi] link DOWN
[23:54:00.564] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:54:02.787] [wifi] link UP  rssi=-48  mac=d8:3a:dd:ae:4b:eb
[23:54:10.592] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:54:20.590] [pf001] window 3/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:54:30.607] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:54:40.605] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:54:50.623] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:54:50.624] [pf001] done: fix compiled OUT, tick cycles now=0
[23:54:50.626] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:54:52.642] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:54:54.646] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[23:54:56.665] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2


## 8th power-cycle
[23:55:04.490] === uT-Kernel 3.0 / RP2040 demo ===
[23:55:04.490] SMP build: 2 processors
[23:55:04.491] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:55:04.506] [pf001] upstream main, fix compiled OUT
[23:55:04.507] [pf001] tick_at_boot=0x61e00 (enable=1 cycles=0 running=1)  tick_now=0x8a600 (enable=1 cycles=0 running=1)
[23:55:04.508] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:55:04.694] [wifi] link DOWN
[23:55:14.530] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:55:24.535] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:55:34.548] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:55:44.550] [pf001] window 4/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:55:54.574] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:56:04.574] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:56:04.575] [pf001] done: fix compiled OUT, tick cycles now=0
[23:56:04.583] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:56:06.593] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:56:08.604] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2

## 9th power-cycle
[23:56:18.089] === uT-Kernel 3.0 / RP2040 demo ===
[23:56:18.092] SMP build: 2 processors
[23:56:18.095] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:56:18.096] [pf001] upstream main, fix compiled OUT
[23:56:18.108] [pf001] tick_at_boot=0xcce00 (enable=1 cycles=0 running=1)  tick_now=0xebe00 (enable=1 cycles=0 running=1)
[23:56:18.110] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:56:18.285] [wifi] link DOWN
[23:56:28.130] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:56:30.329] [wifi] link UP  rssi=-49  mac=d8:3a:dd:ae:4b:eb
[23:56:38.129] [pf001] window 2/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:56:48.142] [pf001] window 3/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:56:58.147] [pf001] window 4/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:57:08.163] [pf001] window 5/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:57:18.163] [pf001] window 6/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:57:18.166] [pf001] done: fix compiled OUT, tick cycles now=0
[23:57:18.179] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:57:20.182] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:57:22.203] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[23:57:24.206] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2

## 10th power-cycle
[23:57:33.519] === uT-Kernel 3.0 / RP2040 demo ===
[23:57:33.536] SMP build: 2 processors
[23:57:33.539] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[23:57:33.541] [pf001] upstream main, fix compiled OUT
[23:57:33.543] [pf001] tick_at_boot=0x1ae00 (enable=1 cycles=0 running=1)  tick_now=0xc2e00 (enable=1 cycles=0 running=1)
[23:57:33.556] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[23:57:33.742] [wifi] link DOWN
[23:57:43.557] [pf001] window 1/6: kernel=10010 ms timer=234440 us -> 23.420 us/ms   cumulative 23.420 us/ms
[23:57:53.581] [pf001] window 2/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.429 us/ms
[23:58:03.578] [pf001] window 3/6: kernel=10010 ms timer=234608 us -> 23.437 us/ms   cumulative 23.431 us/ms
[23:58:13.595] [pf001] window 4/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.433 us/ms
[23:58:23.604] [pf001] window 5/6: kernel=10010 ms timer=234609 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:58:33.616] [pf001] window 6/6: kernel=10010 ms timer=234610 us -> 23.437 us/ms   cumulative 23.434 us/ms
[23:58:33.617] [pf001] done: fix compiled OUT, tick cycles now=0
[23:58:33.619] [monitor] seq=109 produced=109 consumed=109 dropped=0  made_on=prc1 seen_on=prc2
[23:58:35.635] [monitor] seq=113 produced=113 consumed=113 dropped=0  made_on=prc1 seen_on=prc2
[23:58:37.639] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2

# 1st Boot after flash pf001_run4_dhcp_fix1.uf2
[23:59:57.386] [wifi] link UP  rssi=-55  mac=d8:3a:dd:ae:4b:eb
[00:00:01.196] [pf001] window 1/6: kernel=10010 ms timer=10002824 us -> 999.283 us/ms   cumulative 999.283 us/ms
[00:00:06.410] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:00:11.210] [pf001] window 2/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[00:00:21.212] [pf001] window 3/6: kernel=10010 ms timer=10009976 us -> 999.997 us/ms   cumulative 999.760 us/ms
[00:00:31.232] [pf001] window 4/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[00:00:41.228] [pf001] window 5/6: kernel=10010 ms timer=10009981 us -> 999.998 us/ms   cumulative 999.856 us/ms
[00:00:51.250] [pf001] window 6/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.880 us/ms
[00:00:51.251] [pf001] done: fix compiled IN, tick cycles now=12
[00:00:51.253] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:00:53.272] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:00:55.274] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:00:57.285] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2


## 2nd Try Reflash
[00:01:18.286] === uT-Kernel 3.0 / RP2040 demo ===
[00:01:18.295] SMP build: 2 processors
[00:01:18.296] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:01:18.297] [pf001] upstream main, fix compiled IN
[00:01:18.300] [pf001] tick_at_boot=0x660c (enable=1 cycles=12 running=1)  tick_now=0x1e0c (enable=1 cycles=12 running=1)
[00:01:18.314] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:01:18.500] [wifi] link DOWN
[00:01:28.315] [pf001] window 1/6: kernel=10010 ms timer=10002833 us -> 999.284 us/ms   cumulative 999.284 us/ms
[00:01:38.333] [pf001] window 2/6: kernel=10010 ms timer=10010017 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[00:01:48.339] [pf001] window 3/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.761 us/ms
[00:01:58.361] [pf001] window 4/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[00:02:08.355] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.856 us/ms
[00:02:18.375] [pf001] window 6/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.880 us/ms
[00:02:18.376] [pf001] done: fix compiled IN, tick cycles now=12
[00:02:18.377] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:02:20.397] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:02:22.391] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

## 3rd Try Reflash
[00:02:50.331] [wifi] link UP  rssi=-47  mac=d8:3a:dd:ae:4b:eb
[00:02:54.132] [pf001] window 1/6: kernel=10010 ms timer=10002834 us -> 999.284 us/ms   cumulative 999.284 us/ms
[00:02:56.347] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:03:04.149] [pf001] window 2/6: kernel=10010 ms timer=10010015 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[00:03:14.151] [pf001] window 3/6: kernel=10010 ms timer=10009982 us -> 999.998 us/ms   cumulative 999.761 us/ms
[00:03:24.173] [pf001] window 4/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[00:03:34.165] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.856 us/ms
[00:03:44.189] [pf001] window 6/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.880 us/ms
[00:03:44.190] [pf001] done: fix compiled IN, tick cycles now=12
[00:03:44.191] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:03:46.207] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:03:48.211] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:03:50.228] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
## 4th Try Reflash
[00:04:11.762] === uT-Kernel 3.0 / RP2040 demo ===
[00:04:11.763] SMP build: 2 processors
[00:04:11.771] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:04:11.774] [pf001] upstream main, fix compiled IN
[00:04:11.777] [pf001] tick_at_boot=0x660c (enable=1 cycles=12 running=1)  tick_now=0x460c (enable=1 cycles=12 running=1)
[00:04:11.778] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:04:11.956] [wifi] link DOWN
[00:04:21.793] [pf001] window 1/6: kernel=10010 ms timer=10002875 us -> 999.288 us/ms   cumulative 999.288 us/ms
[00:04:31.793] [pf001] window 2/6: kernel=10010 ms timer=10009952 us -> 999.995 us/ms   cumulative 999.641 us/ms
[00:04:41.813] [pf001] window 3/6: kernel=10010 ms timer=10010028 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[00:04:51.812] [pf001] window 4/6: kernel=10010 ms timer=10009969 us -> 999.996 us/ms   cumulative 999.820 us/ms
[00:05:01.833] [pf001] window 5/6: kernel=10010 ms timer=10010027 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[00:05:11.833] [pf001] window 6/6: kernel=10010 ms timer=10009975 us -> 999.997 us/ms   cumulative 999.880 us/ms
[00:05:11.834] [pf001] done: fix compiled IN, tick cycles now=12
[00:05:11.850] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:05:13.854] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:05:15.877] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

## 5th Try Reflash
[00:05:41.788] [wifi] link UP  rssi=-39  mac=d8:3a:dd:ae:4b:eb
[00:05:45.601] [pf001] window 1/6: kernel=10010 ms timer=10002901 us -> 999.290 us/ms   cumulative 999.290 us/ms
[00:05:50.827] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:05:55.608] [pf001] window 2/6: kernel=10010 ms timer=10009944 us -> 999.994 us/ms   cumulative 999.642 us/ms
[00:06:05.628] [pf001] window 3/6: kernel=10010 ms timer=10010027 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[00:06:15.626] [pf001] window 4/6: kernel=10010 ms timer=10009975 us -> 999.997 us/ms   cumulative 999.821 us/ms
[00:06:25.650] [pf001] window 5/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.857 us/ms
[00:06:35.646] [pf001] window 6/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.880 us/ms
[00:06:35.647] [pf001] done: fix compiled IN, tick cycles now=12
[00:06:35.660] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:06:37.664] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:06:39.687] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

## 6th Try Reflash
[00:07:03.889] === uT-Kernel 3.0 / RP2040 demo ===
[00:07:03.890] SMP build: 2 processors
[00:07:03.892] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:07:03.893] [pf001] upstream main, fix compiled IN
[00:07:03.905] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[00:07:03.906] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:07:04.081] [wifi] link DOWN
[00:07:13.928] [pf001] window 1/6: kernel=10010 ms timer=10002844 us -> 999.285 us/ms   cumulative 999.285 us/ms
[00:07:23.931] [pf001] window 2/6: kernel=10010 ms timer=10009988 us -> 999.998 us/ms   cumulative 999.641 us/ms
[00:07:33.943] [pf001] window 3/6: kernel=10010 ms timer=10010029 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[00:07:43.942] [pf001] window 4/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.820 us/ms
[00:07:53.968] [pf001] window 5/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[00:08:03.965] [pf001] window 6/6: kernel=10010 ms timer=10009972 us -> 999.997 us/ms   cumulative 999.880 us/ms
[00:08:03.967] [pf001] done: fix compiled IN, tick cycles now=12
[00:08:03.977] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:08:05.985] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:08:08.006] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:08:10.002] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2

## 7th Try Reflash
[00:08:34.568] === uT-Kernel 3.0 / RP2040 demo ===
[00:08:34.587] SMP build: 2 processors
[00:08:34.603] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:08:34.614] [pf001] upstream main, fix compiled IN
[00:08:34.629] [pf001] tick_at_boot=0x660c (enable=1 cycles=12 running=1)  tick_now=0x3e0c (enable=1 cycles=12 running=1)
[00:08:34.640] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:08:34.786] [wifi] link DOWN
[00:08:40.801] [wifi] link UP  rssi=-47  mac=d8:3a:dd:ae:4b:eb
[00:08:44.602] [pf001] window 1/6: kernel=10010 ms timer=10002828 us -> 999.283 us/ms   cumulative 999.283 us/ms
[00:08:46.823] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:08:54.620] [pf001] window 2/6: kernel=10010 ms timer=10010017 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[00:09:04.618] [pf001] window 3/6: kernel=10010 ms timer=10009979 us -> 999.997 us/ms   cumulative 999.761 us/ms
[00:09:14.643] [pf001] window 4/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[00:09:24.638] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.856 us/ms
[00:09:34.662] [pf001] window 6/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.880 us/ms
[00:09:34.665] [pf001] done: fix compiled IN, tick cycles now=12
[00:09:34.676] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:09:36.685] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:09:38.679] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
## 8th Try Reflash
[00:10:26.926] [pf001] window 1/6: kernel=10010 ms timer=10002834 us -> 999.284 us/ms   cumulative 999.284 us/ms
[00:10:36.940] [pf001] window 2/6: kernel=10010 ms timer=10010015 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[00:10:46.941] [pf001] window 3/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.761 us/ms
[00:10:56.960] [pf001] window 4/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.821 us/ms
[00:11:06.964] [pf001] window 5/6: kernel=10010 ms timer=10009979 us -> 999.997 us/ms   cumulative 999.856 us/ms
[00:11:16.980] [pf001] window 6/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.880 us/ms
[00:11:16.981] [pf001] done: fix compiled IN, tick cycles now=12
[00:11:16.982] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:11:19.004] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:11:21.004] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

## 9th Try Reflash
[00:11:38.102] === uT-Kernel 3.0 / RP2040 demo ===
[00:11:38.121] SMP build: 2 processors
[00:11:38.127] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:11:38.135] [pf001] upstream main, fix compiled IN
[00:11:38.143] [pf001] tick_at_boot=0x260c (enable=1 cycles=12 running=1)  tick_now=0x5e0c (enable=1 cycles=12 running=1)
[00:11:38.147] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:11:38.322] [wifi] link DOWN
[00:11:44.340] [wifi] link UP  rssi=-43  mac=d8:3a:dd:ae:4b:eb
[00:11:48.141] [pf001] window 1/6: kernel=10010 ms timer=10002840 us -> 999.284 us/ms   cumulative 999.284 us/ms
[00:11:50.368] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:11:58.172] [pf001] window 2/6: kernel=10010 ms timer=10010017 us -> 1000.001 us/ms   cumulative 999.643 us/ms
[00:12:08.159] [pf001] window 3/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.761 us/ms
[00:12:18.185] [pf001] window 4/6: kernel=10010 ms timer=10010020 us -> 1000.001 us/ms   cumulative 999.821 us/ms
[00:12:28.181] [pf001] window 5/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.856 us/ms
[00:12:38.206] [pf001] window 6/6: kernel=10010 ms timer=10010024 us -> 1000.002 us/ms   cumulative 999.881 us/ms
[00:12:38.207] [pf001] done: fix compiled IN, tick cycles now=12
[00:12:38.208] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:12:40.222] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:12:42.221] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:12:44.245] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2


## 10th Try Reflash

[00:13:01.414] === uT-Kernel 3.0 / RP2040 demo ===
[00:13:01.420] SMP build: 2 processors
[00:13:01.428] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:13:01.433] [pf001] upstream main, fix compiled IN
[00:13:01.437] [pf001] tick_at_boot=0x660c (enable=1 cycles=12 running=1)  tick_now=0x660c (enable=1 cycles=12 running=1)
[00:13:01.440] [pf001] wd_reason=0x1  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:13:01.615] [wifi] link DOWN
[00:13:11.454] [pf001] window 1/6: kernel=10010 ms timer=10002880 us -> 999.288 us/ms   cumulative 999.288 us/ms
[00:13:21.462] [pf001] window 2/6: kernel=10010 ms timer=10009946 us -> 999.994 us/ms   cumulative 999.641 us/ms
[00:13:31.477] [pf001] window 3/6: kernel=10010 ms timer=10010028 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[00:13:41.471] [pf001] window 4/6: kernel=10010 ms timer=10009971 us -> 999.997 us/ms   cumulative 999.820 us/ms
[00:13:51.493] [pf001] window 5/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.857 us/ms
[00:14:01.494] [pf001] window 6/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.880 us/ms
[00:14:01.495] [pf001] done: fix compiled IN, tick cycles now=12
[00:14:01.508] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:14:03.513] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:14:05.535] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

# fix (pf001_run4_dhcp_fix1.uf2): power-cycle
## Power-cycle methodology
1. unplug USB A that plugs into PC to power down pico
2. Replug USB into PC to power up pico
3. Approx 1-2 secs between plugging and unplugging USB

Attempts after the first powercycle it would be sequential, there is no reflash then powercycle other than the first attempt

## 1st power-cycle (after reflash)
[00:15:23.332] === uT-Kernel 3.0 / RP2040 demo ===
[00:15:23.345] SMP build: 2 processors
[00:15:23.346] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:15:23.347] [pf001] upstream main, fix compiled IN
[00:15:23.349] [pf001] tick_at_boot=0x54600 (enable=1 cycles=0 running=1)  tick_now=0x2e0c (enable=1 cycles=12 running=1)
[00:15:23.364] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:15:23.551] [wifi] link DOWN
[00:15:29.573] [wifi] link UP  rssi=-44  mac=d8:3a:dd:ae:4b:eb
[00:15:33.366] [pf001] window 1/6: kernel=10010 ms timer=10002821 us -> 999.282 us/ms   cumulative 999.282 us/ms
[00:15:35.586] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:15:43.391] [pf001] window 2/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.643 us/ms
[00:15:53.386] [pf001] window 3/6: kernel=10010 ms timer=10009962 us -> 999.996 us/ms   cumulative 999.760 us/ms
[00:16:03.407] [pf001] window 4/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.821 us/ms
[00:16:13.409] [pf001] window 5/6: kernel=10010 ms timer=10009964 us -> 999.996 us/ms   cumulative 999.856 us/ms
[00:16:23.433] [pf001] window 6/6: kernel=10010 ms timer=10010036 us -> 1000.003 us/ms   cumulative 999.881 us/ms
[00:16:23.434] [pf001] done: fix compiled IN, tick cycles now=12
[00:16:23.436] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:16:25.446] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:16:27.444] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
## 2nd power-cycle
[00:16:36.134] === uT-Kernel 3.0 / RP2040 demo ===
[00:16:36.135] SMP build: 2 processors
[00:16:36.138] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:16:36.147] [pf001] upstream main, fix compiled IN
[00:16:36.148] [pf001] tick_at_boot=0x31600 (enable=1 cycles=0 running=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[00:16:36.150] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:16:36.336] [wifi] link DOWN
[00:16:46.173] [pf001] window 1/6: kernel=10010 ms timer=10002885 us -> 999.289 us/ms   cumulative 999.289 us/ms
[00:16:56.175] [pf001] window 2/6: kernel=10010 ms timer=10009950 us -> 999.995 us/ms   cumulative 999.642 us/ms
[00:17:06.187] [pf001] window 3/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.762 us/ms
[00:17:16.189] [pf001] window 4/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.821 us/ms
[00:17:26.212] [pf001] window 5/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[00:17:36.214] [pf001] window 6/6: kernel=10010 ms timer=10009974 us -> 999.997 us/ms   cumulative 999.880 us/ms
[00:17:36.215] [pf001] done: fix compiled IN, tick cycles now=12
[00:17:36.224] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:17:38.225] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:17:40.245] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:17:42.245] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2

## 3rd power-cycle
[00:16:36.134] === uT-Kernel 3.0 / RP2040 demo ===
[00:16:36.135] SMP build: 2 processors
[00:16:36.138] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:16:36.147] [pf001] upstream main, fix compiled IN
[00:16:36.148] [pf001] tick_at_boot=0x31600 (enable=1 cycles=0 running=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[00:16:36.150] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:16:36.336] [wifi] link DOWN
[00:16:46.173] [pf001] window 1/6: kernel=10010 ms timer=10002885 us -> 999.289 us/ms   cumulative 999.289 us/ms
[00:16:56.175] [pf001] window 2/6: kernel=10010 ms timer=10009950 us -> 999.995 us/ms   cumulative 999.642 us/ms
[00:17:06.187] [pf001] window 3/6: kernel=10010 ms timer=10010038 us -> 1000.003 us/ms   cumulative 999.762 us/ms
[00:17:16.189] [pf001] window 4/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.821 us/ms
[00:17:26.212] [pf001] window 5/6: kernel=10010 ms timer=10010026 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[00:17:36.214] [pf001] window 6/6: kernel=10010 ms timer=10009974 us -> 999.997 us/ms   cumulative 999.880 us/ms
[00:17:36.215] [pf001] done: fix compiled IN, tick cycles now=12
[00:17:36.224] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:17:38.225] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:17:40.245] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:17:42.245] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
[00:17:44.269] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[00:17:46.268] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2
[00:17:48.291] [monitor] seq=143 produced=143 consumed=143 dropped=0  made_on=prc1 seen_on=prc2
[00:17:50.286] [monitor] seq=147 produced=147 consumed=147 dropped=0  made_on=prc1 seen_on=prc2
[00:17:58.998] [wifi] link UP  rssi=-55  mac=d8:3a:dd:ae:4b:eb
[00:18:02.816] [pf001] window 1/6: kernel=10010 ms timer=10002886 us -> 999.289 us/ms   cumulative 999.289 us/ms
[00:18:05.016] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:18:12.812] [pf001] window 2/6: kernel=10010 ms timer=10009943 us -> 999.994 us/ms   cumulative 999.641 us/ms
[00:18:22.838] [pf001] window 3/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.762 us/ms
[00:18:32.840] [pf001] window 4/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.820 us/ms
[00:18:42.853] [pf001] window 5/6: kernel=10010 ms timer=10010039 us -> 1000.003 us/ms   cumulative 999.857 us/ms
[00:18:52.856] [pf001] window 6/6: kernel=10010 ms timer=10009963 us -> 999.996 us/ms   cumulative 999.880 us/ms
[00:18:52.858] [pf001] done: fix compiled IN, tick cycles now=12
[00:18:52.869] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:18:54.877] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2

## 4th power-cycle
[00:19:03.782] === uT-Kernel 3.0 / RP2040 demo ===
[00:19:03.803] SMP build: 2 processors
[00:19:03.805] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:19:03.807] [pf001] upstream main, fix compiled IN
[00:19:03.808] [pf001] tick_at_boot=0xd3600 (enable=1 cycles=0 running=1)  tick_now=0xe0c (enable=1 cycles=12 running=1)
[00:19:03.814] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:19:04.000] [wifi] link DOWN
[00:19:13.817] [pf001] window 1/6: kernel=10010 ms timer=10002971 us -> 999.297 us/ms   cumulative 999.297 us/ms
[00:19:23.841] [pf001] window 2/6: kernel=10010 ms timer=10009968 us -> 999.996 us/ms   cumulative 999.647 us/ms
[00:19:33.838] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.764 us/ms
[00:19:43.860] [pf001] window 4/6: kernel=10010 ms timer=10010019 us -> 1000.001 us/ms   cumulative 999.823 us/ms
[00:19:53.857] [pf001] window 5/6: kernel=10010 ms timer=10009981 us -> 999.998 us/ms   cumulative 999.858 us/ms
[00:20:03.880] [pf001] window 6/6: kernel=10010 ms timer=10010025 us -> 1000.002 us/ms   cumulative 999.882 us/ms
[00:20:03.881] [pf001] done: fix compiled IN, tick cycles now=12
[00:20:03.882] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:20:05.896] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
## 5th power-cycle
[00:20:22.192] [wifi] link DOWN
[00:20:28.219] [wifi] link UP  rssi=-43  mac=d8:3a:dd:ae:4b:eb
[00:20:32.011] [pf001] window 1/6: kernel=10010 ms timer=10002828 us -> 999.283 us/ms   cumulative 999.283 us/ms
[00:20:34.238] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:20:42.037] [pf001] window 2/6: kernel=10010 ms timer=10010018 us -> 1000.001 us/ms   cumulative 999.642 us/ms
[00:20:52.040] [pf001] window 3/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.761 us/ms
[00:21:02.060] [pf001] window 4/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.821 us/ms
[00:21:12.058] [pf001] window 5/6: kernel=10010 ms timer=10009980 us -> 999.998 us/ms   cumulative 999.856 us/ms
[00:21:22.073] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.880 us/ms
[00:21:22.074] [pf001] done: fix compiled IN, tick cycles now=12
[00:21:22.075] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:21:24.097] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:21:26.092] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
## 6th power-cycle
[00:25:26.631] === uT-Kernel 3.0 / RP2040 demo ===
[00:25:26.632] SMP build: 2 processors
[00:25:26.634] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:25:26.635] [pf001] upstream main, fix compiled IN
[00:25:26.656] [pf001] tick_at_boot=0xc7600 (enable=1 cycles=0 running=1)  tick_now=0x5e0c (enable=1 cycles=12 running=1)
[00:25:26.658] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:25:26.833] [wifi] link DOWN
[00:25:36.673] [pf001] window 1/6: kernel=10010 ms timer=10002877 us -> 999.288 us/ms   cumulative 999.288 us/ms
[00:25:46.674] [pf001] window 2/6: kernel=10010 ms timer=10009952 us -> 999.995 us/ms   cumulative 999.641 us/ms
[00:25:56.686] [pf001] window 3/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.762 us/ms
[00:26:06.687] [pf001] window 4/6: kernel=10010 ms timer=10009968 us -> 999.996 us/ms   cumulative 999.820 us/ms
[00:26:16.705] [pf001] window 5/6: kernel=10010 ms timer=10010028 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[00:26:26.713] [pf001] window 6/6: kernel=10010 ms timer=10009968 us -> 999.996 us/ms   cumulative 999.880 us/ms
[00:26:26.713] [pf001] done: fix compiled IN, tick cycles now=12
[00:26:26.723] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:26:28.726] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:26:30.750] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2
[00:26:32.753] [monitor] seq=131 produced=131 consumed=131 dropped=0  made_on=prc1 seen_on=prc2
[00:26:34.766] [monitor] seq=135 produced=135 consumed=135 dropped=0  made_on=prc1 seen_on=prc2
[00:26:36.769] [monitor] seq=139 produced=139 consumed=139 dropped=0  made_on=prc1 seen_on=prc2

## 7th power-cycle
[00:26:48.973] === uT-Kernel 3.0 / RP2040 demo ===
[00:26:48.974] SMP build: 2 processors
[00:26:48.975] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:26:48.991] [pf001] upstream main, fix compiled IN
[00:26:48.993] [pf001] tick_at_boot=0xba600 (enable=1 cycles=0 running=1)  tick_now=0x4e0c (enable=1 cycles=12 running=1)
[00:26:48.995] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:26:49.172] [wifi] link DOWN
[00:26:55.201] [wifi] link UP  rssi=-46  mac=d8:3a:dd:ae:4b:eb
[00:26:59.019] [pf001] window 1/6: kernel=10010 ms timer=10002906 us -> 999.291 us/ms   cumulative 999.291 us/ms
[00:27:01.213] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:27:09.019] [pf001] window 2/6: kernel=10010 ms timer=10009933 us -> 999.993 us/ms   cumulative 999.642 us/ms
[00:27:19.035] [pf001] window 3/6: kernel=10010 ms timer=10010030 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[00:27:29.038] [pf001] window 4/6: kernel=10010 ms timer=10009966 us -> 999.996 us/ms   cumulative 999.821 us/ms
[00:27:39.053] [pf001] window 5/6: kernel=10010 ms timer=10010034 us -> 1000.003 us/ms   cumulative 999.857 us/ms
[00:27:49.051] [pf001] window 6/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.880 us/ms
[00:27:49.052] [pf001] done: fix compiled IN, tick cycles now=12
[00:27:49.068] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:27:51.073] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:27:53.097] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

## 8th power-cycle
[00:28:03.143] ning=1)  tick_now=0x160c (enable=1 cycles=12 running=1)
[00:28:03.145] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:28:03.330] [wifi] link DOWN
[00:28:13.166] [pf001] window 1/6: kernel=10010 ms timer=10002885 us -> 999.289 us/ms   cumulative 999.289 us/ms
[00:28:23.172] [pf001] window 2/6: kernel=10010 ms timer=10009953 us -> 999.995 us/ms   cumulative 999.642 us/ms
[00:28:33.186] [pf001] window 3/6: kernel=10010 ms timer=10010024 us -> 1000.002 us/ms   cumulative 999.762 us/ms
[00:28:43.184] [pf001] window 4/6: kernel=10010 ms timer=10009976 us -> 999.997 us/ms   cumulative 999.821 us/ms
[00:28:53.211] [pf001] window 5/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.857 us/ms
[00:29:03.210] [pf001] window 6/6: kernel=10010 ms timer=10009978 us -> 999.997 us/ms   cumulative 999.880 us/ms
[00:29:03.213] [pf001] done: fix compiled IN, tick cycles now=12
[00:29:03.221] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:29:05.229] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2


## 9th power-cycle
[00:29:14.658] === uT-Kernel 3.0 / RP2040 demo ===
[00:29:14.674] SMP build: 2 processors
[00:29:14.676] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[00:29:14.677] [pf001] upstream main, fix compiled IN
[00:29:14.678] [pf001] tick_at_boot=0xd2600 (enable=1 cycles=0 running=1)  tick_now=0x3e0c (enable=1 cycles=12 running=1)
[00:29:14.694] [pf001] wd_reason=0x0  chip_reset=0x00000100 (por=1 run=0 psm_restart=0)
[00:29:14.879] [wifi] link DOWN
[00:29:20.901] [wifi] link UP  rssi=-38  mac=d8:3a:dd:ae:4b:eb
[00:29:24.697] [pf001] window 1/6: kernel=10010 ms timer=10002826 us -> 999.283 us/ms   cumulative 999.283 us/ms
[00:29:29.917] [wifi] ip=10.151.89.211  gw=10.151.89.188  dns=10.151.89.188
[00:29:34.714] [pf001] window 2/6: kernel=10010 ms timer=10010022 us -> 1000.002 us/ms   cumulative 999.642 us/ms
[00:29:44.713] [pf001] window 3/6: kernel=10010 ms timer=10009976 us -> 999.997 us/ms   cumulative 999.761 us/ms
[00:29:54.733] [pf001] window 4/6: kernel=10010 ms timer=10010023 us -> 1000.002 us/ms   cumulative 999.821 us/ms
[00:30:04.735] [pf001] window 5/6: kernel=10010 ms timer=10009977 us -> 999.997 us/ms   cumulative 999.856 us/ms
[00:30:14.756] [pf001] window 6/6: kernel=10010 ms timer=10010021 us -> 1000.002 us/ms   cumulative 999.880 us/ms
[00:30:14.759] [pf001] done: fix compiled IN, tick cycles now=12
[00:30:14.761] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:30:16.772] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:30:18.776] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

## 10th power-cycle
[00:30:53.871] [pf001] window 1/6: kernel=10010 ms timer=10002983 us -> 999.299 us/ms   cumulative 999.299 us/ms
[00:31:03.873] [pf001] window 2/6: kernel=10010 ms timer=10009950 us -> 999.995 us/ms   cumulative 999.647 us/ms
[00:31:13.887] [pf001] window 3/6: kernel=10010 ms timer=10010031 us -> 1000.003 us/ms   cumulative 999.765 us/ms
[00:31:23.886] [pf001] window 4/6: kernel=10010 ms timer=10009967 us -> 999.996 us/ms   cumulative 999.823 us/ms
[00:31:33.913] [pf001] window 5/6: kernel=10010 ms timer=10010033 us -> 1000.003 us/ms   cumulative 999.859 us/ms
[00:31:43.912] [pf001] window 6/6: kernel=10010 ms timer=10009970 us -> 999.997 us/ms   cumulative 999.882 us/ms
[00:31:43.913] [pf001] done: fix compiled IN, tick cycles now=12
[00:31:43.924] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[00:31:45.931] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[00:31:47.944] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2


# Router (Pico2_IoT) runs: pf001_run4_router_fix0.uf2 / pf001_run4_router_fix1.uf2

## Network
- Router `Pico2_IoT`, the same network as the original F1.0 / PF-001 verification
  (0/5 before the fix, 7/7 after, on our firmware)
- Moved here from the phone hotspot because the hotspot rejects every second
  join (see "Phone hotspot" in `logs/F1.0-results-2026-10-16.md`),
  which hides the DHCP result on half of the boots

## Build

| | |
|---|---|
| Source | Same as the hotspot images: upstream `main` @ `2e8e6ad` plus the PF-001 test code, worktree `mtk3smp-rp2040-run4` (branch `test/pf001-run4`, not pushed), plus the done-line change below |
| Profile | DHCP profile: `make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 -j8` → `mtk3pico_smp1_usb_cdc_wifi_dhcp.uf2` (same as the hotspot images) |
| Fix switch | `PF001_APPLY_FIX` in `kernel/sysdepend/pico_rp2040/cpu_clock.c`: `0` → `pf001_run4_router_fix0.uf2`, `1` → `pf001_run4_router_fix1.uf2` |
| Wi-Fi config | `config/wifi_credentials.h` (gitignored): SSID `Pico2_IoT`, `UTK_WIFI_AUTH = CYW43_AUTH_WPA3_WPA2_AES_PSK` (auth unchanged from the hotspot images) |
| Toolchain | Docker image from this repo's `Dockerfile`: Arm GNU Toolchain 13.2.rel1, Pico SDK 2.2.0 |
| Built | 2026-10-20, no warnings |

## Instrumentation change: boot values repeated on the `done` line
The router images print the boot-time evidence a second time, at the end of the
60 s measurement. On a power-on boot of the fix image it should read:

```
[pf001] done: fix compiled IN, tick cycles now=12 | at boot: cycles=0 wd_reason=0x0 por=1 run=0
```

**Why it was added now, not at the start:** the header lines
(`tick_at_boot`, `wd_reason`, `chip_reset`) are printed ~0.5 s after USB
enumeration. The firmware waits for enumeration, not for the terminal to open
the port, and output sent before Serial Monitor reconnects is lost. The test code
was written assuming the header would be captured on every boot. That only
proved false once many boots had been logged: runs 1-3 and the hotspot run 4 lost
the header on roughly half of the boots, so for those boots the boot path
(power-on vs. reflash) rests on the method notes rather than the log. The `done`
line comes ~60 s after boot and is always captured, so it was extended before the
router run.

**What it does not change:** the values are the ones already read at the top of
`pf001_report()`. The tick seen at boot is held in WATCHDOG SCRATCH0 from
`startup_clock()`, and the reason/chip-reset registers are read at the start, so
printing them late doesn't alter them. Nothing about boot, Wi-Fi or timer
behaviour changed, and the header lines are still printed as before. Earlier
images (`pf001_upstream_*`, `pf001_run4_dhcp_*`) don't have this line.

## Planned runs
| # | Image | Do | Expect |
|---|---|---|---|
| A | router_fix0 | reflash, 2 boots | ~1000 us/ms, `ip=` |
| B | router_fix0 | power-cycle x5 | at boot `cycles=0`, 23.437 us/ms, `link UP`, no `ip=` |
| C | router_fix1 | reflash, 1 boot | `fix compiled IN`, `ip=` |
| D | router_fix1 | power-cycle x7 | at boot `cycles=0`, now `cycles=12`, ~1000 us/ms, `ip=` on every boot |
