# Declaration
- most [monitor] was removed, it is to be assumed that monitors exists between logs and afterwards unless stated otherwise
# 1
[20:15:17.568] === uT-Kernel 3.0 / RP2040 demo ===
[20:15:17.569] SMP build: 2 processors
[20:15:17.573] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:15:17.763] [wifi] link DOWN
[20:15:23.776] [wifi] link UP  rssi=-31  mac=d8:3a:dd:ae:4b:eb
[20:15:32.822] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
# 2
[20:20:31.833] [wifi] link DOWN
[20:20:43.870] [wifi] link UP  rssi=-32  mac=d8:3a:dd:ae:4b:eb
# 3
[20:24:04.414] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
# 4
[0:26:50.371] === uT-Kernel 3.0 / RP2040 demo ===
[20:26:50.382] SMP build: 2 processors
[20:26:50.390] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:26:50.555] [wifi] link DOWN
[20:26:56.569] [wifi] link UP  rssi=-37  mac=d8:3a:dd:ae:4b:eb
[20:27:11.629] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
# 5
[20:28:23.779] SMP build: 2 processors
[20:28:23.781] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:28:23.974] [wifi] link DOWN
[20:28:36.018] [wifi] link UP  rssi=-39  mac=d8:3a:dd:ae:4b:eb
# 6
[20:32:08.424] === uT-Kernel 3.0 / RP2040 demo ===
[20:32:08.435] SMP build: 2 processors
[20:32:08.436] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:32:08.642] [wifi] link DOWN
[20:32:20.681] [wifi] link UP  rssi=-44  mac=d8:3a:dd:ae:4b:eb
# 7
[20:41:27.767] === uT-Kernel 3.0 / RP2040 demo ===
[20:41:27.780] SMP build: 2 processors
[20:41:27.793] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:41:27.801] [monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[20:41:27.986] [wifi] link DOWN
[20:41:34.002] [wifi] link UP  rssi=-48  mac=d8:3a:dd:ae:4b:eb
[20:41:34.003] [wifi] waiting for lease: state=8 tries=0 t=1739ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=265 stage=7 rd=0
[20:41:37.026] [wifi] waiting for lease: state=8 tries=0 t=4759ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=416 stage=7 rd=0
[20:41:40.027] [wifi] waiting for lease: state=8 tries=0 t=7779ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=567 stage=7 rd=0
[20:41:43.038] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[20:41:45.881] [monitor] seq=37 produced=37 consumed=37 dropped=0  made_on=prc1 seen_on=prc2
[20:42:28.080] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2
[20:42:30.101] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
# 8
Power cycle (no reflash) of the attempt-7 build: the poll/stage diagnostic is in.
[20:45:20.803] [monitor] stalled: produced=2 consumed=2
[20:45:29.020] [wifi] link UP  rssi=-42  mac=d8:3a:dd:ae:4b:eb
[20:45:29.021] [wifi] waiting for lease: state=1 tries=1 t=55ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=293 stage=7 rd=1
[20:45:32.045] [wifi] waiting for lease: state=1 tries=1 t=126ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=444 stage=7 rd=2
[20:45:35.056] [wifi] waiting for lease: state=1 tries=1 t=196ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=594 stage=7 rd=2
[20:45:38.063] [wifi] waiting for lease: state=1 tries=1 t=267ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=745 stage=7 rd=3
[20:45:41.080] [wifi] waiting for lease: state=1 tries=1 t=337ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=895 stage=7 rd=3
[20:45:44.079] [wifi] waiting for lease: state=1 tries=1 t=408ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1046 stage=7 rd=4
[20:45:47.104] [wifi] waiting for lease: state=1 tries=1 t=479ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1196 stage=7 rd=4
[20:45:50.094] [wifi] waiting for lease: state=1 tries=1 t=549ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1347 stage=7 rd=5
[20:45:53.125] [wifi] waiting for lease: state=1 tries=1 t=620ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1497 stage=7 rd=5
[20:45:56.116] [wifi] waiting for lease: state=1 tries=1 t=690ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1648 stage=7 rd=6
[20:45:59.142] [wifi] waiting for lease: state=1 tries=1 t=761ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1798 stage=7 rd=6
[20:46:02.139] [wifi] waiting for lease: state=1 tries=1 t=832ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=1949 stage=7 rd=7
[20:46:05.156] [wifi] waiting for lease: state=1 tries=1 t=902ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=2099 stage=7 rd=7
[20:46:08.160] [wifi] waiting for lease: state=1 tries=1 t=973ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=2250 stage=7 rd=8
[20:46:11.174] [wifi] waiting for lease: state=1 tries=1 t=1043ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=2400 stage=7 rd=8
[20:46:14.180] [wifi] waiting for lease: state=1 tries=1 t=1114ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=2551 stage=7 rd=9
[20:46:17.198] [wifi] waiting for lease: state=1 tries=1 t=1184ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=2701 stage=7 rd=9
[20:46:20.205] [wifi] waiting for lease: state=1 tries=1 t=1255ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=2852 stage=7 rd=10
(monitor lines continued normally every 2 s throughout)

## Analysis of #8
- **The radio task is alive.** `polls` rises ~150 every line and `stage=7`
  (sleeping) is normal, so the "radio task hung" hypothesis is wrong.
- **DHCP is stuck in `state=1` (REQUESTING), `tries=1`.** The REQUEST went
  out, but the retry never fires.
- **The tell: `t=` only advances ~70 ms per 3 s of real time.** Wall clock
  (log timestamps) is 3.0 s per line, but `t=` (from `time_us_64()`) moves
  55 → 126 → 196 ms. **The microsecond timer is running ~43× too slow.**
  Compare with the successful reflash boot #7: `t=` 1739 → 4759 → 7779 ms,
  3.02 s per 3 s line, which is correct.
- 70.5 ms / 3 s = **23.5 ms per second**. That is exactly 12 MHz / 512 =
  23 437 ticks/s: the timer is ticking once per 512 `clk_ref` cycles
  instead of once per 12.
- Consequences: lwIP's clock (`sys_now()` = `time_us_64()/1000`) runs 43× slow,
  so every lwIP timer is 43× late. A 2 s DHCP retry becomes ~85 s, ACD's ~9 s
  becomes ~6 min, and our own 30 s DHCP timeout becomes ~21 min. That's why
  failing boots never print even `ip=0.0.0.0`. The kernel itself (SysTick)
  is unaffected, so `[monitor]` and the LED look normal.
- Side observation: `polls` rises ~50 per second, so the "2 ms" radio poll
  (`CYW43_POLL_MS`) really runs every ~20 ms. `tk_dly_tsk(2)` is rounded up
  by the 10 ms kernel tick. Not the bug, but relevant to F4 (the feedback's
  1 ms kernel tick point).
# 9
[20:52:39.488] === uT-Kernel 3.0 / RP2040 demo ===
[20:52:39.489] SMP build: 2 processors
[20:52:39.491] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:52:39.494] [monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[20:52:39.680] [wifi] link DOWN
[20:52:45.700] [wifi] link UP  rssi=-33  mac=d8:3a:dd:ae:4b:eb
[20:52:45.701] [wifi] waiting for lease: state=8 tries=0 t=1799ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=264 stage=7 rd=0
[20:52:48.714] [wifi] waiting for lease: state=8 tries=0 t=4819ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=415 stage=7 rd=0
[20:52:51.743] [wifi] waiting for lease: state=8 tries=0 t=7819ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=565 stage=7 rd=0
[20:52:54.741] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[20:53:35.771] [monitor] seq=112 produced=112 consumed=112 dropped=0  made_on=prc1 seen_on=prc2
[20:53:37.772] [monitor] seq=116 produced=116 consumed=116 dropped=0  made_on=prc1 seen_on=prc2

# 10

[20:55:53.769] === uT-Kernel 3.0 / RP2040 demo ===
[20:55:53.770] SMP build: 2 processors
[20:55:53.772] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:55:53.785] [monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[20:55:53.972] [wifi] link DOWN
[20:55:59.993] [wifi] link UP  rssi=-30  mac=d8:3a:dd:ae:4b:eb
[20:55:59.993] [wifi] waiting for lease: state=8 tries=0 t=1799ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=266 stage=7 rd=1
[20:56:03.012] [wifi] waiting for lease: state=8 tries=0 t=4819ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=417 stage=7 rd=2
[20:56:06.035] [wifi] waiting for lease: state=8 tries=0 t=7819ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=567 stage=7 rd=2
[20:56:07.855] [monitor] seq=29 produced=29 consumed=29 dropped=0  made_on=prc1 seen_on=prc2
[20:56:09.035] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[20:56:52.068] [monitor] seq=116 produced=116 consumed=116 dropped=0  made_on=prc1 seen_on=prc2
[20:56:54.092] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

# 11
[20:58:57.754] === uT-Kernel 3.0 / RP2040 demo ===
[20:58:57.755] SMP build: 2 processors
[20:58:57.756] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[20:58:57.772] [monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[20:58:57.958] [wifi] link DOWN
[20:59:03.976] [wifi] link UP  rssi=-33  mac=d8:3a:dd:ae:4b:eb
[20:59:03.977] [wifi] waiting for lease: state=8 tries=0 t=2019ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=277 stage=7 rd=1
[20:59:05.812] [monitor] seq=18 produced=18 consumed=18 dropped=0  made_on=prc1 seen_on=prc2
[20:59:06.995] [wifi] waiting for lease: state=8 tries=0 t=5039ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=428 stage=7 rd=2
[20:59:10.012] [wifi] waiting for lease: state=8 tries=0 t=8039ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=578 stage=7 rd=2
[20:59:11.834] [monitor] seq=30 produced=30 consumed=30 dropped=0  made_on=prc1 seen_on=prc2
[20:59:13.015] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[20:59:56.056] [monitor] seq=117 produced=117 consumed=117 dropped=0  made_on=prc1 seen_on=prc2
[20:59:58.078] [monitor] seq=121 produced=121 consumed=121 dropped=0  made_on=prc1 seen_on=prc2

# 12
[21:00:41.115] [monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
[21:00:43.131] [monitor] seq=10 produced=10 consumed=10 dropped=0  made_on=prc1 seen_on=prc2
[21:00:45.130] [monitor] seq=14 produced=14 consumed=14 dropped=0  made_on=prc1 seen_on=prc2
[21:00:45.316] [wifi] link UP  rssi=-33  mac=d8:3a:dd:ae:4b:eb
[21:00:45.318] [wifi] waiting for lease: state=1 tries=2 t=1759ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=266 stage=7 rd=1
[21:00:47.159] [monitor] seq=18 produced=18 consumed=18 dropped=0  made_on=prc1 seen_on=prc2
[21:00:48.329] [wifi] waiting for lease: state=1 tries=2 t=4779ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=417 stage=7 rd=2
[21:00:49.155] [monitor] seq=22 produced=22 consumed=22 dropped=0  made_on=prc1 seen_on=prc2
[21:00:51.169] [monitor] seq=26 produced=26 consumed=26 dropped=0  made_on=prc1 seen_on=prc2
[21:00:51.357] [wifi] waiting for lease: state=8 tries=0 t=7779ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=567 stage=7 rd=2
[21:00:53.171] [monitor] seq=29 produced=29 consumed=29 dropped=0  made_on=prc1 seen_on=prc2
[21:00:54.357] [wifi] waiting for lease: state=8 tries=0 t=10799ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=718 stage=7 rd=3
[21:00:57.370] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[21:01:41.410] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[21:01:43.430] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2

# 13
[21:02:43.635] [wifi] link DOWN
[21:02:45.464] [monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
[21:02:47.462] [monitor] seq=10 produced=10 consumed=10 dropped=0  made_on=prc1 seen_on=prc2
[21:02:49.483] [monitor] seq=14 produced=14 consumed=14 dropped=0  made_on=prc1 seen_on=prc2
[21:02:49.659] [wifi] link UP  rssi=-32  mac=d8:3a:dd:ae:4b:eb
[21:02:49.661] [wifi] waiting for lease: state=8 tries=0 t=1779ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=265 stage=7 rd=0
[21:02:51.480] [monitor] seq=18 produced=18 consumed=18 dropped=0  made_on=prc1 seen_on=prc2
[21:02:52.676] [wifi] waiting for lease: state=8 tries=0 t=4799ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=416 stage=7 rd=0
[21:02:53.504] [monitor] seq=22 produced=22 consumed=22 dropped=0  made_on=prc1 seen_on=prc2
[21:02:55.495] [monitor] seq=26 produced=26 consumed=26 dropped=0  made_on=prc1 seen_on=prc2
[21:02:55.683] [wifi] waiting for lease: state=8 tries=0 t=7819ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=567 stage=7 rd=0
[21:02:57.523] [monitor] seq=29 produced=29 consumed=29 dropped=0  made_on=prc1 seen_on=prc2
[21:02:58.702] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[21:03:41.733] [monitor] seq=116 produced=116 consumed=116 dropped=0  made_on=prc1 seen_on=prc2
[21:03:43.741] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

# 14

[21:11:58.951] === uT-Kernel 3.0 / RP2040 demo ===
[21:11:58.953] SMP build: 2 processors
[21:11:58.955] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[21:11:58.957] [monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[21:11:59.154] [wifi] link DOWN
[21:12:00.974] [monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
[21:12:02.972] [monitor] seq=10 produced=10 consumed=10 dropped=0  made_on=prc1 seen_on=prc2
[21:12:04.990] [monitor] seq=14 produced=14 consumed=14 dropped=0  made_on=prc1 seen_on=prc2
[21:12:05.176] [wifi] link UP  rssi=-45  mac=d8:3a:dd:ae:4b:eb
[21:12:05.177] [wifi] waiting for lease: state=1 tries=2 t=1739ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=265 stage=7 rd=1
[21:12:06.992] [monitor] seq=18 produced=18 consumed=18 dropped=0  made_on=prc1 seen_on=prc2
[21:12:08.191] [wifi] waiting for lease: state=1 tries=2 t=4759ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=416 stage=7 rd=1
[21:12:09.018] [monitor] seq=22 produced=22 consumed=22 dropped=0  made_on=prc1 seen_on=prc2
[21:12:11.014] [monitor] seq=25 produced=25 consumed=25 dropped=0  made_on=prc1 seen_on=prc2
[21:12:11.191] [wifi] waiting for lease: state=8 tries=0 t=7779ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=567 stage=7 rd=2
[21:12:13.036] [monitor] seq=29 produced=29 consumed=29 dropped=0  made_on=prc1 seen_on=prc2
[21:12:14.214] [wifi] waiting for lease: state=8 tries=0 t=10779ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=717 stage=7 rd=2
[21:12:15.030] [monitor] seq=33 produced=33 consumed=33 dropped=0  made_on=prc1 seen_on=prc2
[21:12:17.050] [monitor] seq=37 produced=37 consumed=37 dropped=0  made_on=prc1 seen_on=prc2
[21:12:17.216] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[21:12:57.253] [monitor] seq=116 produced=116 consumed=116 dropped=0  made_on=prc1 seen_on=prc2
[21:12:59.255] [monitor] seq=120 produced=120 consumed=120 dropped=0  made_on=prc1 seen_on=prc2

# 15
[21:13:40.215] [monitor] seq=7 produced=7 consumed=7 dropped=0  made_on=prc1 seen_on=prc2
[21:13:42.210] [monitor] seq=11 produced=11 consumed=11 dropped=0  made_on=prc1 seen_on=prc2
[21:13:44.226] [monitor] seq=15 produced=15 consumed=15 dropped=0  made_on=prc1 seen_on=prc2
[21:13:44.414] [wifi] link UP  rssi=-60  mac=d8:3a:dd:ae:4b:eb
[21:13:44.415] [wifi] waiting for lease: state=1 tries=1 t=2519ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=305 stage=7 rd=2
[21:13:46.226] [monitor] seq=19 produced=19 consumed=19 dropped=0  made_on=prc1 seen_on=prc2
[21:13:47.435] [wifi] waiting for lease: state=8 tries=0 t=5539ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=456 stage=7 rd=2
[21:13:48.249] [monitor] seq=23 produced=23 consumed=23 dropped=0  made_on=prc1 seen_on=prc2
[21:13:50.246] [monitor] seq=27 produced=27 consumed=27 dropped=0  made_on=prc1 seen_on=prc2
[21:13:50.433] [wifi] waiting for lease: state=8 tries=0 t=8559ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=607 stage=7 rd=3
[21:13:52.269] [monitor] seq=31 produced=31 consumed=31 dropped=0  made_on=prc1 seen_on=prc2
[21:13:53.450] [wifi] waiting for lease: state=8 tries=0 t=11559ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=757 stage=7 rd=3
[21:13:54.268] [monitor] seq=35 produced=35 consumed=35 dropped=0  made_on=prc1 seen_on=prc2
[21:13:56.286] [monitor] seq=39 produced=39 consumed=39 dropped=0  made_on=prc1 seen_on=prc2
[21:13:56.452] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[21:14:28.451] [monitor] seq=102 produced=102 consumed=102 dropped=0  made_on=prc1 seen_on=prc2
[21:14:30.443] [monitor] seq=106 produced=106 consumed=106 dropped=0  made_on=prc1 seen_on=prc2

# 16
[21:14:57.221] === uT-Kernel 3.0 / RP2040 demo ===
[21:14:57.222] SMP build: 2 processors
[21:14:57.224] producer -> [mbf] -> consumer, 2 credits, 500 ms period
[21:14:57.233] [monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[21:14:57.420] [wifi] link DOWN
[21:14:59.241] [monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
[21:15:01.255] [monitor] seq=10 produced=10 consumed=10 dropped=0  made_on=prc1 seen_on=prc2
[21:15:03.258] [monitor] seq=14 produced=14 consumed=14 dropped=0  made_on=prc1 seen_on=prc2
[21:15:03.435] [wifi] link UP  rssi=-50  mac=d8:3a:dd:ae:4b:eb
[21:15:03.436] [wifi] waiting for lease: state=1 tries=2 t=1879ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=267 stage=7 rd=1
[21:15:05.281] [monitor] seq=18 produced=18 consumed=18 dropped=0  made_on=prc1 seen_on=prc2
[21:15:06.458] [wifi] waiting for lease: state=1 tries=2 t=4899ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=418 stage=7 rd=2
[21:15:07.282] [monitor] seq=22 produced=22 consumed=22 dropped=0  made_on=prc1 seen_on=prc2
[21:15:09.297] [monitor] seq=26 produced=26 consumed=26 dropped=0  made_on=prc1 seen_on=prc2
[21:15:09.483] [wifi] waiting for lease: state=8 tries=0 t=7899ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=568 stage=7 rd=2
[21:15:11.303] [monitor] seq=29 produced=29 consumed=29 dropped=0  made_on=prc1 seen_on=prc2
[21:15:12.481] [wifi] waiting for lease: state=8 tries=0 t=10919ms addr=0.0.0.0 gw=0.0.0.0 dns=0.0.0.0 polls=719 stage=7 rd=3
[21:15:13.321] [monitor] seq=33 produced=33 consumed=33 dropped=0  made_on=prc1 seen_on=prc2
[21:15:15.315] [monitor] seq=37 produced=37 consumed=37 dropped=0  made_on=prc1 seen_on=prc2
[21:15:15.503] [wifi] ip=192.168.50.8  gw=192.168.50.1  dns=8.8.8.8
[21:15:59.532] [monitor] seq=124 produced=124 consumed=124 dropped=0  made_on=prc1 seen_on=prc2
[21:16:01.553] [monitor] seq=128 produced=128 consumed=128 dropped=0  made_on=prc1 seen_on=prc2