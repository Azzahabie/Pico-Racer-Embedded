# Results: resolving the Week 6 feedback (Buddy 1)

What actually happened for each step in [`Actions.md`](Actions.md). Use the same
step IDs. Record **observations, not intentions**. Paste the evidence line,
give the date, and say plainly if something failed or was skipped.

**Entry format:**

```
### F1.x <title>
- Date: YYYY-MM-DD
- Build: <.uf2 name / commit>
- Outcome: ✅ pass / ❌ fail / ⛔ blocked / ⏭ skipped
- Evidence: <console lines, log file path, numbers>
- Notes: <surprises, follow-ups>
```

---

## F1: Wi-Fi/DHCP reliability

### F1.0 Baseline (before the fix)
- Date: 16 Oct 2026
- Build: `mtk3pico_smp1_usb_cdc_wifi_dns.uf2` @ `f1cb60f` (+ diagnostic; boots 9+ also + PF-001 fix)
- Log file: [`F1log.md`](F1log.md)

**Pico2_IoT (20 cold boots)**

| Boot | link UP? | RSSI | ip= | Time to ip (s) | Notes                                                                                                            |
| ---- | -------- | ---- | --- | -------------- | ---------------------------------------------------------------------------------------------------------------- |
| 1    | yes      | yes  | yes | 15             | After removing diagnosis, fresh flash, ping reached, router recognize pico                                       |
| 2    | yes      | yes  | no  |                | Quick disconnect/reconnect on pico, ping from PC not reached, after 3mins no IP, router recognize pico           |
| 3    | yes      | yes  | no  |                | Disconnected pico for 30s, ping from pc not reach, after 3mins no ip, router recognize pico                      |
| 4    | yes      | yes  | yes | 20             | Reflash same build, only disconnected for ~20s, ping reached, router recognize pico                              |
| 5    | yes      | yes  | no  |                | Quick disc/rc on pico, ping from pc not reached, after 3 mins no ip, router recognize pico                       |
| 6    | yes      | yes  | no  |                | Waited 1 min after dc to reconnect pico, ping not reached, no ip after 3 mins, router recognize pico             |
| 7    | yes      | yes  | yes | 16             | Reflashed with new build, ping reached, router recognize pico                                                    |
| 8    | yes      | -42  | no  |                | Power cycle, poll diagnostic build. Radio alive, DHCP stuck REQUESTING; `t=` shows timer ~43x slow. See F1log #8 |
| 9    | yes      | -33  | yes | 15             | Reflash with new timer fix build. Radio alive, ping reached                                                      |
| 10   | yes      | -30  | yes | 16             | quick power cycle, radio alive, ping reached                                                                     |
| 11   | yes      | -33  | yes | 16             | Ditto above                                                                                                      |
| 12   | yes      | -33  | yes | 16             | Ditto above                                                                                                      |
| 13   | yes      | -32  | yes | 15             | Ditto above                                                                                                      |
| 14   | yes      | -45  | yes | 18             | Ditto above                                                                                                      |
| 15   | yes      | -60  | yes | 17             | Ditto above                                                                                                      |
| 16   | yes      | -50  | yes | 18             | Ditto above                                                                                                      |

**Phone hotspot (10 cold boots)**

- Date: 19 Oct 2026
- Build: `mtk3pico_smp1_usb_cdc_wifi_dns.uf2` @ `1c01d5a` (timer-fixed, diagnostic
  line still in), credentials switched to the hotspot `Test1`
- Hotspot: phone, 2.4 GHz channel 6, WPA2-Personal (AKM `00-0f-ac:02` PSK),
  CCMP, 802.11n (read from a Windows laptop joined to the same hotspot)
- Times are from the first `[wifi] link DOWN` line (~0.2 s after the banner),
  not from power-on. USB enumeration adds a few seconds before that. Where the
  banner was cut off, a new boot is recognised by the monitor `seq` restarting.

| Boot | link UP? | RSSI | ip= | Time to ip (s) | Notes |
|---|---|---|---|---|---|
| 1 | yes | -58 | yes | 12.0 | link UP +6.0 s, `ip=10.151.89.211` |
| 2 | no | | no | | No link UP in ~54 s. Only failed boot watched past the 30 s join timeout; nothing printed when it expired |
| 3 | yes | -45 | yes | 15.1 | link UP +6.0 s |
| 4 | no | | no | | No link UP in ~18 s watched |
| 5 | yes | -53 | yes | 12.1 | link UP +6.0 s |
| 6 | no | | no | | No link UP in ~20 s watched |
| 7 | yes | -47 | yes | 12.1 | link UP +6.0 s |
| 8 | no | | no | | No link UP in ~20 s watched |
| 9 | yes | -50 | yes | 12.1 | link UP +6.0 s |
| 10 | no | | no | | No link UP in ~16 s watched |
| 11 | yes | -51 | yes | 15.1 | link UP +6.0 s. Extra boot beyond the planned 10 |

**Finding: hotspot boots strictly alternate success / failure (11 in a row).**
- Every failure is at **association** (no `link UP`), never at DHCP. Every
  success looks identical: link UP ~6 s after boot, then `ip=` 6 or 9 s later
  (ACD, `state=8`), always the same address `10.151.89.211`.
- Signal is not the cause (RSSI -45 to -58), and neither is the off time:
  the gap between the last line of one boot and the next boot was 3-8 s
  before both successes and failures. The only predictor is whether the
  **previous** boot succeeded.
- **Hypothesis (unconfirmed): stale session on the phone.** A power cut gives
  the Pico no chance to disassociate, so the hotspot still lists its MAC
  (`d8:3a:dd:ae:4b:eb`) as connected. The next join from the same MAC is
  ignored or rejected, and that failed attempt clears the stale entry, so the
  boot after it succeeds. The router (boots 9-16, 8/8 with equally quick power
  cycles) doesn't show this.
- **Hotspot-restart test (19 Oct 2026): 5 / 5 connected.** Each run: power
  off the Pico, turn the hotspot off, turn the hotspot on, power on the Pico.
  It associated every time, against strict alternation without the restart.
  Per-boot RSSI / timings weren't logged. This puts the failure **on the phone
  side**: restarting the hotspot wipes whatever state it held about the Pico.
  It fits the stale-session hypothesis, but doesn't yet rule out another
  phone-side cause (e.g. the phone temporarily blocking the MAC after a failed
  attempt). Tests still open: (1) after a successful boot, cut power and check
  the phone's connected-devices list for the Pico; (2) after a successful boot,
  wait ~5 min (typical hotspot idle timeout) before the next boot.
- **Consequence for F1.2:** today the join is one-shot and fails silently
  (`cyw43_utk.c:660`, no console line), so every second hotspot boot stays
  offline until power-cycled. A rejoin after back-off is exactly the "next
  boot" in this pattern, so F1.2 should turn these into a one-retry success.
  F1.5 T6 tests this.
- Raw log: [`F1log.md`](F1log.md) → "Hotspot (Test1), 19 Oct 2026", boots
  H1-H11.

**Blocker found during F1.0: ✅ resolved by port fix PF-001.** Boots 2, 3,
5, 6 and 8 (power cycles) associated but never got an IP. The cause was the
RP2040 microsecond timer running ~43× slow after a power-on, because the port
never programmed the watchdog tick, and lwIP's timers run on that timer. The
symptom, the investigation, the fix and the open drill-down items all live in
[`docs/port_fixes/PF-001-timer-tick/`](../../port_fixes/PF-001-timer-tick/README.md).
Raw logs are in [`F1log.md`](F1log.md) #1-#16.

Wi-Fi-specific notes that stay here:
- **Baseline split:** boots 1-8 ran the pre-fix firmware. Boots 9-20 are
  the timer-fixed firmware, which is the "before" for the F1.2/F1.3 retry
  changes.
- **Lease time:** ~9-10 s from link UP to `ip=` (15-18 s from power-on), almost
  all of it lwIP's ACD address check (`state=8`). ACD can be disabled
  (`LWIP_DHCP_DOES_ACD_CHECK 0` in `lwipopts.h`) to help NFR-09 (15 s
  recovery), at the cost of conflict detection. Decide in F1.2 / after F1.5.
- **⚠ Address overlap:** the Pico is getting `192.168.50.8`, which is also the
  broker address in `config/broker_config.h:37`. Before any MQTT build, either
  reserve the Pico at another address or point `broker_config.h` at the
  laptop's real IP.
- **Diagnostic line still in the build:** `[wifi] waiting for lease: state=
  tries= t= addr= gw= dns= polls= stage= rd=` (`app_main.c`, plus the
  `dhcp_state` / `poll_stage` fields in `cyw43_utk.h` / `.c`). Committed as
  `3e40f08` (debug). Fold it into F1.4's retry counters or remove it there.

- Success rate, Pico2_IoT: __ / 20
- Success rate, hotspot: **6 / 11** first attempt (~55%), strictly alternating;
  none of the 5 failures recovered while watched (only boot 2 was watched past
  the 30 s join timeout)

### F1.1 Where DHCP starts/stops
- Date: 2026-10-16
- Outcome: ✅ answered from source. A hardware confirmation of the third answer
  is still wanted (see below).
- Source read (the exact versions the Docker build uses; Pico SDK `2.2.0`
  pins these submodule commits):
  - cyw43-driver `dd75682`:
    [`src/cyw43_lwip.c`](https://github.com/georgerobotics/cyw43-driver/blob/dd7568229f3bf7a37737b9e1ef250c26efe75b23/src/cyw43_lwip.c),
    [`src/cyw43_ctrl.c`](https://github.com/georgerobotics/cyw43-driver/blob/dd7568229f3bf7a37737b9e1ef250c26efe75b23/src/cyw43_ctrl.c)
  - lwIP `77dcd25`:
    [`src/core/netif.c`](https://github.com/lwip-tcpip/lwip/blob/77dcd25a72509eb83f72b033d219b1d40cd8eb95/src/core/netif.c),
    [`src/core/ipv4/dhcp.c`](https://github.com/lwip-tcpip/lwip/blob/77dcd25a72509eb83f72b033d219b1d40cd8eb95/src/core/ipv4/dhcp.c)
  - Our `lwipopts.h` sets no custom DHCP timeouts, so lwIP's defaults apply.

**Q1. Where is `dhcp_start()` called?** ☑ once at init.
`cyw43_cb_tcpip_init()` (`cyw43_lwip.c:225-226`) calls `dhcp_start()` on the
STA netif. That init runs from `cyw43_wifi_set_up()` (`cyw43_ctrl.c:566-568`),
which our `cyw43_utk_task` calls once at boot (`cyw43_utk.c:515`). The link is
still down at that point, so `dhcp_start()` just parks in `DHCP_STATE_INIT`
(`dhcp.c`, `dhcp_start`: "wait for dhcp_network_changed() to call
dhcp_discover()").

**Q2. Does link-down stop DHCP?** ☑ no.
`cyw43_cb_tcpip_set_link_down()` only calls `netif_set_link_down()`
(`cyw43_lwip.c:296-298`), and that function makes no DHCP call at all
(`netif.c`). The client keeps its state. On the next link-up,
`netif_set_link_up()` calls `dhcp_network_changed_link_up()`, which resets
`tries = 0` and either re-requests the old address (if it was BOUND) or
restarts discovery. **So lwIP re-leases on its own after every reconnect.**
Our code doesn't need to call `dhcp_start()` / `dhcp_stop()`.

**Q3. Does lwIP keep retrying after our 30 s "timeout"?** ☑ yes, forever.
lwIP has no give-up. DISCOVER retries back off 1, 2, 4, 8, 16, 32 s, then every
**60 s** (`DHCP_REQUEST_BACKOFF_SEQUENCE`, `dhcp.c:132`). A stuck REQUEST is
released and restarted after 5 tries (`dhcp_timeout`). Our 30 s timeout exists
only in our status code. Underneath it, lwIP is still trying, and when a lease
finally arrives, our latch ignores it (`cyw43_utk.c:446` checks
`!dhcp_complete`).
*Still to confirm on hardware:* a boot that hits our 30 s timeout should show
`dhcp_tries` still rising afterwards. With PF-001 fixed, no boot has timed out
yet; F1.5 T3 (DHCP refused) will exercise it.

**Decision for F1.3:**
- No `dhcp_start()` / `dhcp_release_and_stop()` calls are needed. The fix is to
  **stop the latch**: clear `dhcp_complete` / `dhcp_result` on link loss and
  keep watching for an address after a timeout.
- **New finding (affects NFR-09, recovery within 15 s):** if we only wait,
  lwIP's back-off can grow to 60 s between DISCOVERs, so recovery after the
  DHCP server comes back could take up to ~60 s. F1.2's leave → rejoin on lease
  timeout is therefore still worth keeping. Each link-up resets `tries = 0`,
  so discovery restarts at the short 1-2 s intervals.
- Not covered by F1.1: whether the CYW43 firmware rejoins the AP by itself
  after a disassociation. `cyw43_ctrl.c:353-355` just clears `wifi_join_state`,
  so assume **no**, and keep F1.2's explicit rejoin.

### F1.2 Connection state machine
- Date:
- Commit:
- Build result (DNS profile):
- Notes:

### F1.3 DHCP latch replaced
- Date:
- Commit:
- Build result (DNS profile):
- Build result (MQTT profile):
- Notes:

### F1.4 Retry visibility
- Date:
- Sample console output:
  ```
  ```

### F1.5 Recovery tests

| Test | Date | Outcome | Evidence / timing |
|---|---|---|---|
| T1 AP off at boot | | | router-on → ip = __ s |
| T2 AP drop while online | | | |
| T3 DHCP refused | | | |
| T4 wrong password | | | |
| T5 30 min soak | | | link_losses = __ |
| T6 hotspot back-to-back power cycles | | | boots with ip= __ / 10, retries per boot __ |

### F1.6 A1 re-measured (after the fix)
- Date:
- Build/commit:

| Network | Boots | First-attempt success (before → after) | Success within 60 s (after) | Mean / max retries |
|---|---|---|---|---|
| Pico2_IoT | 20 | __ → __ | | |
| Phone hotspot | 10 | __ → __ | | |

### F1.7 Mission without Wi-Fi
- Date:
- Network readers found in app tasks:
- Rule added to `messages.h`: ☐  `RTOS_INTERFACES.md`: ☐
- Handed to `nav_ctrl_task` owner (who / when):
- Test result (AP off → calibrate → start → drives):

### F1.8 Demo-day network
- Date asked / answered:
- Band: · Security: · PMF:
- Matches A1? ☐ yes ☐ no → action:

**Auth-mode compatibility on Pico2_IoT (19 Oct 2026)**
- Builds: DNS profile @ `1c01d5a`, one per `UTK_WIFI_AUTH` value in
  `config/wifi_credentials.h` (`…_wifi_dns_pico2iot_<mode>.uf2`)
- 5 boots per row. Per-boot RSSI / timings weren't logged.

| Router security | Router cipher | Pico `UTK_WIFI_AUTH` | Joined + `ip=` |
|---|---|---|---|
| WPA/WPA2-Personal | TKIP+AES | `CYW43_AUTH_WPA2_AES_PSK` | ✅ 5 / 5 |
| WPA2/WPA3-Personal | AES | `CYW43_AUTH_WPA3_WPA2_AES_PSK` | ✅ 5 / 5 |
| WPA3-Personal | AES | `CYW43_AUTH_WPA3_SAE_AES_PSK` | ✅ 5 / 5 |
| WPA2/WPA3-Personal | AES | `CYW43_AUTH_WPA2_AES_PSK` | ✅ 5 / 5 (joins through WPA2) |
| WPA3-Personal | AES | `CYW43_AUTH_WPA2_AES_PSK` | ❌ 0 / 5 (negative case, as expected) |
| WPA3-Personal | AES | `CYW43_AUTH_WPA3_WPA2_AES_PSK` | ✅ 5 / 5 |
| WPA/WPA2-Personal | TKIP+AES | `CYW43_AUTH_WPA3_WPA2_AES_PSK` | ✅ 5 / 5 |
| WPA/WPA2-Personal | TKIP+AES | `CYW43_AUTH_WPA3_SAE_AES_PSK` | ❌ 0 / 5 |

Summary by build (router modes tested: WPA/WPA2, WPA2/WPA3, WPA3-only):

| Pico build | WPA/WPA2 | WPA2/WPA3 | WPA3-only |
|---|---|---|---|
| `WPA2_AES_PSK` | ✅ | ✅ | ❌ |
| `WPA3_WPA2_AES_PSK` | ✅ | ✅ | ✅ |
| `WPA3_SAE_AES_PSK` | ❌ | ✅ | ✅ |

- **WPA3 (SAE) works on the Pico W.** It only needs `UTK_WIFI_AUTH` set to a
  WPA3 mode, and the driver then advertises PMF as capable (`cyw43_ll.c:2120`).
  This contradicts A1's "no firmware can fix a WPA3-only / PMF-required AP":
  it's a one-line config change, so a WPA3 demo AP is no longer a blocker.
- **Why the two WPA3 settings differ** (`cyw43_ll_wifi_join()`, cyw43-driver
  `dd75682`): both set the WPA auth mode to SAE (`cyw43_ll.c:2065-2066`) and
  load the SAE password (`:2105-2110`), but only `WPA3_WPA2_AES_PSK` **also**
  loads the WPA2 passphrase (`WLC_SET_WSEC_PMK`, `:2094-2102`, which skips only
  `WPA3_SAE_AES_PSK`). With both keys, the radio firmware falls back to WPA2-PSK
  on a WPA2-only AP. Which method it used on each join isn't visible on the
  console. The fallback is inferred from the 5 / 5 result plus this code.
- **The WPA2 build covers WPA/WPA2 and WPA2/WPA3 networks; only a WPA3-only
  network needs the WPA3 build**, and on one the WPA2 build never joins.
  (Superseded: the default is now `CYW43_AUTH_WPA3_WPA2_AES_PSK`, see below.)
- **`WPA3_WPA2_AES_PSK` is the only build that joined every router mode tested
  (15 / 15)**, which makes it the candidate default: one build regardless of
  the demo AP's mode.
- **Same build on the phone hotspot (`Test1`, WPA2-Personal):** back-to-back
  Pico power cycles **still alternate** success / failure, exactly like the
  WPA2 build in F1.0. With the hotspot restarted every time the Pico is
  cycled, it joined **5 / 5**. So the WPA2 fallback works on the phone too, and
  the auth setting neither causes nor fixes the stale-session alternation (that
  stays F1.2's job).
- **Decision (19 Oct 2026): `CYW43_AUTH_WPA3_WPA2_AES_PSK` is the default** in
  `config/wifi_credentials.example.h`, so one build covers any demo AP mode.
  Still to do: include it in the F1.6 re-measure.
- This firmware prints nothing when a join fails (the reason is only added in
  F1.4 step 4), so a wrong-auth build looks the same on the console as an AP
  that's out of range.

### F1.9 Docs updated
- ☐ Statechart 6.1 spec ☐ Report A1 ☐ Report §5 / FR-02 ☐ GLOSSARY *Latch* ☐ NETWORKING §5 ☐ README / Progress table
- Date:

---

## F2-F6

Not started. Add sections here as the matching steps in `Actions.md` are run.
