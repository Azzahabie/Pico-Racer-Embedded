# Actions: resolving the Week 6 feedback (Buddy 1)

The exact steps to close each feedback point in
[`../WEEK6_FEEDBACK.md`](../WEEK6_FEEDBACK.md). This file says **what to do**.
[`results.md`](results.md) records **what happened** when it was done.

- **Each step has an ID** (`F1.3`). Its outcome goes into `results.md` under the
  same ID.
- **Status:** ⬜ not started · 🟡 in progress · ✅ done · ⛔ blocked (say why in
  `results.md`).
- **Terms** are defined in [`../GLOSSARY.md`](../GLOSSARY.md).
- **Build rule:** every build goes through Docker
  (`docker compose run --rm build bash -lc "cd build_make && make <flags> -j8"`).
  Never use a host toolchain. Edit sources at the repo root, never under
  `build_make/mtkernel_3/`.

## Progress

| Point | Topic | Steps planned | Status |
|---|---|---|---|
| F1 | Wi-Fi/DHCP reliability, mission without Wi-Fi | F1.0-F1.9 | 🟡 F1.1 done |
| F2 | MQTT interface doc + map versioning | not yet planned | ⬜ |
| F3 | Queue sizing, command-full ≠ Fault | not yet planned | ⬜ |
| F4 | Core 1 timing with Wi-Fi active | not yet planned | ⬜ |
| F5 | End-of-course mechanism | not yet planned | ⬜ |
| F6 | GP28 battery jumper | not yet planned | ⬜ |

---

## F1: Wi-Fi/DHCP reliability, and running without Wi-Fi

**Goal.** The car keeps trying until it has Wi-Fi and an IP address, recovers on
its own after losing either, and never needs Wi-Fi to start or run a mission.
Turn A1's "~70%" into a measured number.

**Why (from the code).** All of this is in
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`:

| Problem | Where | Effect |
|---|---|---|
| **Join is one-shot.** The task calls `cyw43_wifi_join()` once, after the first scan. On failure or a 30 s timeout it sets `join_complete = 1` with an error and never tries again. | lines 609-656, `CYW43_JOIN_TIMEOUT_US` line 45 | A boot that fails to associate stays offline until a power cycle. |
| **The DHCP timeout is a dead-end latch.** After 30 s without a lease it sets `dhcp_result = ERR_TIMEOUT` and `dhcp_complete = 1`. | lines 468-473, `CYW43_DHCP_TIMEOUT_US` line 46 | A lease that arrives **later** is ignored: line 446 only looks for an address while `!dhcp_complete`. DNS never starts (line 355 needs `ERR_OK`), and MQTT polls with no address (line 598 gates only on `dhcp_complete`). This matches the A1 log `dhcp=0 addr=0.0.0.0`. |
| **Losing the link resets nothing.** `cyw43_utk_link_state_changed(0)` only clears `link_up`. `dhcp_complete` and the join state stay set. | lines 477-482 | After an AP drop there's no path back to "join → lease". |

**Constraint.** lwIP is single-threaded (`NO_SYS 1`), so **all** retry logic must
live inside `cyw43_utk_task`'s poll loop. Nothing else may call lwIP or CYW43.

---

### F1.0 Baseline: measure today's success rate ⬜

Measure first, so the fix can be compared against real numbers.

1. Build the DNS profile (association + DHCP + DNS, no peers needed):
   ```sh
   docker compose run --rm build bash -lc "cd build_make && make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 -j8"
   ```
   Output: `build_make/mtk3pico_smp1_usb_cdc_wifi_dns.uf2`.
2. Flash it. Open the COM port at 115200 and log the session to a file.
3. Do **20 cold boots** (unplug the power each time, not just reset) on
   `Pico2_IoT`. For each boot, record in `results.md` → F1.0:
   - `[wifi] link UP` seen? (yes/no) and its RSSI
   - `[wifi] ip=` line: a real address, or `0.0.0.0`
   - time from power-on to the `ip=` line (approximate is fine)
4. Repeat on the phone hotspot (at least 10 boots).
5. Save the raw logs to a folder you can cite in the report, e.g.
   `docs/comms/resolve_feedback/logs/`. Check them for secrets before
   committing: no SSID passwords.

**Done when:** `results.md` F1.0 has per-boot rows and a success-rate line for
each network.

### F1.1 Confirm where DHCP is started and stopped ✅

Before changing anything, find which code calls `dhcp_start()`. It isn't in our
repo: it's in the Pico SDK's CYW43 lwIP glue (`cyw43_lwip.c`, built per
`build_make/pico_rp2040.mk:518`). The SDK is cloned only inside the Docker image
(`Dockerfile:42-47`), so read it on GitHub at the pinned versions instead:

- Pico SDK `2.2.0` pins cyw43-driver at `dd75682` and lwIP at `77dcd25`
  (links in `results.md` → F1.1).

1. Read `cyw43_lwip.c` / `cyw43_ctrl.c` (cyw43-driver) and `netif.c` /
   `ipv4/dhcp.c` (lwIP).
2. Record in `results.md` → F1.1:
   - Is `dhcp_start()` called once at netif init, or on every link-up?
   - Does link-down call `dhcp_stop()` / `dhcp_release_and_stop()`?
   - Does lwIP's own DHCP client keep retrying after our 30 s "timeout"? The
     `dhcp_tries` field in the status struct shows this. Log it in F1.0 if you
     can.

**Why it matters:** if lwIP keeps retrying on its own, F1.3 may only need to
remove the latch. If DHCP runs only once at init, F1.3 must restart it.

**Done when:** the three answers are written down. They decide F1.3's exact
code.

**Result (2026-10-16):** DHCP is started once at boot. Link-down doesn't stop
it. lwIP retries forever and re-leases on every link-up, but its back-off grows
to 60 s. So F1.3 only removes the latch, and F1.2's rejoin stays, to keep
recovery fast. Details are in `results.md` → F1.1.

### F1.2 Add a connection state machine in `cyw43_utk_task` ⬜

Replace the one-shot join and latch with an explicit state, matching
Statechart 6.1's WAITING FOR WI-FI.

1. In `cyw43_utk.c`, add a static state variable for the radio loop:
   `NET_JOINING → NET_WAIT_LEASE → NET_ONLINE`, plus `NET_BACKOFF`.
2. Make the timeouts overridable (`#ifndef … #define`) so they can be tuned
   without editing logic:

   | Define | Proposed | Replaces |
   |---|---|---|
   | `CYW43_JOIN_TIMEOUT_US` | 15 s | 30 s |
   | `CYW43_DHCP_TIMEOUT_US` | 10 s | 30 s |
   | `CYW43_RETRY_BACKOFF_US` | 3 s (fixed, within Statechart 6.1's 2-5 s) | new |

3. Transitions:
   - **JOINING:** call `cyw43_wifi_join()`. Link UP → WAIT_LEASE. Join error
     or timeout → BACKOFF.
   - **WAIT_LEASE:** an address arrives (`dhcp_supplied_address()` with gateway
     and DNS set, as now) → ONLINE. No lease within the DHCP timeout → BACKOFF.
     Link down → BACKOFF.
   - **ONLINE:** link down → BACKOFF.
   - **BACKOFF:** wait the back-off, call `cyw43_wifi_leave()`, then go to
     JOINING. A full leave + rejoin is the simple, reliable recovery and also
     resets the AP's view of the car. Per F1.1 it also resets lwIP's DHCP
     back-off (`tries = 0` on link-up). Otherwise that back-off grows to 60 s
     between DISCOVERs, which would break NFR-09's 15 s.
4. Keep the first scan as it is. The join now starts from JOINING instead of
   from the `was_scanning && !scan_active` edge.
5. Keep each poll iteration short. The state machine only checks timestamps
   and flags. It never waits inside the loop.

**Done when:** it builds in the DNS profile (F1.0's command) with no warnings
added.

### F1.3 Replace the DHCP latch with "address valid now" ⬜

1. Stop treating `dhcp_complete` as permanent. Either:
   - **(preferred)** keep `dhcp_complete` but **clear it** (and `dns_complete`,
     `dhcp_result`) when the state leaves ONLINE, or
   - add a new field `addr_valid` and move every reader to it.
2. **No DHCP calls needed** (F1.1). lwIP's client runs for the whole boot and
   restarts discovery by itself on every link-up. Don't add `dhcp_start()` /
   `dhcp_stop()`, because doing so would fight the driver's own call in
   `cyw43_cb_tcpip_init()`. Just keep reading the netif's address after a
   timeout, instead of ignoring it.
3. Reset `dhcp_started_us` and `dhcp_ipv4_adopted` when leaving ONLINE, so the
   next lease is timed and adopted fresh (lines 418-419, 438-440, 457-460).
4. Let DNS restart: reset `dns_started` / `dns_started_us` when leaving ONLINE,
   so `update_dns_status()` (line 355) runs again on the next lease.
5. Update the MQTT gate at line 598 to require a **current** lease
   (`dhcp_complete && dhcp_result == ERR_OK`), not just "DHCP finished".
6. Check `lib/libnet/lwip/lwip_utk_mqtt.c`. It is one-shot (`finish()` sets
   `complete`). Reconnecting MQTT belongs to FR-25 / M9, **not F1**. Just make
   sure a lost lease doesn't crash it: it already checks
   `netif_is_link_up()` at line 168.

**Done when:** builds in both the DNS profile and the MQTT profile:
```sh
docker compose run --rm build bash -lc "cd build_make && make SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_UDP=1 WIFI_TCP=1 WIFI_MQTT=1 -j8"
```

### F1.4 Make retries visible ⬜

Evidence needs counters, not silence.

1. Add to `T_CYW43_UTK_STATUS` (`cyw43_utk.h`): `net_state`, `join_attempts`,
   `lease_attempts`, `link_losses`. Keep the existing `typedef char
   …_size_matches[…]` static asserts compiling. Update them if they cover this
   struct.
2. In the console monitor (`app_program/app_main.c` ~line 290), print one line
   per state change, built with `tm_sprintf` and sent with **one** `tm_printf`
   (AGENTS.md §7):
   `[wifi] state=WAIT_LEASE join#=2 lease#=1 lost=0`
3. Make the existing `[wifi] ip=` line print again on **every** new lease, not
   only the first. It keys on `dhcp_complete` transitions, so F1.3's clearing
   makes this automatic. Check it.

**Done when:** a boot with the AP off, then on, shows the state lines and
ends in `ip=…`.

### F1.5 Recovery tests on hardware ⬜

Flash the F1.3 DNS-profile build and run these, recording each in
`results.md` → F1.5:

| Test | How | Pass |
|---|---|---|
| T1: AP off at boot | Router off, power the Pico, wait 60 s, router on | `ip=` appears without a reset. Note the time from router-on to `ip=` |
| T2: AP drop while online | Get `ip=`, switch the router off for 30 s, then on | `link DOWN`, back-off lines, then a new `ip=` |
| T3: DHCP refused | Router on, but its DHCP pool full or disabled (or the hotspot in its "no lease" state from A1) | Repeated `lease#` increments, no crash, recovers when DHCP returns |
| T4: wrong password | Build with a bad password in `config/wifi_credentials.h` | Repeated `join#` increments at the back-off rate. The rest of the system keeps running (LED/monitor alive) |
| T5: soak | Leave it online for 30 min | Still online, no unexpected `link_losses` |

Restore the correct credentials after T4. Never commit
`config/wifi_credentials.h`.

**Done when:** T1-T5 all pass, or the failures are logged with what was seen.

### F1.6 Re-measure A1 with the fix ⬜

Repeat F1.0 exactly (20 cold boots on `Pico2_IoT`, 10 on the hotspot) with
the new build. Record **first-attempt success**, **eventual success within 60 s**,
and the mean/max retries. Then compare against F1.0 in `results.md`.

**Done when:** `results.md` F1.6 has the before/after table.

### F1.7 Guarantee a mission can start without Wi-Fi ⬜

`nav_ctrl_task` doesn't exist yet, so do this as a contract plus a check:

1. Confirm nothing outside the radio path waits on the network: `grep` the app
   tasks for `cyw43_utk_get_status`, and check that none of them blocks on a
   Wi-Fi result. Today only the console monitor in `app_main.c` reads it.
2. Write the rule into the integration contract
   (`app_program/common/messages.h` comment, plus
   [`../RTOS_INTERFACES.md`](../RTOS_INTERFACES.md)): *"CONN on `nav_flg` is
   informational. No transition into Ready or Running may wait for it."*
3. Tell the `nav_ctrl_task` owner, and add it to their acceptance test: boot
   with the AP off → calibrate → start → the car drives.
4. Once `nav_ctrl_task` exists, run that test and log it in `results.md` →
   F1.7.

**Done when:** the rule is in both files, and the test is either logged or
handed to the owner (note who and when).

### F1.8 Demo-day network check ⬜

1. Ask the instructor for the demo AP's band, security mode, and whether PMF
   is required or optional. Record the answer and date in `results.md`.
2. If it's WPA3-only or PMF-required, raise it immediately. A1 says no firmware
   can fix that. `UTK_WIFI_AUTH` in `config/wifi_credentials.h` sets the auth
   mode we request.
3. If possible, test on the real AP (or an identically configured one) before
   demo day.

**Done when:** the network details are recorded and checked against A1.

### F1.9 Update the docs and the report ⬜

Do this last, from the evidence:

1. **Statechart 6.1** ([`../STATECHART1_DIAGRAM_SPEC.md`](../STATECHART1_DIAGRAM_SPEC.md)):
   add the join/lease timeout → back-off → rejoin edges out of WAITING FOR
   WI-FI, and remove "the latch never clears / never returns to BOOT".
2. **Report A1:** replace "~70%" with the F1.6 numbers and cite the logs.
   Change the status from "Pending validation" to the result.
3. **Report Section 5 / FR-02:** add "Ready → Running does not depend on Wi-Fi".
4. **[`../GLOSSARY.md`](../GLOSSARY.md):** the *Latch* entry cites
   `dhcp_complete` as never clearing. Update it.
5. **[`../NETWORKING.md`](../NETWORKING.md) §5:** add the recovery evidence (T1-T5).
6. Mark F1 ✅ in the Progress table above and in [`../README.md`](../README.md).

**Done when:** all six are updated, and nothing still describes the old
one-shot behaviour.
