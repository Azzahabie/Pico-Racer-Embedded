IS24
This is a very thorough design with single motor ownership, short ISRs, event-flag urgent-stop paths that bypass nav_ctrl_task, and a sensible split of control and radio across cores. The main weakness is that much of it depends on bracketed values and untested assumptions, so turn those into measured numbers quickly.
- Fill in the placeholders that make NFRs untestable (NFR-01/02/03, NFR-05, NFR-06, NFR-08, FR-27 [A] g, FR-28 [T]°, FR-30 [t_stall]) and define "defined course conditions" and "defined course lighting".
- Assumption 1 (end of course) is open, so Finished has no trigger. Pick one mechanism now, as it affects barcode_task, nav_ctrl_task and the MQTT commands.
- The pin map is careful (GP26 via Grove 5 only, IMU on I2C0 GP16/GP17, encoders on Grove 1/2 with USB console and the Maker Port empty). One correction: GP28 reads VBAT/2 only if the jumper on the back of the Robo Pico is soldered; it is not board-assigned by default, so check the jumper before relying on battery telemetry.
- The HC-SR04 is rated at 5 V; on the Grove 3.3 V rail (A5) range may be short or unstable. Test this week, and plan the 5 V supply and ECHO divider now if needed. Budget SG90 settling (100–200 ms per step) into NFR-07's 7 s.
- ISR design is sound (ENC_ISR counts, ECHO_ISR timestamps, BTN_ISR sets a flag), but core 1 timing is unvalidated. The 1 ms cyclic handler needs a 1 ms kernel tick (the template default is 10 ms). Measure WCET of each core-1 task and the quadrature ISR rate at full speed, and show NFR-11 (1 ms ± 0.5 ms) and NFR-18 (20 ms stop) hold with Wi-Fi active on core 0.
- ENC, ECHO and BTN share one GPIO callback per core. Agree a single dispatcher, state BTN_ISR's core, and check encoder bursts cannot delay ECHO timestamps.
- Junctions are "both sensors dark" for [J_n] samples; define the barcode layout relative to the line so bars cannot cause false junctions, and test both together at speed.
- Calculate how many 1 ms samples a narrow bar gives at [X cm/s] and confirm ratio [R] is resolvable, including PID corrections over the barcode.
- Hump height via Σ Δs·sinθ is affected by wheel slip and acceleration at the ramp; demonstrate it early. Validate the hump flag shared with scan_task and motion_task.
- Document the MQTT topics, payloads, command set and validation rules, and define how map versioning stops a retained …/map being re-applied mid-mission.
- Size tele_mbfid, tele_pub_mbfid, bar_mbf and cmd_mbx from worst-case rates (M7) and define bar_mbf's overflow policy; treat "command buffer full" as a rejected command rather than a system Fault.
- With ~70% Wi-Fi association success (A1), add DHCP retry and a timeout in WAITING FOR WI-FI, and ensure the mission can start without Wi-Fi.
- Do not defer the motor watchdog: if motion_task hangs, PWM keeps its last duty.
Key Risks to Validate Early
1. HC-SR04 range and stability at 3.3 V, and total stop distance.
2. Barcode and junction discrimination while moving.
3. Hump-height accuracy from encoder distance × pitch, including slip.
4. Core 1 timing: 1 ms sampling, quadrature ISR load and the 20 ms stop path.
5. Wi-Fi association and DHCP reliability on the demo network.
The design is mature; now prove its assumptions on hardware as early as possible rather than refining the documentation further