# **Team Information**

| Item | Details |
| :---- | :---- |
| Group ID |  IS-24 |
| Repository Link |   |

 

| Buddy | Student ID | Primary subsystem / persona |
| :---- | :---- | :---- |
| Buddy 1 | (removed) | Network Gatekeeper: WiFi, MQTT, telemetry |
| Buddy 2 | (removed) | Real-Time Taskmaster: motors, encoders, PID |
| Buddy 3 | (removed) | Track Purist: IR sensors, line following, barcode |
| Buddy 4 | (removed) | Physics Watchdog: IMU, terrain, hump/motion events |
| Buddy 5 | (removed) | Spatial Optimizer: ultrasonic, servo, obstacle profiling |

# **Version B: Live Zoom-Updated Version**

| Instructions • During the recorded Zoom session, update this section live or immediately from discussion notes. • Record what changed from Version A and why. A change without justification is weak evidence. • Every buddy must contribute at least one challenge, defence, trade-off, or correction. • Any use of AI during the Zoom session must be screen-shared and visible to all participants at the point it is being discussed. • If AI-generated content is introduced during the live discussion, the team must explain the prompt used, the output received, and why the output is being considered. • AI suggestions must be challenged, validated, refined, or rejected through human engineering discussion; “AI said so” is not sufficient justification. |
| :---- |

**Acceptable evidence examples:** GitHub commit link, short test log, video timestamp, oscilloscope or serial monitor screenshot, pin test result, task timing measurement, AI prompt and critique, meeting decision note, or photo of hardware wiring.

**AI-use declaration:** Do not ask AI to complete this template on behalf of the team. Use AI as a reviewer, challenger, or brainstorming partner. The team confirms that all final entries were reviewed, corrected, and accepted by the team and that every member can explain the decisions related to their subsystem.

## **Buddy Contributions During Zoom**

Each buddy must complete **one row**. The contribution must be a real technical challenge, defence, trade-off, or correction raised during Zoom, not only an attendance statement.

| Buddy | Key challenge, concern, or argument raised | Challenge received | Decision or compromise reached |
| :---- | :---- | :---- | :---- |
| Buddy 1 |  I asked buddy 2 on how using packed bits would impact the design of the telemetry buffer and if it changes interaction with the queue. To be exact the total bits size, packing/unpacking time constraint, tele buffer full behaviour and handling of out-of-bound values | Buddy 5 ask how using binary therefore folds one shared fault bitmask. Specifically whats the worst case delay when nav\_task reads the data till motion\_task executes the movement. Leading back to NFR-11. | We will keep the fault word inside a kernel object and never share as a plain variable. The event flag is where the kernel handles the bit write. A record packed by one task is read by another requires no locks |
| Buddy 2 |  Asked Buddy 3 how many bits the line error line\_state needs, how signed error is stored when Barr-C 5.4 bans signed bit-fields, and whether ir\_sense\_task publishes line\_state with one store so motion\_task can read it without the mutex. |  Buddy 1 asked:How many bits my binary telemetry record needs and how often it is sent;What happens to my 10 ms loop if a value overflows or tele\_mbf is full:Whether packing saves memory, given the kernel’s 4-byte header per message. | Telemetry record of about 54 bits (state, two speeds, distance, turn rate, fault and overflow bits), so 7 bytes, which the kernel rounds to 8; the spare bits carry a sequence number.Sent once per second and on each state change, with TMO\_POL, so a full buffer means drop and count.Out-of-range values are clamped, and an overflow bit is set.Line error stored as an offset value with line\_seen in one word, published with one store. |
| Buddy 3 |  Asked about the shared bitmask: between ir\_sense\_task(core1) and barcode\_task (core 0\) setting different bits in 1 word could lose an update. | Buddy 2 asked if the safe set slows the 1ms sampling?  |  Set fault bits through microT-kernel event-flag calls with a short hardware spinlock as the backup.  Fault will be set only when a fault arises not every tick. |
| Buddy 4 |  I asked Buddy 5 if the bit-packing changes how scan\_task stores and shares it’s data, what happens to its execution time and deadlines, and if a packed\_field could race with my huno\_state or the shared fault bits. | Buddy 3 Asked me if my imu\_task and her ir\_sense\_task are both on core 1, hers every 10ms. So when we pack and unpack data every tick, what will happen to my execution time, deadlines and if my shared date delays her 1ms sampling.  |  To come to a descision, i consulted buddy 3 during the discussion that my exection time will go up by the pack and unpack work each tick. My response time would be my own run time and only my part will grow.  Her timing will not changebrcause she outranks me and i won’t disable her interrupts STOP\_SCAN is already a single event-flag bit, so the 20 ms stop in NFR-18 is unaffected. hump\_state is safe. . |
| Buddy 5 | I asked Buddy 1 about how the requirement folds no echo, motor stall and Wi-Fi loss into one shared fault bitmask, and if the no-echo bit is set by scan\_task (core 1\) and the word is read by nav\_ctrl\_task on core 0, then what is the worst-case delay before motion\_task issues the 20 ms brake in NFR-18.  | Buddy 4 asked about the situation on if bit-packing changes, and how scan\_task stores and shares its data, then what happens to its execution time and deadlines, and could a packed field race with my hump\_state or the shared fault bits. | STOP\_SCAN is already one event-flag bit, so the 20 ms stop in NFR-18 is unaffected. Servo settling dominates the 7 s budget, so the extra shifts should be negligible; I’ll confirm with a WCET test. hump\_state is safe because I copy it under its mutex. The fault bits go in one µT-Kernel event flag that I own, with a hardware spinlock as the fallback, because the M0+ has no atomic read-modify-write and writers are on both cores. I’ll compare RAM, message size and timing before and after.  |

## **Live Decision Change Log**

Enter at least **3 changes AND 3 confirmed no-change decisions** from the Zoom discussion. Include the change type and evidence link or timestamp whenever possible. One row may state “No change” if the team defended the Week 6 design successfully.

| Original idea / Week 6 design | Updated decision | Change type | Reason for change | Evidence link / timestamp | Owner / follow-up |
| :---- | :---- | :---- | :---- | :---- | :---- |
| Outbound wire format is JSON | Packed binary record with a decoder script for the broker | Major redesign | Requirements names MQTT payloads in compact binary fields. JSON no longer compiles, custom decoder required unless standardized structure is given to all teams |  | Buddy 1 |
| Radio task is the only caller of IwIP | unchanged | No changes | IwIP is built single threaded so one caller is the hard rule, packed binary does not change this |  | Buddy 1 |
| Fault condition is as a separate flag and fields | One shard fault bitmask holding individual bits held in a kernel object | Major Redesign | Requirements ask for fault conditions in shard fault bitmask |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |
| Encoder (Buddy 2\) | Two 16-bit wrapping counts as the halves of one aligned 32-bit word ENC\_ISR writes each half with one 16-bit store. Motion\_tasks read both with one 32-bit load and takes wrap-safe differences every 10 ms. | Simplified | Meets the minimum-bit rule for data shared between an ISR and a task, removes the interrupts-off copy. At about 40 counts per tick (estimate), the 16-bit wrap is harmless. No C bit-fields, to avoid a hidden read-modify-write. | 11:46 a.m. | Buddy 2\. Logic analyser plus hand-turn test. |
| Motion Command(Buddy 2\) | 13-bit command packed into 16 bits: 3-bit opcode (7 commands), 1-bit side or fault source, 9-bit value (0 to 511 mm, estimated range), built with shifts and masks.  | Clarified  | Requirement item 7\. The µT-Kernel source stores a 4-byte header plus the message rounded up to 4 bytes, so buffer RAM stays 48 bytes before and after. We report that honestly in Week 13\.  | 11:46 a.m. | Buddy 2 with Buddy 5 (messages.h layout)  |
| Fault reporting (Buddy 2\) | Separate motor-stall and encoder-fault bits in the shared fault bitmask, implemented as a µT-Kernel event flag. The brake is written first, then the bit is set  | Clarified  | The requirement’s fault-bitmask example. Writers on both cores need an atomic kernel operation, because a plain read-modify-write would lose bits. Fallback: a hardware spinlock if the port is not safe across cores.  | 11:46 a.m. | Buddy 2 with Buddy 5 (owner of the flag), Buddy 1 and Buddy 4  |
| distance\_state (Buddy 2\) | one 32-bit word (15-bit offset distance of ±16 m, 10-bit offset turn rate of ±511 °/s, both estimated ranges, plus spare bits), built locally and published with one store. Mutex dropped.  | Simplified  | Minimum bits, and one aligned store is atomic, so imu\_task never sees half an update and motion\_task never waits on the mutex.  | 11:46 a.m. | Buddy 2 with Buddy 4  |
| Task priority and tick (Buddy 2\) | No change.  | No change | Packing adds about 1 to 2 µs per 10 ms tick (estimate, under 0.03%), and the encoder interrupt rate is unchanged.  | 11:46 a.m. | Buddy 2\. Measure with a GPIO toggle per tick.  |
| Urgent-stop path(Buddy 2\)  | No change. | No change | The STOP bits are already single bits in a kernel event flag, and the brake is a register write, not packed data.  | 11:46 a.m. | Buddy 2 with Buddy 5\. Redo the 20 ms stop test on the packed build.  |
| PID maths(Buddy 2\) | No change. | No change | Packing working variables would add cycles and overflow risk for no RAM benefit, and they are working variables rather than stored data  | 11:46 a.m. | Buddy 2  |
| Separate fault variables per subsystem (buddy 3 )  | One shared fault bitmask, set via event calls  | Clarified | read \-modify-write on packed data needs atomic access | 11:52 am | Buddy 1,2,3  |
| Barcode table: one entry per pattern | Pack 29 narrow/wide elements (29 bits) plus a 2-bit action into 1 32 bit word; a 26 entry table is about 104 bytes.  | simplified | Smaller table; the numbers come from the 29-width spec  | 11:52 am  | Buddy 3   |
| Ir\_sense\_task: 1ms, priority, priority 10   | No change | No change | Packing adds a few shifts and masks per tick and moves no deadline. WCET is unmeasured. | 11:52 am | Buddy 3  |
| One writer for line-state,mutex held only to copy  | No change | No change /  | Build the value locally store the whole 32-bit word once,motion\_task copies under mutex | 11:52 am | Buddy 3  |
| Classify barcode bars by width ratio; 1 ms width resolution | No change | No change | A narrow bar is only about 10 samples at 30 cm/s (estimated), so coarser widths would hurt the ratio decode | 11:52 am | Buddy 3  |
| Buddy 4 hump\_state is a shared struct with values NONE / CLIMBING / DESCENDING (Section 4). Motion event classes are named in FR-27; no bit widths given.  | hump\_state becomes a 2-bit field (3 values); motion event class a 3-bit field (5 classes). Single writer (imu\_task) and priority-inheritance mutex unchanged. Masks and shifts, not compiler bit-fields, for anything put on a queue or MQTT, because C bit-field layout is implementation-defined.  | Clarified  | Change Requirement: bits \= ceil(log2(number of values)): 3 values → 2 bits, 5 classes → 3 bits.  | Week 6 Section 4 (shared data), FR-27;  timestamp: 12:00pm  | Buddy 4\. Confirm hump\_state layout with Buddies 2 and 5  |
| (buddy 4\) imu\_task runs every 10 ms at priority 12 on core 1 (NFR-11, Section 3).  | No change. Pack/unpack adds a few mask and shift operations per tick  | No change  | Order against ir\_sense\_task (10) and motion\_task (11) is still right. The cyclic handler sets the period, so it does not drift. imu\_task WCET is not measured yet; Week 6 already plans to measure it with Wi-Fi on core 0\.  | Timestamp: 12:00pm | Buddy 4: measure tick interval and WCET before and after packing  |
| (buddy 4\) Hump height \= sum of Δs × sin θ using dist\_mm from distance\_state. Thresholds T \= 20°, N \= 3, θ\_on \= 5°, θ\_off \= 3°, V\_hump \= 15 cm/s (all proposed).  | No change to method or thresholds. dist\_mm stays in mm and the height accumulator keeps sub-mm fixed-point precision.  | No change | Calculated from proposed values: about 1.5 mm per 10 ms at 15 cm/s, and about 0.13 mm of height per sample at 5°. Rounding to cm or whole mm would zero most samples and break NFR-05 (0.5 cm or 20%).  | Week 6 Section 6.4 (Assumption), A4, NFR-05; calculation  |  Buddy 4 with Buddy 2  |
| avoid\_cmd\_mbf as a 4-byte command (Buddy 5\) | One 16-bit packed layout shared with Buddy 2 in messages.h (opcode, side, offset in mm). The offset width is agreed with Buddy 2 and clamped if too large.  | Simplified | Minimum bits (requirement item 7). Buffer RAM is unchanged because of the kernel’s 4-byte header, so this is for compliance, not a RAM saving. The offset range is an estimate until A5.  | 12.12 pm  | Buddy 5 with Buddy 2: agree the field widths; PC round-trip test  |
| Single motor writer, direct STOP\_SCAN wake of motion\_task (Buddy 5\) | No change | No change | Stop path is already one bit; the 20 ms (NFR-18) is the time to issue the brake | 12.12 pm | Buddy 2, Buddy 5 |
| Priorities and cores (scan\_task 13, core 1; radio on core 0\)(Buddy 5\) | No change | No change | Packing doesn’t alter the timing hierarchy; NFR-18 is still timed with WiFi on  | 12.12 pm | Buddy 5  |
| ISRs only timestamp or count (ECHO\_ISR, ENC\_ISR) | No change | No change | Packing happens in tasks, so ISR jitter and the shared core 1 callback are untouched | 12:12 pm  | Buddy 2, Buddy 5 |

## **Resource and Assumption Review**

| Item challenged | Evidence presented | Decision made | Impact on other buddies |
| :---- | :---- | :---- | :---- |
|  does packing motion\_task’s data actually save RAM? (Buddy 2\) | The µT-Kernel 3.0 messagebuf.c stores a 4-byte header plus the payload rounded to 4 bytes. The 2-byte and 4-byte commands therefore both use 8 bytes, so 48 bytes for 6 commands before and after. distance\_state goes from 8 to 4 bytes, and the encoder counts from 8 to 4 bytes (estimates).  |  11:46 and 12:56 | Buddy 1: about 7 to 9 times smaller telemetry records (estimate). Buddy 5: new command layout in messages.h. Buddy 4: distance\_state becomes one packed word.  |
|  Does packing the payloads to compact binary also improves WiFi? | Design only where with cadence of 1Hz and measured JSON of 141 bytes is about 1.1kbits/s, meaning no airtime to save | Keep it as assumptions that it will improve but would require testing | Buddy 2-5, none of them should rely on improvements to enable their design |
|  Is the 1 ms sampling safe with packing ?  | Estimation : 100 extra cycles will be about 0.08% of the 125,000-cycles tick at 125 MHz, WCET isnt measured   |  Measure with a gpio toggle over 1000 ticks, wifi streaming |  Buddy 2 and 5 (core 1 ISRs) |
| Cann a packed shared fault mask be written safely actress cores?  | M0+ has no exclusive load/store; interrupt-off is per core | event \-flag calls or a spinlock  | Buddies 1,2,4,5 |
| Narrow-bar width and 3:1 ratio | Estimated 2.1 mm emitter spot is close to a 3 mm bar | Measure the printed bars before fixing field width | Buddy 3 only |
| RAM saved by packing imu\_task data, against the NFR-12 budget (150 KB of 264 KB).  |  Nothing measured yet. The fields owned (hump\_state, event class, fault bits, a few records) are bytes, while task stacks and message buffers are far larger. Plan: linker map and stack high-water marks, baseline build before packing and packed build after.  |  Expect a real but small saving; claim a number only after measuring. Keep an unpacked baseline commit to compare against.  |  All buddies: the RAM comparison is per whole firmware. Buddies 1 and 5 hold the larger buffers (tele\_mbfid, scan data).  |
| Does a 9-bit offset (up to 511 mm) cover every bypass and reverse distance? (Buddy 5\) |  None yet: \[C\_pass\], \[D\_rev\] and \[L\_pass\] wait on the A5 tests  | To clamp, add a compile-time size check, and resize the field after A5  |  Buddy 2’s unpack code and opcode layout  |

## **Risk Assessment and Subsystem Test Plan**

### **Integration Risk Register**

Enter at least **3 integration risks**. Include at least one timing or scheduling risk, one hardware or wiring risk, and one interface or team-integration risk.

| Risk ID | Risk | Cause | Impact | Likelihood | Severity | Mitigation | Owner | Status |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| R1 | The radio task delay the brakes | Caused by a mutex on lib cyw\_43 at prio 4 | The brake can be late against 20ms baseline | Medium | high | Motion\_task tries each lock without waiting and keep the last copy so it never blocked behind a holder | Buddy 2 | Open |
| R2a | bit widths are sized from unmeasured ranges (counts per revolution, top speed, course length; A2 is still open).  | bit widths are sized from unmeasured ranges (counts per revolution, top speed, course length; A2 is still open)  | wrong distance and turns, and wrong hump height for Buddy 4\.  | Medium  | Medium  | clamp and set an overflow bit; wrap-safe 16-bit differences; confirm the ranges after the A2 hand-turn test; PC tests at the limits.  | Buddy 2  | Open  |
| R2b | fault bits lost between core ss | the event flag’s cross-core atomicity is unverified in our µT-Kernel RP2040 port.  | a fault is not reported or acted on.  | Low (unverified) | High  | brake before setting the bit; stress test with both cores setting bits; hardware spinlock as the fallback  | Buddy 2 and Buddy 5  | Open  |
| R3a | Timing : ir-sense-task misses a 1 ms sample  | Critical sections on packed data or user bursts on core 1 with packing added | Distorted line error or barcode width  | medium | High  | Keep critical sections to a few instructions; measure NFR-11 jitter with Wi-Fi on  | Buddy 3  | open |
| R3b | Shared resource : lost update in the fault bit mask | Two cores doing read-modify-write on one word | A fault is silently missed  | medium | High  | Atomic set via event flags; two-core stress test | Buddy 3 with 1 and 2 | Open  |
| R4 | Interface: the avoidance command is cut off or misread  | Offset bigger than 511 mm, or the two tasks pack it differently  | The car turns back in too early and hits the obstacle  | Medium  | High  | One messages.h definition, clamping, round-trip test  | Buddy 5, Buddy 2  | Open  |
| R5 | Timing: packing adds jitter to the 1 ms and 10 ms loops  | Packing or unpacking inside ir\_sense\_task or the ISRs; shared core 1 GPIO callback  | Misses NFR-11 or the 20 ms stop (NFR-18)  | Medium  | High  | Pack only in tasks, never in ISRs; GPIO-toggle WCET before and after, with WiFi on  | Buddy 2, Buddy 3  | Open |

 

### 

### **Subsystem Test Plan**

Enter **at least 4 cross-subsystem tests**. Each test must involve the work of **at least two buddies** and must include a success criterion and evidence type, such as a video, serial log, screenshot, timing result, or commit link. At least **TWO** test should involve timing, safety, or shared-resource concern.

| Subsystem | Test case | Success criteria | Evidence required | Owner | Status |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **Fault bitmask** Buddies 1, 2, 4, 5; shared resource  | tasks on both cores set and clear their own fault bits repeatedly for 10 minutes.  | every bit set is seen, and no bit is lost or cleared by another task. Logged counts match.  | serial log with counters  | Buddy 2  | Open  |
| Inbound command and remap | Publish map and snap from broker while the car runs | The remap applies with no recompile | Broker log  | 1 & 3 |  |
| IMU stop path (Buddy 4 \+ Buddy 2, Wi-Fi load from Buddy 1\) – timing and safety  | Tilt the car past T \= 20° for 3 samples while motion\_task runs the PID and Wi-Fi is streaming. Run on the unpacked baseline and on the packed build.  | STOP\_IMU raised within 40 ms of the first sample above T (NFR-19) and motor-stop command within 20 ms of the request (NFR-18), worst of 100 trials. Packed build no worse than baseline.  | Logic-analyser capture and serial log with timestamps; commit links for both builds.  | Buddy 4 | Open  |
| Fault flag (Buddies 1, 2, 3, 4, 5\)  | Each task sets and clears its own fault bit at a high rate from both cores  | No bit lost or cleared by another task over a set run time  | Serial log with counters  | Buddy 5  | Open  |

 

## **Constraint Injection / Curveball Response**

| Instructions • Record the injected constraint exactly as given. • Explain how it affects all five buddies, even if the impact differs by subsystem. • A good response shows prioritisation, trade-off reasoning, fallback behaviour, and evidence-aware decision making. |
| :---- |

 

| Injected constraint | Buddy affected | Immediate impact | Adaptation / compromise |
| :---- | :---- | :---- | :---- |
|   | Buddy 1 |   |   |
|   | Buddy 2 | Commands go from 4 bytes to 13 bits (2 bytes), but buffer RAM is unchanged because of the kernel header. distance\_state, the encoder counts and telemetry are packed, and motion fault bits move into the shared bitmask. Packing costs about 1 to 2 µs per 10 ms tick (estimate). The main risk is read-modify-write on shared packed words.  |  One writer per packed word, published with one store. Encoder counts as two 16-bit halves, without bit-fields. Fault bitmask as a µT-Kernel event flag, with a spinlock fallback. Signed values offset-encoded (Barr-C 5.3), and MQTT built with shifts and masks in a fixed byte order (Barr-C 5.5). Clamp plus an overflow bit. Priority, tick and brake path unchanged.  |
| The robot car firmware must minimise the number of bits used to represent application data. Each data field representing application information must use only the number of bits required to represent its valid range, using C bit-fields or explicit masks and shifts where appropriate… For MQTT communication, application data should be represented using compact binary fields rather than sending individual values as text… Read-modify-write operations on shared packed data must not introduce race conditions, and atomic access or appropriate synchronisation must be used where required.   | Buddy 3 |  bar\_mbf (2-bit type \+ 7 or 8-bit width), line\_state, the 2-bit navigation command, and the barcode table (29 bits plus a 2-bit action) are packed  |  Build values locally and store the whole word once. The shared fault mask uses event-flag calls. No change to 1 ms, priority 10 or core 1\. Savings and timing are not measured yet.  |
|  The robot car firmware must minimise the number of bits used to represent application data. Each data field representing application information must use only the number of bits required to represent its valid range, using C bit-fields or explicit masks and shifts where appropriate.” \[…\] “When multiple values are packed into the same storage unit and that data is shared between tasks, or between an ISR and a task, access must be implemented safely. | Buddy 4 | The robot car firmware must minimise the number of bits used to represent application data. Each data field representing application information must use only the number of bits required to represent its valid range, using C bit-fields or explicit masks and shifts where appropriate.” \[…\] “When multiple values are packed into the same storage unit and that data is shared between tasks, or between an ISR and a task, access must be implemented safely.”  | Safety and accuracy come before bits: STOP\_IMU stays a separate event flag, dist\_mm stays in mm, the height accumulator keeps sub-mm precision. One writer per packed word; fault causes via event flags; nav\_ctrl\_task composes the shared mask. Fallback: if timing or accuracy gets worse after packing, leave that field unpacked and record why in the data representation table. Week 13 evidence: representation table, RAM (linker map and stack high-water), message size, re-run of NFR-05, NFR-18 and NFR-19 tests.  |
|  | Buddy 5 | avoid\_cmd\_mbf packed; obstacle-profile and sensor-fault records go binary; no-echo becomes a bit in the shared flag; scan state fits 3 bits  | Scan arrays kept unpacked; clamp offsets that don’t fit; own the fault event flag; the stop path and its ordering are unchanged  |

* 

Integration Readine**ss Self-Assessment**

Complete all rows. Use **Green** only if there is evidence, **Yellow** if partly ready or untested, and **Red** if the area is unclear or blocking Week 10 integration.

| Readiness area | Status | Evidence | Concern |
| :---- | :---- | :---- | :---- |
| Hardware wiring known | Green / Amber / Red |  |  |
| RTOS task ownership clear | Green / Amber / Red | Week 6 Section 9 consistency check: one owner per device; only motion\_task writes the motors; fault flag owner agreed as Buddy 5 in Version B  | None  |
| Shared resources identified | Green / Amber / Red | Shared Resource Allocation Matrix (pre-tribunal)  | Packed shared words (distance\_state, line\_state, encoder counts) and the fault event flag are new and must be added to the matrix  |
| Critical statecharts implementable | Green / Amber / Red | Statecharts 6.1–6.5 drawn with states, events and fault paths; scan\_logic.c designed to be tested on a PC  | Not yet coded or run on hardware  |
| Safety or fault handling defined | Green / Amber / Red |  |  |
| Week 10 partial integration feasible | Green / Amber / Red |  |  |

 

**Updated readiness position before the Week 10 partial integration demo:**

Overall: Amber**.** The Week 6 design survives the change. Single motor ownership, the direct STOP-bit stop path and the split of control tasks on core 1 and radio on core 0 all stay. The minimum-bit requirement mainly changes data layouts and shared-data access, not the task structure. Hardware evidence is still thin. Only the Wi-Fi and MQTT network layers are proven on the board. Line and barcode logic pass PC tests. Motion, IMU and ultrasonic are still design only.

Before Week 10 we will:

1. Freeze one shared messages.h containing every packed message, the shared fault bitmask (as a µT-Kernel event flag) and the binary MQTT record layout, so all five buddies build against the same format.  
2. Build an unpacked baseline first and record its RAM use and message sizes, because Week 13 needs a before-and-after comparison.  
3. Switch the kernel tick to 1 ms.  
4. Run the bring-up tests: encoder hand-turn (A2), HC-SR04 range at 3.3 V (A5), IMU characterisation (A4) and Wi-Fi join with the timer fix (A1).

The Week 10 target is a partial integration: line following with motion\_task, the STOP\_SCAN emergency stop, and binary telemetry reaching the laptop. Packed data will be in place, and timing will be checked against NFR-11 (task timing) and NFR-18 (20 ms stop).

#  

 

 

 

 

 

 

 

 

 

 

