# Phase 4L.6A firmware V3.1 and guarded bench protocol

ESP32-S3 UART1 uses 57600 baud, 8N1 and 3.3 V logic with a common ground.
Connect Pi pin 8 (GPIO14 TX) to ESP GPIO18 RX and Pi pin 10 (GPIO15 RX) to
ESP GPIO17 TX. The Pi device is `/dev/serial0`. The firmware starts with motor
outputs disabled. Bench operation requires both wheels securely raised and a
verified physical motor-power cutoff. Bridge confirmation depends on the operator
having performed those checks.

## UART requests

Send printable ASCII lines terminated by CR or LF. Replies end in CRLF. Use one
request at a time and read its reply before sending another. Commands are
case-sensitive.

| Request | Successful reply | Behavior |
| --- | --- | --- |
| `VER` | `VERSION PV=3 CAP=ARM_CONT,STICKY_FAULT,STATIONARY,BOOT_ID` | Identifies the baseline protocol. V3.1 preserves the exact reply for Bridge V3. |
| `CAPS` | `CAPS PV=3.1 CAP=ARM_LOW,LOW_FIXED_PAIR,LOW_SESSION_15S` | Identifies the extension. Send it only after `Q` advertises `PV=3.1`. A build with characterization disabled reports `CAP=NONE`. |
| `Q` | `STATUS key=value ...` | Reads firmware state and normalized wheel feedback. |
| `T 012345` | `ACK 012345` | Echoes exactly six decimal digits. |
| `ARM` | `OK ARMED 5S` | Legacy arming. Motion must end within five seconds of arming, regardless of velocity renewals. |
| `ARM_CONT` | `OK ARMED CONT` | Arms only while disarmed, stationary and free of latched faults. The first valid motion command must arrive within five seconds. |
| `ARM_LOW <signed_rpm> WHEELS_UP` | `OK ARMED LOW` | Arms a single signed characterization pair while stationary, disarmed and free of faults. |
| `V <left_rpm> <right_rpm>` | `OK V` | Updates signed wheel targets. A valid nonzero frame in an authorized session renews the active heartbeat. Both targets zero stops and disarms. |
| `STOP` | `OK STOP` | Disables both drivers, disarms and preserves any fault latch. |

A continuous session can pass five seconds once motion has started. It must
receive valid authorized velocity frames less than 300 ms apart. `Q`, `T` and
`VER` never renew the heartbeat. An unused arm expires and returns to idle. Loss
of the active heartbeat or expiry of the active legacy motion window latches a
fault. The ESP32 enforces these deadlines independently of the Pi.

In continuous mode each target must be zero or have magnitude 5–40 RPM, the
existing firmware setpoint range. Unsupported nonzero magnitudes below 5 RPM
return `ERR LOW_SPEED`. The legacy near-zero input deadband is still accepted.
The firmware retains each wheel's last nonzero commanded direction through zero
targets and STOP. A reversal requires disarming and verified stationary
feedback, then arming again. Firmware stationary verification requires at least
250 ms with no encoder count changes and both filtered speed magnitudes below
1 RPM. A reversal attempted too early returns `ERR STOP_BEFORE_REVERSE` and
latches a fault. The existing 45% duty cap and forward-straight encoder sync
remain in effect. Continuous mode also trips a stall fault after 800 ms below
1 measured RPM in the commanded direction, even at low duty.

## Status fields

The original `A RUN L R DL DR TL TR UE CF` fields are preserved.

| Field | Meaning |
| --- | --- |
| `A`, `RUN` | Armed and active flags. `RUN=1` reports an accepted motion command. |
| `L`, `R` | Commanded RPM / filtered measured RPM, forward-positive on each wheel. |
| `DL`, `DR` | Applied PWM duty magnitude in percent. Zero while the driver enable is low. |
| `TL`, `TR` | Signed normalized encoder counts at 3960 counts per revolution. |
| `UE`, `CF` | UART and control fault counters since boot. |
| `MODE` | `IDLE`, `LEGACY`, `CONT` or `LOW`. |
| `FL`, `FR` | Sticky fault flag and first fault reason, or `NONE`. |
| `ST` | Firmware stationary verification flag. |
| `BOOT` | Eight-digit hexadecimal boot identifier used to detect resets. |
| `PV` | V3.1 status extension marker, currently `3.1`. |
| `SC` | Current straight-sync correction in RPM, added to left and subtracted from right. Always zero in `LOW`. |

Framing errors, invalid bytes, gaps, overflows and malformed commands latch a
fault. Encoder read failures or disagreement, control timing faults, overspeed,
stall, active deadline expiry and unsafe setpoint changes also latch a fault.
`STOP` remains available after a fault. All arm commands refuse to arm while
`FL=1`.

## Baseline Bridge V3 operation and recovery

By default `amr_ros2_bridge_v3.py` polls telemetry and creates no command
subscription. It can read the legacy status format. It still requests STOP on
shutdown or on a safety failure. The Pi must have ROS 2 Humble and pyserial in
the Python environment that runs the bridge. Stop other UART users first,
including any serial console and earlier bridge. V3 uses pyserial's exclusive
lock and Linux `TIOCEXCL`. A process already holding the device can remain an
owner if it ignores advisory locking.

Enable the bench mode with `--continuous-bench` and confirm `WHEELS_UP`
interactively after securing the stand and testing the physical cutoff.
The only motion input is `geometry_msgs/msg/TwistStamped` on
`/amr_bridge/bench_cmd_vel`. Stamp every sample at publication using the Pi's
system clock. For a remote publisher, synchronize its system clock with the Pi.
The bridge rejects expired, future-dated, repeated and pre-READY stamps.
The subscription uses volatile, best-effort QoS with depth 1. A reliable live
publisher can also connect. The bridge accepts only planar `linear.x` and
`angular.z`, and validates all six components.

The bridge proportionally limits setpoints to 15 RPM, 0.06 m/s and 0.35 rad/s.
From rest each requested active wheel starts at 5 RPM, the smallest supported
nonzero setpoint. Subsequent changes slew at up to 15 RPM/s per wheel. A target
below 5 RPM after limiting is rejected and reported. A change to the commanded
wheel direction or to which wheels are driven requests STOP. During
`STOPPING`, the bridge verifies idle via STOP acknowledgement and a fresh
`Q`. It then requires `ST=1`, measured speeds below 1 RPM and unchanged counts
for at least another 250 ms. `READY` requires a newly stamped nonzero command
to start. Normal zero uses the same stop transition. A 230 ms command gap
during `RUNNING` latches a bridge fault. The 15-second first-arm budget includes
all intervening stops and restarts. The bridge stops accepting motion 400 ms
before the deadline to leave time for UART, the firmware watchdog and the
control loop.

V3 publishes `/joint_states` with the existing joint names, radians and
radians per second. `/amr_bridge/status` reports bridge and verified protocol
versions, state, requested/limited/acknowledged wheel RPM, firmware measurements
and sample age, command age, arm/run flags, and faults and counters. Unknown
measurements and unavailable legacy fields appear as JSON `null`. RPM last
acknowledged by firmware and measured wheel RPM are reported separately.

For recovery, cut motor power physically, stop the command publisher and record
the bridge reason and firmware `FL FR UE CF` fields. Diagnose and correct the
cause with the wheels secured. Restart the ESP32 logic with motor power isolated
to clear a firmware latch, then check `Q` for `A=0 RUN=0 FL=0 ST=1`. A
bridge-only fault also requires closing and restarting V3. Before any later
motion, repeat the supervised setup and interactive confirmation. No automatic
re-arm path is provided. If V3 cannot verify final STOP via an acknowledgement
and fresh idle `Q`, **cut motor power physically**.

## Low-speed characterization

`ARM_LOW` fixes both wheels to the same signed candidate. Select one of
`2`, `3`, `4`, `5`, `-2`, `-3`, `-4` or `-5`. The confirmation token is
an operator attestation. The ESP32 has no stand sensor. After arming, only
the selected pair or two zeros are accepted. For example, a selection of
`-2` accepts `V -2 -2` and `V 0 0`. Changing the candidate, choosing a single
active wheel, or reversing within that selection latches `LOW_SETPOINT`.
Use STOP and complete stationary verification before a new run. The firmware
allows five seconds to send the first motion frame, keeps the independent
300 ms heartbeat, and adds an absolute 15-second limit from arming. An active
run that reaches that limit latches `LOW_SESSION_EXPIRED`. Queries and echo
frames do not renew either deadline.

This mode disables forward straight-wheel sync to make per-wheel behavior
visible. It retains the 20 ms PI loop, 45% output cap, 3 percentage point
duty ramp per control tick, the original high-duty stall check, and the
continuous-mode 800 ms signed-progress check at 1 RPM. It also keeps the
existing encoder, timing and overspeed checks.

At 2–5 RPM the feedforward term is approximately 12.48–13.20% duty, plus the
existing 1.5 percentage point left-forward trim. The proportional and integral
terms add duty if measured speed falls below the target. The integral is
limited to ±15 percentage points, and the signed-progress check can trip
before the PI loop builds enough duty to overcome breakaway friction. The
feedforward bias may also produce a transient overshoot once a wheel breaks
away. Reverse transient asymmetry was observed during Phase 4L.4. Candidate
speeds, tracking and breakaway duty require physical measurement. If the
candidates stall or track poorly, use a separate supervised calibration to
measure breakaway and sustained duty by wheel and direction before changing
feedforward or PI gains.

## Bridge V3.1 and Nav2 preview

`amr_ros2_bridge_v3_1.py` uses the unchanged V3 UART transport and STOP
handshake. It defaults to read-only. Normal continuous bench mode retains
`--continuous-bench`, `WHEELS_UP` and `/amr_bridge/bench_cmd_vel`.
`--low-speed-bench --rpm N --direction forward|reverse` requires a separate
`WHEELS_UP_LOW_SPEED` confirmation and subscribes only to
`/amr_bridge/low_speed_cmd_vel` with `geometry_msgs/msg/TwistStamped`. Send
straight velocity `signed_rpm * 2*pi*0.0435/60` m/s and zero angular velocity.
The selected experiment can start once per bridge invocation. Both ROS bench
modes require fresh system-clock publisher stamps and keep the 230 ms command
deadman and 15-second session budget. Publisher timestamps, command timeouts,
STOP verification, boot identity and fault counters remain part of admission.

V3.1 publishes the existing joint and bridge status topics. `STARTING` means a
velocity frame was acknowledged and the bridge is waiting for a subsequent
`Q` to confirm firmware `RUN=1`. Only that confirmation transitions the bridge
to `RUNNING`. `requested_rpm`, `limited_rpm`, `acknowledged_rpm`,
`measured_rpm` and `pwm_duty_percent` describe different observations. Acknowledged
RPM can retain the last accepted velocity after STOP. Check `stop_verified`,
`firmware_run_confirmed`, feedback age and encoder measurements before
interpreting it as activity. Firmware `RUN=1` indicates an active control
session. Physical wheel movement is established from encoder feedback.

The Nav2 monitor accepts unstamped Twist data for offline evaluation. It
validates all six components, uses the existing differential-drive
kinematics, and applies proportional limiting to 0.06 m/s, 0.35 rad/s and
15 RPM. For a forward pair that could enter firmware sync, it reserves
up to 1 RPM of correction per wheel within that 15 RPM envelope. Monitor
results carry no command timestamp, UART operation or motor authority.

| Input | Proposed near-zero policy |
| --- | --- |
| Exact zero | Request normal STOP and verify idle and stationary feedback. Accept a later fresh nonzero command after READY. |
| Tiny nonzero | Inhibit the pair. If motion is active, request STOP. |
| One nonzero wheel below the supported minimum | Inhibit the entire pair. A wheel requested at exactly zero can stay off if the other wheel is supported. |
| A wheel changes direction or enters or leaves the driven pair | STOP, verify stationary feedback and require fresh input after READY. |
| Invalid, stale or lost input | Inhibit input and STOP active motion. Safety faults require deliberate operator recovery. |

The monitor uses 5 RPM as the current normal firmware threshold. Set any
future ground-control minimum from measured stable results by wheel,
direction and load. Ground-control activation would require explicit
operator enable, exclusive UART ownership and one authorized upstream
command owner or arbiter. Use a volatile depth-one queue and a verifiable
source-age contract across the Nav2 handoff. A trusted adapter could timestamp
on receipt and supervise arrival gaps, but that stamp cannot establish when
Nav2 originally published the command. Treat publisher age and receipt age
separately. On ownership loss, deactivation, stale input or process shutdown,
send STOP and verify acknowledged idle `Q` and stationary feedback. The
firmware watchdog must continue to operate independently.

## Exact verification commands

Run the offline checks from the firmware repository. The Humble integration
tests use mocked UART and do not send motion messages.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
PYTHONPYCACHEPREFIX=$(mktemp -d) python3 -m py_compile amr_ros2_bridge_v3_1.py tools/amr_v3_low_speed_test.py tests/test_amr_ros2_bridge_v3_1.py
native_test_dir=$(mktemp -d)
cc -std=c11 -Wall -Wextra -Werror -pedantic -I main tests/test_low_speed_policy.c -o "$native_test_dir/test_low_speed_policy"
"$native_test_dir/test_low_speed_policy"
python3 -B amr_ros2_bridge_v3_1.py --monitor-twist 0.06 0
python3 -B tools/amr_v3_low_speed_test.py --help
```

Build for ESP32-S3 with 16 MB flash configuration using an external build
directory and temporary SDK config. This compiles the image only.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source ~/.espressif/tools/activate_idf_v6.1.sh
idf_build_dir=$(mktemp -d)
printf 'CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y\n' > "$idf_build_dir/sdkconfig.defaults"
idf.py -B "$idf_build_dir/build" -D "SDKCONFIG=$idf_build_dir/sdkconfig" -D "SDKCONFIG_DEFAULTS=$idf_build_dir/sdkconfig.defaults" -D IDF_TARGET=esp32s3 build
```

For a later supervised Pi check, use a separately commissioned board running
V3.1. Start with motor power isolated, a securely raised stand and the physical
cutoff verified. Start the read-only bridge in one terminal.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
python3 -B amr_ros2_bridge_v3_1.py --port /dev/serial0
```

In a second terminal inspect the bridge and status.

```bash
source /opt/ros/humble/setup.bash
ros2 node info /amr_serial_bridge
ros2 topic echo --once /amr_bridge/status
ros2 topic hz /joint_states
```

Confirm `READ_ONLY`, protocol `3.1`, `low_speed_capable=true`, idle state,
zero PWM and a healthy stationary board. Stop the read-only bridge and verify
its final STOP before the characterization tool takes UART ownership. When
separately authorized to energize motors for a supervised raised-wheel run,
select one candidate and a new CSV path for each invocation.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
python3 -B tools/amr_v3_low_speed_test.py --port /dev/serial0 --rpm 5 --direction forward --duration 3 --csv ./low-speed-5rpm-forward.csv
```

The tool owns UART directly through the V3.1 bridge core. Keep other bridges
and the Pi serial console stopped. It accepts durations from 0.5 to 10
seconds, requires `WHEELS_UP_LOW_SPEED`, and verifies STOP plus stationary
feedback before declaring successful completion. CSV includes per-wheel samples
and summaries. Time to start is measured from the initial velocity ACK to the
first sampled encoder advance of at least three ticks in the commanded
direction with PWM applied. Mean and peak absolute tracking errors use
RUN-confirmed samples, including startup. Compare both wheels and directions.
When either wheel has no encoder-confirmed start, the CSV marks that result
instead of estimating a start time. The tool records fault events and ends the
run. A later experiment requires a new invocation and operator confirmation.

On the physical system, verify each candidate and direction, the sticky fault
and heartbeat behaviors in LOW, stop timing and independent wheel response
before deriving a stable minimum. Raised-wheel data alone cannot establish
tracking under floor load or Nav2 compatibility.

## Phase 4L.6C guarded ground commissioning

Bridge V3.2 reuses the V3/V3.1 UART transport, firmware negotiation and STOP
handshake. Firmware V3.1 and the existing PI and feedforward values are the
baseline. Default V3.2 execution is `READ_ONLY`. Its motor-enabled modes are
`--raised-commissioning` and `--ground-control`. Both use the same control path
with `ARM_CONT`. Raised commissioning retains a 15-second budget. Ground starts
with 10 seconds by default and accepts an explicit `--session-seconds` value
between 1 and 120. The budget starts at the first ARM attempt and includes all
normal stops and restarts. The bridge requests STOP 400 ms before the deadline
to leave time for UART, the 300 ms firmware watchdog and a control tick.

Use the physical cutoff and a supervisor for every commissioning run.
Secure the stand for raised-wheel tests or clear and control the floor area
for floor tests. These sessions establish short, human-supervised control.
Firmware V3.1 is required for motion. The operator starts a fresh bridge
process, selects one source, and confirms the setup before UART is opened.
The bridge verifies idle and stationary feedback before READY and requires
fresh input after READY. STOP bypasses the slew limiter. UART, firmware,
authority and timing faults require an operator to end the session and
diagnose the cause before a new activation.

### Command authority and timestamps

The bridge subscribes to `geometry_msgs/msg/TwistStamped` on
`/amr_bridge/ground_cmd_vel` in enabled modes. The laptop adapter subscribes to
one selected raw `geometry_msgs/msg/Twist` source.

| Selection | Raw source topic |
| --- | --- |
| `teleop` | `/amr/teleop_cmd_vel` |
| `nav2` | `/amr/nav2_cmd_vel` |
| `commissioning` | `/amr/commissioning_cmd_vel` |

The operator enters the Pi's random session ID into the adapter and confirms
activation. The adapter binds its own activation ID, which the bridge
accepts once. Ownership and the motion state are reported separately.
Changing sources requires stopping and starting a new bridge and adapter
session. ROS command queues are volatile, best-effort and depth one.
Use one raw publisher on the selected topic. The adapter and bridge pin
the single observed endpoints after short discovery stabilization and
fault on publisher conflicts or changes. The control and clock-probe
responses use `/amr_bridge/ground_control`, separate from velocity data.

The installed Humble `rclpy` 3.3.22 subscription binding provides DDS source
and receipt timestamps through the subscription handle. The standard
executor drops those timestamps from Python callbacks. V3.2 uses a scoped
executor override for these subscriptions. Unavailable or invalid DDS
timestamps inhibit motion. DDS source time is the middleware publication
event on the raw writer. An unstamped Twist still has no application-level
creation timestamp, so any time spent in the source before its DDS publish
remains unknown. Put the original writer and adapter on the laptop and
verify the raw writer's publish behavior. Avoid sources that replay, hold or
periodically republish old commands. The adapter forwards only when it takes
a new eligible raw sample. Its stamped header carries original DDS source
time in the commissioning metadata and raw adapter receipt time as the
header timestamp. This topic is for the bridge and adapter, so its
`frame_id` is control metadata rather than a TF frame.
See the [Humble rclpy subscription binding](https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/subscription.cpp).

Synchronize both hosts' system clocks before activation. The bridge uses a
fresh four-timestamp round trip to bound the possible laptop/Pi offset
within ±20 ms and requires a renewed proof within one second. It budgets
that uncertainty against the 230 ms command deadline and checks original
and receipt ages separately. Marked session/activation IDs and timestamps
before the post-STOP READY boundary cannot start motion. Source silence,
owner disappearance, expiry or cancellation stops the session. The adapter
can send a fault notification, while the bridge deadman and firmware
watchdog remain independent of delivery of that notification.

Use only an isolated, trusted commissioning LAN with known nodes, one raw
writer on the laptop and one adapter. The tested Humble binding exposes
timestamps but does not provide a publisher GID on each received sample.
Discovery endpoint counts and GIDs, process names and session IDs support
accidental-conflict detection. They are not cryptographic authentication,
and discovery can lag transient or spoofed participants. On middleware
that supplies a sample GID, V3.2 also compares it to the pinned endpoint.
Keep the bridge in READ_ONLY if the single-writer topology or source
freshness cannot be established. A shared or adversarial network requires
a separately secured DDS deployment and validation.

### Motion admission and telemetry

The maximum body targets are 0.06 m/s and 0.35 rad/s. One proportional scale
applies both body and wheel limits. Forward pairs reserve up to 1 RPM per
wheel for the firmware's straight synchronization, including setpoints
encountered during slew. The effective wheel target ceiling is 15 RPM.
Lower limits are available through `--max-linear`, `--max-angular` and
`--max-wheel-rpm` within the firmware's 5 RPM startup constraints.

Initial drive uses 5 RPM on each active wheel. Later setpoint changes use a
15 RPM/s maximum slew. The initial step is from zero to the smallest
supported setpoint, so startup can change heading transiently and provides
no smooth-from-zero guarantee. The firmware keeps its existing duty ramp
and PWM cap. A change in wheel sign or which wheels are driven requires a
normal STOP, verified stationary READY and fresh input.

An exact zero requests normal STOP. A pair containing any nonzero wheel
below 5 RPM after limiting is inhibited in full and, while moving, causes a
normal STOP. `LOW_SPEED_INHIBITED` is distinct from a fault. After stationary
READY, clearing the inhibit requires at least three fresh eligible samples
of the same wheel direction/drive pattern spanning at least 120 ms with
each active wheel at or above 5.5 RPM. A fresh exact zero after READY allows
the same dwell at the standard 5 RPM minimum. Any intervening sub-minimum
request resets admission. This policy preserves the requested path ratio
through saturation and avoids converting arbitrarily small commands into
motion.

`/amr_bridge/status` includes requested and limited body velocity
`[m/s, rad/s]`, requested, limited, acknowledged and measured RPM, actual
PWM, source and receipt ages, feedback age, owner and session state, time
remaining, STOP and fault reasons, inhibition, boot ID and encoder continuity.
`STARTING` reports a velocity acknowledgement without confirmed firmware
RUN. `RUNNING` requires a fresh firmware Q with RUN=1. Update-time measurements
missing from firmware remain null. `/joint_states` retains
`left_wheel_joint` and `right_wheel_joint` with radians and radians per second.
If the firmware boot identity changes or counts are discontinuous, V3.2
invalidates continuity, withholds joint samples and latches a fault. Raw
counts remain visible for diagnosis. Before resuming after such an event,
restart odometry and re-establish localization. The existing odometry node
does not consume the bridge continuity field.

### Offline verification

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
PYTHONPYCACHEPREFIX=$(mktemp -d) python3 -m py_compile amr_ground_protocol.py amr_ros2_bridge_v3_2.py tools/amr_cmd_vel_adapter.py tools/amr_ground_commissioning.py tests/test_amr_ground_control.py
native_test_dir=$(mktemp -d)
cc -std=c11 -Wall -Wextra -Werror -pedantic -I main tests/test_low_speed_policy.c -o "$native_test_dir/test_low_speed_policy"
"$native_test_dir/test_low_speed_policy"
```

The tests use mocked UART. The installed Humble metadata regression
publishes an isolated localhost String on a non-motion topic.

### Manual commissioning commands for later supervised use

On both hosts, use corresponding repository copies and the same isolated
ROS domain. Check clock synchronization on each host.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
timedatectl status
```

Start read-only on the Pi with motor power isolated. In a separate ROS
terminal run `ros2 topic echo --once /amr_bridge/status` and check firmware
protocol 3.1, valid encoder continuity and healthy idle feedback. Close the
read-only process before starting an enabled bridge.

```bash
python3 -B amr_ros2_bridge_v3_2.py --port /dev/serial0
```

Start a new Pi process for a single raised profile. Confirm
`WHEELS_UP_CUTOFF_READY` when prompted. For a later floor run, use the second
command instead and confirm `CLEAR_AREA_CUTOFF_READY`. Use one command per run.

```bash
python3 -B amr_ros2_bridge_v3_2.py --port /dev/serial0 --raised-commissioning --source commissioning
python3 -B amr_ros2_bridge_v3_2.py --port /dev/serial0 --ground-control --source commissioning --session-seconds 10
```

On the laptop, assign the Pi's printed session ID to `SESSION` in each
terminal. Start a single adapter and confirm `ACTIVATE <session-id>`.

```bash
read -r SESSION
python3 -B tools/amr_cmd_vel_adapter.py --session "$SESSION" --source commissioning
```

In another laptop terminal, run exactly one profile from the table and type
its prompted `RUN <test> <session-id>` confirmation. Use a new CSV path and
new bridge and adapter session for each experiment. These are separate runs.

| Test selection | Pi mode | Profile |
| --- | --- | --- |
| `raised` | Raised | 0.04 m/s for 2 s |
| `forward-3s` | Ground | 0.04 m/s for 3 s |
| `forward-5s` | Ground | 0.05 m/s for 5 s |
| `left` | Ground | +0.25 rad/s for 2 s |
| `right` | Ground | −0.25 rad/s for 2 s |
| `normal-stop` | Raised | 0.04 m/s for 1 s, then verify normal STOP and stationary READY |
| `source-loss` | Raised | 0.04 m/s for 1 s, then silence and verify fault inhibition, STOP and stationary encoders |

```bash
read -r SESSION
read -r TEST
python3 -B tools/amr_ground_commissioning.py --test "$TEST" --session "$SESSION" --csv "./phase46c-${TEST}-${SESSION}.csv"
```

Enter the session ID and one test selection at the two `read` prompts. The
publisher waits for the bridge to confirm stationary readiness and command
authority. Normal profiles continue fresh zeros through the final
stationary check. The source-loss profile stays silent and checks the
inhibited state without sending a second motion command. Closing the
publisher after a normal profile can cause a subsequent owner-loss fault.
Use the CSV's final test result and the Pi's shutdown STOP and stationary
verification. If either remains unverified, cut motor power physically.

### Later Nav2 Humble integration

After floor validation, prepare `nav2_navfn_planner/NavfnPlanner` with
Regulated Pure Pursuit and forward-preferred navigation. Start evaluation
with `desired_linear_vel: 0.05`, `rotate_to_heading_angular_vel: 0.25`,
`allow_reversing: false`, `use_rotate_to_heading: true` and
`use_collision_detection: true`. Keep obstacle and costmap processing
active. Remove backup from the Behavior Server and any referenced
`BackUp` recovery action in the chosen behavior trees until reverse
floor motion is validated. Review the `Twist` publisher topology so a
single final Nav2 writer is remapped to `/amr/nav2_cmd_vel`.
See the [Humble Nav2 configuration](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml),
[RPP parameters](https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller)
and [default recovery tree](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml).

The 5 RPM threshold corresponds to about 0.0228 m/s in straight motion and
0.152 rad/s for a symmetric in-place rotation. In curves the inside wheel
can fall below the minimum even above the straight-line threshold. Approach
deceleration and heading alignment can therefore request inhibited setpoints.
Use measured floor stopping and tracking to select goal tolerances and
rotate-to-heading settings. Allow the progress checker enough time and a
realistic distance for these slow speeds, in-place turns and verified stop
transitions. Humble's SimpleProgressChecker measures translational progress.
Persistent inhibition is a navigation failure. Inspect the requested wheel
pair, odometry, local path and progress state before changing parameters or
accepting any goal result. The raised-wheel 2–5 RPM data and the new offline
tests do not establish floor traction, tracking, reverse performance or
autonomous safety.

## Phase 4L.6D.4 link fault investigation

The failed source-loss CSV records a bridge fault in `WAIT_READY`, before
the first arm. The firmware was idle, had zero PWM and reported no firmware
fault. The CSV has a single status sample from that session, so it does not
show where the preceding status samples or clock replies were delayed.

### Confirmed software failure paths

- **Probe-dependent liveness.** Baseline `318a3b5` returns from the status
  callback before updating liveness when a status lacks a clock probe. A
  controlled replay of valid same-session statuses at 61 ms intervals,
  without probes, reproduces `bridge status stream lost`. Status and probe
  are now validated separately. Fresh, authorized status can maintain
  telemetry liveness. Only a valid four-timestamp exchange can renew
  clock authority.
- **Executor ordering.** On the installed Humble executor, the wait set
  yields ready timers before ready subscriptions on the same node. The
  metadata override preserves that order. An isolated ROS regression
  confirms it with a pending String sample. The baseline watchdog then
  reproduces the false timeout if a valid status arrived at 205 ms and the
  timer runs first at 240 ms. At the deadline boundary the adapter now takes
  at most one pending status before declaring timeout. The bridge uses the
  same bounded check for pending clock replies. A baseline replay also
  reproduced clock expiry when a valid reply arrived about 982 ms after the
  previous proof and was processed 40 ms later. Only a validated DDS
  receipt before the applicable deadline can prevent the fault. Samples
  received after a real gap, expired proofs and stale callback data still
  fail closed. Receipt time is tracked separately from processing time.
- **Repeated graph work.** Each baseline adapter status callback queries
  both the raw source graph and the bridge status graph. Status processing
  now checks the status writer only. The regular adapter timer and the raw
  callback still check both endpoints, including immediately before
  forwarding any motion sample. Commissioning bridge control and velocity
  callbacks retain endpoint checks. The eliminated lookup was redundant.
  No field measurements attribute the fault to graph cost.

The hardware trigger remains unconfirmed. The reliable status writer and
best-effort depth-one reader can communicate, but a matched reader can still
lose a sample or replace a pending sample. Status QoS is preserved to avoid
adding reliable-reader backpressure to the bridge. Bridge timers and
subscriptions share one thread. Commissioning adds graph inspection, clock
handling and, during drive, UART work relative to read-only operation.
A delayed callback, executor or OS scheduling, slow UART transactions
and DDS or network delays must be distinguished using an instrumented
commissioning run. An ICMP ping measures a different path. The reported
healthy read-only timing cannot rule out commissioning stalls.

The 230 ms status deadline, 230 ms motion deadman and command freshness
checks, ±20 ms clock bounds, one-second proof lifetime and independent
300 ms firmware heartbeat are preserved. A real gap or validation failure
still latches a fault. The diagnostic code requires an explicit option on
the live bridge or adapter and never clears a fault.

### Deploy and run the isolated link check

Stop existing test participants, then copy the same updated Python sources
to both hosts. For an existing checkout on `robot1`, run from the laptop's
updated checkout.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
scp amr_ground_protocol.py amr_link_metrics.py amr_ros2_bridge_v3_2.py robot1:~/amr_phase_v3_work/ESP32_AMR/
scp tools/amr_cmd_vel_adapter.py tools/amr_link_diagnostics.py robot1:~/amr_phase_v3_work/ESP32_AMR/tools/
```

Use an unused isolated ROS domain and matching diagnostic session on the
trusted laptop/Pi LAN. Start the bridge side first and launch the adapter
within 15 seconds. Choose a new session and output names for each run.
The diagnostic never opens UART or publishes velocity. It advertises an
unused stamped endpoint inside its diagnostic topic prefix so the real
ownership/discovery checks still run. This test exercises the middleware
link and clock exchange without simulating UART workload. At the end of a healthy run, the adapter sends a
diagnostic-only completion marker so deliberate teardown can be separated
from publisher disappearance during the measurement.

On the Raspberry Pi:

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
SESSION=6845d0bd00000000000000000000000d
python3 -B tools/amr_link_diagnostics.py bridge --domain-id 184 --session "$SESSION" --duration 145 --json ./link-pi.json
```

On the laptop:

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
SESSION=6845d0bd00000000000000000000000d
python3 -B tools/amr_link_diagnostics.py adapter --domain-id 184 --session "$SESSION" --duration 120 --json ./link-laptop.json
```

To verify that a real gap still faults, repeat on a new diagnostic session
and give the mock bridge `--pause-status-at 10 --pause-status-for .45`.
Once authority is established, expect nonzero exit and the observed fault
in each JSON. This is a deliberately failing diagnostic scenario.

For a separately authorized commissioning investigation, use the existing
raised-wheel commands in this README and append `--link-diagnostics` to
both V3.2 and the adapter. Leave the raw motion publisher off to compare
stationary commissioning against the isolated test. These processes emit
at most one periodic summary per ten seconds through an asynchronous
single-item queue and a final summary after cleanup. Percentiles use the
latest 4,096 samples per metric, while maxima and counts are lifetime
values. Inspect the bridge and adapter reports together.

`status_published` and sequence gaps describe the producer and the samples
visible to Python. `status_taken` counts samples dequeued from DDS,
`status_callbacks` counts callbacks entered, and the adapter reports
accepted, ignored and rejected dispositions. The DDS receipt-to-callback
delay identifies samples that reached the local middleware but waited for
Python. If nothing was taken, comparing the two profiles can narrow the
failure, but a sequence gap alone cannot distinguish delivery loss from
a replaced depth-one sample. Missing metadata is recorded as JSON null.
Timer delay is measured against nominal cadence on the local monotonic
clock. Offset bounds and round-trip measurements use the real clock probe
timestamps.

### Offline verification commands

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
PYTHONPYCACHEPREFIX=$(mktemp -d) python3 -m py_compile amr_ground_protocol.py amr_link_metrics.py amr_ros2_bridge_v3_2.py tools/amr_cmd_vel_adapter.py tools/amr_link_diagnostics.py tests/test_amr_link_diagnostics.py
```

Verified locally with Humble. All 83 Python tests passed with zero failures
or skips. Syntax checks and the existing native policy test passed. The
120-second localhost diagnostic accepted all 1,999 taken status samples
and verified 780 clock proofs. The maximum DDS status interval was
61.23 ms, with p95 of 60.34 ms. A separate injected 450 ms status pause
latched the adapter at an observed 262.75 ms timeout age and propagated
the fault to the mock bridge. The localhost runs used no UART or motion
traffic. A cross-host stationary commissioning capture is still needed
to identify the field trigger.
