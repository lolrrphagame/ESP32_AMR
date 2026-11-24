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
