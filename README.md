# Phase 4L.1 guarded continuous-motion bench protocol

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
| `VER` | `VERSION PV=3 CAP=ARM_CONT,STICKY_FAULT,STATIONARY,BOOT_ID` | Identifies the guarded firmware. |
| `Q` | `STATUS key=value ...` | Reads firmware state and normalized wheel feedback. |
| `T 012345` | `ACK 012345` | Echoes exactly six decimal digits. |
| `ARM` | `OK ARMED 5S` | Legacy arming. Motion must end within five seconds of arming, regardless of velocity renewals. |
| `ARM_CONT` | `OK ARMED CONT` | Arms only while disarmed, stationary and free of latched faults. The first valid motion command must arrive within five seconds. |
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
| `MODE` | `IDLE`, `LEGACY` or `CONT`. |
| `FL`, `FR` | Sticky fault flag and first fault reason, or `NONE`. |
| `ST` | Firmware stationary verification flag. |
| `BOOT` | Eight-digit hexadecimal boot identifier used to detect resets. |

Framing errors, invalid bytes, gaps, overflows and malformed commands latch a
fault. Encoder read failures or disagreement, control timing faults, overspeed,
stall, active deadline expiry and unsafe setpoint changes also latch a fault.
`STOP` remains available after a fault. Both arm commands refuse to arm while
`FL=1`.

## Bridge V3 operation and recovery

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

## Verification and later phases

From the workspace root, with the ROS 2 Humble environment sourced, run
`PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v`. All UART
traffic in the tests is mocked. The ROS adapter test is skipped if ROS is
unavailable.

Before a later supervised raised-wheel run, validate the physical cutoff, STOP
and watchdog timing, fault/reboot behavior, encoder polarity, coast-down and
reversal, stability through a run exceeding five seconds, and session cutoff
on the actual hardware. Offline tests and a firmware build cannot establish
these properties. The separate low-speed phase should measure breakaway,
repeatable starts and stops, near-zero and single-wheel transitions, tracking
in both directions, and behavior with load before attempting Nav2 integration.
