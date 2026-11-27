# Real Nav2 with Pi-local ground control

The Pi runs the Bridge, Command Adapter and UART watchdogs on localhost ROS domain 43. The separate joint publisher runs on network domain 42 with LiDAR, laptop odometry, AMCL, RViz and Nav2. The bridge publishes its authority status and clock challenges only on domain 43. Keep monitoring of that topic on the Pi.

In the physical 6D.8 run, the laptop adapter logged a 2,425.989 ms maximum DDS status interval and a 40.877 ms maximum timer interval. The supplied Pi measurements show 139.572 ms maximum status publication and a 175.356 ms maximum bridge timer interval. Another subscriber reported a 95 ms maximum interval. This identifies a failure of timely delivery to the authority participant. The summaries cannot distinguish writer queuing from transport or reader delay for the entire gap.

The laptop source reads each live `/amr/nav2_cmd_vel` Twist with its original DDS write time, receipt time and writer GID in a single `rcl_take`. Humble's Python binding on the test host omits the writer GID. The source pins the single `/controller_server` endpoint, checks the Fast DDS host prefix against its local reader and compares the full writer GID on every sample. Keep the existing trusted LAN and standard Fast DDS GUID configuration. Pi and laptop exchange session-authenticated UDP datagrams on ports 46011 and 46010 respectively. Clock replies contain no velocity. Each motion datagram requires a current Pi challenge, clock bounds within ±20 ms, a fresh original timestamp and the pinned owner. A clock step, owner change or lost proof inhibits the session. The proof lifetime is one second. The Bridge independently charges transport and callback delay against the original 230 ms command lifetime.

A second process for authority status would put a queue between the UART observation, clock challenge and DDS write. That would require transporting and checking the observation time separately to prevent a late queued state from looking current. Keeping the writer on localhost removes network subscribers from this call while retaining the original authority semantics. A stalled private DDS write can still delay the executor. The physical stationary trial should measure that call and the timer interval under load.

The laptop also checks scan, wheel odometry and AMCL map-to-odom measurement times. It subtracts AMCL's configured future dating from the TF stamp. The earliest feedback expiry accompanies each nonzero command and bounds how long the Bridge may drive. Wheel odometry now carries the actual encoder measurement stamp. The feedback window uses the deployed RPP transform tolerance of 0.3 s and accounts for clock uncertainty. The controller's collision check, 5 RPM wheel minimum, speed limits, explicit activation, verified STOP and 300 ms firmware heartbeat retain their existing settings.

## Deploy from the laptop

These commands target the existing Phase 6D.8 Pi installation, Humble system Python, Pi `192.168.50.11` and laptop `192.168.50.10`. Use these checkout files, including the new source and native reader, on the laptop. Stop previous motor-control processes and isolate motor power before deploying.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
git switch phase6d-nav2-stable
source /opt/ros/humble/setup.bash
mkdir -p build
includes=(/opt/ros/humble/include/*/)
cc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared "${includes[@]/#/-I}" \
    tools/nav2_take.c -L/opt/ros/humble/lib -lrcl -o build/nav2_take.so
PI_HOST=ubuntu@192.168.50.11
ssh "$PI_HOST" 'mkdir -p ~/amr_uart_v33/tools'
scp amr_ground_protocol.py amr_nav2_peer.py amr_joint_telemetry.py amr_ros2_bridge_v3_2.py \
    "$PI_HOST:~/amr_uart_v33/"
scp tools/amr_cmd_vel_adapter.py "$PI_HOST:~/amr_uart_v33/tools/"
```

The laptop runs the updated `amr_odom_from_joints.py` and `tools/amr_nav2_source.py` directly from the checkout. Only five Python files go to the Pi. The existing V3.3 bridge entry point and flashed firmware are used.

## Start with all terminals closed

Run this **public environment** in every laptop terminal and Pi terminal 1. Source the installed robot overlay when present.

```bash
source /opt/ros/humble/setup.bash
if [ -f ~/dev_ws/install/setup.bash ]; then source ~/dev_ws/install/setup.bash; fi
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset AMR_NAV2_PUBLIC_DOMAIN
```

Run this **private environment** in Pi terminals 2, 3 and 4.

```bash
source /opt/ros/humble/setup.bash
cd ~/amr_uart_v33
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1 AMR_NAV2_PUBLIC_DOMAIN=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
```

**Pi terminal 1 — LiDAR.** Check clock synchronization on both hosts. Keep motor power isolated for the stationary check.

```bash
chronyc tracking
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 \
    -p frame_id:=laser_frame -p scan_mode:=Standard -p angle_compensate:=true
```

**Pi terminal 2 — Bridge.** Type `CLEAR_AREA_CUTOFF_READY` when prompted. Copy the new session ID for laptop terminal 6 and Pi terminal 3.

```bash
python3 -Bu amr_ros2_bridge_v3_3.py --port /dev/serial0 --ground-control \
    --source nav2 --session-seconds 30 --link-diagnostics 2>&1 | tee -a ~/nav2_bridge_6d10.log
```

**Laptop terminal 1 — Robot state publisher.**

```bash
chronyc tracking
ros2 launch my_amr rsp.launch.py sim_mode:=false use_ros2_control:=false use_sim_time:=false
```

**Laptop terminal 2 — Wheel odometry.**

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
python3 -B amr_odom_from_joints.py
```

**Laptop terminal 3 — Saved map and localization.**

```bash
ros2 launch nav2_bringup localization_launch.py \
    map:="$HOME/dev_ws/maps/amr_home_trial_01.yaml" \
    params_file:="$HOME/dev_ws/config/amcl_amr.yaml" use_sim_time:=False autostart:=True
```

**Laptop terminal 4 — RViz.** Select 2D Pose Estimate at the robot's actual position and heading. Confirm the live scan aligns with the map.

```bash
rviz2 -d "$HOME/dev_ws/config/amr_navigation.rviz"
```

**Laptop terminal 5 — Navigation.**

```bash
ros2 launch "$HOME/dev_ws/config/navigation_amr.launch.py" \
    params_file:="$HOME/dev_ws/config/nav2_amr_real.yaml" use_sim_time:=False autostart:=True
```

**Laptop terminal 6 — Nav2 source.** Check for one publisher named `/controller_server` and an active navigator. The deployed AMCL transform tolerance is 0.5 s. Pass the value reported by the live AMCL node. Paste the session ID at the prompt.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
ros2 topic info -v /amr/nav2_cmd_vel
ros2 topic info -v /joint_states
ros2 lifecycle get /bt_navigator
TOL=$(ros2 param get /amcl transform_tolerance | awk '/Double value is:/ {print $NF}')
read -rp 'Bridge session ID: ' SESSION
python3 -Bu tools/amr_nav2_source.py --pi 192.168.50.11 --session "$SESSION" \
    --amcl-tolerance "$TOL" 2>&1 | tee -a ~/nav2_source_6d10.log
```

**Pi terminal 3 — Stationary adapter.** Paste the same session ID and type the requested `ACTIVATE <session-id>`. Stationary mode suppresses all velocity forwarding.

```bash
read -rp 'Bridge session ID: ' SESSION
python3 -Bu tools/amr_cmd_vel_adapter.py --session "$SESSION" --source nav2 --laptop 192.168.50.10 \
    --stationary-check --link-diagnostics 2>&1 | tee ~/nav2_adapter_6d10_stationary.log
```

**Pi terminal 4 — Verify 120 seconds stationary.** Run the following once. It waits up to 30 seconds for ownership, watches fresh status for two minutes and exits on a gap of one second or a change in the required state. The adapter log should show a pinned `remote_nav2_gid` and `remote_clock_bounds_ms`.

```bash
python3 - <<'PY'
import json, time, rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('stationary_observer')
first = last = session = None
def check(msg):
    global first, last, session
    s = json.loads(msg.data)
    now = time.monotonic()
    assert not s['fault_latched'] and not s['firmware_fault'], s
    assert not s['firmware_armed'] and not s['firmware_running'], s
    ready = (s['pi_nav2'] and s['authority_state'] == 'OWNED' and s['clock_status'] == 'VERIFIED'
             and s['state'] == 'READY' and s['stationary_ready'] and s['stop_verified']
             and s['pwm_duty_percent'] == [0., 0.] and s['encoder_continuity_valid'])
    if first is None and ready:
        first, session = now, s['session_id']
    if first is not None:
        assert ready and s['session_id'] == session, s
        assert last is None or now - last < 1., 'status gap'
        last = now
node.create_subscription(String, '/amr_bridge/status', check, qos_profile_sensor_data)
start = time.monotonic()
while last is None or last - first < 120.:
    rclpy.spin_once(node, timeout_sec=.1)
    assert time.monotonic() - (last or start) < (1. if last else 30.), 'status timeout'
print(f'Verified stationary for {last-first:.3f} s')
node.destroy_node()
rclpy.shutdown()
PY
```

## First short floor goal and recovery

1. After the stationary check, Ctrl+C Pi terminal 3, laptop terminal 6 and Pi terminal 2. Confirm `Final STOP and stationary feedback verified`. Leave LiDAR, odometry, AMCL, RViz and Nav2 running.
2. Restart Pi terminal 2 with its command above. Restart laptop terminal 6 with its commands and the **new** session ID. In Pi terminal 3, start the motion adapter below. Type the new explicit activation at its prompt.

   ```bash
   read -rp 'New bridge session ID: ' SESSION
   python3 -Bu tools/amr_cmd_vel_adapter.py --session "$SESSION" --source nav2 --laptop 192.168.50.10 \
       --link-diagnostics 2>&1 | tee ~/nav2_adapter_6d10_motion.log
   ```

3. In Pi terminal 4, inspect private status. Require `pi_nav2=true`, `OWNED`, `VERIFIED`, `READY`, `stationary_ready=true`, valid encoders and disarmed zero PWM. Confirm the clear path and keep the physical cutoff within reach before enabling motor power.

   ```bash
   ros2 topic echo --once /amr_bridge/status
   ```

4. In **laptop terminal 7** with the public environment, inspect the actual map pose. Stop `tf2_echo` with Ctrl+C. Select a clear goal 0.25–0.30 m straight ahead at the same yaw and enter its map X, Y and quaternion Z, W. Keep the motor cutoff accessible throughout.

   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   read -rp 'Goal X Y and current quaternion Z W: ' X Y QZ QW
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
       "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}}" --feedback
   ```

5. Confirm `SUCCEEDED` in laptop terminal 7 and repeat the Pi terminal 4 status command. Require verified stationary STOP, disarmed firmware and zero PWM. A zero from Nav2 permits ordinary goal completion. Loss of a live movement stream latches STOP.

For a fault, use the physical cutoff and cancel outstanding goals from laptop terminal 7. A cancellation request with an empty goal ID and zero timestamp selects all goals.

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

With motor power isolated, inspect Pi terminal 4 for the fault reason and STOP feedback. For a host-only fault with healthy firmware and verified STOP, repeat steps 1–3 with a new session and activation. A firmware fault or unverified STOP requires diagnosis with motor power isolated. Keep the existing localization and saved map for retries.

## Software verification

From the laptop checkout, run the following groups sequentially. The three ROS trials use dedicated domains 191/192, the existing laptop Nav2 overlay, loopback UDP, simulated UART and synthetic LiDAR. Close other instances of the source and Pi adapter on the test host to free the UDP ports.

```bash
cd ~/amr_phase_v3_work/ESP32_AMR
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests
test_dir=$(mktemp -d)
for name in test_low_speed_policy test_uart_startup; do
    cc -std=c11 -Wall -Wextra -Werror -pedantic -I main "tests/$name.c" -o "$test_dir/$name" && "$test_dir/$name" || break
done
bash tests/pi_nav2/run.sh idle
bash tests/pi_nav2/run.sh goal
bash tests/pi_nav2/run.sh loss
```

All **118 Python tests** and **both native C suites** passed. The local full-stack trials used the installed Humble controller, planner, navigator and AMCL with the V3.3 mock UART.

| Trial | Observed result |
|---|---|
| Stationary with injected joint writes of 450 ms and 1,200 ms | 120.008 s owned and stationary. Mock UART received zero arm or velocity frames. Maximum bridge timer interval was 60.169 ms, private status publish was 0.069 ms, and adapter DDS status interval was 60.572 ms. |
| 0.28 m NavigateToPose | Action returned SUCCEEDED. Mock firmware received one arm and 110 velocity frames. Bridge returned to READY with verified STOP, disarmed firmware and zero PWM. |
| Remote source suspended during motion | Bridge latched the command deadman and verified STOP. Source resumed, and the trial observed one more second disarmed. Mock UART recorded one arm for the entire session. |

These runs used loopback between two ROS domains on the laptop. The two-host latency, physical stationary run, hardware loss/STOP and short floor goal require verification on the robot. Noninteractive SSH reached the Pi and was denied authentication.
