#!/usr/bin/env bash
# Real Humble/Nav2, synthetic room and UART. Requires the existing my_amr overlay.
set -eo pipefail
mode=${1:-goal}
case "$mode" in idle|loss|scan-loss|goal) ;; *) exit 2 ;; esac
cd "$(dirname "$0")/../.."
fixture=$PWD/tests/pi_nav2
dir=$(mktemp -d)
echo "Trial logs in $dir"
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$PWD/tests:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS RMW_FASTRTPS_USE_QOS_FROM_XML=0
# Public worker uses network-enabled discovery. Keep the trial on dedicated domains.
unset AMR_NAV2_PUBLIC_DOMAIN
includes=(/opt/ros/humble/include/*/)
cc -std=c11 -Wall -Wextra -Werror -fPIC -shared "${includes[@]/#/-I}" tools/nav2_take.c \
    -L/opt/ros/humble/lib -lrcl -o "$dir/nav2_take.so"
cp "$fixture/room.yaml" "$dir/"
python3 - "$dir/room.pgm" <<'PY'
from pathlib import Path
import sys
pixels = bytes(0 if x in (0, 79) or y in (0, 79) else 254 for y in range(80) for x in range(80))
Path(sys.argv[1]).write_bytes(b'P5\n80 80\n255\n' + pixels)
PY
pids=()
start() {
    local name=$1
    shift
    setsid "$@" >"$dir/$name.log" 2>&1 &
    pids+=("$!")
}
cleanup() {
    for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT
private=(env ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 AMR_NAV2_PUBLIC_DOMAIN=191)
bridge_env=()
if [[ "$mode" == idle ]]; then
    c++ -std=c++17 -shared -fPIC "${includes[@]/#/-I}" "$fixture/block_joint_write.cpp" \
        -L/opt/ros/humble/lib -lfastrtps -ldl -o "$dir/block_joint_write.so"
    bridge_env=(LD_PRELOAD="$dir/block_joint_write.so")
fi
start bridge "${private[@]}" "${bridge_env[@]}" python3 -u "$fixture/mock_driver.py" bridge
start rsp ros2 launch my_amr rsp.launch.py sim_mode:=false use_ros2_control:=false use_sim_time:=false
start odom python3 -u amr_odom_from_joints.py
start loc ros2 launch nav2_bringup localization_launch.py map:="$dir/room.yaml" \
    params_file:="$HOME/dev_ws/config/amcl_amr.yaml" use_sim_time:=False autostart:=True
start nav ros2 launch "$HOME/dev_ws/config/navigation_amr.launch.py" \
    params_file:="$HOME/dev_ws/config/nav2_amr_real.yaml" use_sim_time:=False autostart:=True
for attempt in {1..50}; do
    session=$(awk '/^Session / {gsub(",", "", $2); print $2}' "$dir/bridge.log")
    [[ -z "$session" ]] || break
    sleep .2
done
test -n "$session"
start source python3 -u tools/amr_nav2_source.py --pi 127.0.0.1 --session "$session" --library "$dir/nav2_take.so"
export AMR_TEST_SOURCE_PID=$!
start adapter "${private[@]}" python3 -u "$fixture/mock_driver.py" adapter "$session" "$mode"
python3 -u "$fixture/trial.py" "$mode"
cleanup
trap - EXIT
if [[ "$mode" == idle ]]; then
    rg -q '^Test joint write delay 1200 ms' "$dir/bridge.log"
    rg -q '"remote_clock_proven":' "$dir/adapter.log"
    if rg -q '"remote_prearm_expiry":' "$dir/adapter.log"; then exit 1; fi
    counts=$(rg '^UART COUNTS' "$dir/bridge.log")
    [[ "$counts" != *"'ARM_CONT'"* && "$counts" != *"'V'"* ]]
fi
