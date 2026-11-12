#!/usr/bin/env bash
# Runs INSIDE the ros2_humble container on the Pi (sent by mapping.py).
# Base + LiDAR + SLAM (the group's balance_slam_bringup). Stops cleanly; map is NOT saved here.
set -o pipefail
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0

# --- stop a whole process tree (the launch AND every node under it) -------------------------------
descendants() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do echo "$c"; descendants "$c"; done; }
stop_tree() {   # stop_tree PID [first_signal]: graceful signal, then TERM, then KILL, for PID + all descendants
    local root="$1" first="${2:-INT}" tree p i any
    [[ -n "$root" ]] || return 0
    tree="$root $(descendants "$root" | tr '\n' ' ')"
    kill -"$first" "$root" 2>/dev/null || true
    for i in 1 2 3 4 5 6 7 8; do kill -0 "$root" 2>/dev/null || break; sleep 0.5; done
    for p in $tree; do kill -TERM "$p" 2>/dev/null || true; done
    for i in 1 2 3 4 5 6; do
        any=0; for p in $tree; do kill -0 "$p" 2>/dev/null && any=1; done
        [[ $any -eq 0 ]] && return 0
        sleep 0.5
    done
    for p in $tree; do kill -KILL "$p" 2>/dev/null || true; done
}
LOCK=/tmp/ee3033_slam.pid
if [[ -f "$LOCK" ]]; then
    old="$(cat "$LOCK" 2>/dev/null || true)"
    if [[ "$old" =~ ^[0-9]+$ ]] && kill -0 "$old" 2>/dev/null; then
        echo "[Pi] SLAM already running (PID $old)" >&2
        exit 4
    fi
    rm -f "$LOCK"
fi
echo $$ > "$LOCK"
LAUNCH_PID=''
cleanup() {
    trap - EXIT TERM INT
    echo '[Pi] stopping SLAM...' >&2
    stop_tree "$LAUNCH_PID" INT
    [[ "$(cat "$LOCK" 2>/dev/null)" == "$$" ]] && rm -f "$LOCK"
    exit "${EXIT_CODE:-0}"
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
ros2 launch balance_bringup balance_slam_bringup.launch.py < /dev/null &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
EXIT_CODE=$?
echo "[Pi] SLAM launch exited with code $EXIT_CODE" >&2
exit "$EXIT_CODE"
