#!/usr/bin/env bash




set -o pipefail
MAP_YAML="$1"
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0


descendants() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do echo "$c"; descendants "$c"; done; }
stop_tree() {
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
LOCK=/tmp/ee3033_nav.pid
if [[ -f "$LOCK" ]]; then
    old="$(cat "$LOCK" 2>/dev/null || true)"
    if [[ "$old" =~ ^[0-9]+$ ]] && kill -0 "$old" 2>/dev/null; then
        echo "[Pi] navigation stack already running (PID $old). Run: python3 ee3033.py down" >&2
        exit 4
    fi
    rm -f "$LOCK"
fi
if [[ ! -s "$MAP_YAML" ]]; then echo "[Pi] map file missing: $MAP_YAML" >&2; exit 5; fi
IMG="$(sed -n 's/^image:[[:space:]]*//p' "$MAP_YAML" | head -n1 | tr -d "\"'")"
[[ "$IMG" = /* ]] || IMG="$(dirname "$MAP_YAML")/$IMG"
if [[ ! -s "$IMG" ]]; then echo "[Pi] map image missing: $IMG" >&2; exit 5; fi
if ! python3 -c 'import numpy, rclpy' < /dev/null 2>/dev/null; then echo '[Pi] numpy/rclpy missing in container' >&2; exit 8; fi
echo $$ > "$LOCK"
LOC_PID=''
ASTAR_LOOP=''

cleanup() {
    trap - EXIT TERM INT
    echo '[Pi] stopping A* and localization...' >&2
    if [[ -n "$ASTAR_LOOP" ]]; then
        kill -TERM "$ASTAR_LOOP" 2>/dev/null || true
        for i in $(seq 1 24); do kill -0 "$ASTAR_LOOP" 2>/dev/null || break; sleep 0.5; done
        kill -KILL "$ASTAR_LOOP" 2>/dev/null || true
    fi
    stop_tree "$LOC_PID" INT
    [[ "$(cat "$LOCK" 2>/dev/null)" == "$$" ]] && rm -f "$LOCK"
    exit "${EXIT_CODE:-0}"
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT




LOCALIZATION_CMD="${EE3033_LOCALIZATION_CMD:-ros2 launch balance_bringup balance_known_map_waypoint_bringup.launch.py map:=$MAP_YAML start_waypoints:=false}"
echo "[Pi] localization: $LOCALIZATION_CMD"
eval "exec $LOCALIZATION_CMD" < /dev/null &
LOC_PID=$!
sleep 4

(
    child=''
    trap 'stop_tree "$child" TERM; exit 0' TERM
    while true; do
        python3 /tmp/ee3033/astar_realtime.py --ros-args --params-file /tmp/ee3033/astar.yaml < /dev/null &
        child=$!
        wait "$child"
        echo "[Pi] A* exited (code $?); restarting in 2 s" >&2
        sleep 2 &
        wait $!
    done
) &
ASTAR_LOOP=$!
echo '[Pi] navigation stack running (A* IDLE until the mission sends a command)'
while kill -0 "$LOC_PID" 2>/dev/null; do sleep 1; done
echo '[Pi] localization stack exited unexpectedly' >&2
EXIT_CODE=1
exit 1
