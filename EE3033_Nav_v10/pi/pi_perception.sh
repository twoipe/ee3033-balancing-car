#!/usr/bin/env bash



set -o pipefail
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
LOCK=/tmp/ee3033_perception.pid
if [[ -f "$LOCK" ]]; then
    old="$(cat "$LOCK" 2>/dev/null || true)"
    if [[ "$old" =~ ^[0-9]+$ ]] && kill -0 "$old" 2>/dev/null; then
        echo "[Pi] perception already running (PID $old)" >&2
        exit 4
    fi
    rm -f "$LOCK"
fi
echo $$ > "$LOCK"
LOOPS=()

supervise() {
    local name="$1"; shift
    (
        child=''
        trap 'stop_tree "$child" TERM; exit 0' TERM
        while true; do
            "$@" < /dev/null &
            child=$!
            wait "$child"
            echo "[Pi] $name exited (code $?); restarting in 3 s" >&2
            sleep 3 &
            wait $!
        done
    ) &
    LOOPS+=($!)
}

cleanup() {
    trap - EXIT TERM INT
    echo '[Pi] stopping perception...' >&2
    for p in "${LOOPS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    for i in $(seq 1 24); do
        any=0; for p in "${LOOPS[@]}"; do kill -0 "$p" 2>/dev/null && any=1; done
        [[ $any -eq 0 ]] && break
        sleep 0.5
    done
    for p in "${LOOPS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
    [[ "$(cat "$LOCK" 2>/dev/null)" == "$$" ]] && rm -f "$LOCK"
    exit 0
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT

supervise camera ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -r image_raw:=/camera/image_raw -p video_device:=/dev/video0 -p image_size:='[640,480]' -p time_per_frame:='[1,10]'
supervise compressor ros2 run image_transport republish raw compressed --ros-args \
    -r in:=/camera/image_raw -r /out/compressed:=/camera/image_raw/compressed
echo '[Pi] perception running: /camera/image_raw and /camera/image_raw/compressed'
while true; do sleep 1; done
