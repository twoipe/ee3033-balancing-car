#!/usr/bin/env bash
# Manually save the current SLAM map (run in a 2nd terminal while mapping.py is running).
# Usage: bash save_map.sh [map_name]      (default name: ee3033_assessment)
exec python3 "$(dirname "$(readlink -f "$0")")/mapping.py" save-map ${1:+--name "$1"}
