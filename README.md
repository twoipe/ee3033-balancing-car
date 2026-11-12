# EE3033: Balancing Car, Mapping, Autonomous Navigation and Target Localization

ROS 2 Humble project for the EE3033 System Integration and Design Lab. A balancing car builds a
LiDAR SLAM map of a sandbox, then autonomously searches for a target object (YOLO + camera
triangulation), reports its (x, y) in the map frame, and returns to the start.

## Repository layout

| Folder | Purpose |
|---|---|
| `EE3033_Mapping_v7.4/` | Mapping launcher: SLAM + RViz2 + keyboard teleop + map save (runs on the Ubuntu VM; not a ROS package) |
| `EE3033_Nav_v10/` | Navigation launcher + `ee3033_search` ROS 2 package (mission node), Pi-side A*, YOLO perception, config, RViz |
| `docs/` | v10 walkthrough and reference (checks, TF, calibration, parameters). v10 README is `EE3033_Nav_v10/README.md` |

Clone anywhere on the VM (the READMEs assume `~/ee3033-balancing-car`). Generated maps are saved to
`~/ee3033_maps/` on the VM.

## The three commands

```bash
# 1. Mapping: SLAM + RViz2 + keyboard teleop, all in one command
cd ~/ee3033-balancing-car/EE3033_Mapping_v7.4 && python3 mapping.py

# 2. Save the map (Ctrl+C in the mapping terminal, press 's'; or from a second terminal:)
cd ~/ee3033-balancing-car/EE3033_Mapping_v7.4 && bash save_map.sh

# 3. YOLO + navigation + mission, all in one command (replace bottle with the YOLO class)
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py run --target bottle
```

Then set the start pose and begin (second terminal):
`python3 ee3033.py set-pose --x 0 --y 0 --yaw-deg 0`, `python3 ee3033.py status`, `python3 ee3033.py start`.

## One-time setup (VM)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py offline   # offline unit tests
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py install   # copies ee3033_search into ~/ros2_ws/src and builds
```

Requires the existing ROS 2 Humble workspace at `~/ros2_ws`, the Pi robot bringup, YOLO model weights
(`EE3033_YOLO_MODEL`) and passwordless SSH to `balance@balance10.local`.
Re-run `install` after any change to `ee3033_search/` or `ee3033_search/config/search.yaml`.

See `EE3033_Mapping_v7.4/README.md` and `EE3033_Nav_v10/README.md` for full usage, calibration and troubleshooting.

## Additional notes

The assessment map and target position are never pre-programmed. No map file is included in this repo.
During the autonomous run: no teleoperation, no manual goals, no map edits, no typed target coordinates.

## Demonstration Video
[Watch the robot demonstration video](https://github.com/twoipe/ee3033-balancing-car/raw/refs/heads/main/docs/navigation-video-demo.mp4)
