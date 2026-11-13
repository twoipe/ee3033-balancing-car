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

The assessment environment and target position were not known in advance or pre-programmed into the system. Mapping was performed on-site during the assessment, before the target object was placed in the environment.

The repository includes maps generated during the actual assessment for documentation and reproducibility. These maps were recorded during the demonstration and were not supplied to the robot before the mapping stage. During autonomous operation, no manual teleoperation, navigation-goal selection, map modification or manual specification of target coordinates was performed.

The recorded results include both successful and unsuccessful runs to document the system's capabilities and limitations.

## Recorded demonstration

[Watch the robot demonstration video](https://github.com/twoipe/ee3033-balancing-car/raw/refs/heads/main/docs/navigation-video-demo.mp4)

**Date:** 12 November 2026

**Result:** Target successfully localized; robot returned to the starting area.

```text
2026-11-12 20:07:21  Returned to the start area.
TARGET REPORT: (x, y) = (1.919, -0.954) m in the map frame.
Mission time: 87.1 s.
```

The robot detected the target, collected observations from different camera viewpoints and estimated its position through triangulation. After confirming the target coordinates, it autonomously returned to the starting area.

## Actual assessment results

**Date:** 13 November 2026

### Attempt 1 — Target localization unsuccessful

**Time:** 17:19:18  
**Mission duration:** 327.9 s  
**Outcome:** Returned to the starting area without a confirmed target position.

```text
2026-11-13 17:19:18  Returned to the start area.
No target was confirmed.
Mission time: 327.9 s.
```

The robot detected the target during the search but was unable to obtain a second usable observation for triangulation. The available observation viewpoint was too constrained for reliable target reacquisition, preventing the system from estimating and reporting the target coordinates.

Although target localization was unsuccessful, the robot autonomously returned to the starting area.

### Attempt 2 — Navigation and return failure

**Time:** 17:26:18  
**Mission duration:** 126.8 s  
**Outcome:** Failed to return to the starting area; no target position was confirmed.

```text
2026-11-13 17:26:18  FAILED: could not return to the start area.
No target was confirmed.
Mission time: 126.8 s.
```
During navigation, the robot collided with a mapped obstacle and became stuck. The navigation system was unable to recover and return to the starting area. A possible contributing factor was the navigation clearance configuration, which used a hard obstacle-inflation radius (0.075m) smaller than the robot's physical half-width (0.0925) to allow narrow passages. 

This failure highlights limitations in obstacle clearance and navigation recovery in confined environments. Further investigation is needed to determine whether the collision resulted from insufficient clearance, localization error or path-following behaviour.

## Assessment maps

The map files generated during the actual assessment are preserved in this repository as records of the tested environment.

Each saved map consists of a `.yaml` metadata file and its corresponding `.pgm` occupancy-grid image.

These files document the environment mapped during the assessment. They were generated on-site and were not preloaded or used as a substitute for the required mapping procedure.
