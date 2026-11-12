# EE3033 Navigation v10

## 1. What changed from v8.2

Nav v10 retains the v8.2/v9 mission, A* navigation, AMCL, YOLO, camera-only multi-view triangulation, automatic sideways viewpoint, and return-home behavior. **Changes in v10 (tooling only):** `ee3033.py rviz` now looks up the live YOLO annotated-image topic (`/yolo/dbg_image` vs `/yolo/debug_image` differs between yolo_ros versions) and feeds it to RViz instead of a hard-coded name; a new `ee3033.py image` command opens the annotated image in `rqt_image_view` (the same viewer used by the one-command startup) as a fallback when RViz image panels stay black in a VM; The YOLO launch now passes `detections_topic:=detections` so the debug node reads the raw detections (with `use_tracking:=False` the default `tracking` topic never publishes, which left `/yolo/debug_image` empty). **No navigation algorithm or safety thresholds were changed.** The integrated system has been fully tested on the physical robot in this release.

The robot uses LiDAR for mapping, localization and obstacle avoidance, **not for measuring target range**. Camera bearings from distinct robot positions determine the target's map coordinates. The target must be a recognized YOLO class, and the method assumes one stationary object of that class.

## 2. Quick start: one command for navigation

First complete mapping with Mapping v7.4, save the map, stop mapping, and put the robot at its intended start mark. Install Nav v10 at least once and verify physical calibration before enabling the mission. Then use separate terminals:

**Terminal 1 — combined launch (keep open):**

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py run --target bottle
```

`run` starts perception, the Pi navigation stack, the VM mission node and their required processes. Replace `bottle` with the exact class recognized by YOLO (e.g. `cup`, `apple`, `sports ball`). It **does not** start mapping, RViz, or arm the robot. Do not also start `perception` or `nav` separately when using `run`.

**Terminal 2 — pose and checks (each command is a complete one-liner):**

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py set-pose --x 0 --y 0 --yaw-deg 0
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py status
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py preview
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py start
```

`set-pose` assumes the robot really is at `(0,0)` facing map +x. **Do not use those coordinates if the actual start pose differs.** Alternatively, open RViz in another terminal and use **2D Pose Estimate**. Before `start`, check that the saved map is correct, the LiDAR scan overlays walls in RViz, the robot is stationary, the target class is correct, camera TF is available, the A* heartbeat is present, and calibration has been verified. The `start` command asks you to type `START` before motion.

**Optional RViz terminal** (start YOLO first via `perception` or `run`, so the annotated-image topic can be detected; if the image panels are black, use `python3 ee3033.py image` instead):

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py rviz
```

**Stop and shutdown (separate commands):**

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py stop
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py down
```

`stop` is a software cancel, **not an emergency stop**. Keep access to the physical power switch. `down` stops the Pi stacks; close the combined-launch terminal as appropriate. Do not teleoperate, set manual goals, edit the map, or type target coordinates during the autonomous assessment.

## 3. Mapping and installation

**Mapping, before the navigation run:**

```bash
cd ~/ee3033-balancing-car/EE3033_Mapping_v7.4 && python3 mapping.py
```

Use the mapping program's `Ctrl+C` menu, choose `s` and confirm `MAP SAVED` and the map check, then `q` and `STOP`. Mapping v7.4 is in the sibling `EE3033_Mapping_v7.4/` folder.

**One-time installation and offline checks (VM):**

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py offline
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py install
ssh balance@balance10.local echo ok
```

`install` copies the navigation ROS package into `~/ros2_ws/src/ee3033_search` (backing up the prior copy) and builds the workspace. **Run `install` again after every change to `ee3033_search` Python files or `ee3033_search/config/search.yaml`.** The archive assumes the existing ROS 2 Humble workspace, Pi robot bringup, YOLO dependencies, model weights and SSH connection. Set an actual model path if needed:

```bash
export EE3033_YOLO_MODEL="$HOME/ros2_ws/yolo26s.onnx" && ls -lh "$EE3033_YOLO_MODEL"
```

The path above is an example; use the location of the model actually installed on your VM. The environment variable must be available in the launch terminal (for example, configure it in `~/.bashrc`).

## 4. ROS 2 terminal environment: when sourcing is needed

**`python3 ee3033.py` commands:** `status`, `set-pose`, `preview`, `start`, `stop`, `rviz`, `run` and other ROS-facing commands already use the launcher’s `ros_shell`, which sources `/opt/ros/humble/setup.bash`, then `~/ros2_ws/install/setup.bash`, and exports `ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0`. You only need `cd ... && python3 ee3033.py ...` in a new terminal. No extra manual `source` is required for these commands.

**Direct `ros2` commands:** source the environment once per new terminal:

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0
```

Or use a fully self-contained one-liner, for example:

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 && ros2 run tf2_ros tf2_echo map camera_link
```

Further direct checks (after sourcing):

```bash
ros2 topic hz /yolo/detections
ros2 topic echo --once /astar/status
ros2 topic info -v /cmd_vel
ros2 run tf2_ros tf2_echo base_footprint camera_link
```

## 5. Calibration: exact parameters and procedures

Edit `~/ee3033-balancing-car/EE3033_Nav_v10/ee3033_search/config/search.yaml` for mission parameters and `~/ee3033-balancing-car/EE3033_Nav_v10/config/astar.yaml` for the Pi A* parameters. Start with the defaults. **The values below are starting settings, not verified physical measurements.** Record the old value, change only one variable at a time, repeat the same trial, and compare position error and safety behavior. Re-run `install` after edits to `search.yaml`; `astar.yaml` is transferred to the Pi when navigation starts. Restart the navigation stack after changes.

### 5.1 Camera, geometry and target estimation

| Parameter (default) | Significance | Specific calibration / verification instructions |
|---|---|---|
| `camera_frame: camera_link` | TF origin and axes for every measured ray | With the robot still, run `ros2 run tf2_ros tf2_echo base_footprint camera_link` in a sourced terminal. Check that camera position matches tape-measured offsets from the robot base and its forward axis points along the actual optical direction. Correct the **static TF in robot bringup**, not this frame name, if the physical mounting transform is wrong. Then check `map camera_link` after AMCL pose initialization. |
| `camera_horizontal_fov_deg: 48.3` | Converts YOLO box-centre x pixel to a horizontal bearing | Put a small detected object at the camera's height, approximately **1.00 m forward and 0.40 m left**, measured from the camera optical centre; keep the robot still. Read the box-centre x pixel `u` in a 640 px image from `/yolo/detections`. Compute `FOV = 2 atan((320 × 0.40/1.00)/(320-u))` in degrees. Repeat with 0.40 m right (`u>320`; use absolute values) and at several offsets. Example calculator: `python3 -c 'import math;u=181;print(math.degrees(2*math.atan(128/abs(320-u))))'`. **This formula assumes zero camera yaw bias and negligible distortion**; calibrate mounting yaw separately and repeat. The current 48.3° value is unverified for v10 triangulation. |
| `camera_yaw_offset_deg: 0.0` | Residual angular correction added to camera TF yaw | Face the robot along a measured map +x line; put a target directly on the camera's forward optical axis. Its YOLO centre should be near pixel 320. Measure target positions left and right and compare computed map bearings with tape-measured `atan2(target_y-camera_y,target_x-camera_x)`. If the same signed angular error appears across positions, correct the physical TF yaw first; use this offset only for a small residual. Re-test both sides, avoiding double compensation. |
| `image_width: 640`, `image_height: 480` | Pixel scale used for box-to-bearing conversion | Check the actual camera image dimensions and detection coordinates. If resolution changes, match both settings and recalibrate FOV. Do not tune width/height to compensate for geometric error. |
| `visual_min_baseline_m: 0.20` | Minimum separation between observation camera positions | Mark the two physical camera positions on the floor and compare their distance with TF/AMCL displacement. Aim for at least 0.20 m **actual** separation; if a 0.40 m commanded move finishes short, first verify A* goal tolerance and AMCL accuracy. Increasing this threshold rejects weak baselines but may reduce successful fixes. |
| `visual_min_cross_angle_deg: 6.0` | Minimum angle between two viewing rays | Test one target around 1 m away using two viewpoints separated laterally by ~0.40 m. The rays should intersect at a visibly nonparallel angle; repeat with targets at 1.5–2 m. Keep 6° initially; increasing it rejects poorly conditioned distant pairs, while lowering it makes range estimates more sensitive to bearing noise. |
| `visual_max_bearing_error_deg: 3.0` | Maximum disagreement of an observation with the fitted target | At three tape-measured target positions, repeat trials and inspect scatter in estimated positions. If good detections are rejected, first check FOV, yaw, TF timestamps and camera motion. Tighten only if false matches persist; relaxing admits inconsistent rays and can worsen accuracy. |
| `visual_min_observations: 4` | Minimum ray observations for a triangulation solution | Observe the same stationary target from two separated locations; confirm at least four usable detections across the two positions. If detections are too sparse, improve lighting/model rate before reducing the threshold. Four frames from one pose cannot determine range. |
| `visual_history_sec: 35.0` | Rolling time window for candidate rays | Measure the time from first detection to arrival and observation at the sideways viewpoint. Keep the history longer than that elapsed time so both views coexist. If the move routinely takes over 35 s, consider increasing it, but longer windows can retain stale poses or unrelated detections. |
| `visual_sample_interval_sec: 0.35` | Minimum interval between saved detections | Check `/yolo/detections` frequency; 0.35 s permits at most ~2.9 samples/s. If YOLO is slower than this, the setting is not the bottleneck. Avoid making it very short just to collect nearly identical frames. |
| `visual_max_distance_m: 2.5` | Rejects solutions beyond this camera-to-target range | Place targets at measured 1.0, 1.5 and 2.0 m, then near the farthest required distance. Confirm reliable solutions inside the intended range. Increase beyond 2.5 m only if camera detection and triangulation remain accurate there; longer ranges amplify bearing error. |
| `target_max_spread_m: 0.35` | Allowed spread among successive candidate target positions | Run 3+ trials at each measured position. Compare accepted estimates with tape truth; reduce only after pose/FOV errors are fixed. A smaller spread rejects inconsistent fixes but may prevent confirmation. |
| `confirmations: 3`, `refine_sec: 2.0` | Consecutive estimates needed, then stationary refinement wait | At a fixed object, check that confirmation is repeatable and the robot remains still for refinement. If reports are unstable, examine independent-view geometry first: successive confirmations reuse much of the same observation history. Increasing these values alone does not prove independence. |
| `min_confidence: 0.45` | Minimum YOLO confidence accepted by the mission | Check `/yolo/detections` for the chosen class under normal arena lighting, at centre and image edges. Raise if false positives occur; lower only if true detections are consistently rejected and misclassifications remain acceptable. This is distinct from the detector's own confidence threshold. |
| `lost_clear_sec: 1.5` | Time without a target before trigger/confirmation continuity expires | Observe target detections while stationary and during a short turn. If legitimate detections are separated by more than 1.5 s, improve YOLO throughput first; increase only enough to tolerate real gaps, since old observations may then be treated as continuous. |

### 5.2 Automatic viewpoint acquisition and coverage

| Parameter (default) | Significance | Specific calibration / verification instructions |
|---|---|---|
| `viewpoint_trigger_frames: 2` | Consecutive qualifying detections before leaving coverage search | Put one bottle clearly in view; inspect the mission log/status and verify it enters `REPOSITIONING` after two usable samples. Increase if single-frame false positives cause unnecessary moves. |
| `viewpoint_trigger_max_bearing_deg: 12.0` | Allowed angular disagreement in trigger observations | With robot still and target fixed, watch the detected box centre over several frames. If bearings vary more than 12°, check tracking/TF before relaxing. Narrowing reduces false triggers but can reject detections during rotation. |
| `viewpoint_move_m: 0.40` | Preferred sideways translation for parallax | Tape-mark a free 0.40 m lateral destination from the camera's initial position. Confirm the actual arrival displacement with TF and tape, and check target remains reacquirable. If the robot cannot safely make this move, adjust for the actual arena width, but keep actual displacement above `visual_min_baseline_m`. |
| `viewpoint_max_attempts: 2` | Number of candidate alternative viewpoints | Intentionally block one side during a supervised trial and verify it tries the other candidate. Increase only if there are safe, reachable alternatives and enough demo time. |
| `viewpoint_observe_sec: 2.5` | Hold after aiming/sweeps to collect detections | Measure how long YOLO takes to return at least two valid samples once stationary. Use a hold longer than this; if detections remain absent, inspect camera aim and model rather than only increasing wait. |
| `viewpoint_sweep_deg: 16.0`, `viewpoint_max_sweeps: 4` | Reacquisition turn size and number | With target placed near each camera FOV edge, check whether the target is found within the sweep sequence. Verify enough physical clearance for the balancing robot to turn. Larger sweeps cover more angle but can skip a narrow target between observations. |
| `viewpoint_range_hint_m: 1.2` | **Aiming-only** distance along first camera ray | Test targets at 0.6, 1.0 and 2.0 m. Observe whether the camera points toward the target after sideways movement; adjust the hint to improve reacquisition at typical range. **It is not a measured range and does not change triangulated distance directly.** |
| `viewpoint_cooldown_sec: 5.0` | Delay before another reposition episode | Simulate a failed viewpoint; time the return to coverage search and next trigger. Increase if the robot repeatedly oscillates into the same failed move; reduce only if retries are useful and safe. |
| `robot_clearance_m: 0.13` | Free-space clearance required for candidate search/viewpoint sites | Measure wall-to-robot clearance at candidate stops and check the entire turn, not just centre-point clearance. In a narrow passage, sites may be excluded even if a transit path exists. Do not reduce based only on grid visualization. |
| `search_spacing_m: 0.30`, `max_sites: 40` | Search-site lattice and cap | Use `preview` to inspect planned points on the actual map. Reduce spacing if camera blind spots remain; increase if too many redundant stops exceed the time budget. Confirm sites stay reachable and safely clear. |
| `coverage_range_m: 2.0`, `coverage_fov_fraction: 0.8` | Camera search visibility range and conservative fraction of nominal FOV | Place a detectable target at increasing distances and near left/right edges, then check actual reliable detections. Set coverage range no farther than reliable YOLO recognition; reduce FOV fraction if edge detections are poor. Re-run `preview` and a full coverage trial. |
| `min_gain_fraction: 0.12`, `spin_step_deg: 90.0` | Coverage gain needed to continue scanning; angular sweep increment | In RViz, inspect uncovered sectors after each stop. If the robot skips obvious unseen regions, lower the gain threshold or use smaller turns; time the extra scanning and check turn clearance. |
| `max_navigation_failures: 6`, `max_home_retries: 6`, `home_retry_delay_sec: 3.0` | Failure budget and return-home retries | In supervised trials with blocked sites, verify failure transitions and return behavior. Choose limits from observed recovery time and assessment duration; never bypass collision checks to force a retry. |
| `reject_wall_hits: true`, `wall_hit_radius_m: 0.10` | Rejects target estimates in or near mapped obstacles | Tape-place target at least 0.30 m from walls for initial calibration, then test closer placements. If true targets near walls are rejected, first inspect map alignment and camera geometry. Reducing radius risks accepting false wall positions. |
| `calibration_verified: false` | Explicit mission-start gate | Leave `false` by default. **Current code refuses `start` while false, including calibration trials.** For a supervised physical trial only, set it `true` temporarily after checking TF, LiDAR and stationary start, run `install`, and supervise motion. Restore `false` if any test fails. Keep `true` for demonstration only after successful physical validation. This flag is a manual declaration, not a measurement. |

### 5.3 A* motion, obstacle and corridor calibration

These parameters are in `config/astar.yaml`; some A* defaults such as `soft_margin_m` and `soft_cost` are defined in `pi/astar_realtime.py` and are not currently listed in the YAML. Test motion on a clear floor before any narrow passage.

| Parameter (default) | Significance | Specific calibration / verification instructions |
|---|---|---|
| `robot_radius: 0.07`, `safety_margin: 0.005` | Hard obstacle inflation for A* | Robot's measured wheel-to-wheel width is **0.185 m**, so half-width is 0.0925 m; the configured hard inflation totals only **0.075 m**. This is deliberately smaller than physical half-width to leave a 0.22 m gap traversable on a coarse grid, **not proof of collision clearance**. Map and measure the narrowest passage, then supervised-test at minimum speed with a person ready to cut power. Raise inflation if it clips walls; reducing it increases collision risk. |
| `guard_half_width_m: 0.10` | Width of LiDAR forward/rear safety lane | Check front and rear obstacle detection with a test obstacle centred in the robot's actual travel lane and near each wheel edge. Keep the lane broad enough to cover the robot body while avoiding false stops from side walls. |
| `stop_distance: 0.20`, `slow_distance: 0.45` | Forward obstacle stop and slow-down thresholds | Approach a stationary obstacle slowly with measured clearance and log where speed decreases and motion stops. Increase thresholds if braking distance or balance overshoot is too large. Do not decrease simply to fit a tight corridor. |
| `max_linear_speed: 0.10`, `max_angular_speed: 0.35`, `spin_speed: 0.35` | Speed limits in m/s and rad/s | Tape-mark 1 m travel and a 90° rotation; measure speed, overshoot and stability on a clear floor. Lower if balance or localization becomes unreliable. Test both turn directions. |
| `goal_tolerance: 0.15` | A* accepts a waypoint within 15 cm | Command a supervised 0.40 m lateral move and measure actual displacement. At the tolerance limit it may stop about 0.25 m from the original position, reducing triangulation baseline. Tighten only if the balancing controller can settle reliably without oscillation. |
| `reverse_speed: 0.05`, `reverse_clear_m: 0.35` | Back-off speed and required rear clearance | In a controlled blocked-front test, verify rear LiDAR clearance before reversing and measure actual backward movement. Lower speed if balance is unstable; ensure rear obstacle checks work. |
| `goal_timeout_sec: 120`, `blocked_timeout_sec: 8`, `progress_window_sec: 15` | Navigation timeout and stuck detection | Time normal routes, then safely obstruct a route and check the controller stops/fails instead of driving indefinitely. Tune to observed progress while preserving a bounded failure response. |
| `scan_timeout: 1.0` (A* code default), `max_scan_age_sec: 2.0` (mission YAML) | Stale LiDAR protection | While stationary, verify scan publication frequency. Under supervision, interrupt the scan feed and confirm A* ceases driving; never test by driving blindly. Do not extend timeouts to hide intermittent sensor faults. |
| `soft_margin_m: 0.20`, `soft_cost: 1.5` (A* code defaults) | Soft A* penalty near obstacles | Compare RViz paths in a wide aisle and narrow passage. Increase penalty to favour the aisle centre; if the path is infeasible, inspect hard inflation and map accuracy first. |

### 5.4 Physical calibration sequence

1. **Check geometry without motion.** Confirm the TF chain `map -> odom -> base_footprint -> camera_link`, the camera offset, map/scan alignment, the 640×480 image, and YOLO class. Set the pose only to the robot's measured map pose.
2. **Calibrate bearing.** Put the object at measured offsets from the **camera**: `(forward=1.0 m, left=0)`, `(1.0 m, +0.40 m)` and `(1.0 m, -0.40 m)`. Record box centres, compute FOV and verify yaw symmetry. Repeat at another distance.
3. **Calibrate movement.** Test the A* 0.40 m lateral viewpoint in clear space and measure actual camera translation, stop accuracy and whether YOLO reacquires the target. Do not infer travel distance from commanded distance alone.
4. **Supervised target-position trials.** Tape-measure the object's **map** coordinates for at least three locations, for example `(1.0, 0.0)`, `(1.0, -0.5)` and `(1.5, 0.5)` if those positions are safe and free. Run the complete mission and record each reported coordinate and Euclidean error. Repeat trials. Ten centimetres is an **internal accuracy target**, not an established assessment tolerance.
5. **Check failure behavior.** Test no target, target loss during a viewpoint move, a blocked viewpoint, and return home. Do not treat a successful offline test as proof of physical safety.

**Temporary supervised calibration unlock** (only after checking the robot and preparing a safe test area):

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && sed -i -E 's/^([[:space:]]*calibration_verified:).*/\1 true/' ee3033_search/config/search.yaml && python3 ee3033.py install
```

**Re-lock if the test fails:**

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && sed -i -E 's/^([[:space:]]*calibration_verified:).*/\1 false/' ee3033_search/config/search.yaml && python3 ee3033.py install
```

**Compare against tape-measured truth (only after a fresh confirmed target report):**

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py error --x 1.0 --y 0.0
```

`error` writes `~/ee3033_calibration_log.csv`. **Known v8.2/v9/v10 limitation:** the tool searches the report file for the last coordinate-bearing `TARGET REPORT` and can reuse a prior trial after a no-target run. Confirm the report belongs to the latest trial before running `error`; for an explicit reading, use `--rx REPORTED_X --ry REPORTED_Y` with the same command.

## 6. Troubleshooting and expected outputs

| Symptom | What to check |
|---|---|
| `status` shows a blank mission section | Mission node is not responding; do not type START. Check combined-launch terminal, workspace build and ROS domain. |
| `calibration_verified is false` | Physical calibration has not been declared complete; follow the supervised calibration procedure above. |
| `missing_camera_tf` | Check time-matched `map -> camera_link` TF and camera static mounting transform. Do not replace historical transforms with latest transforms during movement. |
| `insufficient_parallax_or_inconsistent_bearings` | Actual camera baseline may be under 0.20 m, crossing angle too small, or FOV/yaw/time alignment wrong. Measure the lateral move and check both camera poses. |
| `no_target_class_or_low_conf` | Check exact YOLO class name, confidence, lighting and `/yolo/detections`. |
| `rejected_on_wall` | Check actual object-to-wall clearance, map/scan alignment, camera TF and yaw. |
| `no fresh LiDAR scan` | Check `/scan` or `/scan_fixed`; do not start without working obstacle sensing. |
| A* fails in narrow passage | Check mapped width, inflation and physical 18.5 cm wheel-to-wheel width. A map path alone does not guarantee clearance. |
| Object lost after sideways move | Inspect viewpoint aim hint, FOV edge, YOLO rate and sweep behavior. The mission may try another viewpoint then resume searching. |
| Wrong reported `(x,y)` | Recheck map origin/AMCL pose, camera extrinsics, FOV, timestamp alignment, actual baseline and target association. |

## 7. Source and verification notes

- `ee3033.py`: VM command launcher, combined `run`, environment setup, status/pose/error commands.
- `ee3033_search/ee3033_search/mission_node.py`: ROS integration and target detections.
- `ee3033_search/ee3033_search/monocular_geometry.py`: bearing and camera-ray triangulation.
- `ee3033_search/ee3033_search/viewpoint_planner.py`: alternate free-space viewpoints.
- `ee3033_search/ee3033_search/mission_logic.py`: coverage, viewpoint, confirmation and return-home states.
- `pi/astar_realtime.py` and `config/astar.yaml`: Pi-side A* and motion control.

The YOLO launch now passes `detections_topic:=detections` so the debug node reads the raw detections (with `use_tracking:=False` the default `tracking` topic never publishes, which left `/yolo/debug_image` empty). Offline tests are supplied under `ee3033_search/tests/`; hardware, timing, camera mounting, corridor clearance and real-world localization must be checked on the actual robot. The v8.2 known limitations (including stale-pose checks and stale error reports) have **not** been repaired in this release.
