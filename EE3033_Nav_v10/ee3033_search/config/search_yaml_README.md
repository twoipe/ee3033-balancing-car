# search.yaml: How to tune and apply it

File: `EE3033_Nav_v10/ee3033_search/config/search.yaml`
Full background: `docs/Nav_v10_Reference.md` (sections 3 to 6). Copy-paste walkthrough: `docs/README_Nav_v10_walkthrough.md`.

This file holds the ROS 2 parameters for the `ee3033_search` node (camera calibration, target triangulation, search coverage, retries). Nothing in it takes effect until you run `install`.

## 1. How the file reaches the robot

1. You edit the file in the repo: `EE3033_Nav_v10/ee3033_search/config/search.yaml`.
2. `python3 ee3033.py install` copies the `ee3033_search/` package into `~/ros2_ws/src` (the previous copy is moved to `~/.ee3033_backups/`) and runs `colcon build --symlink-install`.
3. `setup.py` installs `config/*.yaml` to `share/ee3033_search/config/`.
4. `search.launch.py` passes that installed file to the node. The node declares built-in defaults, and values in this file override them.
5. `ee3033.py run --target bottle` sets `EE3033_TARGET`, which overrides only `target_class` for that run.

Rule: **run `install` after every edit to `search.yaml` or to anything under `ee3033_search/`.** The repo copy is the source of truth; edits made only inside `~/ros2_ws` will be overwritten by the next `install`.

## 2. Edit workflow

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10
nano ee3033_search/config/search.yaml
python3 ee3033.py install
```

Check the values that matter:

```bash
grep -n "calibration_verified\|target_class\|camera_horizontal_fov_deg\|camera_yaw_offset_deg" ee3033_search/config/search.yaml
```

Ground rules:

- Change **one** parameter at a time and repeat the same trial.
- Keep YAML formatting: two-space indent under `ros__parameters:`, strings in quotes, `true`/`false` in lowercase.
- **Never edit `target_class`.** It must stay `''`; pass the class with `--target`.
- **Freeze rule:** stop editing at least 30 minutes before the demonstration, run `install`, then do two rehearsals with the frozen values.

## 3. Calibrating the two camera values

These are the only parameters the reference tells you to calibrate rather than tune.

| Key | Meaning |
|---|---|
| `camera_horizontal_fov_deg` | Converts a pixel column to a bearing |
| `camera_yaw_offset_deg` | Residual yaw added on top of the camera TF yaw |

Steps (robot still, perception running):

1. Start perception: `python3 ee3033.py perception`
2. Place one clearly detectable object (a bottle works) at points measured **from the camera**, not the robot centre. X is forward, Y is left. Suggested placements (X, Y in metres): A 1.0, +0.25; B 1.0, 0.00; C 1.0, -0.25; D 0.7, -0.15. Keep the object fully inside the image.
3. For each placement, read the box-centre pixel `x` three times and average:
   ```bash
   source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 && ros2 topic echo /yolo/detections --once | grep -E "class_name|score|x:|y:"
   ```
   Use the `x` under `bbox: center:` and ignore `size`.
4. Fit:
   ```bash
   python3 bearing_calibration.py --obs 1.0,0.25,U_A 1.0,0.0,U_B 1.0,-0.25,U_C 0.7,-0.15,U_D
   ```
   Replace `U_A` ... `U_D` with your averaged pixels. At least 3 placements at 3 different true bearings are required.
5. Decide from the output:

| Result | Action |
|---|---|
| RMS below 0.7 deg and yaw offset under 3 deg | Write both values into `search.yaml`, then `install` |
| Yaw offset above 3 deg | Fix the camera static TF yaw (reference section 1.3) first, then repeat |
| RMS above 1 deg | Re-measure from the camera, re-read pixels, check lighting |

The helper assumes a pinhole camera with the principal point at pixel 320 and no lens distortion.

## 4. The `calibration_verified` gate

`start` refuses to run while `calibration_verified` is `false`, including calibration trials. It is your manual declaration that calibration and trials look sane.

Unlock for supervised trials:

```bash
sed -i -E 's/^([[:space:]]*calibration_verified:).*/\1 true/' ee3033_search/config/search.yaml && python3 ee3033.py install
```

Lock again if trials fail:

```bash
sed -i -E 's/^([[:space:]]*calibration_verified:).*/\1 false/' ee3033_search/config/search.yaml && python3 ee3033.py install
```

Note: the reference table lists the default as `false`, but the file currently ships with `true`. Check it before every session.

## 5. Parameter reference

"Advice" is taken from the reference's demo advice. "Keep" means leave the shipped value.

### Run gate and identity

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `target_class` | `''` | Set per run by `--target` | Never edit |
| `calibration_verified` | `true` | Start gate | `true` only after trials |

### Camera and bearing

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `camera_horizontal_fov_deg` | 57.9 | Pixel to bearing | Calibrate (section 3) |
| `camera_yaw_offset_deg` | -2.05 | Residual yaw | Calibrate (section 3) |
| `camera_frame` | `camera_link` | TF frame of the camera | Change only if the frame name differs |
| `image_width`, `image_height` | 640, 480 | Pixel scale | Must match the camera |

### Detection and confirmation

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `min_confidence` | 0.45 | Mission YOLO threshold (the YOLO node itself uses 0.3) | Raise if false positives |
| `confirmations` | 3 | Confirmations needed | Keep |
| `refine_sec` | 2.0 | Stationary refine time | Can drop to 1.0 to save time |
| `lost_clear_sec` | 1.5 | Time before a target counts as lost | Keep |
| `target_max_spread_m` | 0.35 | Allowed spread of estimates | Keep |

### Triangulation (visual_*)

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `visual_min_baseline_m` | 0.20 | Minimum camera separation | Keep |
| `visual_min_cross_angle_deg` | 6.0 | Minimum angle between rays | Keep |
| `visual_max_bearing_error_deg` | 3.0 | Inlier tolerance | Relax to 4 only if good detections are rejected |
| `visual_min_observations` | 4 | Observations needed | Keep |
| `visual_history_sec` | 35.0 | Observation window | Keep |
| `visual_sample_interval_sec` | 0.35 | Spacing between samples | Keep |
| `visual_max_distance_m` | 2.5 | Maximum range | Keep unless the sandbox is larger |

### Viewpoint (parallax) behaviour

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `viewpoint_move_m` | 0.40 | Sideways move for parallax | Keep |
| `viewpoint_max_attempts` | 2 | Candidate viewpoints | Keep |
| `viewpoint_observe_sec` | 2.5 | Wait for detections after aiming | Raise to 3.5 if YOLO is slow |
| `viewpoint_sweep_deg`, `viewpoint_max_sweeps` | 16.0, 4 | Reacquisition sweeps | Keep |
| `viewpoint_range_hint_m` | 1.2 | Aim only, not a range | Lower to about 0.9 if objects usually appear close |
| `viewpoint_cooldown_sec` | 5.0 | Delay before another reposition | Keep |
| `viewpoint_trigger_frames` | 2 | Detections before leaving search | Keep |
| `viewpoint_trigger_max_bearing_deg` | 12.0 | Bearing agreement for the trigger | Keep |

### Wall rejection and clearance

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `reject_wall_hits` | `true` | Reject targets on mapped walls | Keep |
| `wall_hit_radius_m` | 0.10 | Wall-rejection radius | See section 6 |
| `robot_clearance_m` | 0.13 | Clearance for stops and viewpoints | Keep |

### Search coverage

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `search_spacing_m` | 0.3 | Search lattice spacing | Larger spacing means fewer stops and less time |
| `max_sites` | 40 | Maximum search sites | Keep |
| `coverage_range_m` | 2.0 | Trusted detection range | Lower if YOLO is poor at range |
| `coverage_fov_fraction` | 0.8 | Share of FOV counted as covered | Keep |
| `min_gain_fraction` | 0.12 | Minimum new coverage to scan | Raise to 0.2 to speed up |
| `spin_step_deg` | 90.0 | Spin increment | Keep |

### Retries and timeouts

| Key | Shipped | Meaning | Advice |
|---|---|---|---|
| `max_navigation_failures` | 6 | Failures before heading home | Keep |
| `max_home_retries`, `home_retry_delay_sec` | 6, 3.0 | Return-home retries | Keep |
| `cmd_ack_timeout_sec` | 5.0 | A* silence timeout | Keep |
| `max_scan_age_sec` | 2.0 | LiDAR freshness at start | Keep |

### Frames and topics (leave unless your ROS setup differs)

`map_frame` (`map`), `base_frame` (`base_footprint`), `map_topic` (`/map`), `scan_topic` (`auto`), `detections_topic` (`/yolo/detections`), `detections_message_type` (`yolo_msgs/msg/DetectionArray`).

## 6. Which key to change when a trial goes wrong

After a trial, compare the reported position with the true one:

```bash
python3 ee3033.py error --x 1.0 --y 0.0 --rx REPORTED_X --ry REPORTED_Y
```

Always pass `--rx --ry`; without them the tool can score a previous run.

| Pattern | Likely cause | Change |
|---|---|---|
| Same sideways shift at every position | Camera yaw | `camera_yaw_offset_deg` (or the TF yaw) |
| Sideways error grows toward the edges | FOV | Recalibrate `camera_horizontal_fov_deg` |
| Error mostly along the viewing direction | Short or inaccurate baseline, or yaw timing | Check the actual lateral displacement; prefer a stationary view |
| Large and random | Localization wrong | Re-set the AMCL pose; check the scan lies on the walls |
| `rejected_on_wall` | Object within `wall_hit_radius_m` of a wall, or pose error | Optionally `wall_hit_radius_m: 0.05`; test once with an object near a wall first |
| No `TARGET CONFIRMED` | See the counters below | |

Counters printed every 5 seconds during a run (`detection stats (last 5 s)`):

| Counter | Meaning | Action |
|---|---|---|
| nothing printed | No detections arrive | `ros2 topic hz /yolo/detections`; restart perception |
| `no_target_class_or_low_conf` high | YOLO names the object differently, or low score | Use the class YOLO prints with `--target`; check lighting; maybe lower `min_confidence` |
| `missing_camera_tf` | No time-matched `map -> camera_link` | Check the static TF, the AMCL pose and `camera_frame` |
| `invalid_box` | Box centre outside the image | Check `image_width` and `image_height` |
| `insufficient_parallax_or_inconsistent_bearings` | Rays too parallel, too few views, or bearings disagree | Check actual displacement, FOV and yaw; see the `visual_*` keys |
| `estimates_accepted` grows but no `TARGET CONFIRMED` | Estimates scatter by more than `target_max_spread_m` | Pose, FOV or yaw problem |

`status` also shows `camera_views=` and `camera_baseline_m=`; a baseline under 0.20 means triangulation cannot succeed.

## 7. Related file: `config/astar.yaml`

Motion limits (speeds, safety margins, goal tolerance, watchdogs) live in `EE3033_Nav_v10/config/astar.yaml`, not here. It is shipped to the Pi at each start, so it does not need `install`. See reference section 4.2.
