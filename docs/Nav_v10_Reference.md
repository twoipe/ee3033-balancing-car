# EE3033 Nav v10: Reference (checks, TF, calibration, parameters)

Companion to `README_Nav_v10_walkthrough.md`.

Direct `ros2` commands need the environment in each new terminal:

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0
```

`python3 ee3033.py ...` commands set this up themselves.

---

## 1. Pre-flight checks (10 minutes)

Start the stack in one terminal (`python3 ee3033.py run --target bottle`), set the AMCL pose (RViz *2D Pose Estimate* or `set-pose`), then run the helper in another terminal:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && bash preflight_checks.sh 2>&1 | tee ~/preflight_out.txt
```

`preflight_checks.sh` and `bearing_calibration.py` are already in `EE3033_Nav_v10/`. The script is read-only: it echoes topics, TF and parameters and commands nothing.

### 1.1 What the first outputs already show

| Observation | Reading |
|---|---|
| `/camera_static_tf` exists and publishes `/tf_static` | A camera static transform is being published. Its **values and frame names are not yet verified**. |
| First `/tf` message: `odom_combined -> base_footprint`, identity | The robot's odometry frame is named `odom_combined`, not `odom`. Identity just means it had not moved at that instant. **AMCL must use `odom_frame_id: odom_combined`** (check 4 of the script). |
| `A message was lost!!!` on `ros2 topic echo /tf` | Normal for a CLI echo over Wi-Fi. Not a mission problem on its own. |
| `/cmd_vel` has 6 publishers and 1 subscriber (`balance_robot_node`) | See section 2. |

### 1.2 Reading the script output

| Check | Pass | Fail meaning and action |
|---|---|---|
| 1. static TF names | a pair with child `camera_link` (or the child name you actually use) | Child has another name: set `camera_frame` in `search.yaml` to that name, then `install`. |
| 2. `base_footprint -> camera_link` | prints Translation and Rotation | "Invalid frame" or hangs: parent frame is not connected to `base_footprint`; see 1.3. |
| 3. LiDAR height | z of the LiDAR frame about 0.15 to 0.20 m | Tells you `base_footprint` is on the floor, so a camera z of about 0.14 m is consistent. |
| 4. AMCL frames | `global_frame_id: map`, `base_frame_id: base_footprint`, `odom_frame_id: odom_combined` | `odom` here while the robot publishes `odom_combined` breaks `map -> base_footprint`. Fix in the lab bringup's AMCL config. |
| 5. `map -> base_footprint`, `map -> camera_link` | both print, repeatedly | Not available: pose not set, or chain broken (see 4 and 1.3). |
| 6. `/cmd_vel` idle | nothing printed | Anything printed while A* is idle: stop, see section 2. |
| 7. `/goal_pose` listeners | A* and (probably) `bt_navigator` | Reminder: never click *2D Goal Pose* during the mission. |
| 8. smoother and behavior server | list their topics | See section 2. |
| 9. YOLO and A* | `average rate` several Hz; `/astar/status` shows `IDLE` | YOLO below about 3 Hz: expect weak triangulation (see section 5). |

### 1.3 Camera TF: expected values and fixes

Your measurements: camera about **14 cm above the ground**, robot about **20 cm tall and 18 cm wide**.

Expected `base_footprint -> camera_link` (check 2):

| Field | Expected | Matters for the mission? |
|---|---|---|
| Translation x | tape-measured camera distance **forward** of the `base_footprint` origin (v7.6 notes used about -0.035 m, i.e. slightly behind) | Yes, a few cm of error |
| Translation y | measured left offset, about 0 | Yes, a few cm of error |
| Translation z | about **0.14** | **No.** The mission uses only x, y and yaw of the camera pose. v7.6 notes used 0.2655 m, which contradicts your 14 cm. A wrong z is cosmetic. |
| Rotation (RPY) | roll about 0, pitch about 0 (a small downward tilt is fine), **yaw about 0** | Yaw: **critical** |

Triage, in priority order:

| What you see | Fix |
|---|---|
| Yaw is about +/-90 deg, or the rotation looks like roll -90 / yaw -90 | This is an **optical-frame convention**. The mission needs `camera_link` with **x forward**. Correct the static TF to a forward-facing x-axis. Do not use `camera_yaw_offset_deg` for this. |
| Yaw is off by a small constant (under 5 deg) | Leave the TF. Correct it with `camera_yaw_offset_deg` from the bearing calibration (section 3.2). |
| x or y more than 3 cm from tape | Correct the static TF (below), or accept up to about 3 cm. |
| z not 0.14 | Optional. Fix only if time allows. |
| "Invalid frame" for `camera_link` | Child named differently: set `camera_frame` to it. Parent not connected: see the next row. |
| Camera parent frame (for example `base_link`) is not connected to `base_footprint` | Add the missing link only if `base_link` has no other parent, in a separate terminal: `ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_footprint --child-frame-id base_link` |

To find where `camera_static_tf` is defined on the Pi (to edit its x, y, yaw):

```bash
ssh balance@balance10.local "docker exec ros2_humble bash -lc 'grep -rIn camera_static_tf /home/ros2_ws/src | head -n 10'"
```

Edit the numbers in the file it names, rebuild that package in the container if it is installed without `--symlink-install`, then `python3 ee3033.py down` and restart `run`. If the grep shows nothing useful, paste the script output and I can give the exact edit.

After any change, repeat checks 2 and 5.

---

## 2. `behavior_server` and `velocity_smoother` on `/cmd_vel`

**Evidence you collected:**

- Publishers on `/cmd_vel`: `ee3033_astar_realtime`, `behavior_server` x4, `velocity_smoother`. Only subscriber: `balance_robot_node`.
- `ros2 topic echo /cmd_vel` for 10 s: silent (twice).
- Both nodes `active [3]`.

**What they are.** Both are Nav2 navigation-stack nodes from the lab bringup. `behavior_server` publishes velocity only while executing a recovery behavior (spin, backup, wait), which `bt_navigator` requests when it has a navigation goal. `velocity_smoother` publishes only after it receives a command on its input topic. Our system uses neither: A* is the planner and controller; only `map_server` and AMCL from Nav2 are needed.

**The `/cmd_vel` subscriber count is 1** (`balance_robot_node` only). So `velocity_smoother` does **not** read `/cmd_vel`, and there is no feedback loop through it.

**Verdict:** they are not needed, but they are **harmless while idle**. **Leave them alone.**

Reasons not to deactivate or kill them:

1. Nav2's lifecycle manager keeps a bond with each managed node. Deactivating one node can make the manager shut down its whole group, which risks disturbing the bringup 3 hours before the demonstration.
2. Nothing in the pipeline sends them goals, so they never publish.
3. Removing them would need edits to the lab launch file, which I have not seen.

**The only ways they can become active:**

- Clicking *2D Goal Pose* in RViz (both `bt_navigator` and A* listen to `/goal_pose`). **Never click it** during mission or trials.
- Someone sending a Nav2 goal or action. Nobody does.

**Rule:** if `ros2 topic echo /cmd_vel` ever prints anything while A* is `IDLE`, run `python3 ee3033.py stop`, power-cycle the robot stack with `down`, and re-check.

**Still to confirm** (all printed by `preflight_checks.sh`, checks 6 to 8): `ros2 node info /velocity_smoother` (its input topic) and `ros2 topic info -v /goal_pose`. If the smoother's input turns out to be a topic that `behavior_server` or something else of ours publishes to, that is a further way the robot could move while A* is `IDLE`.

---

## 3. Calibration

### 3.1 Order and time

| Step | Time | Why |
|---|---|---|
| Camera TF (section 1.3) | 10 min | Wrong yaw or frame breaks every ray |
| Bearing: FOV and yaw (3.2) | 15 min | Directly sets target accuracy |
| Movement (3.3) | 10 min | Sets the actual baseline |
| Full trials (3.4) | 45 to 60 min | Proves the integrated mission |

### 3.2 Bearing: FOV and yaw offset (robot still)

Run only perception (lighter than `run`):

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py perception
```

Place one clearly detectable object (a bottle works) at points measured **from the camera**, not from the robot centre. Use **X = forward**, **Y = left**. Recommended placements:

| Placement | X (m) | Y (m) |
|---|---|---|
| A | 1.0 | +0.25 |
| B | 1.0 | 0.00 |
| C | 1.0 | -0.25 |
| D | 0.7 | -0.15 |

For each placement read the box-centre pixel (3 times, average):

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 && ros2 topic echo /yolo/detections --once | grep -E "class_name|score|x:|y:"
```

The box centre is the `x` under `bbox: center:` (it may appear as `position: x:`). Ignore the `size` values. Then:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 bearing_calibration.py --obs 1.0,0.25,U_A 1.0,0.0,U_B 1.0,-0.25,U_C 0.7,-0.15,U_D
```

Replace `U_A` ... with your pixel values. Output gives `camera_horizontal_fov_deg` and `camera_yaw_offset_deg` and the fit error.

| Result | Action |
|---|---|
| RMS below 0.7 deg, yaw offset under 3 deg | Write both values into `search.yaml`. |
| Yaw offset above 3 deg | Fix the camera TF yaw first (section 1.3), repeat. |
| RMS above 1 deg | Re-measure from the camera, re-read pixels, check lighting. |

Edit and rebuild:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && nano ee3033_search/config/search.yaml
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py install
```

(The helper assumes a pinhole camera with the principal point at pixel 320 and no lens distortion.)

### 3.3 Movement (clear floor, robot still on the start mark, pose set)

With the stack running, from a third terminal (home test only, never in the assessment):

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 && ros2 topic pub --once /astar/command std_msgs/msg/String "{data: '{\"type\":\"goto\",\"id\":\"t1\",\"x\":0.0,\"y\":0.4}'}"
```

Measure with tape how far the robot actually moved. The goal tolerance is 0.15 m, so it may stop about 0.25 m away. Pass: actual lateral displacement of at least 0.25 m and no oscillation. Then a 90 degree spin in both directions:

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 && ros2 topic pub --once /astar/command std_msgs/msg/String "{data: '{\"type\":\"spin\",\"id\":\"t2\",\"angle\":1.57}'}"
```

Use `-1.57` for the other direction. Check that the robot stays upright and the rotation is close to 90 degrees.

### 3.4 Full trials

Unlock only after 3.1 to 3.3 look sane (section 3.5). Always run:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py run --target bottle 2>&1 | tee ~/trial_N.log
```

| Trial | Object position (map, m) | Purpose |
|---|---|---|
| 1 | (1.0, 0.0) | Object visible almost immediately; exercises the whole viewpoint cycle |
| 2 | (1.0, -0.5) | Other side |
| 3 | (1.5, 0.5) | Longer range |
| 4 | remove the object after `Camera viewpoint 1/2` | Lost-target behaviour and time cost |
| 5 | no object at all | Check it returns and does not invent a report |

For trials 1 to 3: after `Returned to the start area`, compare:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py error --x 1.0 --y 0.0 --rx REPORTED_X --ry REPORTED_Y
```

Always pass `--rx --ry`: without them the tool can score a previous run after a no-target run.

Reading the error:

| Pattern | Cause | Change |
|---|---|---|
| Same sideways shift at every position | camera yaw | adjust `camera_yaw_offset_deg` (or TF yaw) |
| Sideways error grows toward the edges | FOV | recalibrate FOV (3.2) |
| Error mostly along the viewing direction | short or inaccurate baseline, or yaw timing | check actual displacement (3.3); prefer a stationary view |
| Large and random | localization wrong | re-set the AMCL pose, check the scan lies on the walls |
| `rejected_on_wall` | object within 0.10 m of a wall, or a pose error | see section 5 |
| No `TARGET CONFIRMED` | see section 6 | |

Change **one** parameter at a time and repeat the same trial. 10 cm is an internal target, not the examiners' tolerance.

### 3.5 Unlock and lock `calibration_verified`

`start` refuses to run while it is `false`, including calibration trials. This flag is your manual declaration. Changing it is your decision.

Unlock for supervised trials:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && sed -i -E 's/^([[:space:]]*calibration_verified:).*/\1 true/' ee3033_search/config/search.yaml && python3 ee3033.py install
```

Lock again if trials fail:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && sed -i -E 's/^([[:space:]]*calibration_verified:).*/\1 false/' ee3033_search/config/search.yaml && python3 ee3033.py install
```

Verify the current setting: `grep -n "calibration_verified\|target_class\|camera_horizontal_fov_deg\|camera_yaw_offset_deg" ~/ee3033-balancing-car/EE3033_Nav_v10/ee3033_search/config/search.yaml`. `target_class` must stay `''`; set the class with `--target`.

---

## 4. Parameters

### 4.1 `ee3033_search/config/search.yaml` (run `install` after edits)

| Parameter | Default | Meaning | Demo advice |
|---|---|---|---|
| `target_class` | `''` | Empty; set per run by `--target` | Never edit |
| `calibration_verified` | `false` | Start gate | `true` only after trials |
| `camera_horizontal_fov_deg` | 57.9 | Pixel to bearing | **Calibrate** (3.2) |
| `camera_yaw_offset_deg` | -2.05 | Residual yaw added to the camera TF | **Calibrate** (3.2) |
| `camera_frame` | `camera_link` | TF frame of the camera | Change only if the frame name differs |
| `image_width`, `image_height` | 640, 480 | Pixel scale | Must match the camera |
| `min_confidence` | 0.45 | Mission YOLO threshold (YOLO node itself uses 0.3) | Raise if false positives |
| `visual_min_baseline_m` | 0.20 | Minimum camera separation | Keep |
| `visual_min_cross_angle_deg` | 6.0 | Minimum angle between rays | Keep |
| `visual_max_bearing_error_deg` | 3.0 | Inlier tolerance | Relax to 4 only if good detections are rejected |
| `visual_min_observations` | 4 | Observations needed | Keep |
| `visual_history_sec` | 35 | Observation window | Keep |
| `visual_sample_interval_sec` | 0.35 | Spacing between samples | Keep |
| `visual_max_distance_m` | 2.5 | Maximum range | Keep unless the sandbox is larger |
| `viewpoint_move_m` | 0.40 | Sideways move for parallax | Keep |
| `viewpoint_max_attempts` | 2 | Candidate viewpoints | Keep |
| `viewpoint_observe_sec` | 2.5 | Wait for detections after aiming | Raise to 3.5 if YOLO is slow |
| `viewpoint_sweep_deg`, `viewpoint_max_sweeps` | 16, 4 | Reacquisition sweeps | Keep |
| `viewpoint_range_hint_m` | 1.2 | **Aim only**, not a range | Lower to about 0.9 if objects usually appear close |
| `viewpoint_cooldown_sec` | 5.0 | Delay before another reposition | Keep |
| `viewpoint_trigger_frames` | 2 | Detections before leaving search | Keep |
| `viewpoint_trigger_max_bearing_deg` | 12 | Bearing agreement for the trigger | Keep |
| `confirmations`, `refine_sec` | 3, 2.0 | Confirmation and still time | `refine_sec` can drop to 1.0 to save time |
| `target_max_spread_m` | 0.35 | Allowed spread of estimates | Keep |
| `reject_wall_hits` | `true` | Reject targets on mapped walls | Keep |
| `wall_hit_radius_m` | 0.10 | Wall-rejection radius | See section 5 |
| `robot_clearance_m` | 0.13 | Clearance for stops and viewpoints | Keep |
| `search_spacing_m`, `max_sites` | 0.30, 40 | Search lattice | Larger spacing means fewer stops and less time |
| `coverage_range_m` | 2.0 | Trusted detection range | Lower if YOLO is poor at range |
| `coverage_fov_fraction` | 0.8 | Share of FOV counted as covered | Keep |
| `min_gain_fraction` | 0.12 | Minimum new coverage to scan | Raise to 0.2 to speed up |
| `spin_step_deg` | 90 | Spin increment | Keep |
| `max_navigation_failures` | 6 | Failures before heading home | Keep |
| `max_home_retries`, `home_retry_delay_sec` | 6, 3 | Return-home retries | Keep |
| `cmd_ack_timeout_sec` | 5.0 | A* silence timeout | Keep |
| `max_scan_age_sec` | 2.0 | LiDAR freshness at start | Keep |

### 4.2 `config/astar.yaml` (shipped to the Pi at each start)

| Parameter | Default | Meaning | Demo advice |
|---|---|---|---|
| `robot_radius`, `safety_margin` | 0.07, 0.005 | Hard obstacle inflation, 0.075 m total | **Below the half-width of 0.0925 m.** Keep it unless the robot scrapes walls; if it does, raise it |
| `guard_half_width_m` | 0.10 | Width of the safety lane | Keep |
| `stop_distance`, `slow_distance` | 0.20, 0.45 | Stop and slow distances | Raise if braking overshoots; never lower |
| `max_linear_speed` | 0.10 | m/s | Raise in 0.02 steps **only** if driving is smooth |
| `max_angular_speed`, `spin_speed` | 0.35, 0.35 | rad/s | Keep |
| `goal_tolerance` | 0.15 | Goal accepted within 15 cm | Keep |
| `reverse_speed`, `reverse_clear_m` | 0.05, 0.35 | Back-off | Keep |
| `goal_timeout_sec`, `blocked_timeout_sec`, `progress_window_sec` | 120, 8, 15 | Watchdogs | Keep |

Hard-coded in `pi/astar_realtime.py` (not in the YAML): `scan_timeout` 1.0 s, `soft_margin_m` 0.20, `soft_cost` 1.5, `new_obstacle_margin_m` 0.10, `target_obstacle_m` 0.06.

**Freeze rule:** stop editing parameters at least 30 minutes before the demonstration, run `install`, and do two rehearsals with the frozen values.

---

## 5. Known limitations and workarounds

| Limitation | Workaround |
|---|---|
| **No localization freshness or quality check.** A stale AMCL pose is not detected. | Set the pose immediately before `start`; confirm the scan lies on the walls in RViz. |
| `error` can score a previous run's report | Always pass `--rx --ry`. |
| Estimates within 0.10 m of a mapped wall are rejected, with no fallback. The mission then returns home with no report. | Optional config-only mitigation: `wall_hit_radius_m: 0.05` (less rejection, more risk of accepting a bad fix). Test once with an object near a wall before using it. |
| Samples taken **while the robot is rotating** carry a heading bias from camera latency (about 2 deg at 100 ms latency is roughly 12 cm of range error). | Nothing to change in config. Be aware when interpreting trial errors. Errors that depend on the turn direction point at this. |
| Boxes clipped by the image border are not rejected | Keep calibration objects fully inside the image. |
| The aim hint (1.2 m) can miss targets closer than about 0.6 m | Lower `viewpoint_range_hint_m` if close targets are likely. |
| No cap on repeated reposition episodes and no global mission timeout | Watch the clock. After about 2 failed episodes, decide whether to let it run. |
| LiDAR plane is about 19 cm up (v7.6 notes); the camera is at 14 cm. Objects shorter than that (apple, ball, cup) are invisible to the LiDAR. | A* cannot see them. Keep the sandbox clear of other low objects; after the report A* keeps a 0.06 m keep-out around the reported point. |
| One stationary object of the target class is assumed | Do not leave other bottles, cups, etc. in view. |

---

## 6. Troubleshooting counters

While the mission runs, Terminal 1 prints `detection stats (last 5 s)` every 5 seconds.

| Counter | Meaning | Action |
|---|---|---|
| nothing printed | no detections arrive | `ros2 topic hz /yolo/detections`; restart perception |
| `no_target_class_or_low_conf` high, `other classes seen: {'cup@0.55'}` | YOLO names the object differently, or low score | Use the class YOLO prints with `--target`; check lighting |
| `missing_camera_tf` | no time-matched `map -> camera_link` | Section 1 checks 2 and 5; check the static TF and the AMCL pose |
| `missing_detection_stamp` | detections without a timestamp | YOLO node issue; restart perception |
| `invalid_box` | box centre outside the image | Check `image_width` and `image_height` |
| `insufficient_parallax_or_inconsistent_bearings` | rays too parallel, too few views, or bearings disagree | Check actual lateral displacement (3.3), FOV and yaw (3.2) |
| `rejected_on_wall` | estimate on a wall or in unknown space | Section 5 |
| `estimates_accepted` grows, but no `TARGET CONFIRMED` | estimates scatter by more than 0.35 m | Pose, FOV or yaw problem |

`status` also shows `camera_views=` (observations held) and `camera_baseline_m=` (largest camera separation seen); a baseline under 0.20 means triangulation cannot succeed.

---

## 7. Quick reference

**ROS topics:** `/map`, `/scan` or `/scan_fixed`, `/tf`, `/tf_static`, `/yolo/detections`, `/astar/command` (JSON `goto`, `spin`, `cancel`), `/astar/status`, `/cmd_vel`, `/ee3033/target_map`, `/ee3033/target_report`, `/ee3033/mission_state`, `/ee3033/search_sites`, `/ee3033/coverage`.

**Services:** `/ee3033_search/start`, `/ee3033_search/stop`, `/ee3033_search/status`, `/ee3033_search/preview`.

**Frames:** `map -> odom_combined -> base_footprint -> camera_link`, where AMCL provides `map -> odom_combined` (check 4), and the LiDAR frame hangs from `base_footprint`.

**Files written:** `~/ee3033_target_report.txt`, `~/ee3033_calibration_log.csv`.

**Emergency:** physical power switch first; `python3 ee3033.py stop`; `python3 ee3033.py down`.
