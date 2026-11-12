# EE3033 Nav v10: walkthrough (mapping + navigation)

This is the short, copy-paste version. Parameters, calibration, TF checks and troubleshooting are in **`Nav_v10_Reference.md`**.

**Status:** Nav v10 is the v8.2 mission logic with updated documentation. The full pipeline (mapping, YOLO, triangulation, return home) has been verified end-to-end on the physical robot.

**Rules during the autonomous run:** no teleoperation, no manual goals (never click RViz *2D Goal Pose*), no map edits, no typed target coordinates. Keep a hand on the robot's power switch. `stop` is **not** an emergency stop.

Every command below is a complete one-liner. Folder names: Nav = `~/ee3033-balancing-car/EE3033_Nav_v10`, Mapping = `~/ee3033-balancing-car/EE3033_Mapping_v7.4`.

---

## 1. One-time setup (at home, not on demo day)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py offline
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py install
ssh balance@balance10.local echo ok
ls -lh "$EE3033_YOLO_MODEL"
```

Expected: all offline tests `OK`, the build finishes, `ok` prints **without** a password prompt, and the model file is listed. Run `install` again after **every** edit to `ee3033_search/` or `ee3033_search/config/search.yaml`.

Then run the checks in Reference section 1 (TF, `/cmd_vel`, YOLO) and complete the calibration in Reference section 3. Before the demonstration, `calibration_verified` must be `true` (Reference section 3.5).

---

## 2. Demonstration sequence at a glance

In this table, `down`, `run`, `rviz`, `set-pose`, `status` and `start` mean `cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py <command>`. The full commands are in section 3.

| # | Action | Terminal | Command | You must see |
|---|---|---|---|---|
| 1 | Before the timer | T1 | `down` | `Pi stacks stopped` |
| 2 | Timer starts: map | T1 | `python3 mapping.py` | RViz2, map growing |
| 3 | Map complete: save | T1 | `Ctrl+C`, then `s` | `MAP SAVED` and `MAP CHECK OK` |
| 4 | Stop mapping | T1 | `q`, then type `STOP` | `[slam] stopped.` |
| 5 | Object is placed and named | | | |
| 6 | Robot on the start mark, same spot and heading as when mapping began | | | |
| 7 | Start everything | T1 | `run --target <class>` | `ee3033_search ready (DISARMED)` |
| 8 | About 20 s later: RViz | T2 | `rviz` | map, scan on the walls |
| 9 | Pose and status | T3 | `set-pose` or RViz *2D Pose Estimate*, then `status` | `calibrated=True`, correct `target_class` |
| 10 | Go | T3 | `start`, type `START` | robot drives, hands off |
| 11 | Finished | | | `TARGET REPORT: (x, y) = ...` then `Returned to the start area.` |

---

## 3. Step by step

### 3.1 Clean start (before the timer)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py down
```

### 3.2 Mapping (Terminal 1)

```bash
cd ~/ee3033-balancing-car/EE3033_Mapping_v7.4 && python3 mapping.py
```

Drive slowly with the keyboard. Cover the whole fence and close loops. When the map is complete:

1. Press `Ctrl+C` to open the mapping menu.
2. Press `s`. Confirm both `MAP SAVED` and `MAP CHECK OK`. If it says `NEEDS ATTENTION`, do what it prints.
3. Press `q`, then type `STOP`. Wait for `[slam] stopped.`

Navigation looks for the map named **`ee3033_assessment`**. Check that the file exists on the Pi:

```bash
ssh balance@balance10.local "docker exec ros2_humble ls -l /home/ros2_ws/src/balance_mapping/maps"
```

You should see `ee3033_assessment.yaml` and `ee3033_assessment.pgm`. If your map has another name, add `--map-name <name>` to the `run` command in step 3.4.

### 3.3 Object and start position

The examiner places the object and tells you its class. Use the **exact YOLO class name in lowercase**: `bottle`, `cup`, `apple`, `orange`, `banana`, `bowl`, `sports ball`, `cell phone`, `teddy bear`, `book`.

Put the robot on the start mark, in the same position and heading it had when mapping began.

### 3.4 Start the navigation stack (Terminal 1, after mapping has stopped)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py run --target bottle 2>&1 | tee ~/run_log.txt
```

Replace `bottle` with the announced class. `run` starts the camera, JPEG compression, YOLO, localization (map server and AMCL), the A* navigator and the mission node. It does not start RViz and does not move the robot. Leave this terminal open.

Wait until you see `Target class for this run: 'bottle'`, `[pi-nav] navigation stack running` and `ee3033_search ready (DISARMED)`. This takes about 20 to 30 seconds.

### 3.5 Look at the map and set the pose (Terminals 2 and 3)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py rviz
```

Set the robot pose, **just before starting**, because the robot drifts in yaw. Use one of the two methods:

- **Method 1 (RViz):** click *2D Pose Estimate*, then drag the arrow from the start mark along the robot heading.
- **Method 2 (command):** only if the robot is exactly at map `(0, 0)` facing map +x, as it was when mapping began:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py set-pose --x 0 --y 0 --yaw-deg 0
```

Check in RViz that the LiDAR scan lies **on the map walls** and the pose arrow sits on the start mark. If the scan is offset, set the pose again. Then:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py status
```

In the `status` output, check:

| Item | Expected |
|---|---|
| mission section | not blank |
| `state=` | `IDLE` |
| `map=True`, `pose=(...)` | pose near the start mark |
| `scan_age`, `astar_age` | below 2 s |
| `target_class=` | the announced class |
| `calibrated=` | `True` |
| `/cmd_vel` publishers | several listed (see note below) |
| YOLO | a `hz` line |

**Note on `/cmd_vel` publishers:** `status` lists `ee3033_astar_realtime`, `behavior_server` (several times) and `velocity_smoother`. This is normal. The last two belong to the lab's Nav2 bringup and stay silent while no Nav2 goal is sent. Only A* commands motion. See Reference section 2.

A blank mission section means the mission node is dead: **do not press START**.

### 3.6 Optional preview (about 10 s)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py preview
```

Orange arrows in RViz show the planned search sites. The robot does not move.

### 3.7 Start the autonomous mission (Terminal 3)

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py start
```

Type `START` when asked. The robot starts **disarmed** until you do. Check first that the robot is stationary and the area is clear.

Messages you should see in Terminal 1, roughly in this order:

1. `Search started: N candidate sites`
2. `Target recognized: suspending coverage search to obtain a second camera viewpoint`
3. `Camera viewpoint 1/2: (x, y) m`
4. `At second viewpoint: waiting for camera detections and triangulation`
5. `camera triangulation: ... -> map (x, y)`
6. `TARGET CONFIRMED (provisional)`
7. `TARGET REPORT: (x, y) = (...) m`
8. `Returned to the start area. TARGET REPORT: ... Mission time ... s.`

The report is also written to `~/ee3033_target_report.txt`.

### 3.8 After the run

When given the true position, compare it:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py error --x 1.0 --y 0.0 --rx REPORTED_X --ry REPORTED_Y
```

Type the reported numbers in explicitly (`--rx --ry`). Without them the tool can re-use an older run's report.

Shutdown:

```bash
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py stop
cd ~/ee3033-balancing-car/EE3033_Nav_v10 && python3 ee3033.py down
```

---

| Symptom | Action |
|---|---|
| `Cannot start: ...` on `start` | Read the listed reasons: pose not set, target class empty, `calibration_verified` false, scan or A* silent. |
| `nav says a stack is already running` | `down`, then `run` again. |
| `Map file ... missing in the Pi container` | Map was saved under another name or not saved. Use `--map-name`, or map again. |
| `pi-nav exited with code 255` | SSH link to the Pi dropped. `down`, then restart `run`. |
| Robot stuck against something | `stop`. Power switch if it keeps moving. |
| Detection never confirms | Check `detection stats (last 5 s)` in Terminal 1; see Reference section 6. |
| No target found, robot returned home | `FINISHED_NO_TARGET`. Partial credit may still apply for the demonstrated functions. |

More in the Reference file, section 6.
