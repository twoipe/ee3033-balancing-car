# EE3033 Mapping launcher v7.4 (RViz2 + keyboard teleop + manual map save)

Runs on **Ubuntu VMware**. Separate from `EE3033_Nav_v10` (camera + YOLO + A* navigation).

## Where to put it
Clone the repo anywhere on the Ubuntu VM, e.g. `~/ee3033-balancing-car/EE3033_Mapping_v7.4/`
(**not** inside `~/ros2_ws/src` - it is not a ROS package).

## One-time requirements (Ubuntu)
    sudo apt install -y ros-humble-rviz2        # usually already installed
Teleop is now your **Lab 4 node**, started on the Pi like in the lab manual
(`ros2 run balance_robot_base balance_keyboard_control`). Nothing extra to install.

## Use
    cd ~/ee3033-balancing-car/EE3033_Mapping_v7.4
    python3 mapping.py
1. Connects to `balance@balance10.local`, starts the Pi container, starts **SLAM** (`balance_slam_bringup`).
2. Opens **RViz2** (map, LiDAR, TF) after ~8 s.
3. Drops you into the **Lab 4 keyboard teleop** in the same terminal (it runs on the Pi over SSH; use the keys it prints).
   Be ready to stop the robot before it reaches a wall.
4. **Ctrl+C** leaves teleop and shows a menu:
   `[t]` teleop again, `[s]` **save map now**, `[r]` reopen RViz2, `[q]` stop SLAM+RViz2, `[k]` leave everything running.
   You can also save from a second terminal at any time: `bash save_map.sh` (or `python3 mapping.py save-map`).

## Saving the map (manual, as you asked)
Nothing is saved automatically. Save when the map looks complete:
    bash save_map.sh                  # name: ee3033_assessment
    bash save_map.sh my_practice_map  # other name
- Saved on the Pi in `/home/ros2_ws/src/balance_mapping/maps/<name>.yaml/.pgm`, and copied to `~/ee3033_maps/`.
- An existing map with the same name is **kept** as `<name>_prev_<timestamp>`, never overwritten.
- It reports `MAP SAVED` only if both files exist and are non-empty; otherwise `MAP NOT SAVED` and SLAM stays up.
- `[q]` warns you if no map was saved from the menu. Stopping SLAM discards the live map.

## MAP CHECK after every save (new in v7.4)
After `MAP SAVED` the launcher inspects the PC backup copy and prints, for example:

    MAP CHECK OK:
      free_thresh 0.19 OK (unknown space stays unknown)
      map 57 x 50 cells = 2.85 m x 2.50 m; free 41 %, occupied 6 %, unknown 53 %
      origin (map x, y, yaw) = [-0.952, -1.76, 0]  <- this is where the robot started mapping: set-pose 0 0 0 there

- `free_thresh 0.19` is applied on the Pi right after `map_saver_cli` (Nav2's default 0.25 makes the grey unknown pixels load as FREE,
  so the planner would think the area outside the fence is open floor).
- `NEEDS ATTENTION` means: wrong `free_thresh` (the fix command is printed) or almost no walls mapped (drive along the fence, save again).
- Offline test of the check: `python3 -m unittest discover -s tests`.

## If it says SLAM "is already running"
v7.3+ checks the network directly (`ros2 node list --no-daemon`). If it still reports `/slam_toolbox`, it is real: either a
stale SLAM on the car (next section), a SLAM on this PC (`pgrep -af slam_toolbox`), or someone else on your ROS_DOMAIN_ID.

## Stale processes on the car
`python3 mapping.py stop` stops the SLAM launch **and sweeps leftover base / LiDAR / SLAM processes** on the Pi
(v7.1 left them running after `[q]`). Last resort: `ssh balance@balance10.local "docker restart ros2_humble"`.
Do not run this while another stack (navigation) is using the car.

## Safety / behaviour
- Does **not** start camera, YOLO or navigation, and refuses to start if `/slam_toolbox`, `/amcl`, the A* node
  or another teleop is already running.
- If the launcher terminal closes, SLAM keeps running on the Pi (so the map is not lost): `python3 mapping.py stop`.
- Other teleop: `export EE3033_PI_TELEOP_CMD="ros2 run <pkg> <node>"` (runs on the Pi), or `export EE3033_TELEOP_CMD="..."`
  to run one on this PC instead. (The generic `teleop_twist_keyboard` is NOT the default: it keeps the last velocity until
  you press `k`, and this robot's base does not stop by itself.)
- Env overrides: `EE3033_PI_HOST`, `EE3033_CONTAINER`, `EE3033_MAP_NAME`, `ROS2_WS`.

## Verified on the real robot
Written and unit-tested both offline and on the robot (save-map logic, shell supervision with fake ROS). RViz shows the scan, and the Lab 4 teleop starts and moves the car.
The RViz config is hand-written; if the displays look empty set *Fixed Frame = map* and add Map(/map) and LaserScan manually.
