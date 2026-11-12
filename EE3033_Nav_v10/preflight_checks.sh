#!/usr/bin/env bash
source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0
echo "== 1. static TF parent/child names"
timeout 6 ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable 2>/dev/null | grep -E "^\s+(frame_id|child_frame_id):" | sort -u
echo "== 2. base_footprint -> camera_link (expect x,y = tape measure, z about 0.14, yaw 0)"
timeout 6 ros2 run tf2_ros tf2_echo base_footprint camera_link 2>&1 | head -n 10
echo "== 3. LiDAR frame and its height (confirms base_footprint is on the floor)"
LIDAR_FRAME=$(timeout 5 ros2 topic echo --once --field header.frame_id /scan 2>/dev/null | head -n 1)
[ -z "$LIDAR_FRAME" ] && LIDAR_FRAME=$(timeout 5 ros2 topic echo --once --field header.frame_id /scan_fixed 2>/dev/null | head -n 1)
echo "lidar frame: ${LIDAR_FRAME:-NOT FOUND}"
[ -n "$LIDAR_FRAME" ] && timeout 6 ros2 run tf2_ros tf2_echo base_footprint "$LIDAR_FRAME" 2>&1 | head -n 6
echo "== 4. AMCL frames (odom frame must equal the one the robot publishes, odom_combined)"
for p in global_frame_id odom_frame_id base_frame_id; do echo -n "$p: "; timeout 5 ros2 param get /amcl $p 2>&1 | head -n 1; done
echo "== 5. map -> base_footprint and map -> camera_link (needs the AMCL pose to be set)"
timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -n 6
timeout 6 ros2 run tf2_ros tf2_echo map camera_link 2>&1 | head -n 6
echo "== 6. /cmd_vel must be silent while A* is idle (no lines below = good)"
timeout 5 ros2 topic echo /cmd_vel 2>/dev/null | head -n 5
echo "== 7. who listens to RViz goals (never click 2D Goal Pose during the mission)"
ros2 topic info -v /goal_pose 2>/dev/null | grep -E "Node name|Endpoint type"
echo "== 8. velocity_smoother and behavior_server connections"
ros2 node info /velocity_smoother 2>/dev/null | sed -n '1,22p'
ros2 node info /behavior_server 2>/dev/null | sed -n '1,22p'
echo "== 9. YOLO rate and A* heartbeat"
timeout 8 ros2 topic hz /yolo/detections 2>&1 | head -n 3
timeout 4 ros2 topic echo --once /astar/status 2>&1 | head -n 4
