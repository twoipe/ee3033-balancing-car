from math import atan2, cos, hypot, radians, sin

from .grid_planner import clearance_mask, reachable_cells


def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def viewpoint_options(grid, camera_xy, robot_xy, heading, clearance_m=0.13,
                      move_m=0.4, minimum_baseline_m=0.2, max_options=8):
    if not (0 < minimum_baseline_m < move_m and max_options > 0):
        return []
    safe = clearance_mask(grid, clearance_m)
    free = [0 <= v <= 20 for v in grid.values]
    sx, sy = grid.cell(*robot_xy)
    if not grid.in_bounds(sx, sy) or not free[grid.index(sx, sy)]:
        return []
    connected = reachable_cells(grid, free, sx, sy)
    if not connected:
        return []
    forward = (cos(heading), sin(heading))
    lateral = (-sin(heading), cos(heading))
    proposals = []
    for sideways in (move_m, -move_m, 1.3 * move_m, -1.3 * move_m):
        for forward_offset in (0.0, -0.15, 0.15):
            goal_x = robot_xy[0] + sideways * lateral[0] + forward_offset * forward[0]
            goal_y = robot_xy[1] + sideways * lateral[1] + forward_offset * forward[1]
            cx, cy = grid.cell(goal_x, goal_y)
            for ox in (-1, 0, 1):
                for oy in (-1, 0, 1):
                    xx, yy = cx + ox, cy + oy
                    if not grid.in_bounds(xx, yy):
                        continue
                    index = grid.index(xx, yy)
                    if not safe[index] or index not in connected:
                        continue
                    x, y = grid.world(xx, yy)
                    baseline = hypot(x - camera_xy[0], y - camera_xy[1])
                    travel = hypot(x - robot_xy[0], y - robot_xy[1])
                    if baseline < minimum_baseline_m + 0.05 or travel < minimum_baseline_m + 0.05:
                        continue
                    error = hypot(x - goal_x, y - goal_y)
                    score = abs(travel - move_m) + 2 * error + abs(forward_offset) * 0.5
                    proposals.append((score, x, y))
    proposals.sort(key=lambda p: p[0])
    selected = []
    for _, x, y in proposals:
        if all(hypot(x - ax, y - ay) >= max(grid.resolution * 2, 0.12) for ax, ay in selected):
            selected.append((x, y))
        if len(selected) >= max_options:
            break
    return selected


def aim_delta(robot_pose, anchor_camera_xy, anchor_heading, camera_to_robot_yaw=0.0, assumed_range_m=1.2):
    proxy_x = anchor_camera_xy[0] + assumed_range_m * cos(anchor_heading)
    proxy_y = anchor_camera_xy[1] + assumed_range_m * sin(anchor_heading)
    desired = atan2(proxy_y - robot_pose[1], proxy_x - robot_pose[0]) - camera_to_robot_yaw
    return wrap_angle(desired - robot_pose[2])
