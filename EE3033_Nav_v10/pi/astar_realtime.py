#!/usr/bin/env python3


import heapq
import json
import math
import signal
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate(x, y, angle):
    return (math.cos(angle) * x - math.sin(angle) * y,
            math.sin(angle) * x + math.cos(angle) * y)


def inflate(blocked, rad):

    if rad <= 0:
        return blocked.copy()
    n = int(math.floor(rad + 1e-9))
    h, w = blocked.shape
    if n == 0:
        return blocked.copy()
    padded = np.pad(blocked, n, constant_values=True)
    out = np.zeros_like(blocked)
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if dx * dx + dy * dy <= rad * rad + 1e-9:
                out |= padded[n + dy:n + dy + h, n + dx:n + dx + w]
    return out


def clear_disc(expanded, raw, center, r):

    h, w = expanded.shape
    cx, cy = center
    y0, y1, x0, x1 = max(0, cy - r), min(h, cy + r + 1), max(0, cx - r), min(w, cx + r + 1)
    if y0 >= y1 or x0 >= x1:
        return
    yy, xx = np.ogrid[y0:y1, x0:x1]
    mask = ((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r
    sub = expanded[y0:y1, x0:x1]
    sub[mask] = raw[y0:y1, x0:x1][mask]


def nearest_free(blocked, cell, max_r):
    h, w = blocked.shape
    cx, cy = cell
    if 0 <= cx < w and 0 <= cy < h and not blocked[cy, cx]:
        return (cx, cy)
    for r in range(1, max_r + 1):
        best, best_d = None, None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                x, y = cx + dx, cy + dy
                if 0 <= x < w and 0 <= y < h and not blocked[y, x]:
                    d = dx * dx + dy * dy
                    if best is None or d < best_d:
                        best, best_d = (x, y), d
        if best is not None:
            return best
    return None


def astar(blocked, start, goal, max_expansions=200000, extra=None):
    h, w = blocked.shape

    def inside(c):
        return 0 <= c[0] < w and 0 <= c[1] < h
    if not inside(start) or not inside(goal) or blocked[start[1], start[0]] or blocked[goal[1], goal[0]]:
        return None
    dist = {start: 0.0}
    parent = {}
    counter = 0
    q = [(math.hypot(start[0] - goal[0], start[1] - goal[1]), counter, start)]
    closed = set()
    s2 = math.sqrt(2)
    offsets = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, s2), (-1, 1, s2), (1, -1, s2), (-1, -1, s2)]
    while q and len(closed) < max_expansions:
        _, _, current = heapq.heappop(q)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        closed.add(current)
        x, y = current
        for dx, dy, cost in offsets:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                continue
            if dx and dy and (blocked[y, nx] or blocked[ny, x]):
                continue
            n = (nx, ny)
            tentative = dist[current] + cost + (float(extra[ny, nx]) if extra is not None else 0.0)
            if tentative < dist.get(n, math.inf):
                parent[n] = current
                dist[n] = tentative
                counter += 1
                heapq.heappush(q, (tentative + math.hypot(nx - goal[0], ny - goal[1]), counter, n))
    return None


PARAMS = {
    'scan_topic': 'auto', 'robot_frame': 'base_footprint', 'map_frame': 'map',
    'robot_radius': 0.07, 'safety_margin': 0.005,
    'max_linear_speed': 0.10, 'max_angular_speed': 0.35, 'spin_speed': 0.35,
    'stop_distance': 0.20, 'front_half_angle_deg': 30.0, 'goal_tolerance': 0.15,
    'obstacle_ttl': 2.0, 'max_map_cells': 600000, 'scan_timeout': 1.0,
    'goal_timeout_sec': 120.0, 'plan_fail_limit': 5, 'blocked_timeout_sec': 8.0,
    'progress_window_sec': 15.0, 'progress_min_m': 0.10,
    'endpoint_clear_m': 0.30, 'goal_snap_m': 0.30,

    'slow_distance': 0.45,
    'avoid_gain': 0.5,
    'reverse_speed': 0.05,
    'reverse_clear_m': 0.35,
    'guard_half_width_m': 0.10,
    'soft_margin_m': 0.20, 'soft_cost': 1.5,
    'new_obstacle_margin_m': 0.10,
    'target_obstacle_m': 0.06,
}


class AStarNavigator(Node):
    def __init__(self):
        super().__init__('ee3033_astar_realtime')
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        self.map_frame = self.p('map_frame')
        self.robot_frame = self.p('robot_frame')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.scan_sub = None
        self.create_timer(1.0, self.resolve_scan_topic)
        self.create_subscription(String, '/astar/command', self.on_command, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_rviz_goal, 10)
        self.create_subscription(PointStamped, '/ee3033/target_map', self.on_target, latched)
        self.target_xy = None
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)
        self.local_pub = self.create_publisher(Path, '/astar_local_path', 10)
        self.obs_pub = self.create_publisher(PoseArray, '/astar_obstacles', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/astar/status', 10)
        self.grid = self.info = None
        self.job = None
        self.path = []
        self.obstacles = deque()
        self.last_scan = 0.0
        self.front_clearance = 0.0
        self.front_y = 0.0
        self.rear_clearance = float('inf')
        self.last_warn = 0.0
        self.status = ('IDLE', '', '')
        self.rviz_seq = 0
        self.create_timer(1.0, self.plan)
        self.create_timer(0.1, self.drive)
        self.create_timer(0.5, self.heartbeat)
        self.get_logger().info('A* navigator ready (IDLE). Waiting for /astar/command')


    def stop(self):
        self.cmd_pub.publish(Twist())

    def warn(self, text):
        if time.monotonic() - self.last_warn > 3:
            self.get_logger().warn(text)
            self.last_warn = time.monotonic()

    def set_status(self, state, jid, detail=''):
        self.status = (state, jid, detail)
        self.heartbeat()

    def heartbeat(self):
        msg = String()
        msg.data = json.dumps({'state': self.status[0], 'id': self.status[1], 'detail': self.status[2]})
        self.status_pub.publish(msg)

    def publish_local(self, points):

        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = float(x), float(y), 1.0
            msg.poses.append(pose)
        self.local_pub.publish(msg)

    def publish_obstacles(self, points):

        step = max(1, len(points) // 200)
        msg = PoseArray()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points[::step]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.orientation.w = float(x), float(y), 1.0
            msg.poses.append(pose)
        self.obs_pub.publish(msg)

    def finish(self, state, detail=''):
        jid = self.job['id'] if self.job else ''
        self.job = None
        self.path = []
        self.publish_local([])
        self.publish_obstacles([])
        self.stop()
        self.set_status(state, jid, detail)
        self.get_logger().info(f'Job {jid}: {state} {detail}')

    def resolve_scan_topic(self):
        if self.scan_sub is not None:
            return
        wanted = self.p('scan_topic')
        names = dict(self.get_topic_names_and_types())
        for cand in (['/scan', '/scan_fixed'] if wanted == 'auto' else [wanted]):
            if cand in names or wanted != 'auto':
                self.scan_sub = self.create_subscription(LaserScan, cand, self.on_scan, qos_profile_sensor_data)
                self.get_logger().info(f'Using LiDAR topic {cand}')
                return

    def tf_pose(self, frame):
        t = self.buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)


    def on_map(self, msg):
        n = msg.info.width * msg.info.height
        if n <= 0 or n > self.p('max_map_cells') or msg.info.resolution <= 0 or len(msg.data) != n:
            self.grid = None
            self.path = []
            self.stop()
            self.warn('Map missing, malformed or too large')
            return
        self.info = msg.info
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    def on_scan(self, msg):
        if not msg.ranges:
            return
        self.last_scan = time.monotonic()
        try:
            sx, sy, syaw = self.tf_pose(msg.header.frame_id)
        except TransformException:
            self.warn('Missing map-to-LiDAR TF; dynamic obstacle updates unavailable')
            return
        lane = float(self.p('guard_half_width_m'))
        front = rear = float('inf')
        front_y = 0.0
        points = []
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d) or not (msg.range_min < d < msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            fx, fy = d * math.cos(a), d * math.sin(a)
            if abs(fy) < lane:
                if fx > 0 and fx < front:
                    front, front_y = fx, fy
                elif fx < 0 and -fx < rear:
                    rear = -fx
            if i % 2 == 0:
                points.append((sx + d * math.cos(a + syaw), sy + d * math.sin(a + syaw)))
        self.front_clearance, self.front_y, self.rear_clearance = front, front_y, rear
        self.obstacles.append((time.monotonic(), points))
        self.prune_obstacles()

    def prune_obstacles(self):
        cutoff = time.monotonic() - float(self.p('obstacle_ttl'))
        while self.obstacles and self.obstacles[0][0] < cutoff:
            self.obstacles.popleft()

    def on_target(self, msg):

        x, y = msg.point.x, msg.point.y
        self.target_xy = (x, y) if math.isfinite(x) and math.isfinite(y) else None

    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
            kind = cmd['type']
        except (ValueError, KeyError, TypeError):
            self.warn('Ignoring malformed /astar/command')
            return
        cid = str(cmd.get('id', ''))
        now = time.monotonic()
        if kind == 'cancel':
            if self.job:
                self.finish('CANCELLED', 'cancelled by mission')
            return
        if self.job:
            self.finish('CANCELLED', 'replaced by a new command')
        try:
            if kind == 'goto':
                self.job = {'id': cid, 'type': 'goto', 'goal': (float(cmd['x']), float(cmd['y'])), 't0': now,
                            'timeout': float(self.p('goal_timeout_sec')), 'best': math.inf, 'best_t': now,
                            'plan_fails': 0, 'blocked_t': None}
            elif kind == 'spin':
                angle = float(cmd['angle'])
                self.job = {'id': cid, 'type': 'spin', 'angle_abs': abs(angle), 'sign': 1.0 if angle >= 0 else -1.0,
                            'accum': 0.0, 'last_yaw': None, 't0': now,
                            'timeout': abs(angle) / max(0.05, float(self.p('spin_speed'))) * 3.0 + 8.0}
            else:
                self.warn(f'Unknown command type {kind!r}')
                return
        except (KeyError, ValueError, TypeError):
            self.warn('Command is missing numeric fields')
            return
        self.path = []
        self.set_status('ACTIVE', cid, kind)
        self.get_logger().info(f'New job {cid}: {msg.data}')
        if kind == 'goto':
            self.plan()

    def on_rviz_goal(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.warn(f'Goal rejected: frame must be {self.map_frame}, got {msg.header.frame_id!r}')
            return
        self.rviz_seq += 1
        fake = String()
        fake.data = json.dumps({'type': 'goto', 'id': f'rviz{self.rviz_seq}',
                                'x': msg.pose.position.x, 'y': msg.pose.position.y})
        self.on_command(fake)


    def world_to_cell(self, x, y):
        o = self.info.origin
        dx, dy = rotate(x - o.position.x, y - o.position.y, -yaw_of(o.orientation))
        return int(math.floor(dx / self.info.resolution)), int(math.floor(dy / self.info.resolution))

    def cell_to_world(self, x, y):
        o = self.info.origin
        dx, dy = rotate((x + 0.5) * self.info.resolution, (y + 0.5) * self.info.resolution, yaw_of(o.orientation))
        return o.position.x + dx, o.position.y + dy

    def plan_failed(self, reason):
        job = self.job
        job['plan_fails'] += 1
        self.path = []
        self.stop()
        self.warn(f'Planning failed ({reason}) {job["plan_fails"]}/{self.p("plan_fail_limit")}')
        if job['plan_fails'] >= int(self.p('plan_fail_limit')):
            self.finish('FAILED', f'no path: {reason}')

    def plan(self):
        job = self.job
        if job is None or job['type'] != 'goto':
            return
        if self.grid is None:
            self.warn('No /map yet')
            return
        if time.monotonic() - self.last_scan > float(self.p('scan_timeout')):
            self.path = []
            self.stop()
            self.warn('Scan timeout; holding position')
            return
        try:
            rx, ry, _ = self.tf_pose(self.robot_frame)
        except TransformException:
            self.path = []
            self.stop()
            self.warn(f'Missing {self.map_frame}->{self.robot_frame} TF; holding (set the AMCL initial pose)')
            return
        res = self.info.resolution
        start = self.world_to_cell(rx, ry)
        goal = self.world_to_cell(*job['goal'])
        raw = np.logical_or(self.grid >= 50, self.grid < 0)
        live = np.zeros_like(raw)
        self.prune_obstacles()
        near_wall = inflate(raw, max(1, math.ceil(float(self.p('new_obstacle_margin_m')) / res)))
        shown = []
        for _, points in self.obstacles:
            for px, py in points:
                x, y = self.world_to_cell(px, py)
                if 0 <= y < raw.shape[0] and 0 <= x < raw.shape[1] and not near_wall[y, x]:
                    live[y, x] = True
                    shown.append((px, py))
        if self.target_xy is not None:
            tx, ty = self.world_to_cell(*self.target_xy)
            tr = max(1, math.ceil(float(self.p('target_obstacle_m')) / res))
            h, w = live.shape
            for yy in range(max(0, ty - tr), min(h, ty + tr + 1)):
                for xx in range(max(0, tx - tr), min(w, tx + tr + 1)):
                    if (xx - tx) ** 2 + (yy - ty) ** 2 <= tr * tr:
                        live[yy, xx] = True
        self.publish_obstacles(shown)
        rad = (self.p('robot_radius') + self.p('safety_margin')) / res
        if rad > 30:
            self.plan_failed('inflation radius > 30 cells; check map resolution')
            return
        clear_r = math.ceil(float(self.p('endpoint_clear_m')) / res)
        expanded = inflate(raw, rad)
        clear_disc(expanded, raw, start, clear_r)
        clear_disc(expanded, raw, goal, clear_r)
        live_exp = inflate(live, rad)
        clear_disc(live_exp, live, start, clear_r)
        expanded = np.logical_or(expanded, live_exp)
        start_ok = nearest_free(expanded, start, 5)
        goal_ok = nearest_free(expanded, goal, math.ceil(float(self.p('goal_snap_m')) / res))
        if start_ok is None or goal_ok is None:
            self.plan_failed('start or goal blocked')
            return
        soft = inflate(np.logical_or(raw, live), float(self.p('soft_margin_m')) / res)
        extra = np.where(np.logical_and(soft, np.logical_not(expanded)), float(self.p('soft_cost')), 0.0)
        result = astar(expanded, start_ok, goal_ok, extra=extra)
        if not result:
            self.plan_failed('A* found no safe path')
            return
        job['plan_fails'] = 0
        self.path = [self.cell_to_world(x, y) for x, y in result]
        job['reach'] = self.path[-1]
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = x, y, 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)


    def drive(self):
        job = self.job
        if job is None:
            return
        now = time.monotonic()
        if now - job['t0'] > job['timeout']:
            self.finish('FAILED', 'timeout')
            return
        try:
            x, y, yaw = self.tf_pose(self.robot_frame)
        except TransformException:
            self.stop()
            self.warn('Missing robot TF; holding')
            return
        if job['type'] == 'spin':
            self.drive_spin(job, yaw)
            return
        reach = job.get('reach', job['goal'])
        dist = math.hypot(reach[0] - x, reach[1] - y)
        if dist < float(self.p('goal_tolerance')):
            self.finish('REACHED', f'dist={dist:.2f}')
            return
        if not self.path or now - self.last_scan > float(self.p('scan_timeout')):
            self.stop()
            return
        if dist < job['best'] - float(self.p('progress_min_m')):
            job['best'], job['best_t'] = dist, now
        elif now - job['best_t'] > float(self.p('progress_window_sec')):
            self.finish('FAILED', 'no progress')
            return
        nearest = min(range(len(self.path)), key=lambda i: (self.path[i][0] - x) ** 2 + (self.path[i][1] - y) ** 2)
        waypoint = self.path[-1]
        for cand in self.path[nearest:]:
            waypoint = cand
            if math.hypot(cand[0] - x, cand[1] - y) > 0.35:
                break
        error = wrap(math.atan2(waypoint[1] - y, waypoint[0] - x) - yaw)
        self.publish_local([(x, y), waypoint])
        cmd = Twist()
        wmax = float(self.p('max_angular_speed'))
        cmd.angular.z = max(-wmax, min(wmax, 1.2 * error))
        stop_d, slow_d = float(self.p('stop_distance')), max(float(self.p('slow_distance')), float(self.p('stop_distance')) + 0.05)
        front = self.front_clearance
        away = -1.0 if self.front_y > 0 else 1.0
        if front < stop_d:
            job['blocked_t'] = job['blocked_t'] or now
            if now - job['blocked_t'] > float(self.p('blocked_timeout_sec')):
                self.finish('FAILED', 'front blocked')
                return

            if self.rear_clearance > float(self.p('reverse_clear_m')):
                cmd.linear.x = -float(self.p('reverse_speed'))
            cmd.angular.z = max(-wmax, min(wmax, cmd.angular.z + away * float(self.p('avoid_gain'))))
        else:
            job['blocked_t'] = None
            if abs(error) < 0.45:
                cmd.linear.x = min(float(self.p('max_linear_speed')), 0.7 * math.hypot(waypoint[0] - x, waypoint[1] - y))
                cmd.linear.x *= max(0.0, 1.0 - abs(error) / 0.6)
            if front < slow_d:
                k = (front - stop_d) / (slow_d - stop_d)
                cmd.linear.x *= max(0.2, k)
                cmd.angular.z = max(-wmax, min(wmax, cmd.angular.z + away * float(self.p('avoid_gain')) * (1.0 - k)))
        self.cmd_pub.publish(cmd)

    def drive_spin(self, job, yaw):
        if job['last_yaw'] is None:
            job['last_yaw'] = yaw
        job['accum'] += wrap(yaw - job['last_yaw']) * job['sign']
        job['last_yaw'] = yaw
        remaining = job['angle_abs'] - job['accum']
        if remaining <= 0.05:
            self.finish('REACHED', 'spin done')
            return
        cmd = Twist()
        cmd.angular.z = job['sign'] * min(float(self.p('spin_speed')), max(0.12, 1.2 * remaining))
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        for _ in range(3):
            self.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = AStarNavigator()

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
