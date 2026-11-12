

import json
import math
from collections import Counter, deque
from pathlib import Path
from time import monotonic, strftime

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from yolo_msgs.msg import DetectionArray

from .grid_planner import Grid
from .mission_logic import DEFAULTS, RUNNABLE, MissionLogic, State
from .monocular_geometry import area_fraction, pixel_to_bearing, triangulate_target


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RosLink:


    def __init__(self, node):
        self.n = node

    def now(self):
        return monotonic()

    def log(self, msg):
        self.n.get_logger().info(msg)
        self.n.publish_state(msg)

    def pose(self):
        return self.n.robot_pose()

    def astar_age(self):
        return monotonic() - self.n.astar_seen

    def _command(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.n.cmd_pub.publish(msg)

    def goto(self, x, y, cid):
        self._command({'type': 'goto', 'id': cid, 'x': float(x), 'y': float(y)})

    def spin(self, angle, cid):
        self._command({'type': 'spin', 'id': cid, 'angle': float(angle)})

    def cancel(self):
        self._command({'type': 'cancel', 'id': ''})

    def publish_sites(self, sites):
        msg = PoseArray()
        msg.header.frame_id = self.n.p('map_frame')
        msg.header.stamp = self.n.get_clock().now().to_msg()
        for x, y in sites:
            pose = Pose()
            pose.position.x, pose.position.y, pose.orientation.w = float(x), float(y), 1.0
            msg.poses.append(pose)
        self.n.sites_pub.publish(msg)

    def publish_target(self, x, y):
        point = PointStamped()
        point.header.frame_id = self.n.p('map_frame')
        point.header.stamp = self.n.get_clock().now().to_msg()
        point.point.x, point.point.y = float(x), float(y)
        self.n.target_pub.publish(point)

    def report(self, text):
        msg = String()
        msg.data = text
        self.n.report_pub.publish(msg)
        try:
            with (Path.home() / 'ee3033_target_report.txt').open('a') as f:
                f.write(f'{strftime("%Y-%m-%d %H:%M:%S")}  {text}\n')
        except OSError as exc:
            self.n.get_logger().warning(f'Could not write report file: {exc}')


class SearchNode(Node):
    def __init__(self):
        super().__init__('ee3033_search')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = lambda key: self.get_parameter(key).value
        self.logic = MissionLogic({k: self.p(k) for k in DEFAULTS}, RosLink(self))
        self.map_data = None
        self.visual_bearings = deque(maxlen=60)
        self.last_visual_sample = -1e9
        self.last_target_seen = -1e9
        self.last_apparent_area = 0.0
        self.scan_rx = 0.0
        self.astar_seen = -1e9
        self.last_det_stamp = None
        self.wall_rejects = 0
        self.last_warn = 0.0
        self.stats = Counter()
        self.other_classes = Counter()
        self.last_stats_print = 0.0
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.p('map_topic'), self.on_map, latched)
        self.scan_sub = None
        self.create_timer(1.0, self.resolve_scan_topic)
        det_type = DetectionArray
        if self.p('detections_message_type') == 'vision_msgs/msg/Detection2DArray':
            from vision_msgs.msg import Detection2DArray
            det_type = Detection2DArray
        self.create_subscription(det_type, self.p('detections_topic'), self.on_detections, 10)
        self.create_subscription(String, '/astar/status', self.on_astar_status, 10)
        self.cmd_pub = self.create_publisher(String, '/astar/command', 10)
        self.state_pub = self.create_publisher(String, '/ee3033/mission_state', 10)
        self.sites_pub = self.create_publisher(PoseArray, '/ee3033/search_sites', latched)
        self.target_pub = self.create_publisher(PointStamped, '/ee3033/target_map', latched)
        self.report_pub = self.create_publisher(String, '/ee3033/target_report', latched)
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/ee3033/coverage', latched)
        self.create_timer(1.0, self.publish_coverage)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/preview', self.srv_preview)
        self.create_service(Trigger, '~/status', self.srv_status)
        self.create_service(Trigger, '~/stop', self.srv_stop)
        self.create_timer(0.25, self.tick)
        self.get_logger().info('ee3033_search ready (DISARMED). Waiting for /ee3033_search/start')


    def publish_state(self, note=''):
        msg = String()
        msg.data = f'{self.logic.state.value}: {note}' if note else self.logic.state.value
        self.state_pub.publish(msg)

    def tick(self):
        self.logic.tick()
        self.publish_state()
        if self.logic.state in (State.SEARCHING, State.REPOSITIONING, State.REFINING) and monotonic() - self.last_stats_print > 5.0:
            self.last_stats_print = monotonic()
            if self.stats or self.other_classes:
                self.get_logger().info(f'detection stats (last 5 s): {dict(self.stats)}; other classes seen: {dict(self.other_classes)}')
                self.stats.clear(); self.other_classes.clear()

    def publish_coverage(self):

        L = self.logic
        if L.coverage is None or L.grid is None:
            return
        g = L.grid
        msg = OccupancyGrid()
        msg.header.frame_id = self.p('map_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(g.resolution)
        msg.info.width, msg.info.height = int(g.width), int(g.height)
        msg.info.origin.position.x, msg.info.origin.position.y = float(g.origin_x), float(g.origin_y)
        msg.info.origin.orientation.z = math.sin(g.origin_yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(g.origin_yaw / 2.0)
        msg.data = [60 if v else 0 for v in L.coverage.seen]
        self.coverage_pub.publish(msg)

    def resolve_scan_topic(self):
        if self.scan_sub is not None:
            return
        wanted = self.p('scan_topic')
        names = dict(self.get_topic_names_and_types())
        for candidate in (['/scan', '/scan_fixed'] if wanted == 'auto' else [wanted]):
            if candidate in names or wanted != 'auto':
                self.scan_sub = self.create_subscription(LaserScan, candidate, self.on_scan, qos_profile_sensor_data)
                self.get_logger().info(f'Using LiDAR topic {candidate}')
                return

    def warn(self, text, period=3.0):
        if monotonic() - self.last_warn > period:
            self.get_logger().warning(text)
            self.last_warn = monotonic()

    def on_map(self, msg):
        try:
            self.map_data = Grid(msg.info.width, msg.info.height, msg.info.resolution,
                                 msg.info.origin.position.x, msg.info.origin.position.y,
                                 tuple(msg.data), yaw_from_q(msg.info.origin.orientation))
        except ValueError as exc:
            self.map_data = None
            self.warn(f'Rejected malformed /map: {exc}')

    def on_scan(self, msg):
        self.scan_rx = monotonic()


    def on_astar_status(self, msg):
        self.astar_seen = monotonic()
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self.logic.on_status(str(data.get('state', '')), str(data.get('id', '')), str(data.get('detail', '')))

    def robot_pose(self):
        try:
            t = self.buffer.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_from_q(t.transform.rotation))


    def srv_start(self, request, response):
        L = self.logic
        pose = self.robot_pose()
        problems = []
        if L.state not in RUNNABLE:
            problems.append(f'already {L.state.value}')
        if self.map_data is None:
            problems.append('no /map')
        if pose is None:
            problems.append(f'no TF {self.p("map_frame")}->{self.p("base_frame")} (set the AMCL initial pose)')
        try:
            self.buffer.lookup_transform(self.p('map_frame'), self.p('camera_frame'), Time())
        except TransformException:
            problems.append(f'no TF {self.p("map_frame")}->{self.p("camera_frame")} (check camera transform and localization)')
        if monotonic() - self.scan_rx > float(self.p('max_scan_age_sec')):
            problems.append('no fresh LiDAR scan')
        if monotonic() - self.astar_seen > 3.0:
            problems.append('A* navigator not running on the Pi (/astar/status silent)')
        if not bool(self.p('calibration_verified')):
            problems.append('calibration_verified is false in search.yaml')
        if not str(self.p('target_class')).strip():
            problems.append('target_class is empty in search.yaml')
        if problems:
            response.success, response.message = False, 'Cannot start: ' + '; '.join(problems)
            return response
        self.wall_rejects = 0
        self.visual_bearings.clear()
        self.last_visual_sample = -1e9
        self.last_target_seen = -1e9
        self.last_det_stamp = None
        ok, message = L.begin(self.map_data, pose)
        response.success, response.message = ok, message
        return response

    def srv_preview(self, request, response):
        pose = self.robot_pose()
        if self.map_data is None or pose is None:
            response.success, response.message = False, 'Missing /map or map->base TF'
            return response
        sites = self.logic.plan_sites(self.map_data, pose[:2])
        self.logic.link.publish_sites(sites)
        response.success = bool(sites)
        response.message = f'Preview: {len(sites)} candidate sites published on /ee3033/search_sites; no movement commanded'
        return response

    def srv_status(self, request, response):
        response.success = True
        response.message = (f'{self.logic.status_text()}; map={self.map_data is not None}; '
                            f'pose={self.robot_pose()}; scan_age={monotonic() - self.scan_rx:.1f}s; '
                            f'astar_age={monotonic() - self.astar_seen:.1f}s; '
                            f'target_class={self.p("target_class")!r}; calibrated={self.p("calibration_verified")}; '
                            f'wall_hit_rejects={self.wall_rejects}; camera_views={len(self.visual_bearings)}; '
                            f'camera_baseline_m={self.camera_baseline():.2f}')
        return response

    def srv_stop(self, request, response):
        self.logic.stop()
        response.success, response.message = True, 'Cancel sent to A*; verify the robot is stationary (NOT an emergency stop)'
        return response


    def is_wall(self, xy):

        g = self.map_data
        if g is None:
            return False
        cx, cy = g.cell(*xy)
        if not g.in_bounds(cx, cy) or g.values[g.index(cx, cy)] < 0:
            return True
        r = max(1, math.ceil(float(self.p('wall_hit_radius_m')) / g.resolution))
        for yy in range(cy - r, cy + r + 1):
            for xx in range(cx - r, cx + r + 1):
                if g.in_bounds(xx, yy) and g.values[g.index(xx, yy)] > 50:
                    return True
        return False

    def map_tf(self, frame, stamp):
        try:
            return self.buffer.lookup_transform(self.p('map_frame'), frame, Time.from_msg(stamp))
        except TransformException as exc:
            self.warn(f'No time-matched map->{frame} TF: {exc}')
            return None

    @staticmethod
    def box_geometry(det):

        box = det.bbox
        centre = box.center
        cx = centre.position.x if hasattr(centre, 'position') else centre.x
        cy = centre.position.y if hasattr(centre, 'position') else centre.y
        w = box.size.x if hasattr(box, 'size') else box.size_x
        h = box.size.y if hasattr(box, 'size') else box.size_y
        return float(cx), float(cy), float(w), float(h)

    def camera_baseline(self):
        if not self.visual_bearings:
            return 0.0
        first = self.visual_bearings[0]
        return max(math.hypot(x - first[1], y - first[2]) for _, x, y, _ in self.visual_bearings)

    def on_detections(self, msg):
        logic = self.logic
        if logic.state not in (State.SEARCHING, State.REPOSITIONING, State.REFINING):
            return
        now = monotonic()
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_det_stamp:
            return
        self.last_det_stamp = stamp
        if stamp == (0, 0):
            self.stats['missing_detection_stamp'] += 1
            self.warn('Detection timestamp missing; camera pose cannot be associated reliably')
            return
        if not msg.detections:
            logic.on_lost()
            return
        self.stats['msgs_with_detections'] += 1
        target_class = str(self.p('target_class')).strip().lower()
        confidence = float(self.p('min_confidence'))
        options = []
        for det in msg.detections:
            if hasattr(det, 'class_name'):
                name, score = str(det.class_name).lower().strip(), float(det.score)
                if name == target_class and score >= confidence:
                    options.append((score, det))
                else:
                    self.other_classes[f'{name}@{score:.2f}'] += 1
            else:
                for result in det.results:
                    h = result.hypothesis
                    if str(h.class_id).lower().strip() == target_class and float(h.score) >= confidence:
                        options.append((float(h.score), det))
        if not options:
            self.stats['no_target_class_or_low_conf'] += 1
            logic.on_lost()
            return
        self.stats['target_class_ok'] += 1
        self.last_target_seen = now
        if now - self.last_visual_sample < float(self.p('visual_sample_interval_sec')):
            return
        self.last_visual_sample = now
        score, det = max(options, key=lambda o: o[0])
        cx, cy, box_width, box_height = self.box_geometry(det)
        width, height = int(self.p('image_width')), int(self.p('image_height'))
        if not (0 <= cx <= width and 0 <= cy <= height):
            self.stats['invalid_box'] += 1
            return
        try:
            bearing = pixel_to_bearing(cx, width, math.radians(float(self.p('camera_horizontal_fov_deg'))))
            self.last_apparent_area = area_fraction(box_width, box_height, width, height)
        except ValueError:
            self.stats['invalid_camera_geometry'] += 1
            return
        transform = self.map_tf(self.p('camera_frame'), msg.header.stamp)
        if transform is None:
            self.stats['missing_camera_tf'] += 1
            return
        camera_x = transform.transform.translation.x
        camera_y = transform.transform.translation.y
        world_heading = (yaw_from_q(transform.transform.rotation) +
                         math.radians(float(self.p('camera_yaw_offset_deg'))) + bearing)
        self.visual_bearings.append((now, camera_x, camera_y, world_heading))
        robot_transform = self.map_tf(self.p('base_frame'), msg.header.stamp)
        if robot_transform is not None:
            base_yaw = yaw_from_q(robot_transform.transform.rotation)
            axis_offset = math.atan2(math.sin(world_heading - bearing - base_yaw),
                                     math.cos(world_heading - bearing - base_yaw))
            logic.on_visual_detection((camera_x, camera_y), world_heading, axis_offset)
        cutoff = now - float(self.p('visual_history_sec'))
        while self.visual_bearings and self.visual_bearings[0][0] < cutoff:
            self.visual_bearings.popleft()
        estimate = triangulate_target(
            ((x, y, heading) for _, x, y, heading in self.visual_bearings),
            min_baseline_m=float(self.p('visual_min_baseline_m')),
            min_cross_angle_deg=float(self.p('visual_min_cross_angle_deg')),
            max_distance_m=float(self.p('visual_max_distance_m')),
            max_bearing_error_deg=float(self.p('visual_max_bearing_error_deg')),
            min_observations=int(self.p('visual_min_observations')),
        )
        if estimate is None:
            self.stats['insufficient_parallax_or_inconsistent_bearings'] += 1
            return
        if bool(self.p('reject_wall_hits')) and self.is_wall(estimate):
            self.wall_rejects += 1
            self.stats['rejected_on_wall'] += 1
            self.warn('Camera triangulation rejected: estimate on a mapped wall or unknown map region', 5.0)
            return
        self.stats['estimates_accepted'] += 1
        self.get_logger().info(
            f'camera triangulation: score={score:.2f} px={cx:.0f} '
            f'bearing={math.degrees(bearing):.1f}deg area={self.last_apparent_area:.4f} '
            f'views={len(self.visual_bearings)} baseline={self.camera_baseline():.2f}m '
            f'-> map ({estimate[0]:.2f}, {estimate[1]:.2f})',
            throttle_duration_sec=1.0)
        logic.on_estimate(estimate)


def main():
    rclpy.init()
    node = SearchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
