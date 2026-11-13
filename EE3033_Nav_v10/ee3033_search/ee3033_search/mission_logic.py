

from enum import Enum
from math import atan2, cos, hypot, pi, radians, sin
from statistics import median

from .coverage import Coverage
from .grid_planner import search_sites, visible_cells
from .viewpoint_planner import aim_delta, viewpoint_options


class State(str, Enum):
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    REPOSITIONING = 'REPOSITIONING'
    REFINING = 'REFINING'
    RETURNING = 'RETURNING'
    FINISHED = 'FINISHED'
    FINISHED_NO_TARGET = 'FINISHED_NO_TARGET'
    FAILED = 'FAILED'
    STOPPED = 'STOPPED'


RUNNABLE = (State.IDLE, State.STOPPED, State.FINISHED, State.FINISHED_NO_TARGET, State.FAILED)

DEFAULTS = {

    'target_class': '', 'calibration_verified': False,
    'min_confidence': 0.45, 'lost_clear_sec': 1.5, 'confirmations': 3, 'refine_sec': 2.0,
    'image_width': 640, 'image_height': 480, 'camera_horizontal_fov_deg': 57.9,
    'camera_yaw_offset_deg': -2.05, 'camera_frame': 'camera_link',
    'max_scan_age_sec': 2.0,
    'visual_min_baseline_m': 0.20, 'visual_min_cross_angle_deg': 6.0,
    'visual_max_bearing_error_deg': 3.0, 'visual_history_sec': 35.0,
    'visual_min_observations': 4, 'visual_max_distance_m': 2.5,
    'visual_sample_interval_sec': 0.35,
    'viewpoint_move_m': 0.40, 'viewpoint_max_attempts': 2,
    'viewpoint_observe_sec': 2.5, 'viewpoint_sweep_deg': 16.0,
    'viewpoint_max_sweeps': 4, 'viewpoint_range_hint_m': 1.2,
    'viewpoint_cooldown_sec': 5.0, 'viewpoint_trigger_frames': 2,
    'viewpoint_trigger_max_bearing_deg': 12.0,
    'target_max_spread_m': 0.35, 'reject_wall_hits': True, 'wall_hit_radius_m': 0.10,

    'robot_clearance_m': 0.13, 'search_spacing_m': 0.30, 'max_sites': 40,
    'coverage_range_m': 2.0, 'coverage_fov_fraction': 0.8, 'min_gain_fraction': 0.12,
    'spin_step_deg': 90.0, 'max_navigation_failures': 6,

    'max_home_retries': 6, 'home_retry_delay_sec': 3.0, 'cmd_ack_timeout_sec': 5.0,

    'map_frame': 'map', 'base_frame': 'base_footprint', 'map_topic': '/map',
    'scan_topic': 'auto', 'detections_topic': '/yolo/detections',
    'detections_message_type': 'yolo_msgs/msg/DetectionArray',
}


class Site:
    def __init__(self, xy, cells):
        self.xy = xy
        self.cells = cells
        self.done = False


def robust_center(points, spread):

    mx, my = median(p[0] for p in points), median(p[1] for p in points)
    inliers = [p for p in points if hypot(p[0] - mx, p[1] - my) <= spread]
    if not inliers:
        return (mx, my)
    return (median(p[0] for p in inliers), median(p[1] for p in inliers))


class MissionLogic:
    def __init__(self, cfg, link):
        self.c = {**DEFAULTS, **cfg}
        self.link = link
        self.state = State.IDLE
        self._seq = 0
        self.reset()


    def reset(self):
        self.cmd = None
        self.sites = []
        self.current_site = None
        self.coverage = None
        self.grid = None
        self.home = None
        self.positions = []
        self._last_est_t = 0.0
        self.estimates = []
        self.refine_until = None
        self.target_xy = None
        self.nav_failures = 0
        self.home_attempts = 0
        self.home_retry_at = None
        self.spun = 0.0
        self.t_start = None
        self.reason = ''
        self.viewpoint_anchor = None
        self.viewpoint_options = []
        self.viewpoint_attempt = 0
        self.viewpoint_sweep = 0
        self.viewpoint_offset_deg = 0.0
        self.viewpoint_phase = ''
        self.viewpoint_hold_until = None
        self.viewpoint_next_allowed = 0.0
        self.visual_candidates = []

    def plan_sites(self, grid, start_xy):
        c = self.c
        return search_sites(grid, start_xy, c['robot_clearance_m'], c['search_spacing_m'], c['max_sites'])

    def begin(self, grid, pose):
        self.reset()
        c = self.c
        xy_list = self.plan_sites(grid, pose[:2])
        if not xy_list:
            return False, 'No reachable search sites; inspect map, clearance and localization'
        self.grid = grid
        self.home = pose
        self.coverage = Coverage(grid)
        self.sites = [Site(xy, visible_cells(grid, xy[0], xy[1], c['coverage_range_m'])) for xy in xy_list]
        self.t_start = self.link.now()
        self.state = State.SEARCHING
        self.link.log(f'Search started: {len(self.sites)} candidate sites; home=({pose[0]:.2f}, {pose[1]:.2f})')
        self.link.publish_sites([s.xy for s in self.sites])
        self.link.publish_target(float('nan'), float('nan'))
        self._next_task()
        return True, f'Started; {len(self.sites)} candidate sites'

    def stop(self):
        self.link.cancel()
        self.cmd = None
        self.state = State.STOPPED
        self.link.log('Mission stopped by operator (cancel sent; NOT an emergency stop)')


    def _new_id(self):
        self._seq += 1
        return f'm{self._seq}'

    def _send(self, kind, purpose, *args):
        cid = self._new_id()
        self.cmd = {'id': cid, 'kind': kind, 'purpose': purpose, 't': self.link.now(), 'acked': False}
        if kind == 'goto':
            self.link.goto(args[0], args[1], cid)
        else:
            self.link.spin(args[0], cid)

    def _send_home(self):
        self.home_retry_at = None
        self._send('goto', 'home', self.home[0], self.home[1])


    def _update_coverage(self):
        pose = self.link.pose()
        if pose is None or self.coverage is None:
            return
        c = self.c
        fov = radians(c['camera_horizontal_fov_deg']) * c['coverage_fov_fraction']
        heading = pose[2] + radians(c['camera_yaw_offset_deg'])
        self.coverage.update(pose[0], pose[1], heading, fov, c['coverage_range_m'])

    def _gain_fraction(self, site):
        return self.coverage.unseen(site.cells) / max(1, len(site.cells))

    def _select_site(self):
        pose = self.link.pose()
        px, py = (pose[0], pose[1]) if pose else (self.home[0], self.home[1])
        best, best_score = None, -1.0
        for s in self.sites:
            if s.done:
                continue
            frac = self._gain_fraction(s)
            if not s.cells or frac < self.c['min_gain_fraction']:
                s.done = True
                continue
            gain = frac * len(s.cells)
            score = gain / (hypot(s.xy[0] - px, s.xy[1] - py) + 1.0)
            if score > best_score:
                best, best_score = s, score
        return best

    def _next_task(self):
        if self.state != State.SEARCHING:
            return
        self._update_coverage()
        site = self._select_site()
        if site is None:
            self._abort_to_home('search complete: no uncovered area left')
            return
        self.current_site = site
        self.spun = 0.0
        self._send('goto', 'site', site.xy[0], site.xy[1])

    def _spin_step(self):
        step = radians(self.c['spin_step_deg'])
        self._send('spin', 'sweep', step)

    def _search_event(self, cmd, state, detail):
        site = self.current_site
        if cmd['kind'] == 'goto':
            if state == 'REACHED':
                self.nav_failures = 0
                self._update_coverage()
                if self._gain_fraction(site) < self.c['min_gain_fraction']:
                    site.done = True
                    self._next_task()
                else:
                    self._spin_step()
            else:
                site.done = True
                self.nav_failures += 1
                self.link.log(f'Navigation to a search site failed ({state}: {detail}); '
                              f'failure {self.nav_failures}/{self.c["max_navigation_failures"]}')
                if self.nav_failures > int(self.c['max_navigation_failures']):
                    self._abort_to_home('too many navigation failures')
                else:
                    self._next_task()
        else:
            if state == 'REACHED':
                self.spun += radians(self.c['spin_step_deg'])
                self._update_coverage()
                if self.spun >= 2 * pi - 0.05 or self._gain_fraction(site) < self.c['min_gain_fraction']:
                    site.done = True
                    self._next_task()
                else:
                    self._spin_step()
            else:
                site.done = True
                self._next_task()



    def on_visual_detection(self, camera_xy, world_heading, camera_axis_robot_yaw):
        now = self.link.now()
        if self.state == State.REPOSITIONING:
            return
        if self.state != State.SEARCHING or now < self.viewpoint_next_allowed:
            return
        if self.viewpoint_anchor is not None:
            return
        sample = (now, tuple(camera_xy), world_heading, camera_axis_robot_yaw)
        self.visual_candidates.append(sample)
        self.visual_candidates = [s for s in self.visual_candidates if now - s[0] <= float(self.c['lost_clear_sec'])][-5:]
        if len(self.visual_candidates) < int(self.c['viewpoint_trigger_frames']):
            return
        first = self.visual_candidates[0]
        if any(hypot(s[1][0] - first[1][0], s[1][1] - first[1][1]) > 0.18 or
               abs(atan2(sin(s[2] - first[2]), cos(s[2] - first[2]))) >
               radians(float(self.c['viewpoint_trigger_max_bearing_deg'])) for s in self.visual_candidates):
            self.visual_candidates = [sample]
            return
        self.visual_candidates = []
        self._begin_reposition(sample)

    def _begin_reposition(self, sample):
        pose = self.link.pose()
        if pose is None or self.grid is None:
            return
        _, camera_xy, heading, camera_axis_robot_yaw = sample
        candidates = viewpoint_options(
            self.grid, camera_xy, pose[:2], heading,
            clearance_m=float(self.c['robot_clearance_m']),
            move_m=float(self.c['viewpoint_move_m']),
            minimum_baseline_m=float(self.c['visual_min_baseline_m']),
        )
        if not candidates:
            self.viewpoint_next_allowed = self.link.now() + float(self.c['viewpoint_cooldown_sec'])
            self.link.log('Target seen, but no safe camera viewpoint in the known free map; continuing search')
            return
        self.viewpoint_anchor = (camera_xy, heading, camera_axis_robot_yaw)
        self.viewpoint_options = candidates
        self.viewpoint_attempt = 0
        self.state = State.REPOSITIONING
        self.link.cancel()
        self.cmd = None
        self.link.log('Target recognized: suspending coverage search to obtain a second camera viewpoint')
        self._next_viewpoint()

    def _next_viewpoint(self):
        pose = self.link.pose()
        maximum = min(len(self.viewpoint_options), int(self.c['viewpoint_max_attempts']))
        while self.viewpoint_attempt < maximum:
            target = self.viewpoint_options[self.viewpoint_attempt]
            self.viewpoint_attempt += 1
            if pose is None or hypot(pose[0] - target[0], pose[1] - target[1]) < float(self.c['visual_min_baseline_m']):
                continue
            self.viewpoint_phase = 'moving'
            self.viewpoint_hold_until = None
            self.viewpoint_sweep = 0
            self.viewpoint_offset_deg = 0.0
            self.link.log(f'Camera viewpoint {self.viewpoint_attempt}/{maximum}: ({target[0]:.2f}, {target[1]:.2f}) m')
            self._send('goto', 'viewpoint_move', *target)
            return
        self._resume_search('No usable second camera viewpoint; resuming coverage search')

    def _resume_search(self, reason):
        self.link.cancel()
        self.cmd = None
        self.viewpoint_anchor = None
        self.viewpoint_options = []
        self.viewpoint_phase = ''
        self.viewpoint_hold_until = None
        self.visual_candidates = []
        self.positions.clear()
        self.viewpoint_next_allowed = self.link.now() + float(self.c['viewpoint_cooldown_sec'])
        self.state = State.SEARCHING
        self.link.log(reason)
        self._next_task()

    def _viewpoint_observe(self):
        self.viewpoint_phase = 'observing'
        self.viewpoint_hold_until = self.link.now() + float(self.c['viewpoint_observe_sec'])
        self.link.log('At second viewpoint: waiting for camera detections and triangulation')

    def _viewpoint_event(self, cmd, state, detail):
        if state != 'REACHED':
            self.link.log(f'Camera viewpoint command failed ({state}: {detail}); trying another safe site')
            self._next_viewpoint()
            return
        if cmd['purpose'] == 'viewpoint_move':
            pose = self.link.pose()
            if pose is None:
                self._next_viewpoint()
                return
            camera_xy, heading, axis_offset = self.viewpoint_anchor
            delta = aim_delta(pose, camera_xy, heading, axis_offset,
                              float(self.c['viewpoint_range_hint_m']))
            if abs(delta) < radians(3.0):
                self._viewpoint_observe()
            else:
                self.viewpoint_phase = 'aiming'
                self._send('spin', 'viewpoint_aim', delta)
        elif cmd['purpose'] in ('viewpoint_aim', 'viewpoint_sweep'):
            self._viewpoint_observe()

    def _viewpoint_tick(self, now):
        if self.cmd is not None or self.viewpoint_phase != 'observing':
            return
        if self.viewpoint_hold_until is None or now < self.viewpoint_hold_until:
            return
        n = int(self.c['viewpoint_max_sweeps'])
        if self.viewpoint_sweep >= n:
            self.link.log('Camera target not triangulated at this viewpoint')
            self._next_viewpoint()
            return
        step = float(self.c['viewpoint_sweep_deg'])
        direction = 1.0 if self.viewpoint_sweep % 2 == 0 else -1.0
        amplitude = (self.viewpoint_sweep // 2 + 1) * step
        desired_offset = direction * amplitude
        delta = radians(desired_offset - self.viewpoint_offset_deg)
        self.viewpoint_offset_deg = desired_offset
        self.viewpoint_sweep += 1
        self.viewpoint_phase = 'scanning'
        self._send('spin', 'viewpoint_sweep', delta)

    def _abort_to_home(self, reason):

        self.reason = reason
        self.state = State.RETURNING
        self.home_attempts = 0
        self.link.cancel()
        self.link.log(f'Returning home WITHOUT a target report: {reason}' if self.target_xy is None
                      else f'Returning home: {reason}')
        self._send_home()

    def _home_event(self, state, detail):
        if state == 'REACHED':
            self._finish()
            return
        self.home_attempts += 1
        self.link.log(f'Return-home attempt {self.home_attempts} failed ({state}: {detail})')
        if self.home_attempts > int(self.c['max_home_retries']):
            self.state = State.FAILED
            self._emit_final('FAILED: could not return to the start area')
        else:
            self.home_retry_at = self.link.now() + float(self.c['home_retry_delay_sec'])

    def _finish(self):
        self.state = State.FINISHED if self.target_xy else State.FINISHED_NO_TARGET
        self._emit_final('Returned to the start area.')

    def _emit_final(self, headline):
        elapsed = self.link.now() - self.t_start if self.t_start is not None else 0.0
        if self.target_xy:
            text = (f'{headline} TARGET REPORT: (x, y) = ({self.target_xy[0]:.3f}, {self.target_xy[1]:.3f}) m '
                    f'in the map frame. Mission time {elapsed:.1f} s.')
        else:
            text = f'{headline} No target was confirmed. Mission time {elapsed:.1f} s.'
        self.link.log(text)
        self.link.report(text)


    def on_lost(self):

        if self.state in (State.SEARCHING, State.REPOSITIONING) and self.positions and \
                self.link.now() - self._last_est_t > float(self.c['lost_clear_sec']):
            self.positions.clear()

    def on_estimate(self, xy):

        n = int(self.c['confirmations'])
        if self.state in (State.SEARCHING, State.REPOSITIONING):
            if self.positions and self.link.now() - self._last_est_t > float(self.c['lost_clear_sec']):
                self.positions.clear()
            self._last_est_t = self.link.now()
            self.positions.append(xy)
            self.positions = self.positions[-n:]
            if len(self.positions) < n:
                return
            mx, my = median(p[0] for p in self.positions), median(p[1] for p in self.positions)
            if all(hypot(p[0] - mx, p[1] - my) <= self.c['target_max_spread_m'] for p in self.positions):
                self._confirm((mx, my))
            else:
                self.positions.clear()
        elif self.state == State.REFINING:
            self.estimates.append(xy)
            self.estimates = self.estimates[-60:]

    def _confirm(self, provisional):
        self.state = State.REFINING
        self.estimates = list(self.positions) + [provisional]
        self.viewpoint_anchor = None
        self.viewpoint_options = []
        self.viewpoint_phase = ''
        self.positions.clear()
        self.cmd = None
        self.link.cancel()
        self.refine_until = self.link.now() + float(self.c['refine_sec'])
        self.link.log(f'TARGET CONFIRMED (provisional): x={provisional[0]:.3f}, y={provisional[1]:.3f}; '
                      f'standing still {self.c["refine_sec"]} s to refine')

    def _finish_refine(self):
        self.target_xy = robust_center(self.estimates, float(self.c['target_max_spread_m']))
        self.link.publish_target(*self.target_xy)
        self.link.log(f'TARGET REPORT: (x, y) = ({self.target_xy[0]:.3f}, {self.target_xy[1]:.3f}) m '
                      f'(map frame, {len(self.estimates)} samples)')
        self.state = State.RETURNING
        self.home_attempts = 0
        self.reason = 'target reported'
        self._send_home()


    def on_status(self, state, cid, detail=''):
        cmd = self.cmd
        if state == 'IDLE' and cmd and cmd['acked'] and self.link.now() - cmd['t'] > 2.0:

            self.cmd = None
            self._dispatch(cmd, 'FAILED', 'A* restarted / lost the job')
            return
        if cmd is None or cid != cmd['id']:
            return
        cmd['acked'] = True
        if state == 'ACTIVE':
            return
        self.cmd = None
        self._dispatch(cmd, state, detail)

    def _dispatch(self, cmd, state, detail):
        if self.state == State.SEARCHING:
            self._search_event(cmd, state, detail)
        elif self.state == State.REPOSITIONING:
            self._viewpoint_event(cmd, state, detail)
        elif self.state == State.RETURNING and cmd['purpose'] == 'home':
            self._home_event(state, detail)


    def tick(self):
        now = self.link.now()
        if self.state == State.SEARCHING:
            self._update_coverage()
        elif self.state == State.REPOSITIONING:
            self._viewpoint_tick(now)
        elif self.state == State.REFINING:
            if now >= self.refine_until:
                self._finish_refine()
        elif self.state == State.RETURNING:
            if self.cmd is None and self.home_retry_at is not None and now >= self.home_retry_at:
                self._send_home()
        cmd = self.cmd
        if cmd is not None and self.state in (State.SEARCHING, State.REPOSITIONING, State.RETURNING):
            timeout = float(self.c['cmd_ack_timeout_sec'])
            silent = (not cmd['acked'] and now - cmd['t'] > timeout) or self.link.astar_age() > timeout
            if silent:
                self.cmd = None
                self.link.log('No response from the A* navigator; treating command as failed')
                self._dispatch(cmd, 'FAILED', 'no response from A*')

    def status_text(self):
        t = (self.link.now() - self.t_start) if self.t_start is not None else 0.0
        done = sum(1 for s in self.sites if s.done)
        cov = f'{100 * self.coverage.fraction():.0f}%' if self.coverage else 'n/a'
        return (f'state={self.state.value}; sites_done={done}/{len(self.sites)}; coverage={cov}; '
                f'home={tuple(round(v, 2) for v in self.home[:2]) if self.home else None}; '
                f'target={tuple(round(v, 3) for v in self.target_xy) if self.target_xy else None}; '
                f'elapsed={t:.0f}s; viewpoint_attempt={self.viewpoint_attempt if self.state == State.REPOSITIONING else 0}; '
                f'viewpoint_phase={self.viewpoint_phase!r}; reason={self.reason!r}')
