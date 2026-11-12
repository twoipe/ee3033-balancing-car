
import math
import unittest

from ee3033_search.grid_planner import Grid, search_sites, visible_cells
from ee3033_search.coverage import Coverage
from ee3033_search.mission_logic import MissionLogic, State


def box_grid(w_m=3.0, h_m=3.0, res=0.05, wall=0.05, ox=-1.5, oy=-1.5, blocks=()):
    w, h = int(w_m / res), int(h_m / res)
    data = [0] * (w * h)
    for y in range(h):
        for x in range(w):
            if x < wall / res or y < wall / res or x >= w - wall / res or y >= h - wall / res:
                data[y * w + x] = 100
    for (x0, y0, x1, y1) in blocks:
        for y in range(y0, y1):
            for x in range(x0, x1):
                data[y * w + x] = 100
    return Grid(w, h, res, ox, oy, tuple(data))


class FakeLink:

    def __init__(self, pose=(0.0, 0.0, 0.0)):
        self.t = 0.0
        self.pose_xyz = pose
        self.cmds = []
        self.logs = []
        self.targets = []
        self.reports = []
        self.age = 0.0

    def now(self): return self.t
    def goto(self, x, y, cid): self.cmds.append(('goto', x, y, cid))
    def spin(self, a, cid): self.cmds.append(('spin', a, cid))
    def cancel(self): self.cmds.append(('cancel',))
    def pose(self): return self.pose_xyz
    def astar_age(self): return self.age
    def log(self, m): self.logs.append(m)
    def publish_target(self, x, y): self.targets.append((x, y))
    def publish_sites(self, s): self.sites = s
    def report(self, t): self.reports.append(t)

    def last_motion_cmd(self):
        for c in reversed(self.cmds):
            if c[0] in ('goto', 'spin'):
                return c
        return None


def run_until(logic, link, predicate, outcome=lambda cmd: ('REACHED', ''), limit=400):

    for _ in range(limit):
        if predicate():
            return True
        cmd = link.last_motion_cmd()
        if logic.cmd is None or cmd is None:
            link.t += 4.0
            logic.tick()
            continue
        state, detail = outcome(cmd)
        if state == 'REACHED':
            if cmd[0] == 'goto':
                link.pose_xyz = (cmd[1], cmd[2], link.pose_xyz[2])
            else:
                link.pose_xyz = (link.pose_xyz[0], link.pose_xyz[1], link.pose_xyz[2] + cmd[1])
        link.t += 3.0
        logic.on_status('ACTIVE', cmd[-1])
        logic.on_status(state, cmd[-1], detail)
        logic.tick()
    return predicate()


class PlannerTests(unittest.TestCase):
    def test_sites_free_and_clear_of_walls(self):
        g = box_grid()
        sites = search_sites(g, (0, 0), 0.38, 0.8)
        self.assertGreater(len(sites), 3)
        for x, y in sites:
            cx, cy = g.cell(x, y)
            self.assertEqual(g.values[g.index(cx, cy)], 0)
            self.assertGreaterEqual(min(x - (-1.5), 1.5 - x, y - (-1.5), 1.5 - y), 0.38 - 0.06)

    def test_start_close_to_wall_still_gets_sites(self):
        g = box_grid()
        self.assertTrue(search_sites(g, (-1.40, -1.40), 0.38, 0.8))

    def test_max_sites_respected(self):
        g = box_grid(4, 4)
        self.assertLessEqual(len(search_sites(g, (0, 0), 0.2, 0.3, max_sites=10)), 10)

    def test_visibility_blocked_by_wall(self):
        g = box_grid(blocks=[(30, 0, 32, 60)])
        cells = visible_cells(g, -0.8, 0.0, 2.0)
        right = [i for i in cells if (i % g.width) > 34]
        self.assertEqual(right, [])

    def test_coverage_marks_cone_only(self):
        g = box_grid()
        cov = Coverage(g)
        cov.update(0.0, 0.0, 0.0, math.radians(48), 1.0)
        ahead = g.index(*g.cell(0.6, 0.0))
        behind = g.index(*g.cell(-0.6, 0.0))
        self.assertTrue(cov.seen[ahead])
        self.assertFalse(cov.seen[behind])


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.grid = box_grid()
        self.link = FakeLink((-1.0, -1.0, 0.0))
        self.logic = MissionLogic({'confirmations': 3, 'refine_sec': 2.0, 'max_home_retries': 3,
                                   'home_retry_delay_sec': 1.0, 'max_navigation_failures': 2}, self.link)

    def begin(self):
        ok, msg = self.logic.begin(self.grid, self.link.pose_xyz)
        self.assertTrue(ok, msg)

    def test_starts_with_goto_and_publishes_sites(self):
        self.begin()
        self.assertEqual(self.logic.state, State.SEARCHING)
        self.assertEqual(self.link.last_motion_cmd()[0], 'goto')
        self.assertTrue(self.link.sites)

    def test_search_uses_fewer_goals_than_fixed_lattice(self):

        self.begin()
        n_sites = len(self.logic.sites)
        run_until(self.logic, self.link, lambda: self.logic.state != State.SEARCHING)
        motion = [c for c in self.link.cmds if c[0] in ('goto', 'spin')]
        gotos = [c for c in motion if c[0] == 'goto']
        self.assertLess(len(gotos) - 1, n_sites * 4)
        self.assertIn(self.logic.state, (State.RETURNING, State.FINISHED_NO_TARGET))

    def test_full_mission_reports_target_and_returns_home(self):
        self.begin()
        run_until(self.logic, self.link, lambda: len(self.link.cmds) > 3)
        for _ in range(3):
            self.logic.on_estimate((0.62, 0.41))
        self.assertEqual(self.logic.state, State.REFINING)
        self.assertEqual(self.link.cmds[-1], ('cancel',))
        for p in [(0.63, 0.40), (0.61, 0.42), (0.90, 0.9)]:
            self.logic.on_estimate(p)
        self.link.t += 2.5
        self.logic.tick()
        self.assertEqual(self.logic.state, State.RETURNING)
        self.assertAlmostEqual(self.logic.target_xy[0], 0.62, delta=0.03)
        self.assertAlmostEqual(self.logic.target_xy[1], 0.41, delta=0.03)
        run_until(self.logic, self.link, lambda: self.logic.state == State.FINISHED)
        self.assertEqual(self.logic.state, State.FINISHED)
        self.assertTrue(any('TARGET REPORT' in r for r in self.link.reports))
        home_goto = [c for c in self.link.cmds if c[0] == 'goto'][-1]
        self.assertAlmostEqual(home_goto[1], -1.0)
        self.assertAlmostEqual(home_goto[2], -1.0)

    def test_inconsistent_estimates_do_not_confirm(self):
        self.begin()
        for p in [(0.0, 0.0), (1.0, 1.0), (-1.0, 0.5)]:
            self.logic.on_estimate(p)
        self.assertEqual(self.logic.state, State.SEARCHING)

    def test_home_goal_rejected_after_target_is_retried_not_stuck(self):

        self.begin()
        for _ in range(3):
            self.logic.on_estimate((0.5, 0.5))
        self.link.t += 2.5
        self.logic.tick()
        self.assertEqual(self.logic.state, State.RETURNING)
        first = self.link.last_motion_cmd()
        self.logic.on_status('FAILED', first[-1], 'rejected')
        self.link.t += 1.5
        self.logic.tick()
        second = self.link.last_motion_cmd()
        self.assertNotEqual(first[-1], second[-1])
        self.logic.on_status('REACHED', second[-1])
        self.assertEqual(self.logic.state, State.FINISHED)

    def test_failed_state_still_tries_to_go_home(self):

        self.begin()
        run_until(self.logic, self.link, lambda: self.logic.state == State.RETURNING,
                  outcome=lambda cmd: ('FAILED', 'blocked') if cmd[0] == 'goto' and
                  (abs(cmd[1] + 1.0) > 1e-6 or abs(cmd[2] + 1.0) > 1e-6) else ('REACHED', ''))
        self.assertEqual(self.logic.state, State.RETURNING)
        last = self.link.last_motion_cmd()
        self.assertEqual((round(last[1], 2), round(last[2], 2)), (-1.0, -1.0))
        self.logic.on_status('REACHED', last[-1])
        self.assertEqual(self.logic.state, State.FINISHED_NO_TARGET)

    def test_home_retries_exhausted_ends_failed_but_reports(self):
        self.begin()
        self.logic._abort_to_home('test')
        for _ in range(10):
            if self.logic.state == State.FAILED:
                break
            cmd = self.link.last_motion_cmd()
            if self.logic.cmd is not None:
                self.logic.on_status('FAILED', cmd[-1], 'blocked')
            self.link.t += 2.0
            self.logic.tick()
        self.assertEqual(self.logic.state, State.FAILED)
        self.assertTrue(self.link.reports)

    def test_silent_astar_is_treated_as_failure(self):
        self.begin()
        n = len(self.link.cmds)
        self.link.t += 6.0
        self.logic.tick()
        self.assertGreater(len(self.link.cmds), n)

    def test_stale_status_is_ignored(self):
        self.begin()
        n = len(self.link.cmds)
        self.logic.on_status('REACHED', 'm999')
        self.assertEqual(len(self.link.cmds), n)

    def test_astar_restart_idle_heartbeat_fails_acked_command(self):
        self.begin()
        cmd = self.link.last_motion_cmd()
        self.logic.on_status('ACTIVE', cmd[-1])
        self.link.t += 3.0
        n = len(self.link.cmds)
        self.logic.on_status('IDLE', '')
        self.assertGreater(len(self.link.cmds), n)

    def test_operator_stop(self):
        self.begin()
        self.logic.stop()
        self.assertEqual(self.logic.state, State.STOPPED)
        ok, _ = self.logic.begin(self.grid, self.link.pose_xyz)
        self.assertTrue(ok)


if __name__ == '__main__':
    unittest.main()
