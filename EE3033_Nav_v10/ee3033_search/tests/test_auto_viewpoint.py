import math
import unittest

from test_mission_logic import FakeLink, box_grid
from ee3033_search.grid_planner import Grid
from ee3033_search.mission_logic import MissionLogic, State
from ee3033_search.viewpoint_planner import aim_delta, viewpoint_options


class ViewpointPlannerTests(unittest.TestCase):
    def test_selects_safe_lateral_sites_on_both_sides(self):
        grid = box_grid()
        options = viewpoint_options(grid, (0.0, 0.0), (0.0, 0.0), 0.0)
        self.assertGreaterEqual(len(options), 2)
        self.assertGreater(options[0][1] * options[1][1], -1.0)
        self.assertTrue(any(p[1] > 0.2 for p in options))
        self.assertTrue(any(p[1] < -0.2 for p in options))
        for x, y in options:
            cx, cy = grid.cell(x, y)
            self.assertEqual(grid.values[grid.index(cx, cy)], 0)
            self.assertGreater(math.hypot(x, y), 0.25)

    def test_blocked_lateral_sites_rejected(self):
        g = box_grid(w_m=1.1, h_m=1.1, ox=-0.55, oy=-0.55)
        options = viewpoint_options(g, (0.0, 0.0), (0.0, 0.0), 0.0,
                                    clearance_m=0.35, move_m=0.4)
        self.assertFalse(options)

    def test_camera_aim_uses_heading_and_camera_mounting_offset(self):
        target = (1.2, 0.0)
        delta = aim_delta((0.0, 0.4, 0.0), (0.0, 0.0), 0.0)
        self.assertAlmostEqual(delta, math.atan2(-0.4, 1.2))
        delta_offset = aim_delta((0.0, 0.4, 0.0), (0.0, 0.0), 0.0, 0.1)
        self.assertAlmostEqual(delta_offset, delta - 0.1)


class AutomaticRepositionTests(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink((0.0, 0.0, 0.0))
        self.logic = MissionLogic({'viewpoint_move_m': 0.4, 'robot_clearance_m': 0.13,
                                   'viewpoint_max_attempts': 2, 'viewpoint_observe_sec': 1.0,
                                   'viewpoint_max_sweeps': 2}, self.link)
        ok, message = self.logic.begin(box_grid(), self.link.pose_xyz)
        self.assertTrue(ok, message)

    def detect(self):
        self.logic.on_visual_detection((0.0, 0.0), 0.0, 0.0)
        self.link.t += 0.4
        self.logic.on_visual_detection((0.0, 0.0), 0.0, 0.0)

    def reach_command(self):
        cmd = self.link.last_motion_cmd()
        self.assertIsNotNone(cmd)
        if cmd[0] == 'goto':
            self.link.pose_xyz = (cmd[1], cmd[2], self.link.pose_xyz[2])
        else:
            self.link.pose_xyz = (self.link.pose_xyz[0], self.link.pose_xyz[1],
                                  self.link.pose_xyz[2] + cmd[1])
        self.link.t += 1.0
        self.logic.on_status('ACTIVE', cmd[-1])
        self.logic.on_status('REACHED', cmd[-1])
        return cmd

    def test_two_valid_detections_interrupt_search_and_choose_safe_view(self):
        self.logic.on_visual_detection((0.0, 0.0), 0.0, 0.0)
        self.assertEqual(self.logic.state, State.SEARCHING)
        self.detect()
        self.assertEqual(self.logic.state, State.REPOSITIONING)
        self.assertIn(('cancel',), self.link.cmds)
        cmd = self.link.last_motion_cmd()
        self.assertEqual(cmd[0], 'goto')
        self.assertGreater(abs(cmd[2]), 0.25)
        self.assertTrue(any('second camera viewpoint' in x for x in self.link.logs))

    def test_unrelated_bearings_do_not_trigger_reposition(self):
        self.logic.on_visual_detection((0.0, 0.0), 0.0, 0.0)
        self.link.t += 0.4
        self.logic.on_visual_detection((0.0, 0.0), math.radians(45.0), 0.0)
        self.assertEqual(self.logic.state, State.SEARCHING)

    def test_aim_observe_then_sweep_if_no_target_position(self):
        self.detect()
        self.reach_command()
        self.assertEqual(self.logic.state, State.REPOSITIONING)
        self.assertEqual(self.link.last_motion_cmd()[0], 'spin')
        self.reach_command()
        self.assertEqual(self.logic.viewpoint_phase, 'observing')
        self.link.t += 1.1
        self.logic.tick()
        self.assertEqual(self.logic.viewpoint_phase, 'scanning')
        self.assertEqual(self.link.last_motion_cmd()[0], 'spin')

    def test_valid_triangulation_while_moving_cancels_and_returns(self):
        self.detect()
        for _ in range(3):
            self.logic.on_estimate((1.0, 0.3))
        self.assertEqual(self.logic.state, State.REFINING)
        self.assertEqual(self.link.cmds[-1], ('cancel',))
        self.link.t += 2.1
        self.logic.tick()
        self.assertEqual(self.logic.state, State.RETURNING)
        self.assertAlmostEqual(self.link.last_motion_cmd()[1], 0.0)
        self.assertAlmostEqual(self.link.last_motion_cmd()[2], 0.0)

    def test_reposition_goal_failure_tries_other_side(self):
        self.detect()
        first = self.link.last_motion_cmd()
        self.logic.on_status('FAILED', first[-1], 'blocked')
        second = self.link.last_motion_cmd()
        self.assertEqual(self.logic.state, State.REPOSITIONING)
        self.assertNotEqual(first[-1], second[-1])
        self.assertGreater(abs(second[2] - first[2]), 0.3)

    def test_fallback_resumes_search_when_viewpoints_exhausted(self):
        self.detect()
        for _ in range(2):
            cmd = self.link.last_motion_cmd()
            self.logic.on_status('FAILED', cmd[-1], 'blocked')
        self.assertEqual(self.logic.state, State.SEARCHING)
        self.assertTrue(any('resuming coverage search' in x for x in self.link.logs))
        self.assertEqual(self.link.last_motion_cmd()[0], 'goto')

    def test_no_safe_viewpoint_does_not_cancel_exploration(self):
        self.logic.grid = box_grid(w_m=1.0, h_m=1.0, ox=-0.5, oy=-0.5)
        self.logic.c['robot_clearance_m'] = 0.35
        previous_cmd = self.logic.cmd
        self.detect()
        self.assertEqual(self.logic.state, State.SEARCHING)
        self.assertIs(self.logic.cmd, previous_cmd)
        self.assertTrue(any('no safe camera viewpoint' in x for x in self.link.logs))

    def test_operator_stop_cancels_reposition(self):
        self.detect()
        self.logic.stop()
        self.assertEqual(self.logic.state, State.STOPPED)
        self.assertEqual(self.link.cmds[-1], ('cancel',))


if __name__ == '__main__':
    unittest.main()
