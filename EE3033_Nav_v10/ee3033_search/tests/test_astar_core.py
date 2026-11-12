
import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np


class _Auto(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        cls = type(name, (), {})
        setattr(self, name, cls)
        return cls


for mod in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
            'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf2_ros'):
    sys.modules.setdefault(mod, _Auto(mod))

SRC = Path(__file__).resolve().parents[2] / 'pi' / 'astar_realtime.py'
spec = importlib.util.spec_from_file_location('astar_realtime', SRC)
A = importlib.util.module_from_spec(spec)
spec.loader.exec_module(A)


def room(h=60, w=60, wall=1):
    g = np.zeros((h, w), dtype=bool)
    g[:wall], g[-wall:], g[:, :wall], g[:, -wall:] = True, True, True, True
    return g


class AStarCoreTests(unittest.TestCase):
    def test_straight_path(self):
        p = A.astar(room(), (10, 10), (50, 10))
        self.assertIsNotNone(p)
        self.assertEqual(p[0], (10, 10))
        self.assertEqual(p[-1], (50, 10))

    def test_no_path_through_wall(self):
        g = room()
        g[:, 30] = True
        self.assertIsNone(A.astar(g, (10, 10), (50, 10)))

    def test_no_diagonal_corner_cutting(self):
        g = room()
        g[20, 21] = g[21, 20] = True
        p = A.astar(g, (20, 20), (21, 21))
        self.assertTrue(p is None or len(p) > 2)

    def test_inflate_radius(self):
        g = room()
        e = A.inflate(g, 5)
        self.assertTrue(e[5, 30])
        self.assertFalse(e[8, 30])

    def test_start_next_to_wall_is_recoverable(self):

        raw = room()
        expanded = A.inflate(raw, 7)
        start = (30, 4)
        self.assertTrue(expanded[start[1], start[0]])
        self.assertIsNone(A.astar(expanded, start, (30, 30)))
        A.clear_disc(expanded, raw, start, 6)
        self.assertFalse(expanded[start[1], start[0]])
        self.assertIsNotNone(A.astar(expanded, start, (30, 30)))

    def test_clear_disc_keeps_real_obstacles(self):
        raw = room()
        raw[10, 10] = True
        expanded = A.inflate(raw, 4)
        A.clear_disc(expanded, raw, (10, 12), 5)
        self.assertTrue(expanded[10, 10])

    def test_nearest_free_snaps_goal(self):
        e = A.inflate(room(), 7)
        c = A.nearest_free(e, (30, 3), 10)
        self.assertIsNotNone(c)
        self.assertFalse(e[c[1], c[0]])
        self.assertIsNone(A.nearest_free(e, (30, 3), 1))


if __name__ == '__main__':
    unittest.main()


class LiveObstacleGoalTests(unittest.TestCase):

    def test_live_obstacle_next_to_goal_stays_inflated(self):
        raw = room(60, 60)
        live = np.zeros_like(raw)
        live[30, 40] = True
        static_exp = A.inflate(raw, 3)
        A.clear_disc(static_exp, raw, (40, 30), 6)
        live_exp = A.inflate(live, 3)
        blocked = np.logical_or(static_exp, live_exp)
        self.assertTrue(blocked[30, 42])
        self.assertFalse(blocked[30, 30])


class NarrowCorridorTests(unittest.TestCase):
    def test_22cm_gap_passable_with_10cm_inflation(self):
        res = 0.05
        g = np.zeros((40, 60), dtype=bool)
        g[0, :] = g[-1, :] = True
        g[:, 0] = g[:, -1] = True

        g[:, 30] = True
        g[18:22, 30] = False
        blocked = A.inflate(g, 1.5)
        p = A.astar(blocked, (10, 20), (50, 20))
        self.assertIsNotNone(p, 'robot 0.185 m wide must fit a 0.22 m gap')

    def test_3_free_cells_also_pass(self):
        g = np.zeros((40, 60), dtype=bool)
        g[0, :] = g[-1, :] = True
        g[:, 0] = g[:, -1] = True
        g[:, 30] = True
        g[18:21, 30] = False
        self.assertIsNotNone(A.astar(A.inflate(g, 1.5), (10, 19), (50, 19)))
