import math
import unittest

from ee3033_search.monocular_geometry import area_fraction, pixel_to_bearing, triangulate_target


def observations_for(target, positions, error=None):
    out = []
    for i, (x, y) in enumerate(positions):
        bearing = math.atan2(target[1] - y, target[0] - x)
        if error is not None:
            bearing += error[i % len(error)]
        out.append((x, y, bearing))
    return out


class CameraGeometryTests(unittest.TestCase):
    def test_pixels_define_horizontal_bearing(self):
        fov = math.radians(48.3)
        self.assertAlmostEqual(pixel_to_bearing(320, 640, fov), 0.0)
        self.assertGreater(pixel_to_bearing(181.978, 640, fov), 0.0)
        self.assertLess(pixel_to_bearing(500, 640, fov), 0.0)

    def test_invalid_fov_rejected(self):
        with self.assertRaises(ValueError):
            pixel_to_bearing(320, 640, 0)

    def test_area_ratio_does_not_require_real_object_dimensions(self):
        self.assertAlmostEqual(area_fraction(64, 48, 640, 480), 0.01)

    def test_position_recovered_without_object_size(self):
        expected = (1.2, 0.4)
        positions = [(0.0, 0.0), (0.0, 0.25), (0.0, 0.40), (0.32, -0.1), (0.25, 0.15)]
        got = triangulate_target(observations_for(expected, positions))
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got[0], expected[0], delta=0.001)
        self.assertAlmostEqual(got[1], expected[1], delta=0.001)

    def test_multiple_frames_from_stationary_camera_cannot_give_range(self):
        expected = (1.2, 0.4)
        positions = [(0.0, 0.0)] * 20
        self.assertIsNone(triangulate_target(observations_for(expected, positions)))

    def test_insufficient_translation_cannot_give_range(self):
        expected = (1.2, 0.4)
        positions = [(i * 0.01, 0.0) for i in range(8)]
        self.assertIsNone(triangulate_target(observations_for(expected, positions)))

    def test_noisy_bearings_can_still_estimate(self):
        expected = (1.2, 0.4)
        positions = [(0.0, 0.0), (0.0, 0.25), (0.0, 0.4), (0.2, -0.05), (0.35, 0.2), (0.1, -0.1)]
        errors = [0.002, -0.005, 0.004, 0.001, -0.002, 0.003]
        got = triangulate_target(observations_for(expected, positions, errors))
        self.assertIsNotNone(got)
        self.assertLess(math.hypot(got[0] - expected[0], got[1] - expected[1]), 0.05)

    def test_one_wrong_bearing_is_ignored(self):
        expected = (1.2, 0.4)
        positions = [(0.0, 0.0), (0.0, 0.25), (0.0, 0.40), (0.32, -0.1), (0.25, 0.15), (0.45, 0.2)]
        obs = observations_for(expected, positions)
        obs[-1] = (obs[-1][0], obs[-1][1], obs[-1][2] + 0.9)
        got = triangulate_target(obs)
        self.assertIsNotNone(got)
        self.assertLess(math.hypot(got[0] - expected[0], got[1] - expected[1]), 0.05)

    def test_wrong_direction_rejected(self):
        observations = [(0.0, 0.0, math.pi), (0.0, 0.3, math.pi),
                        (0.3, 0.0, math.pi), (0.3, 0.3, math.pi)]
        self.assertIsNone(triangulate_target(observations))


if __name__ == '__main__':
    unittest.main()
