
import importlib.util
import math
import os
import tempfile
import unittest
from pathlib import Path

from ee3033_search.coverage import Coverage
from ee3033_search.grid_planner import Grid

ROOT = Path(__file__).resolve().parents[2]


def load_launcher():
    spec = importlib.util.spec_from_file_location('ee3033_launcher', ROOT / 'ee3033.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class CoverageFraction(unittest.TestCase):
    def test_fraction_counts_only_free_cells(self):
        grid = Grid(4, 4, 0.05, 0.0, 0.0, tuple([0] * 8 + [100] * 4 + [-1] * 4))
        cov = Coverage(grid)
        self.assertEqual(cov.free_cells, 8)
        self.assertEqual(cov.fraction(), 0.0)
        for i in range(4):
            cov.seen[i] = 1
        self.assertAlmostEqual(cov.fraction(), 0.5)
        for i in range(16):
            cov.seen[i] = 1
        self.assertEqual(cov.fraction(), 1.0)


class TargetError(unittest.TestCase):
    def test_parse_report_takes_the_last_one(self):
        text = ('2026 Returned. TARGET REPORT: (x, y) = (1.000, -0.500) m in the map frame.\n'
                '2026 Returned. TARGET REPORT: (x, y) = (1.234, 0.050) m in the map frame. Mission time 90.0 s.\n')
        launcher = load_launcher()
        self.assertEqual(launcher.parse_report(text), (1.234, 0.05))
        self.assertIsNone(launcher.parse_report('No target was confirmed.'))

    def test_error_in_cm_and_log_file(self):
        launcher = load_launcher()
        with tempfile.TemporaryDirectory() as home:
            old = os.environ.get('HOME')
            os.environ['HOME'] = home
            try:
                self.assertEqual(launcher.cmd_error(1.0, 0.0, 1.03, 0.04), 0)
                log = (Path(home) / 'ee3033_calibration_log.csv').read_text().splitlines()
                self.assertEqual(len(log), 2)
                self.assertTrue(log[1].endswith('0.0500'))
            finally:
                if old is None:
                    os.environ.pop('HOME')
                else:
                    os.environ['HOME'] = old


if __name__ == '__main__':
    unittest.main()
