"""Offline test for the post-save MAP CHECK (run: python3 -m unittest discover -s tests)."""
import importlib.util
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mapping', ROOT / 'mapping.py')
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


def write_map(folder, free_thresh, walls=True):
    w, h = 10, 8
    pix = bytearray([205] * (w * h))                 # unknown
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            pix[y * w + x] = 254                     # free interior
    if walls:
        for x in range(w):
            pix[x] = pix[(h - 1) * w + x] = 0       # occupied top and bottom rows
    (folder / 'm.pgm').write_bytes(b'P5\n# CREATOR: test\n10 8\n255\n' + bytes(pix))
    (folder / 'm.yaml').write_text(f'image: m.pgm\nmode: trinary\nresolution: 0.05\norigin: [-0.952, -1.76, 0]\n'
                                   f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: {free_thresh}\n')
    return folder / 'm.yaml'


class MapCheck(unittest.TestCase):
    def test_good_map_passes(self):
        with tempfile.TemporaryDirectory() as d:
            ok, lines = mapping.map_check(write_map(Path(d), 0.19))
            self.assertTrue(ok, lines)
            self.assertTrue(any('free_thresh 0.19 OK' in l for l in lines))
            self.assertTrue(any('10 x 8 cells' in l for l in lines))

    def test_default_nav2_threshold_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            ok, lines = mapping.map_check(write_map(Path(d), 0.25))
            self.assertFalse(ok)
            self.assertTrue(any('free_thresh=0.25' in l for l in lines))

    def test_map_without_walls_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            ok, lines = mapping.map_check(write_map(Path(d), 0.19, walls=False))
            self.assertFalse(ok)


if __name__ == '__main__':
    unittest.main()
