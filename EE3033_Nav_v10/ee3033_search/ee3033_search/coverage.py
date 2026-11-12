
from math import degrees

from .grid_planner import ray_free_cells


class Coverage:
    def __init__(self, grid, free_threshold=20):
        self.grid = grid
        self.free_threshold = free_threshold
        self.seen = bytearray(grid.width * grid.height)
        self.free_cells = sum(1 for v in grid.values if 0 <= v <= free_threshold)

    def update(self, x, y, heading, fov_rad, max_range):

        n = max(2, int(degrees(fov_rad)) + 1)
        for k in range(n):
            angle = heading - fov_rad / 2 + fov_rad * k / (n - 1)
            for idx in ray_free_cells(self.grid, x, y, angle, max_range, self.free_threshold):
                self.seen[idx] = 1

    def unseen(self, cells):
        seen = self.seen
        return sum(1 for i in cells if not seen[i])

    def fraction(self):

        if not self.free_cells:
            return 0.0
        return min(1.0, sum(self.seen) / self.free_cells)
