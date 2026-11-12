
from collections import deque
from dataclasses import dataclass
from math import ceil, cos, sin, pi


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    values: tuple
    origin_yaw: float = 0.0

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError('Invalid grid dimensions or resolution')
        if len(self.values) != self.width * self.height:
            raise ValueError('Grid cell count does not match dimensions')
        object.__setattr__(self, '_c', cos(self.origin_yaw))
        object.__setattr__(self, '_s', sin(self.origin_yaw))

    def index(self, x, y):
        return y * self.width + x

    def world(self, x, y):
        dx, dy = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        return (self.origin_x + self._c * dx - self._s * dy,
                self.origin_y + self._s * dx + self._c * dy)

    def cell(self, world_x, world_y):
        dx, dy = world_x - self.origin_x, world_y - self.origin_y
        mx = self._c * dx + self._s * dy
        my = -self._s * dx + self._c * dy
        return int(mx // self.resolution), int(my // self.resolution)

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height


def clearance_mask(grid, clearance_m=0.38, free_threshold=20):


    n = grid.width * grid.height
    radius = max(0, ceil(clearance_m / grid.resolution))
    dist = [-1] * n
    q = deque()
    for i, v in enumerate(grid.values):
        if v < 0 or v > free_threshold:
            dist[i] = 0
            q.append(i)
    if radius:
        while q:
            idx = q.popleft()
            if dist[idx] >= radius:
                continue
            x, y = idx % grid.width, idx // grid.width
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    xx, yy = x + dx, y + dy
                    if (dx or dy) and grid.in_bounds(xx, yy):
                        ni = grid.index(xx, yy)
                        if dist[ni] == -1:
                            dist[ni] = dist[idx] + 1
                            q.append(ni)
    mask = [0 <= v <= free_threshold and dist[i] == -1
            for i, v in enumerate(grid.values)]
    if radius > 0:
        for y in range(grid.height):
            for x in range(grid.width):
                if min(x, y, grid.width - 1 - x, grid.height - 1 - y) < radius:
                    mask[grid.index(x, y)] = False
    return mask


def nearest_valid(grid, valid, cx, cy, max_r):

    best, best_d = None, None
    for r in range(0, max_r + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                x, y = cx + dx, cy + dy
                if grid.in_bounds(x, y) and valid[grid.index(x, y)]:
                    d = dx * dx + dy * dy
                    if best is None or d < best_d:
                        best, best_d = (x, y), d
        if best is not None:
            return best
    return None


def reachable_cells(grid, valid, start_x, start_y):
    if not grid.in_bounds(start_x, start_y):
        return set()
    start = grid.index(start_x, start_y)
    if not valid[start]:
        return set()
    q, seen = deque([start]), {start}
    while q:
        i = q.popleft()
        x, y = i % grid.width, i // grid.width
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            xx, yy = x + dx, y + dy
            if grid.in_bounds(xx, yy):
                j = grid.index(xx, yy)
                if valid[j] and j not in seen:
                    seen.add(j)
                    q.append(j)
    return seen


def search_sites(grid, start_xy, clearance_m=0.38, spacing_m=0.8, max_sites=40):


    if spacing_m <= 0 or max_sites < 1:
        raise ValueError('Invalid search parameters')
    valid = clearance_mask(grid, clearance_m)
    sx, sy = grid.cell(*start_xy)
    seed = nearest_valid(grid, valid, sx, sy, max(1, ceil(1.0 / grid.resolution)))
    if seed is None:
        return []
    component = reachable_cells(grid, valid, *seed)
    if not component:
        return []
    stride = max(1, round(spacing_m / grid.resolution))
    sites = []
    for y in range(stride // 2, grid.height, stride):
        for x in range(stride // 2, grid.width, stride):
            if grid.index(x, y) in component:
                sites.append(grid.world(x, y))
    if not sites:
        i = min(component, key=lambda k: (k // grid.width - seed[1]) ** 2 + (k % grid.width - seed[0]) ** 2)
        sites = [grid.world(i % grid.width, i // grid.width)]
    if len(sites) > max_sites:
        step = len(sites) / max_sites
        sites = [sites[int(k * step)] for k in range(max_sites)]
    return sites


def ray_free_cells(grid, x0, y0, angle, max_range, free_threshold=20):

    step = grid.resolution * 0.5
    dx, dy = cos(angle), sin(angle)
    s, last = 0.0, -1
    values, width = grid.values, grid.width
    while s <= max_range:
        cx, cy = grid.cell(x0 + s * dx, y0 + s * dy)
        if not grid.in_bounds(cx, cy):
            return
        idx = cy * width + cx
        v = values[idx]
        if v < 0 or v > free_threshold:
            return
        if idx != last:
            yield idx
            last = idx
        s += step


def visible_cells(grid, x, y, max_range, free_threshold=20, n_rays=360):

    out = set()
    for k in range(n_rays):
        out.update(ray_free_cells(grid, x, y, 2 * pi * k / n_rays, max_range, free_threshold))
    return out
