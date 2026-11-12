from math import atan2, cos, hypot, pi, radians, sin, tan
from statistics import median


def pixel_to_bearing(x_px, image_width, horizontal_fov_rad):
    if not (image_width > 0 and 0 < horizontal_fov_rad < pi):
        raise ValueError('Invalid camera geometry')
    focal_px = image_width / (2.0 * tan(horizontal_fov_rad / 2.0))
    return atan2(image_width / 2.0 - x_px, focal_px)


def area_fraction(width_px, height_px, image_width, image_height):
    if image_width <= 0 or image_height <= 0:
        raise ValueError('Invalid image dimensions')
    return max(0.0, width_px) * max(0.0, height_px) / (image_width * image_height)


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _ray_intersection(a, b, min_cross_angle_rad, max_distance_m):
    ax, ay, ath = a
    bx, by, bth = b
    adx, ady, bdx, bdy = cos(ath), sin(ath), cos(bth), sin(bth)
    denominator = _cross(adx, ady, bdx, bdy)
    if abs(denominator) < sin(min_cross_angle_rad):
        return None
    difference_x, difference_y = bx - ax, by - ay
    distance_a = _cross(difference_x, difference_y, bdx, bdy) / denominator
    distance_b = _cross(difference_x, difference_y, adx, ady) / denominator
    if not (0.0 < distance_a <= max_distance_m and 0.0 < distance_b <= max_distance_m):
        return None
    return ax + distance_a * adx, ay + distance_a * ady


def _bearing_error(view, xy):
    dx, dy = xy[0] - view[0], xy[1] - view[1]
    if dx * cos(view[2]) + dy * sin(view[2]) <= 0:
        return pi
    angle = atan2(dy, dx) - view[2]
    return abs(atan2(sin(angle), cos(angle)))


def _refine(views):
    a = b = c = d = ex = ey = 0.0
    for x, y, theta in views:
        nx, ny = -sin(theta), cos(theta)
        a += nx * nx
        b += nx * ny
        d += ny * ny
        projection = nx * x + ny * y
        ex += nx * projection
        ey += ny * projection
    determinant = a * d - b * b
    if determinant <= 1e-9:
        return None
    return (d * ex - b * ey) / determinant, (a * ey - b * ex) / determinant


def triangulate_target(views, min_baseline_m=0.20, min_cross_angle_deg=6.0,
                       max_distance_m=2.5, max_bearing_error_deg=3.0,
                       min_observations=4):
    observations = tuple(views)[-60:]
    if len(observations) < min_observations:
        return None
    tolerance = radians(max_bearing_error_deg)
    cross_min = radians(min_cross_angle_deg)
    best = None
    best_score = None
    for i, first in enumerate(observations):
        for second in observations[i + 1:]:
            baseline = hypot(second[0] - first[0], second[1] - first[1])
            if baseline < min_baseline_m:
                continue
            candidate = _ray_intersection(first, second, cross_min, max_distance_m)
            if candidate is None:
                continue
            inliers = [v for v in observations if _bearing_error(v, candidate) <= tolerance and
                       0.0 < hypot(candidate[0] - v[0], candidate[1] - v[1]) <= max_distance_m]
            if len(inliers) < min_observations:
                continue
            spread = max(hypot(v[0] - first[0], v[1] - first[1]) for v in inliers)
            if spread < min_baseline_m:
                continue
            estimate = _refine(inliers)
            if estimate is None:
                continue
            refined_inliers = [v for v in observations if _bearing_error(v, estimate) <= tolerance and
                               0.0 < hypot(estimate[0] - v[0], estimate[1] - v[1]) <= max_distance_m]
            if len(refined_inliers) < min_observations:
                continue
            if max(hypot(v[0] - refined_inliers[0][0], v[1] - refined_inliers[0][1])
                   for v in refined_inliers) < min_baseline_m:
                continue
            estimate = _refine(refined_inliers)
            if estimate is None:
                continue
            errors = [_bearing_error(v, estimate) for v in refined_inliers]
            if any(e > tolerance for e in errors):
                continue
            score = (len(refined_inliers), -median(errors), min(baseline, 1.0))
            if best_score is None or score > best_score:
                best, best_score = estimate, score
    return best
