#!/usr/bin/env python3
import argparse
import math
import sys


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_obs(text):
    try:
        x, y, u = (float(v) for v in text.split(','))
    except ValueError:
        raise argparse.ArgumentTypeError(f'"{text}" must look like X,Y,U (forward m, left m, pixel)')
    if x <= 0:
        raise argparse.ArgumentTypeError('X (forward distance from the camera) must be positive')
    return x, y, u


def bearing(u, width, fov_deg):
    focal = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    return math.atan2(width / 2.0 - u, focal)


def evaluate(observations, width, fov_deg):
    diffs = [wrap(math.atan2(y, x) - bearing(u, width, fov_deg)) for x, y, u in observations]
    offset = sum(diffs) / len(diffs)
    residuals = [wrap(d - offset) for d in diffs]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    return offset, rms, residuals


def main():
    ap = argparse.ArgumentParser(description='Fit camera_horizontal_fov_deg and camera_yaw_offset_deg from static placements.')
    ap.add_argument('--obs', type=parse_obs, nargs='+', required=True,
                    help='X,Y,U: object forward X (m) and left Y (m) from the camera, and YOLO box-centre pixel U')
    ap.add_argument('--width', type=float, default=640.0)
    ap.add_argument('--fov-min', type=float, default=30.0)
    ap.add_argument('--fov-max', type=float, default=100.0)
    a = ap.parse_args()
    obs = a.obs
    angles = {round(math.degrees(math.atan2(y, x)), 1) for x, y, _ in obs}
    if len(obs) < 3 or len(angles) < 3:
        print('Need at least 3 placements at 3 different true bearings (for example left, centre, right).')
        return 2
    best = None
    steps = int(round((a.fov_max - a.fov_min) / 0.05))
    for k in range(steps + 1):
        fov = a.fov_min + 0.05 * k
        offset, rms, residuals = evaluate(obs, a.width, fov)
        if best is None or rms < best[0]:
            best = (rms, fov, offset, residuals)
    rms, fov, offset, residuals = best
    print(f'camera_horizontal_fov_deg: {fov:.1f}')
    print(f'camera_yaw_offset_deg:     {math.degrees(offset):.2f}   (residual added on top of the camera TF yaw)')
    print(f'fit RMS error:             {math.degrees(rms):.2f} deg')
    for (x, y, u), r in zip(obs, residuals):
        print(f'  X={x:.2f} Y={y:+.2f} U={u:.0f}: true bearing {math.degrees(math.atan2(y, x)):+.1f} deg, residual {math.degrees(r):+.2f} deg')
    if math.degrees(rms) > 1.0:
        print('WARNING: RMS above 1 deg. Re-measure placements (from the camera, not the robot centre) and repeat the pixel readings.')
    if fov <= a.fov_min + 0.1 or fov >= a.fov_max - 0.1:
        print('WARNING: FOV hit the search limit; the data are inconsistent.')
    if abs(math.degrees(offset)) > 3.0:
        print('NOTE: yaw offset above 3 deg: fix the camera static TF yaw or the mounting first, then repeat.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
