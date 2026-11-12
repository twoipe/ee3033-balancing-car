#!/usr/bin/env python3

import argparse
import base64
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WORKSPACE = Path(os.environ.get('ROS2_WS', Path.home() / 'ros2_ws'))
PI_HOST = os.environ.get('EE3033_PI_HOST', 'balance@balance10.local')
CONTAINER = os.environ.get('EE3033_CONTAINER', 'ros2_humble')
MAP_DIR = '/home/ros2_ws/src/balance_mapping/maps'
DEFAULT_MAP = os.environ.get('EE3033_MAP_NAME', 'ee3033_assessment')
YOLO_MODEL = os.environ.get('EE3033_YOLO_MODEL', 'yolov8n.pt')
DOMAIN = '10'

CONTROL_DIR = Path(tempfile.gettempdir()) / f'ee3033_ssh_{os.getuid()}'
CONTROL_PATH = str(CONTROL_DIR / 'ctl-%C')
STOP = threading.Event()
LOCK = threading.Lock()


def say(msg):
    with LOCK:
        print(msg, flush=True)


def ssh_base(extra=()):
    CONTROL_DIR.mkdir(mode=0o700, exist_ok=True)
    return ['ssh', '-o', 'ControlMaster=auto', '-o', f'ControlPath={CONTROL_PATH}', '-o', 'ControlPersist=15m',
            '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=6', *extra, PI_HOST]


def container_exec(command, interactive=False, env=None):
    parts = ['docker', 'exec']
    if interactive:
        parts.append('-i')
    for k, v in {'ROS_DOMAIN_ID': DOMAIN, 'ROS_LOCALHOST_ONLY': '0', **(env or {})}.items():
        parts += ['-e', f'{k}={v}']
    parts += [CONTAINER, 'bash', '-lc', command]
    return ' '.join(shlex.quote(p) for p in parts)


def pi_run(command, timeout=60, capture=True):

    return subprocess.run(ssh_base() + [command], text=True, capture_output=capture, timeout=timeout)


def pi_container(command, timeout=60):
    return pi_run(container_exec(command), timeout=timeout)


def payload_for(script, files):

    lines = ['mkdir -p /tmp/ee3033']
    for name, path in files:
        encoded = base64.encodebytes(Path(path).read_bytes()).decode()
        lines += [f"base64 -d > /tmp/ee3033/{name} <<'EE3033_B64'", encoded.rstrip(), 'EE3033_B64']
    lines.append(Path(script).read_text())
    return ('\n'.join(lines) + '\n').encode()


def remote_stop(kind):
    pid_file = {'nav': '/tmp/ee3033_nav.pid', 'perception': '/tmp/ee3033_perception.pid'}[kind]
    script = (f'f={pid_file}; [ -f $f ] || exit 0; p=$(cat $f); kill -TERM $p 2>/dev/null; '
              'for i in $(seq 1 30); do kill -0 $p 2>/dev/null || exit 0; sleep 0.3; done; '
              'pkill -KILL -P $p 2>/dev/null; kill -KILL $p 2>/dev/null; rm -f $f; echo forced')
    try:
        r = pi_container(script, timeout=30)
        if r.stdout.strip():
            say(f'[{kind}] {r.stdout.strip()}')
    except (subprocess.TimeoutExpired, OSError) as exc:
        say(f'WARNING: could not confirm the Pi {kind} stack stopped ({exc}). Run: python3 ee3033.py down')


ROS_SETUP = ('source /opt/ros/humble/setup.bash && '
             f'source {shlex.quote(str(WORKSPACE / "install" / "setup.bash"))} && '
             f'export ROS_DOMAIN_ID={DOMAIN} ROS_LOCALHOST_ONLY=0 && ')


def ros_shell(command):
    return ['bash', '-lc', ROS_SETUP + command]


def ros_call(command, timeout=60):
    return subprocess.run(ros_shell(command), text=True, capture_output=True, timeout=timeout)


def ros_cli(command):
    return subprocess.call(ros_shell(command))


class Managed:


    def __init__(self, name, argv, stdin_data=None, critical=False, restarts=0):
        self.name, self.argv, self.stdin_data = name, argv, stdin_data
        self.critical, self.restarts_left = critical, restarts
        self.proc, self.dead, self.next_start = None, False, 0.0

    def start(self):
        self.proc = subprocess.Popen(
            self.argv, stdin=subprocess.PIPE if self.stdin_data else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=False, start_new_session=True)
        if self.stdin_data:
            threading.Thread(target=self._feed, args=(self.proc,), daemon=True).start()
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        say(f'[{self.name}] started')

    def _feed(self, proc):
        try:
            proc.stdin.write(self.stdin_data)
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass

    def _pump(self, proc):
        for raw in iter(proc.stdout.readline, b''):
            say(f'[{self.name}] {raw.decode(errors="replace").rstrip()}')

    def signal(self, sig):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                pass


class CriticalFailure(RuntimeError):
    pass


def supervise(procs):
    for m in procs:
        m.start()
    while not STOP.is_set():
        now = time.monotonic()
        for m in procs:
            if m.dead:
                continue
            if m.proc is None:
                if now >= m.next_start:
                    m.start()
                continue
            code = m.proc.poll()
            if code is None:
                continue
            m.proc = None
            if m.critical:
                hints = {4: 'a navigation stack is already running on the Pi - run: python3 ee3033.py down',
                         5: 'the map file/image is missing in the Pi container',
                         8: 'numpy/rclpy missing in the Pi container'}
                raise CriticalFailure(f'{m.name} exited with code {code}' + (f' ({hints[code]})' if code in hints else ''))
            if m.name.startswith('pi-') and code == 4:
                say(f'[{m.name}] already running on the Pi - attaching to the existing one.')
                m.dead = True
            elif m.restarts_left > 0:
                m.restarts_left -= 1
                m.next_start = now + 3.0
                say(f'WARNING [{m.name}] exited (code {code}); restarting in 3 s '
                    f'({m.restarts_left} restarts left). Navigation is NOT affected.')
            else:
                m.dead = True
                say(f'WARNING [{m.name}] exited (code {code}) and will not be restarted. '
                    'Navigation is NOT affected; detection may be unavailable.')
        STOP.wait(0.5)


def shutdown(procs, pi_kinds):
    say('\nShutting down...')
    for m in procs:
        m.signal(signal.SIGINT)
    for kind in pi_kinds:
        remote_stop(kind)
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and any(m.proc and m.proc.poll() is None for m in procs):
        time.sleep(0.2)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for m in procs:
            m.signal(sig)
        time.sleep(1.0)
    say('Stopped.')


def preflight(need_nav, need_perception, map_name):
    say('Checking dependencies on Ubuntu...')
    pkgs = []
    if need_perception:
        pkgs += ['yolo_bringup', 'image_transport']
    if need_nav:
        pkgs += ['ee3033_search']
    probe = ('for p in ' + ' '.join(pkgs) + '; do ros2 pkg prefix $p >/dev/null 2>&1 || echo MISSING $p; done; '
             'python3 -c "import rclpy, yolo_msgs, tf2_ros, std_srvs" 2>&1 | tail -1')
    out = ros_call(probe, timeout=90)
    if not (WORKSPACE / 'install' / 'setup.bash').is_file():
        raise SystemExit(f'Missing {WORKSPACE}/install/setup.bash (set ROS2_WS if your workspace is elsewhere).')
    problems = [l for l in (out.stdout + out.stderr).splitlines() if l.startswith('MISSING') or 'Error' in l]
    if problems:
        hint = ('\nRun `python3 ee3033.py install` for ee3033_search.' if any('ee3033_search' in p for p in problems) else '')
        raise SystemExit('Ubuntu dependency problem:\n  ' + '\n  '.join(problems) + hint)
    if need_perception and '/' not in YOLO_MODEL and not any(
            p.exists() for p in (Path.cwd() / YOLO_MODEL, WORKSPACE / YOLO_MODEL, Path.home() / YOLO_MODEL)):
        say(f'NOTE: model {YOLO_MODEL!r} not found locally; YOLO may try to download it. Pre-download the '
            'weights (or set EE3033_YOLO_MODEL to an absolute path) because the lab network may be offline. '
            'A failed download only restarts YOLO - it cannot stop navigation.')
    say(f'Connecting to {PI_HOST} (SSH password may be requested once)...')
    r = subprocess.run(ssh_base() + ['true'])
    if r.returncode:
        raise SystemExit('SSH failed. Check balance10.local (or set EE3033_PI_HOST=balance@<ip>).')
    running = pi_run(f"docker inspect -f '{{{{.State.Running}}}}' {CONTAINER}").stdout.strip()
    if running != 'true':
        say('Starting the Pi ROS 2 container...')
        pi_run(f'docker start {CONTAINER} >/dev/null')
    if need_nav:
        for ext in ('yaml', 'pgm'):
            if pi_container(f'test -s {MAP_DIR}/{map_name}.{ext}').returncode:
                raise SystemExit(f'Map file {MAP_DIR}/{map_name}.{ext} missing in the Pi container. '
                                 'Save the map with the mapping launcher first.')

    names = ros_call('timeout 20 ros2 node list --no-daemon --spin-time 3', timeout=30).stdout.split()
    unsafe = {'/ee3033_astar_realtime', '/slam_toolbox', '/balance_keyboard_control', '/teleop_twist_keyboard'}
    if need_nav:
        clash = sorted(unsafe & set(names))
        if clash:
            raise SystemExit(f'These nodes are already running: {clash}. Stop mapping/teleop first '
                             '(and `python3 ee3033.py down` for stale navigation).')


def pi_process(name, script, files, args='', critical=False, env=None):
    argv = ssh_base() + [container_exec(f'bash -s -- {args}', interactive=True, env=env)]
    return Managed(name, argv, stdin_data=payload_for(ROOT / 'pi' / script, files), critical=critical)


def perception_procs():
    decompress = ros_shell(
        'exec ros2 run image_transport republish compressed raw --ros-args '
        '-r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_decompressed')
    yolo = ros_shell(
        'exec ros2 launch yolo_bringup yolo.launch.py '
        f'model:={shlex.quote(YOLO_MODEL)} device:=cpu input_image_topic:=/camera/image_decompressed '
        'use_tracking:=False use_3d:=False use_debug:=True threshold:=0.3 detections_topic:=detections')
    return [
        pi_process('pi-camera', 'pi_perception.sh', []),
        Managed('decompress', decompress, restarts=20),
        Managed('yolo', yolo, restarts=20),
    ]


def nav_procs(map_name, target=''):
    env = {'EE3033_LOCALIZATION_CMD': os.environ['EE3033_LOCALIZATION_CMD']} if 'EE3033_LOCALIZATION_CMD' in os.environ else None
    prefix = f'EE3033_TARGET={shlex.quote(target)} ' if target else ''
    return [
        pi_process('pi-nav', 'pi_nav.sh', [('astar_realtime.py', ROOT / 'pi' / 'astar_realtime.py'),
                                           ('astar.yaml', ROOT / 'config' / 'astar.yaml')],
                   args=shlex.quote(f'{MAP_DIR}/{map_name}.yaml'), critical=True, env=env),
        Managed('mission', ros_shell(f'{prefix}exec ros2 launch ee3033_search search.launch.py'), restarts=3),
    ]


def run_stack(mode, map_name, target=''):
    need_nav, need_perc = mode in ('run', 'nav'), mode in ('run', 'perception')
    preflight(need_nav, need_perc, map_name)
    procs, pi_kinds = [], []
    if need_perc:
        procs += perception_procs()
        pi_kinds.append('perception')
    if need_nav:
        procs += nav_procs(map_name, target)
        pi_kinds.append('nav')
        if target:
            say(f'Target class for this run: {target!r} (overrides search.yaml; confirm with `status`)')
        if mode == 'nav':
            say('NOTE: `nav` does not start YOLO/camera. Start them with `ee3033.py perception` (or the one-command startup) in another terminal.')
    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    signal.signal(signal.SIGTERM, lambda *_: STOP.set())
    say('\nStarting. Ctrl+C stops everything started by this command.')
    if need_nav:
        say('NEXT (in another terminal):  python3 ee3033.py set-pose   ->  status  ->  start')
    code = 0
    try:
        supervise(procs)
    except CriticalFailure as exc:
        say(f'\nERROR: {exc}. The navigation stack on the Pi is gone; stopping.')
        code = 1
    finally:
        shutdown(procs, pi_kinds)
    return code


def cmd_install():
    pkg = ROOT / 'ee3033_search'
    dest = WORKSPACE / 'src' / 'ee3033_search'
    check = ros_call('python3 -c "import yolo_msgs, std_srvs" && which colcon', timeout=60)
    if check.returncode:
        raise SystemExit('yolo_msgs / colcon not found. Source your YOLO workspace and install colcon first.\n'
                         + check.stdout + check.stderr)
    (WORKSPACE / 'src').mkdir(parents=True, exist_ok=True)
    if dest.exists():
        backup = Path.home() / '.ee3033_backups' / f'ee3033_search_{time.strftime("%Y%m%d_%H%M%S")}'
        backup.parent.mkdir(exist_ok=True)
        dest.rename(backup)
        say(f'Previous copy moved to {backup}')
    subprocess.check_call(['cp', '-r', str(pkg), str(dest)])
    subprocess.check_call(['find', str(dest), '-name', '__pycache__', '-prune', '-exec', 'rm', '-rf', '{}', '+'])
    say('Building ee3033_search (no Nav2 packages are needed)...')
    return ros_cli(f'cd {shlex.quote(str(WORKSPACE))} && colcon build --symlink-install --packages-select ee3033_search')


def cmd_offline():
    env = {**os.environ, 'PYTHONPATH': str(ROOT / 'ee3033_search')}
    return subprocess.call([sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT / 'ee3033_search' / 'tests'), '-v'],
                           env=env)


def cmd_set_pose(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.25
    cov[35] = 0.0685
    msg = ('{header: {frame_id: map}, pose: {pose: {position: {x: %s, y: %s, z: 0.0}, '
           'orientation: {x: 0.0, y: 0.0, z: %s, w: %s}}, covariance: %s}}'
           % (x, y, math.sin(yaw / 2), math.cos(yaw / 2), cov))
    say(f'Publishing AMCL initial pose x={x} y={y} yaw={yaw_deg} deg (map frame)...')
    rc = ros_cli(f'ros2 topic pub -t 6 -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped {shlex.quote(msg)} >/dev/null')
    say('Checking TF map -> base_footprint (6 s)...')
    ros_cli('timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -n 8')
    say('If no transform appeared: wait 5-10 s and run status; set the pose in RViz (2D Pose Estimate) if needed.')
    return rc


def cmd_service(name, confirm=False):
    if confirm:
        if input('The robot will drive AUTONOMOUSLY. Clear the area. Type START to continue: ').strip() != 'START':
            say('Not started.')
            return 1
    return ros_cli(f"timeout 25 ros2 service call /ee3033_search/{name} std_srvs/srv/Trigger '{{}}' 2>&1 | grep -E 'success|message|response|not available'")


def cmd_status():
    say('--- mission node ---')
    cmd_service('status')
    say('--- A* navigator ---')
    ros_cli('timeout 5 ros2 topic echo --once /astar/status 2>&1 | head -n 6 || true')
    say('--- /cmd_vel publishers (only the A* node should be commanding motion) ---')
    ros_cli('timeout 8 ros2 topic info -v /cmd_vel 2>&1 | grep -E "Node name|Publisher count|Subscription count"')
    say('--- YOLO ---')
    ros_cli('timeout 6 ros2 topic hz /yolo/detections 2>&1 | head -n 3 || true')
    say('--- image topics available for RViz (YOLO annotated image + decompressed camera) ---')
    ros_cli('ros2 topic list 2>/dev/null | grep -E "^/yolo/.*(dbg|debug).*image|^/camera/image_decompressed" || echo "(none yet - is perception running?)"')
    say('(`ee3033.py nav` alone does not start YOLO; use `perception` or `run`.)')
    return 0


DEBUG_IMAGE_PLACEHOLDER = '/yolo/debug_image'


def find_debug_image_topic():
    """Return the YOLO annotated-image topic that actually exists right now, or None.

    The topic name differs between yolo_ros versions (/yolo/dbg_image vs /yolo/debug_image),
    so it is looked up from the live graph instead of being hard-coded.
    """
    try:
        r = ros_call('ros2 topic list -t 2>/dev/null', timeout=25)
    except (subprocess.TimeoutExpired, OSError):
        return None
    found = []
    for line in r.stdout.splitlines():
        m = re.match(r'^(\S+)\s+\[(.*)\]\s*$', line.strip())
        if not m or 'sensor_msgs/msg/Image' not in m.group(2):
            continue
        if re.match(r'^/yolo/', m.group(1)) and re.search(r'(dbg|debug)', m.group(1)) and 'image' in m.group(1):
            found.append(m.group(1))
    return sorted(found)[0] if found else None


def cmd_rviz():
    template = ROOT / 'rviz' / 'navigation.rviz'
    topic = find_debug_image_topic()
    if topic:
        say(f'YOLO annotated image topic found: {topic}')
    else:
        say('WARNING: no YOLO annotated image topic exists yet, so the "YOLO annotated image" display will stay empty.\n'
            '  - `ee3033.py nav` does NOT start YOLO. Start it with `ee3033.py perception` (or use `run`), wait for\n'
            '    detections, then run `ee3033.py rviz` again so the topic can be detected.\n'
            '  - Check with: ros2 topic list | grep -i -E "dbg|debug"')
        topic = DEBUG_IMAGE_PLACEHOLDER
    fd, tmp = tempfile.mkstemp(prefix='ee3033_rviz_', suffix='.rviz')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(template.read_text().replace(DEBUG_IMAGE_PLACEHOLDER, topic))
        say('If RViz shows the image panels black (common in VMware without 3D acceleration), run:  python3 ee3033.py image')
        return ros_cli(f'rviz2 -d {shlex.quote(tmp)}')
    finally:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def cmd_image(topic=''):
    topic = topic or find_debug_image_topic()
    if not topic:
        say('No YOLO annotated image topic found. Start YOLO first (`ee3033.py perception` or `run`), then retry.\n'
            'Camera-only fallback: python3 ee3033.py image --topic /camera/image_decompressed')
        return 1
    say(f'Opening rqt_image_view on {topic} (same viewer as the one-command startup).')
    return ros_cli(f'exec ros2 run rqt_image_view rqt_image_view {shlex.quote(topic)}')


REPORT_RE = re.compile(r'TARGET REPORT: \(x, y\) = \(\s*(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)\s*\)')


def parse_report(text):

    found = REPORT_RE.findall(text)
    return (float(found[-1][0]), float(found[-1][1])) if found else None


def cmd_error(true_x, true_y, rx=None, ry=None):

    if rx is None or ry is None:
        path = Path.home() / 'ee3033_target_report.txt'
        reported = parse_report(path.read_text()) if path.exists() else None
        if reported is None:
            say(f'No TARGET REPORT found in {path}. Pass it by hand:  error --x X --y Y --rx RX --ry RY')
            return 1
        rx, ry = reported
    dx, dy = rx - true_x, ry - true_y
    err = math.hypot(dx, dy)
    say(f'reported=({rx:.3f}, {ry:.3f})  true=({true_x:.3f}, {true_y:.3f})  '
        f'dx={100 * dx:+.1f} cm  dy={100 * dy:+.1f} cm  ERROR={100 * err:.1f} cm')
    say('Within the 10 cm internal target' if err <= 0.10 else 'Above the 10 cm internal target; see README.md')
    log = Path.home() / 'ee3033_calibration_log.csv'
    new = not log.exists()
    with log.open('a') as f:
        if new:
            f.write('time,true_x,true_y,reported_x,reported_y,dx_m,dy_m,error_m\n')
        f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{true_x},{true_y},{rx},{ry},{dx:.4f},{dy:.4f},{err:.4f}\n')
    say(f'Logged to {log} (use it as experimental evidence in the report)')
    return 0


def main():
    ap = argparse.ArgumentParser(description='EE3033 Nav v10 automatic camera viewpoint acquisition and A* navigation', formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('run', 'nav'):
        p = sub.add_parser(name)
        p.add_argument('--map-name', default=DEFAULT_MAP)
        p.add_argument('--target', default='', help="YOLO class to search for this run, e.g. 'apple'")
    sub.add_parser('perception')
    ip = sub.add_parser('image', help='view the YOLO annotated image in rqt_image_view')
    ip.add_argument('--topic', default='', help='image topic (default: auto-detect the YOLO debug image)')
    sub.add_parser('install')
    sub.add_parser('offline')
    for name in ('status', 'preview', 'start', 'stop', 'rviz', 'down'):
        sub.add_parser(name)
    ep = sub.add_parser('error')
    ep.add_argument('--x', type=float, required=True, help='true target x (m, map frame)')
    ep.add_argument('--y', type=float, required=True, help='true target y (m, map frame)')
    ep.add_argument('--rx', type=float, default=None, help='reported x (default: last report file)')
    ep.add_argument('--ry', type=float, default=None, help='reported y (default: last report file)')
    sp = sub.add_parser('set-pose')
    sp.add_argument('--x', type=float, default=0.0)
    sp.add_argument('--y', type=float, default=0.0)
    sp.add_argument('--yaw-deg', type=float, default=0.0)
    a = ap.parse_args()

    if a.cmd in ('run', 'nav', 'perception'):
        return run_stack(a.cmd, getattr(a, 'map_name', DEFAULT_MAP), getattr(a, 'target', '').strip().lower())
    if a.cmd == 'error':
        return cmd_error(a.x, a.y, a.rx, a.ry)
    if a.cmd == 'install':
        return cmd_install()
    if a.cmd == 'offline':
        return cmd_offline()
    if a.cmd == 'set-pose':
        return cmd_set_pose(a.x, a.y, a.yaw_deg)
    if a.cmd == 'status':
        return cmd_status()
    if a.cmd in ('preview', 'stop'):
        return cmd_service(a.cmd)
    if a.cmd == 'start':
        return cmd_service('start', confirm=True)
    if a.cmd == 'rviz':
        return cmd_rviz()
    if a.cmd == 'image':
        return cmd_image(a.topic.strip())
    if a.cmd == 'down':
        subprocess.run(ssh_base() + ['true'])
        for kind in ('nav', 'perception'):
            remote_stop(kind)
        say('Pi stacks stopped (if any were running).')
        return 0
    return 2


if __name__ == '__main__':
    sys.exit(main())
