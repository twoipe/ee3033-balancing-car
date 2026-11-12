#!/usr/bin/env python3
"""EE3033 v7.4 - MAPPING launcher: SLAM + RViz2 + keyboard teleoperation. Run on Ubuntu VMware.

    python3 mapping.py                 start SLAM on the Pi, open RViz2, drive with the keyboard
    python3 mapping.py save-map        save the map NOW (run from a 2nd terminal, or use menu [s])
    python3 mapping.py stop            stop SLAM + base + LiDAR on the Pi, incl. leftovers (only after saving the map!)

This launcher does NOT start the camera, YOLO or navigation, and it never saves the map by itself:
you decide when the map is good and save it manually. It is independent of EE3033_Nav_v10.
"""
import argparse
import os
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
LOCAL_MAP_DIR = Path.home() / 'ee3033_maps'
DOMAIN = '10'
# Keyboard teleop. DEFAULT = your Lab 4 node, run on the Pi inside the container (same as the lab manual):
#   ros2 run balance_robot_base balance_keyboard_control
# Override the Pi command:   export EE3033_PI_TELEOP_CMD="ros2 run <pkg> <node>"
# Or run a teleop on this PC instead:  export EE3033_TELEOP_CMD="ros2 run <pkg> <node>"
PI_TELEOP_CMD = os.environ.get('EE3033_PI_TELEOP_CMD', 'ros2 run balance_robot_base balance_keyboard_control')
TELEOP_CMD = os.environ.get('EE3033_TELEOP_CMD')   # None = use the Pi teleop above

CONTROL_DIR = Path(tempfile.gettempdir()) / f'ee3033_ssh_{os.getuid()}'
CONTROL_PATH = str(CONTROL_DIR / 'ctl-%C')
PRINT_LOCK = threading.Lock()


def say(msg):
    with PRINT_LOCK:
        print(msg, flush=True)


def ssh_base(extra=()):
    CONTROL_DIR.mkdir(mode=0o700, exist_ok=True)
    return ['ssh', '-o', 'ControlMaster=auto', '-o', f'ControlPath={CONTROL_PATH}', '-o', 'ControlPersist=15m',
            '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3', *extra, PI_HOST]


def container_exec(command, interactive=False, tty=False):
    parts = ['docker', 'exec']
    if interactive or tty:
        parts.append('-it' if tty else '-i')
    parts += ['-e', f'ROS_DOMAIN_ID={DOMAIN}', '-e', 'ROS_LOCALHOST_ONLY=0', CONTAINER, 'bash', '-lc', command]
    return ' '.join(shlex.quote(p) for p in parts)


def pi_run(command, timeout=60):
    return subprocess.run(ssh_base() + [command], text=True, capture_output=True, timeout=timeout)


def pi_container(command, timeout=60):
    return pi_run(container_exec(command), timeout=timeout)


ROS_SETUP = ('source /opt/ros/humble/setup.bash && '
             f'source {shlex.quote(str(WORKSPACE / "install" / "setup.bash"))} && '
             f'export ROS_DOMAIN_ID={DOMAIN} ROS_LOCALHOST_ONLY=0 && ')


def ros_shell(command):
    return ['bash', '-lc', ROS_SETUP + command]


def ros_call(command, timeout=60):
    return subprocess.run(ros_shell(command), text=True, capture_output=True, timeout=timeout)


class Managed:
    def __init__(self, name, argv, stdin_data=None):
        self.name, self.argv, self.stdin_data, self.proc = name, argv, stdin_data, None
        self.reported = False

    def start(self):
        self.proc = subprocess.Popen(
            self.argv, stdin=subprocess.PIPE if self.stdin_data else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        if self.stdin_data:
            threading.Thread(target=self._feed, daemon=True).start()
        threading.Thread(target=self._pump, daemon=True).start()
        self.reported = False

    def _feed(self):
        try:
            self.proc.stdin.write(self.stdin_data)
            self.proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass

    def _pump(self):
        for raw in iter(self.proc.stdout.readline, b''):
            say(f'[{self.name}] {raw.decode(errors="replace").rstrip()}')

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def signal(self, sig):
        if self.alive():
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                pass


def payload_for(script):
    return ('mkdir -p /tmp/ee3033\n' + Path(script).read_text()).encode()


# Matches every process of the SLAM bring-up. The [x] bracket trick stops pkill/pgrep from matching
# (and killing) the very shell that carries this pattern on its command line.
STRAY_PATTERN = ('[a]sync_slam_toolbox_node|[b]alance_robot_node|[l]slidar_driver_node|[s]can_resampler|'
                 '[s]tatic_transform_publisher|[b]alance_slam_bringup')
STOP_SCRIPT = (
    'f=/tmp/ee3033_slam.pid; '
    'if [ -f $f ]; then p=$(cat $f); kill -TERM $p 2>/dev/null; '
    'for i in $(seq 1 40); do kill -0 $p 2>/dev/null || break; sleep 0.3; done; '
    'kill -KILL $p 2>/dev/null; rm -f $f; fi; '
    f"pat='{STRAY_PATTERN}'; n=$(pgrep -fc \"$pat\"); "
    'if [ "$n" -gt 0 ]; then pkill -TERM -f "$pat"; sleep 3; pkill -KILL -f "$pat"; '
    'echo "swept $n leftover SLAM/base/LiDAR process(es)"; fi; '
    'echo STOP_DONE')


def remote_stop_slam():
    try:
        r = pi_container(STOP_SCRIPT, timeout=60)
        out = r.stdout.replace('STOP_DONE', '').strip()
        say(f'[slam] stopped. {out}'.strip())
        left = pi_container(f"pgrep -fc '{STRAY_PATTERN}'", timeout=20).stdout.strip()
        if left not in ('', '0'):
            say(f'WARNING: {left} SLAM-related process(es) still running on the Pi. '
                f'Last resort: ssh {PI_HOST} "docker restart {CONTAINER}"')
    except (subprocess.TimeoutExpired, OSError) as exc:
        say(f'WARNING: could not confirm SLAM stopped ({exc}). Run: python3 mapping.py stop')


# --------------------------------------------------------------------------------- map saving
def map_check(yaml_path):
    """Quality check of a saved map (reads the PC backup copy). Returns (ok, lines).

    Verifies free_thresh (0.19 keeps grey 'unknown' pixels unknown, 0.25 would make the planner treat everything
    outside the walls as free) and reports the size and the share of free / occupied / unknown cells.
    """
    yaml_path = Path(yaml_path)
    lines, ok = [], True
    meta = {}
    for raw in yaml_path.read_text().splitlines():
        if ':' in raw and not raw.lstrip().startswith('#'):
            key, _, val = raw.partition(':')
            meta[key.strip()] = val.strip()
    free_thresh = float(meta.get('free_thresh', 'nan'))
    if abs(free_thresh - 0.19) > 1e-6:
        ok = False
        lines.append(f'WARNING free_thresh={meta.get("free_thresh")} (expected 0.19): unknown space would load as FREE. '
                     f'Fix: sed -i "s/^free_thresh:.*/free_thresh: 0.19/" {yaml_path}  (also on the Pi copy)')
    else:
        lines.append('free_thresh 0.19 OK (unknown space stays unknown)')
    pgm = yaml_path.with_name(meta.get('image', yaml_path.stem + '.pgm').strip('"\''))
    data = pgm.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:                     # P5 <width> <height> <maxval>, then ONE whitespace byte, then pixels
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    if tokens[0] != b'P5':
        return False, lines + [f'WARNING {pgm.name} is not a binary PGM (P5)']
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[pos:pos + width * height]
    total = max(1, len(pixels))
    free = sum(1 for v in pixels if v >= 250)
    occupied = sum(1 for v in pixels if v <= 10)
    unknown = total - free - occupied
    res = float(meta.get('resolution', '0.05'))
    lines.append(f'map {width} x {height} cells = {width * res:.2f} m x {height * res:.2f} m; '
                 f'free {100 * free / total:.0f} %, occupied {100 * occupied / total:.0f} %, unknown {100 * unknown / total:.0f} %')
    lines.append(f'origin (map x, y, yaw) = {meta.get("origin")}  <- this is where the robot started mapping: set-pose 0 0 0 there')
    if occupied < 20:
        ok = False
        lines.append('WARNING almost no occupied cells: the walls were not mapped. Drive along the fence and save again.')
    return ok, lines


def save_map(name):
    stem = f'{MAP_DIR}/{name}'
    script = (
        'source /opt/ros/humble/setup.bash; source /home/ros2_ws/install/setup.bash; '
        f'mkdir -p {MAP_DIR}; ts=$(date +%Y%m%d_%H%M%S); '
        f'for ext in yaml pgm; do [ -f {stem}.$ext ] && mv {stem}.$ext {stem}_prev_$ts.$ext && echo "old $ext kept as {name}_prev_$ts.$ext"; done; '
        f'timeout 60 ros2 run nav2_map_server map_saver_cli -f {stem} || {{ echo SAVE_FAILED_saver; exit 3; }}; '
        f'sed -i "s/^free_thresh:.*/free_thresh: 0.19/" {stem}.yaml; '
        f'[ -s {stem}.yaml ] || {{ echo SAVE_FAILED_no_yaml; exit 3; }}; '
        f'[ -s {stem}.pgm ] || {{ echo SAVE_FAILED_no_pgm; exit 3; }}; '
        f'ls -l {stem}.yaml {stem}.pgm; echo SAVED_OK')
    say(f'Saving map "{name}" from the running SLAM (this can take up to a minute)...')
    r = pi_container(script, timeout=120)
    say((r.stdout + r.stderr).strip())
    if r.returncode != 0 or 'SAVED_OK' not in r.stdout:
        say('\nMAP NOT SAVED. Is SLAM still running and is /map being published? SLAM stays up - try again.')
        return False
    LOCAL_MAP_DIR.mkdir(exist_ok=True)
    for ext in ('yaml', 'pgm'):
        dest = LOCAL_MAP_DIR / f'{name}.{ext}'
        with dest.open('wb') as f:
            c = subprocess.run(ssh_base() + [container_exec(f'cat {stem}.{ext}')], stdout=f, timeout=60)
        if c.returncode or dest.stat().st_size == 0:
            say(f'WARNING: backup copy to {dest} failed (the map on the Pi is fine).')
            break
    else:
        say(f'Backup copy on this PC: {LOCAL_MAP_DIR}/{name}.yaml / .pgm')
        try:
            ok, notes = map_check(LOCAL_MAP_DIR / f'{name}.yaml')
            say('MAP CHECK ' + ('OK' if ok else 'NEEDS ATTENTION') + ':\n  ' + '\n  '.join(notes))
        except (OSError, ValueError, IndexError) as exc:
            say(f'MAP CHECK skipped ({exc}); the saved map itself is fine.')
    say(f'\nMAP SAVED on the Pi: {stem}.yaml  (the navigation launcher uses --map-name {name})')
    return True


# --------------------------------------------------------------------------------- run
def preflight():
    if not (WORKSPACE / 'install' / 'setup.bash').is_file():
        raise SystemExit(f'Missing {WORKSPACE}/install/setup.bash (set ROS2_WS if elsewhere).')
    probe = ros_call('which rviz2 >/dev/null || echo MISSING rviz2')
    missing = [l for l in probe.stdout.splitlines() if l.startswith('MISSING')]
    if missing:
        raise SystemExit('Ubuntu dependency problem: ' + ', '.join(missing) + '\n  sudo apt install -y ros-humble-rviz2')
    say(f'Connecting to {PI_HOST} (SSH password may be requested once)...')
    if subprocess.run(ssh_base() + ['true']).returncode:
        raise SystemExit('SSH failed. Check balance10.local (or set EE3033_PI_HOST=balance@<ip>).')
    if pi_run(f"docker inspect -f '{{{{.State.Running}}}}' {CONTAINER}").stdout.strip() != 'true':
        say('Starting the Pi ROS 2 container...')
        pi_run(f'docker start {CONTAINER} >/dev/null')
    # --no-daemon: ask the network directly. The ROS daemon caches nodes and keeps listing dead ones.
    names = set(ros_call('timeout 20 ros2 node list --no-daemon --spin-time 3', timeout=30).stdout.split())
    clash = sorted(names & {'/slam_toolbox', '/amcl', '/ee3033_astar_realtime', '/balance_keyboard_control',
                            '/teleop_twist_keyboard'})
    if clash:
        raise SystemExit(f'Already running (fresh check): {clash}. Stop navigation/teleop/old SLAM first '
                         '(`python3 mapping.py stop` for a stale SLAM on the Pi). If another group or a teammate '
                         'shares ROS_DOMAIN_ID, ask for a different ID.')


def teleop_argv():
    if TELEOP_CMD:
        return ros_shell(f'exec {TELEOP_CMD}')
    inner = ('source /opt/ros/humble/setup.bash; source /home/ros2_ws/install/setup.bash; '
             f'exec {PI_TELEOP_CMD}')
    return ssh_base(['-t']) + [container_exec(inner, tty=True)]


def run_teleop():
    where = 'on this PC' if TELEOP_CMD else 'on the Pi (Lab 4 node)'
    say(f'\n=== KEYBOARD TELEOP {where} - focus THIS terminal ===\n'
        'Use the keys the teleop prints. Be ready to STOP the robot (stop key / Ctrl+C) before it reaches a wall.\n'
        'Ctrl+C leaves teleop and returns to the menu (a zero velocity is then sent to the robot).')
    old = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        subprocess.call(teleop_argv(), preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    finally:
        signal.signal(signal.SIGINT, old)
    for _ in range(2):   # make sure the robot is told to stop
        ros_call('ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1', timeout=15)


def start_rviz():
    rviz = Managed('rviz2', ros_shell(f'exec rviz2 -d {shlex.quote(str(ROOT / "rviz" / "mapping.rviz"))}'))
    rviz.start()
    return rviz


def monitor(slam, stop):
    while not stop.is_set():
        if slam.proc is not None and not slam.alive() and not slam.reported:
            slam.reported = True
            say(f'\n!!! SLAM EXITED (code {slam.proc.returncode}). The live map is lost unless you already saved it. '
                '!!!\n    Check the [slam] lines above; restart with `python3 mapping.py`.')
        stop.wait(1.0)


def run(map_name):
    preflight()
    slam = Managed('slam', ssh_base() + [container_exec('bash -s', interactive=True)], payload_for(ROOT / 'pi' / 'pi_slam.sh'))
    slam.start()
    stop = threading.Event()
    threading.Thread(target=monitor, args=(slam, stop), daemon=True).start()
    say('Waiting ~8 s for SLAM, then opening RViz2...')
    time.sleep(8)
    rviz = start_rviz()
    saved = False
    keep_running = False
    try:
        run_teleop()
        while True:
            say('\n[t] teleop again   [s] SAVE MAP now   [r] reopen RViz2   [q] stop SLAM + RViz2   [k] leave running & exit')
            choice = input('> ').strip().lower()
            if choice == 't':
                run_teleop()
            elif choice == 's':
                saved = save_map(map_name) or saved
            elif choice == 'r':
                if rviz.alive():
                    say('RViz2 is still open.')
                else:
                    rviz = start_rviz()
            elif choice == 'k':
                keep_running = True
                say('Leaving SLAM running on the Pi. Save from any terminal with: python3 mapping.py save-map')
                break
            elif choice == 'q':
                if not saved:
                    answer = input('No map was saved from THIS menu. Type STOP to stop anyway '
                                   '(if you saved from another terminal): ').strip()
                    if answer != 'STOP':
                        continue
                break
    except (EOFError, KeyboardInterrupt):
        say('\nInput closed; leaving SLAM running so the map is not lost. Use `python3 mapping.py save-map` / `stop`.')
        keep_running = True
    finally:
        stop.set()
        rviz.signal(signal.SIGINT)
        if not keep_running:
            remote_stop_slam()
            slam.signal(signal.SIGTERM)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd')
    for name in ('run', 'save-map'):
        p = sub.add_parser(name)
        p.add_argument('--name', default=DEFAULT_MAP, help=f'map name (default {DEFAULT_MAP})')
    sub.add_parser('stop')
    a = ap.parse_args()
    cmd = a.cmd or 'run'
    name = getattr(a, 'name', DEFAULT_MAP)
    if cmd == 'run':
        return run(name)
    if cmd == 'save-map':
        subprocess.run(ssh_base() + ['true'])
        return 0 if save_map(name) else 1
    if cmd == 'stop':
        subprocess.run(ssh_base() + ['true'])
        remote_stop_slam()
        return 0
    return 2


if __name__ == '__main__':
    sys.exit(main())
