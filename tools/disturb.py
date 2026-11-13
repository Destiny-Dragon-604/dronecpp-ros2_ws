#!/usr/bin/env python3
"""Send wind / disturbance commands to the SuaveDisturbance Gazebo plugin (SITL only).

All vectors are NED (north, east, down) in SI units, the same frame suave_main uses.
Wind is the velocity of the air (the direction it blows toward).

  disturb.py wind 0 5 0            steady 5 m/s wind blowing toward east
  disturb.py turb 1.5 [L] [sd]     turbulence, sigma 1.5 m/s, length scale L m (20), vertical sigma sd (sigma/2)
  disturb.py gust 0 8 0 2          1-cosine gust peaking at 8 m/s east, lasting 2 s
  disturb.py push 0 0 -5 0.5       5 N upward force for 0.5 s
  disturb.py torque 0 0 0.3 1      0.3 N m yaw torque (body FRD) for 1 s
  disturb.py drag 1.0 0.05 0.12    drag coefficient, side area m^2, top area m^2
  disturb.py calm                  remove all disturbances
  disturb.py status                show the plugin state
  disturb.py run tools/scenarios/gust_rejection.txt
                                   play a timed scenario (lines: "<seconds> <command>")

Every command and the resulting wind, forces and Gazebo ground truth are logged to the
current flight's data folder (disturbance.csv, disturbance_events.csv).
"""

import os
import shutil
import subprocess
import sys
import time

TOPIC = "/suave/disturbance"
STATUS_TOPIC = "/suave/disturbance/status"
COMMANDS = {"wind", "turb", "gust", "push", "torque", "drag", "calm", "seed", "status"}


def gz_env():
    env = dict(os.environ)
    env.setdefault("GZ_IP", "127.0.0.1")  # must match PX4/Gazebo (tools/sim.sh)
    return env


def send(command: str) -> bool:
    result = subprocess.run(
        ["gz", "topic", "-t", TOPIC, "-m", "gz.msgs.StringMsg", "-p", f'data: "{command}"'],
        env=gz_env(), capture_output=True, text=True, timeout=10)
    if result.returncode != 0:
        print(f"failed to publish: {result.stderr.strip()}", file=sys.stderr)
        return False
    return True


def send_and_reply(command: str, timeout_s: float = 3.0) -> str:
    """Publish a command and return the plugin's answer from the status topic."""
    echo = subprocess.Popen(
        ["gz", "topic", "-e", "-n", "1", "-t", STATUS_TOPIC],
        env=gz_env(), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    time.sleep(0.3)  # let the subscriber connect
    send(command)
    try:
        out, _ = echo.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        echo.kill()
        return "(no reply: is the simulator running via tools/sim.sh?)"
    line = out.strip()
    if line.startswith("data:"):
        line = line[len("data:"):].strip().strip('"')
    return line or "(empty reply)"


def run_scenario(path: str) -> None:
    steps = []
    with open(path) as f:
        for n, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            t_str, _, cmd = line.partition(" ")
            try:
                t = float(t_str)
            except ValueError:
                sys.exit(f"{path}:{n}: expected '<seconds> <command>', got: {raw.strip()}")
            if cmd.split()[0] not in COMMANDS:
                sys.exit(f"{path}:{n}: unknown command '{cmd.split()[0]}'")
            steps.append((t, cmd.strip()))
    steps.sort()
    print(f"Running {path}: {len(steps)} steps over {steps[-1][0] if steps else 0:.1f} s (Ctrl+C sends 'calm')")
    start = time.monotonic()
    try:
        for t, cmd in steps:
            delay = start + t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            print(f"  t={t:6.2f}s  {cmd:30s} -> {send_and_reply(cmd)}")
    except KeyboardInterrupt:
        print("\ninterrupted, calming")
        send("calm")


def main() -> None:
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help", "help"):
        print(__doc__)
        return
    if shutil.which("gz") is None:
        sys.exit("gz (Gazebo) is not on PATH")
    if args[0] == "run":
        if len(args) != 2:
            sys.exit("usage: disturb.py run <scenario.txt>")
        run_scenario(args[1])
        return
    if args[0] not in COMMANDS:
        sys.exit(f"unknown command '{args[0]}'. Try: disturb.py --help")
    command = " ".join(args)
    print(send_and_reply(command))


if __name__ == "__main__":
    main()
