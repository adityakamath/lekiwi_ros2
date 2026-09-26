#!/usr/bin/env python3
"""Inspect an existing LeKiwi MJCF in MuJoCo's passive native viewer.

On macOS run with mjpython. No waypoint follower or ROS controller is included.

Standalone, no install needed: `python3 -m lekiwi_mujoco.mujoco_preview --variant pt101`,
run from this package's root dir (-m puts the cwd on sys.path). After `pip install -e .`
(or a colcon build), the same tool is also `mujoco_preview` on PATH / `ros2 run lekiwi_mujoco
mujoco_preview`.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
from queue import SimpleQueue
from threading import Lock
import time
from tempfile import TemporaryDirectory

from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import RobotControl, Simulation

import glfw
import mujoco
import mujoco.viewer

from lekiwi_mujoco.paths import package_share

PACKAGE = package_share('lekiwi_mujoco')


class Trail:
    """Bounded visual-only XY trail for the flat scene; never adds physics bodies."""
    def __init__(self):
        self.points = deque(maxlen=1000)

    def clear(self):
        self.points.clear()

    def draw(self, scene, position):
        point = (float(position[0]), float(position[1]), .003)
        if not self.points or math.dist(point, self.points[-1]) >= .01:
            self.points.append(point)
        scene.ngeom = 0
        points = list(self.points)
        for start, end in zip(points, points[1:]):
            if scene.ngeom == scene.maxgeom:
                break
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, [0, 0, 0],
                               [0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0, 1], [.1, .85, 1., 1.])
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 2., start, end)
            scene.ngeom += 1


class KeyboardControl:
    """Compute commands from held keys, independent of keyboard repeat settings."""
    def __init__(self, model, control=None):
        self.model = model
        self.control = control or RobotControl(model)
        self.wheels = self.control.wheels
        self.base_limits = self.control.base_limits
        self.wheel_limits = self.control.wheel_limits
        self.kinematics = self.control.kinematics
        self.payload_limits = self.control.payload_limits
        self.active = False
        self.estop = False

    def enforce_limits(self, data, dt):
        self.control.apply(data, data.ctrl.copy(), dt)

    def reset_targets(self, data):
        self.control.reset_targets(data)

    def stop(self, data):
        data.ctrl[self.wheels] = 0
        self.active = False

    def set_estop(self, data, active):
        """Match the real robot's emergency stop: torque disabled on every motor via
        mjDSBL_ACTUATION, same as sts_hardware_interface's real EnableTorque(motor, 0)
        and mujoco_ros2_plugins/EmergencyStopPlugin. Wheels and payload drift/coast
        freely rather than locking or holding."""
        self.estop = active
        if active:
            self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
        else:
            self.model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)

    def update(self, data, held, dt):
        held = {ord(chr(k).upper()) if 97 <= k <= 122 else k for k in held}
        alt = bool(held & {glfw.KEY_LEFT_ALT, glfw.KEY_RIGHT_ALT})
        shift = bool(held & {glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT})
        forward = int(glfw.KEY_UP in held) - int(glfw.KEY_DOWN in held)
        lateral = int(glfw.KEY_LEFT in held) - int(glfw.KEY_RIGHT in held)
        command = self.base_limits * [forward, 0 if shift else lateral, lateral if shift else 0]
        if alt:
            command[:] = 0
        moving = not alt and bool(forward or lateral)
        if moving or self.active or alt:
            data.ctrl[self.wheels] = self.control.wheel_speeds(command)
        self.active = moving
        if alt:
            rates = {'shoulder_pan_joint': lateral, 'tilt_joint': forward}
            self.control.integrate_payload(data, [rates[name] for name in self.payload_limits], dt)


class HeldKeys:
    """Attach GLFW press/release callbacks on the viewer's own UI thread.

    The passive callback bootstraps access to its window on the first key press.
    Unbound keys and focus notifications are forwarded to MuJoCo's callbacks.
    """
    BOUND = {glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT,
             glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT, glfw.KEY_LEFT_ALT, glfw.KEY_RIGHT_ALT,
             glfw.KEY_SPACE, *map(ord, 'XP')}

    def __init__(self):
        self.lock = Lock()
        self.held = set()
        self.events = SimpleQueue()
        self.window = None
        self.previous_key = None
        self.previous_focus = None

    def snapshot(self):
        with self.lock:
            return self.held.copy()

    def clear(self):
        with self.lock:
            self.held.clear()

    def install_callbacks(self, window):
        # Python GLFW's public setters discard callbacks installed by C++.
        # Retain both native function pointers and Python callback lifetimes.
        self.key_callback = glfw._GLFWkeyfun(self.on_key)
        self.focus_callback = glfw._GLFWwindowfocusfun(self.on_focus)
        self.previous_key = glfw._glfw.glfwSetKeyCallback(window, self.key_callback)
        self.previous_focus = glfw._glfw.glfwSetWindowFocusCallback(window, self.focus_callback)

    def bootstrap(self, key):
        if self.window is not None:
            return
        window = glfw.get_current_context()
        if not window:
            return
        self.window = window
        self.install_callbacks(window)
        with self.lock:
            self.held = {k for k in self.BOUND if glfw.get_key(window, k) == glfw.PRESS}
        if key in self.BOUND:
            self.record(key, glfw.PRESS)
        print('Held-key input attached: press/release and focus tracking enabled', flush=True)

    def record(self, key, action):
        with self.lock:
            if action == glfw.RELEASE:
                self.held.discard(key)
            elif action == glfw.PRESS:
                self.held.add(key)
        if action == glfw.PRESS and key in (ord('P'), ord('X'), glfw.KEY_SPACE):
            self.events.put(key)

    def on_key(self, window, key, scancode, action, mods):
        if key in self.BOUND:
            self.record(key, action)
        elif self.previous_key:
            self.previous_key(window, key, scancode, action, mods)

    def on_focus(self, window, focused):
        if not focused:
            self.clear()
            self.events.put('focus_lost')
        if self.previous_focus:
            self.previous_focus(window, focused)


# Two label/value columns, like MuJoCo's own built-in Info overlay - set_texts' 3rd/4th tuple
# fields render as left/right-aligned columns, not one run-on wrapped line.
CONTROL_LABELS_BASE = 'Drive\nStrafe\nRotate\nE-Stop\nReset\nPause'
CONTROL_VALUES_BASE = 'Up/Down\nLeft/Right\nShift + Left/Right\nSpace\nX\nP'
CONTROL_LABELS_PAYLOAD = 'Drive\nStrafe\nRotate\nPan\nTilt\nE-Stop\nReset\nPause'
CONTROL_VALUES_PAYLOAD = 'Up/Down\nLeft/Right\nShift + Left/Right\nAlt + Left/Right\nAlt + Up/Down\nSpace\nX\nP'
STATUS_LABELS = 'Model\nTime\nStatus'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['base', 'pt100', 'pt101'], default='pt101')
    parser.add_argument('--control-package', type=Path)
    parser.add_argument('--description-package', type=Path)
    parser.add_argument('--pt-package', type=Path)
    parser.add_argument('--scene', default='flat', help='flat, arena, home (apartment), maze, none, or scene MJCF path')
    parser.add_argument('--model', type=Path, help='Use an explicit prebuilt XML and its embedded limits instead of regenerating')
    parser.add_argument('--island-colors', action='store_true', help='Debug constraint islands instead of displaying robot materials')
    parser.add_argument('--telemetry', type=Path, help='Optional live model-state JSON')
    args = parser.parse_args()
    filename = 'lekiwi_base.xml' if args.variant == 'base' else f'lekiwi_{args.variant}_oakd_s2.xml'
    # Keep generated files alive for the viewer lifetime; mesh paths are absolute.
    generated = TemporaryDirectory(prefix='lekiwi_preview_') if args.model is None else None
    path = (args.model.resolve() if args.model else
            build(args.variant, Path(generated.name) / filename, absolute=True, scene=args.scene, control_dir=args.control_package,
                  description_dir=args.description_package, pt_package=args.pt_package))
    model = mujoco.MjModel.from_xml_path(str(path))
    simulation = Simulation(model)
    simulation.reset()
    data = simulation.data
    keyboard = KeyboardControl(model, simulation.control)
    keys = HeldKeys()
    trail = Trail()
    paused = False
    frames = 0
    control_labels, control_values = ((CONTROL_LABELS_PAYLOAD, CONTROL_VALUES_PAYLOAD)
                                       if simulation.robot.payload else
                                       (CONTROL_LABELS_BASE, CONTROL_VALUES_BASE))
    if args.telemetry:
        args.telemetry.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.viewer.launch_passive(model, data, key_callback=keys.bootstrap) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = data.xpos[simulation.robot.base] + [0, 0, .18]
            viewer.cam.distance = 1.25
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
            viewer.opt.geomgroup[3] = 0  # Hide collision proxies, keep CAD visuals.
            # The payload's blind arc is baked out of the model itself (mask_payload_lidar,
            # applied at build time), so MuJoCo's own native rendering is already correct.
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = True
        print(f'Native viewer ready: {path}', flush=True)
        for label, value in zip(control_labels.split('\n'), control_values.split('\n')):
            print(f'{label}: {value}', flush=True)
        steps = max(1, round(1 / (60 * model.opt.timestep)))
        period = steps * model.opt.timestep
        while viewer.is_running():
            start = time.monotonic()
            with viewer.lock():
                # Island debug coloring strips robot materials; keep normal colors by default.
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND] = args.island_colors
                while not keys.events.empty():
                    key = keys.events.get()
                    if key in (ord('P'), ord('p')):
                        paused = not paused
                        keys.clear()
                        keyboard.stop(data)
                        simulation.stop()
                    elif key in (ord('X'), ord('x')):
                        keyboard.set_estop(data, False)  # release first, or reset()'s settle step can't move the joints
                        keyboard.stop(data)
                        simulation.stop()
                        keys.clear()
                        simulation.reset()
                        trail.clear()
                    elif key == glfw.KEY_SPACE:
                        keyboard.set_estop(data, not keyboard.estop)
                        keys.clear()
                    elif key == 'focus_lost':
                        keyboard.stop(data)
                        simulation.stop()
                if not paused:
                    held = keys.snapshot()
                    for _ in range(steps):
                        keyboard.update(data, held, model.opt.timestep)
                        simulation.step()
                trail.draw(viewer.user_scn, data.xpos[simulation.robot.base])
                info = simulation.info()
                scan = simulation.scan()
                state = {'model': str(path), 'sim_time': info['sim_time'],
                         'paused': paused, 'estop': keyboard.estop, 'contacts': info['contacts'],
                         'island_colors': bool(viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND]),
                         'warnings': info['warning_count'],
                         'held_keys': sorted(keys.snapshot()),
                         'actuators': {model.actuator(i).name: float(data.ctrl[i])
                                       for i in range(model.nu)},
                         'scan': None if scan is None else scan.tolist()}
                status = 'E-STOP' if keyboard.estop else ('Paused' if paused else 'Running')
            viewer.set_texts([
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                 STATUS_LABELS, f'{path.stem}\n{data.time:.1f} s\n{status}'),
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                 control_labels, control_values),
            ])
            viewer.sync()
            if args.telemetry and frames % 30 == 0:
                args.telemetry.write_text(json.dumps(state, indent=2))
            frames += 1
            time.sleep(max(0, period - (time.monotonic() - start)))


if __name__ == '__main__':
    main()
