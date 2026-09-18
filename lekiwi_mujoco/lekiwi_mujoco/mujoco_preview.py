#!/usr/bin/env python3
"""Inspect an existing LeKiwi MJCF in MuJoCo's passive native viewer.

On macOS run with mjpython. No waypoint follower or ROS controller is included.
"""
import argparse
import json
from pathlib import Path
from queue import SimpleQueue
from threading import Lock
import time
from tempfile import TemporaryDirectory

from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.instance import RobotInstance
from lekiwi_mujoco.simulation import RobotControl, Simulation

import glfw
import mujoco
import mujoco.viewer

from lekiwi_mujoco.paths import package_share

PACKAGE = package_share('lekiwi_mujoco')


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

    def enforce_limits(self, data, dt):
        self.control.apply(data, data.ctrl.copy(), dt)

    def reset_targets(self, data):
        self.control.reset_targets(data)

    def stop(self, data):
        data.ctrl[self.wheels] = 0
        self.active = False

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
             *map(ord, 'XP')}

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
        if action == glfw.PRESS and key in (ord('P'), ord('X')):
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


CONTROLS = ('Up/Down: forward/back | Left/Right: strafe | Shift+Left/Right: rotate\n'
            'Alt/Option+Left/Right: pan | Alt/Option+Up/Down: tilt | X: reset | P: pause')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['base', 'pt100', 'pt101'], default='pt101')
    parser.add_argument('--control-package', type=Path)
    parser.add_argument('--description-package', type=Path)
    parser.add_argument('--pt-package', type=Path)
    parser.add_argument('--robot-name', default='lekiwi')
    parser.add_argument('--prefix', default='')
    parser.add_argument('--spawn', type=float, nargs=3, default=[0, 0, 0], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--spawn-quat', type=float, nargs=4, default=[1, 0, 0, 0])
    parser.add_argument('--scene', default='flat', help='flat, none, or scene MJCF path')
    parser.add_argument('--model', type=Path, help='Use an explicit prebuilt XML and its embedded limits instead of regenerating')
    parser.add_argument('--island-colors', action='store_true', help='Debug constraint islands instead of displaying robot materials')
    parser.add_argument('--telemetry', type=Path, help='Optional live model-state JSON')
    args = parser.parse_args()
    instance = RobotInstance(args.robot_name, args.prefix, tuple(args.spawn), tuple(args.spawn_quat))
    if args.model and (args.spawn != [0, 0, 0] or args.spawn_quat != [1, 0, 0, 0]):
        parser.error("Spawn options apply to generated models; prebuilt models retain their compiled spawn")
    filename = 'lekiwi_base.xml' if args.variant == 'base' else f'lekiwi_{args.variant}_oakd_s2.xml'
    # Keep generated files alive for the viewer lifetime; mesh paths are absolute.
    generated = TemporaryDirectory(prefix='lekiwi_preview_') if args.model is None else None
    path = (args.model.resolve() if args.model else
            build(args.variant, Path(generated.name) / filename, absolute=True, scene=args.scene, control_dir=args.control_package,
                  description_dir=args.description_package, pt_package=args.pt_package, instance=instance))
    model = mujoco.MjModel.from_xml_path(str(path))
    simulation = Simulation(model, instance)
    simulation.reset()
    data = simulation.data
    keyboard = KeyboardControl(model, simulation.control)
    keys = HeldKeys()
    paused = False
    frames = 0
    if args.telemetry:
        args.telemetry.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.viewer.launch_passive(model, data, key_callback=keys.bootstrap) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = data.xpos[simulation.robot.base] + [0, 0, .18]
            viewer.cam.distance = 1.25
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
            viewer.opt.geomgroup[3] = 0  # Hide collision proxies, keep CAD visuals.
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        print(f'Native viewer ready: {path}', flush=True)
        print(CONTROLS, flush=True)
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
                        keyboard.stop(data)
                        simulation.stop()
                        keys.clear()
                        simulation.reset()
                    elif key == 'focus_lost':
                        keyboard.stop(data)
                        simulation.stop()
                if not paused:
                    held = keys.snapshot()
                    for _ in range(steps):
                        keyboard.update(data, held, model.opt.timestep)
                        simulation.step()
                info = simulation.info()
                state = {'model': str(path), 'sim_time': info['sim_time'],
                         'paused': paused, 'contacts': info['contacts'],
                         'island_colors': bool(viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND]),
                         'warnings': info['warning_count'],
                         'held_keys': sorted(keys.snapshot()),
                         'actuators': {model.actuator(i).name: float(data.ctrl[i])
                                       for i in range(model.nu)}}
            viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,
                               mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                               f'{path.stem} | {data.time:.2f} s | {"PAUSED" if paused else "RUNNING"}\n'
                               + CONTROLS, '')])
            viewer.sync()
            if args.telemetry and frames % 30 == 0:
                args.telemetry.write_text(json.dumps(state, indent=2))
            frames += 1
            time.sleep(max(0, period - (time.monotonic() - start)))


if __name__ == '__main__':
    main()
