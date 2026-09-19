"""Held-key motion, release, focus and payload-limit checks without a GUI."""
from pathlib import Path
import sys

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')
glfw = pytest.importorskip('glfw')
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from lekiwi_mujoco.mujoco_preview import HeldKeys, KeyboardControl  # noqa: E402


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(PACKAGE / 'mjcf/lekiwi_pt101_oakd_s2.xml'))


@pytest.mark.parametrize('key,shift,command', [
    (glfw.KEY_UP, False, [.2, 0, 0]), (glfw.KEY_DOWN, False, [-.2, 0, 0]),
    (glfw.KEY_LEFT, False, [0, .17, 0]), (glfw.KEY_RIGHT, False, [0, -.17, 0]),
    (glfw.KEY_LEFT, True, [0, 0, .68]), (glfw.KEY_RIGHT, True, [0, 0, -.68])])
def test_hold_for_seconds_without_repeat_then_release(model, key, shift, command):
    data = mujoco.MjData(model)
    control, keys = KeyboardControl(model), HeldKeys()
    keys.on_key(None, key, 0, glfw.PRESS, 0)
    if shift:
        keys.on_key(None, glfw.KEY_LEFT_SHIFT, 0, glfw.PRESS, 0)
    for _ in range(300):
        control.update(data, keys.snapshot(), .016)
        assert np.allclose(data.ctrl[control.wheels], control.control.wheel_speeds(command), atol=1e-5)
    keys.on_key(None, key, 0, glfw.RELEASE, 0)
    control.update(data, keys.snapshot(), .016)
    assert np.all(data.ctrl[control.wheels] == 0)


def test_shift_changes_while_arrow_stays_held(model):
    data = mujoco.MjData(model)
    control, keys = KeyboardControl(model), HeldKeys()
    keys.record(glfw.KEY_LEFT, glfw.PRESS)
    for shift_action, expected in [(glfw.RELEASE, [0, .17, 0]),
                                    (glfw.PRESS, [0, 0, .68]),
                                    (glfw.RELEASE, [0, .17, 0])]:
        keys.record(glfw.KEY_RIGHT_SHIFT, shift_action)
        control.update(data, keys.snapshot(), .016)
        assert np.allclose(data.ctrl[control.wheels], control.control.wheel_speeds(expected))


def test_focus_loss_stops_and_other_events_forward(model):
    data = mujoco.MjData(model)
    control, keys = KeyboardControl(model), HeldKeys()
    forwarded = []
    keys.previous_key = lambda *args: forwarded.append(args)
    keys.record(glfw.KEY_UP, glfw.PRESS)
    control.update(data, keys.snapshot(), .016)
    keys.on_key(None, ord('W'), 0, glfw.PRESS, 0)
    assert forwarded[0][1] == ord('W')
    keys.on_focus(None, False)
    control.update(data, keys.snapshot(), .016)
    assert np.all(data.ctrl[control.wheels] == 0)
    assert keys.events.get() == 'focus_lost'


@pytest.mark.parametrize('key,name,sign', [(glfw.KEY_LEFT, 'shoulder_pan_joint', 1),
    (glfw.KEY_RIGHT, 'shoulder_pan_joint', -1), (glfw.KEY_DOWN, 'tilt_joint', -1),
    (glfw.KEY_UP, 'tilt_joint', 1)])
def test_payload_continuous_motion_release_and_limits(model, key, name, sign):
    for alt in [glfw.KEY_LEFT_ALT, glfw.KEY_RIGHT_ALT]:
        data = mujoco.MjData(model)
        control = KeyboardControl(model)
        actuator = model.actuator(name).id
        for _ in range(6):
            control.update(data, {key, alt}, 1 / 60)
        assert data.ctrl[actuator] == pytest.approx(sign * control.payload_limits[name] * .1)
        control.update(data, set(), 1.)
        assert data.ctrl[actuator] == pytest.approx(sign * control.payload_limits[name] * .1)
        for _ in range(600):
            control.update(data, {key, alt}, 1 / 60)
        low, high = model.actuator_ctrlrange[actuator]
        assert data.ctrl[actuator] == pytest.approx(high if sign > 0 else low)


def test_reset_pause_are_edges_not_repeat():
    keys = HeldKeys()
    for key in [ord('X'), ord('P')]:
        for action in [glfw.PRESS, glfw.REPEAT, glfw.RELEASE]:
            keys.on_key(None, key, 0, action, 0)
        assert keys.events.get_nowait() == key
        assert keys.events.empty()


def test_bootstrap_installs_release_and_focus_callbacks(monkeypatch):
    keys, window = HeldKeys(), object()
    installed = {}
    monkeypatch.setattr(glfw, 'get_current_context', lambda: window)
    monkeypatch.setattr(glfw, 'get_key', lambda w, k: glfw.PRESS if k == glfw.KEY_UP else glfw.RELEASE)
    monkeypatch.setattr(keys, 'install_callbacks',
                        lambda w: installed.update(key=keys.on_key, focus=keys.on_focus))
    keys.bootstrap(glfw.KEY_UP)
    assert keys.snapshot() == {glfw.KEY_UP}
    installed['key'](window, glfw.KEY_UP, 0, glfw.RELEASE, 0)
    assert not keys.snapshot()


def test_all_key_combinations_respect_body_and_wheel_limits(model):
    from itertools import combinations
    control = KeyboardControl(model)
    keys = [glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT,
            glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT]
    for count in range(len(keys) + 1):
        for held in combinations(keys, count):
            data = mujoco.MjData(model)
            control.update(data, held, .016)
            rates = data.ctrl[control.wheels]
            body = np.linalg.solve(control.kinematics, rates)
            assert np.all(np.abs(rates) <= control.wheel_limits + 1e-10)
            assert np.all(np.abs(body) <= control.base_limits + 1e-10)


def test_slider_commands_are_limited_and_payload_targets_slew(model):
    control = KeyboardControl(model)
    data = mujoco.MjData(model)
    data.ctrl[control.wheels] = [100, -100, 50]
    data.ctrl[model.actuator('tilt_joint').id] = 1.5
    previous = 0.
    for _ in range(20):
        control.enforce_limits(data, .002)
        assert np.max(np.abs(data.ctrl[control.wheels]) / control.wheel_limits) <= 1. + 1e-10
        body = np.linalg.solve(control.kinematics, data.ctrl[control.wheels])
        assert np.all(np.abs(body) <= control.base_limits + 1e-10)
        assert abs(data.ctrl[model.actuator('tilt_joint').id] - previous) <= control.payload_limits['tilt_joint'] * .002 + 1e-10
        previous = data.ctrl[model.actuator('tilt_joint').id]
        data.ctrl[model.actuator('tilt_joint').id] = 1.5


def test_motor_limit_is_baked_into_xml(model):
    expected = 2890 * 2 * np.pi / 4096
    for name in ['left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint']:
        actuator = model.actuator(name).id
        assert model.actuator_ctrllimited[actuator]
        assert np.allclose(model.actuator_ctrlrange[actuator], [-expected, expected])
    assert model.numeric('velocity_limit_tilt_joint').data[0] == pytest.approx(expected, abs=1e-5)


def test_alt_priority_and_modifier_switch_while_arrow_held(model):
    data = mujoco.MjData(model)
    control = KeyboardControl(model)
    arrow = {glfw.KEY_LEFT}
    control.update(data, arrow, .01)
    assert np.any(data.ctrl[control.wheels])
    control.update(data, arrow | {glfw.KEY_LEFT_SHIFT, glfw.KEY_LEFT_ALT}, .01)
    assert np.all(data.ctrl[control.wheels] == 0)
    pan = model.actuator('shoulder_pan_joint').id
    angle = data.ctrl[pan]
    assert angle == pytest.approx(control.payload_limits['shoulder_pan_joint'] * .01)
    control.update(data, arrow | {glfw.KEY_LEFT_SHIFT}, .01)
    assert np.allclose(data.ctrl[control.wheels], control.control.wheel_speeds([0, 0, .68]))
    assert data.ctrl[pan] == angle


def test_old_payload_letters_are_only_viewer_shortcuts(model):
    data = mujoco.MjData(model)
    control, keys = KeyboardControl(model), HeldKeys()
    forwarded = []
    keys.previous_key = lambda w, k, s, a, m: forwarded.append(k)
    for key in map(ord, 'RFTG'):
        keys.on_key(None, key, 0, glfw.PRESS, 0)
    control.update(data, keys.snapshot(), .1)
    assert np.all(data.ctrl == 0)
    assert forwarded == list(map(ord, 'RFTG'))
