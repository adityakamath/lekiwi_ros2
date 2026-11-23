#!/usr/bin/env python3
"""Node-level unit tests for MapSaverNode's save-generation guard: a save/serialize callback
whose gen no longer matches _save_generation must no-op instead of racing a newer save."""

from unittest.mock import MagicMock

import pytest

from lekiwi_navigation.map_saver_node import MapSaverNode

# rclpy is initialized by conftest.py (session-scoped, idempotent).


@pytest.fixture
def node():
    n = MapSaverNode()
    n._save_client = MagicMock()
    n._serialize_client = MagicMock()
    yield n
    n.destroy_node()


class TestSaveWatchdogGuard:

    def test_stale_gen_leaves_saving_state_untouched(self, node):
        node._save_generation = 2
        node._saving = True
        node._save_watchdog = MagicMock()
        node._on_save_watchdog(gen=1)
        assert node._saving is True
        node._save_watchdog.cancel.assert_not_called()

    def test_current_gen_clears_saving_state(self, node):
        node._save_generation = 1
        node._saving = True
        node._save_watchdog = MagicMock()
        node._on_save_watchdog(gen=1)
        assert node._saving is False
        assert node._save_watchdog is None


class TestOnSaveDoneGuard:

    def test_stale_gen_ignores_result_and_does_not_serialize(self, node):
        node._save_generation = 2
        future = MagicMock()
        node._on_save_done(future, gen=1)
        future.result.assert_not_called()
        node._serialize_client.call_async.assert_not_called()

    def test_current_gen_processes_result_and_serializes(self, node, tmp_path):
        node._save_generation = 1
        node._map_name = str(tmp_path / 'map')
        node._create_placeholder_filters = MagicMock()
        future = MagicMock()
        future.result.return_value = MagicMock(result=0)
        node._on_save_done(future, gen=1)
        future.result.assert_called_once()
        node._create_placeholder_filters.assert_called_once()
        node._serialize_client.call_async.assert_called_once()


class TestOnSerializeDoneGuard:

    def test_stale_gen_skips_starting_pose_and_clear(self, node):
        node._save_generation = 2
        node._save_starting_pose = MagicMock()
        node._saving = True
        future = MagicMock()
        node._on_serialize_done(future, gen=1)
        future.result.assert_not_called()
        node._save_starting_pose.assert_not_called()
        assert node._saving is True  # _clear_saving() must not have run either

    def test_current_gen_saves_starting_pose_and_clears(self, node):
        node._save_generation = 1
        node._save_starting_pose = MagicMock()
        node._saving = True
        node._save_watchdog = None
        future = MagicMock()
        future.result.return_value = MagicMock(result=0)
        node._on_serialize_done(future, gen=1)
        node._save_starting_pose.assert_called_once()
        assert node._saving is False


class TestStartingPoseFrames:

    def test_frames_default_to_the_unprefixed_names(self, node):
        assert node.get_parameter('map_frame').value == 'map'
        assert node.get_parameter('base_frame').value == 'base_footprint'

    @pytest.mark.parametrize('prefix', ['', 'robot1/'])
    def test_starting_pose_is_looked_up_between_the_configured_frames(self, node, tmp_path, prefix):
        from geometry_msgs.msg import TransformStamped
        from unittest.mock import patch
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('map_frame', value=f'{prefix}map'),
                             Parameter('base_frame', value=f'{prefix}base_footprint')])
        node._map_name = str(tmp_path / 'map')
        pose = TransformStamped()
        pose.transform.translation.x, pose.transform.translation.y = 1.0, 2.0
        pose.transform.rotation.w = 1.0
        with patch('lekiwi_navigation.map_saver_node.lookup_transform', return_value=pose) as lookup:
            node._save_starting_pose()
        lookup.assert_called_once_with(f'{prefix}map', f'{prefix}base_footprint')
        assert (tmp_path / 'starting_pose.yaml').read_text().startswith('theta: 0.0')
