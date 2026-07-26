#!/usr/bin/env python3
"""Tests for lekiwi_audio's phrases.yaml structure and rendered audio.

Validates the config file's shape without starting any nodes, and cross-checks
that every phrase it references actually has a rendered WAV under sounds/ -
catching exactly the kind of bug a phrase edit without a rebuild would cause.
"""

import hashlib
import os

import yaml

# Resolve source package root (works with both symlink-install and regular install).
_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_PHRASES_PATH = os.path.join(_PKG_SRC, 'config', 'phrases.yaml')
_SOUNDS_DIR = os.path.join(_PKG_SRC, 'sounds')


def _phrase_filename(phrase):
    # Must match render_phrases.phrase_filename() / indicator_node._phrase_filename().
    return hashlib.sha1(phrase.encode('utf-8')).hexdigest()[:12] + '.wav'


def _load_data():
    with open(_PHRASES_PATH) as f:
        return yaml.safe_load(f)


def _load_phrases():
    return _load_data()['services']


def _load_nav_goal_phrases():
    return _load_data().get('nav_goal', {})


class TestPhrasesConfig:

    def setup_method(self):
        self.services = _load_phrases()

    def test_top_level_is_nonempty_dict(self):
        assert isinstance(self.services, dict)
        assert len(self.services) > 0

    def test_service_names_start_with_slash(self):
        for service in self.services:
            assert service.startswith('/'), f'{service!r} should be a fully-qualified service name'

    def test_request_keys_are_true_or_false(self):
        for service, state_map in self.services.items():
            assert set(state_map).issubset({'true', 'false'}), (
                f'{service}: unexpected keys {set(state_map) - {"true", "false"}}'
            )

    def test_outcome_keys_are_success_or_failure(self):
        for service, state_map in self.services.items():
            for state, outcome_map in state_map.items():
                unexpected = set(outcome_map) - {'success', 'failure'}
                assert not unexpected, f'{service}.{state}: unexpected keys {unexpected}'

    def test_all_phrases_are_nonempty_strings(self):
        for service, state_map in self.services.items():
            for state, outcome_map in state_map.items():
                for outcome, phrase in outcome_map.items():
                    assert isinstance(phrase, str) and phrase.strip(), (
                        f'{service}.{state}.{outcome}: phrase must be a non-empty string'
                    )


class TestNavGoalPhrases:

    def setup_method(self):
        self.nav_goal = _load_nav_goal_phrases()

    def test_keys_are_success_or_failure(self):
        assert set(self.nav_goal).issubset({'success', 'failure'}), (
            f'nav_goal: unexpected keys {set(self.nav_goal) - {"success", "failure"}}'
        )

    def test_all_phrases_are_nonempty_strings(self):
        for outcome, phrase in self.nav_goal.items():
            assert isinstance(phrase, str) and phrase.strip(), (
                f'nav_goal.{outcome}: phrase must be a non-empty string'
            )


class TestRenderedAudio:

    def setup_method(self):
        self.services = _load_phrases()
        self.phrases = {
            phrase
            for state_map in self.services.values()
            for outcome_map in state_map.values()
            for phrase in outcome_map.values()
        }
        self.phrases.update(_load_nav_goal_phrases().values())

    def test_every_phrase_has_a_rendered_file(self):
        missing = {p: _phrase_filename(p) for p in self.phrases if not os.path.exists(
            os.path.join(_SOUNDS_DIR, _phrase_filename(p))
        )}
        assert not missing, (
            f'{len(missing)} phrase(s) have no rendered audio - run colcon build: {missing}'
        )

    def test_no_orphaned_sound_files(self):
        wanted = {_phrase_filename(p) for p in self.phrases}
        on_disk = set(os.listdir(_SOUNDS_DIR))
        orphaned = on_disk - wanted
        assert not orphaned, (
            f'{len(orphaned)} sound file(s) no longer match any phrase - '
            f'run colcon build: {orphaned}'
        )
