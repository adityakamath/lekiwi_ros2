#!/usr/bin/env python3
"""Unit tests for render_phrases.py's build-time rendering/pruning.

Stubs lekiwi_audio.tts.Synthesizer so these run without the real Kokoro model
files (116 MB, downloaded on first use) or onnxruntime inference.
"""

import os
import tempfile
from pathlib import Path

import yaml
from lekiwi_audio import render_phrases


class _FakeSynthesizer:
    """Mimics Synthesizer.synthesize()'s real contract: returns a fresh tempfile path."""

    def __init__(self, model_path=None, voices_path=None):
        pass

    def synthesize(self, text):
        fd, path = tempfile.mkstemp(suffix='.wav')
        os.close(fd)
        Path(path).write_bytes(text.encode('utf-8'))
        return path


def _write_phrases_yaml(tmp_path, phrases):
    """Write a minimal phrases.yaml with one service/state/outcome per phrase."""
    data = {'services': {
        f'/svc{i}': {'true': {'success': phrase}} for i, phrase in enumerate(phrases)
    }}
    path = tmp_path / 'phrases.yaml'
    path.write_text(yaml.dump(data))
    return path


class TestRenderMissing:

    def test_renders_missing_phrase_and_cleans_up_tempfile(self, tmp_path, monkeypatch):
        monkeypatch.setattr(render_phrases, '_ensure_voice_files', lambda voices_dir: None)
        monkeypatch.setattr('lekiwi_audio.tts.Synthesizer', _FakeSynthesizer)

        phrases_path = _write_phrases_yaml(tmp_path, ['Hello there'])
        sounds_dir = tmp_path / 'sounds'

        # Patch tempfile.mkstemp to confirm no tempfile is left behind after rendering
        # (the bug: synthesize()'s tempfile was never unlinked).
        created = []
        orig_mkstemp = tempfile.mkstemp

        def tracking_mkstemp(*args, **kwargs):
            fd, path = orig_mkstemp(*args, **kwargs)
            created.append(path)
            return fd, path

        monkeypatch.setattr(tempfile, 'mkstemp', tracking_mkstemp)

        render_phrases.render_missing(phrases_path=phrases_path, sounds_dir=sounds_dir)

        expected_wav = sounds_dir / render_phrases.phrase_filename('Hello there')
        assert expected_wav.exists(), 'rendered phrase should land in sounds_dir'
        assert created, 'synthesize() should have been called via a tempfile'
        for temp_path in created:
            assert not Path(temp_path).exists(), \
                f'temp file {temp_path} was not cleaned up after rendering'

    def test_skips_rendering_when_already_present(self, tmp_path, monkeypatch):
        calls = []
        monkeypatch.setattr(render_phrases, '_ensure_voice_files', lambda voices_dir: None)

        class _CountingSynthesizer(_FakeSynthesizer):
            def synthesize(self, text):
                calls.append(text)
                return super().synthesize(text)

        monkeypatch.setattr('lekiwi_audio.tts.Synthesizer', _CountingSynthesizer)

        phrases_path = _write_phrases_yaml(tmp_path, ['Already rendered'])
        sounds_dir = tmp_path / 'sounds'
        sounds_dir.mkdir()
        (sounds_dir / render_phrases.phrase_filename('Already rendered')).write_bytes(b'x')

        render_phrases.render_missing(phrases_path=phrases_path, sounds_dir=sounds_dir)

        assert calls == [], 'already-rendered phrases must not be re-synthesized'


class TestPruneOrphans:

    def test_removes_wav_with_no_matching_phrase(self, tmp_path):
        sounds_dir = tmp_path / 'sounds'
        sounds_dir.mkdir()
        orphan = sounds_dir / 'deadbeefdeadbe.wav'
        orphan.write_bytes(b'x')

        render_phrases._prune_orphans(phrases=set(), sounds_dir=sounds_dir)

        assert not orphan.exists()

    def test_keeps_wav_matching_a_current_phrase(self, tmp_path):
        sounds_dir = tmp_path / 'sounds'
        sounds_dir.mkdir()
        kept_name = render_phrases.phrase_filename('Keep me')
        kept = sounds_dir / kept_name
        kept.write_bytes(b'x')

        render_phrases._prune_orphans(phrases={'Keep me'}, sounds_dir=sounds_dir)

        assert kept.exists()

    def test_cleans_stale_colcon_build_and_install_symlinks(self, tmp_path):
        # Mimic a colcon workspace (src/.../sounds + sibling build/install trees) with a
        # leftover --symlink-install farm from before a phrase was renamed/removed.
        ws_root = tmp_path / 'ws'
        sounds_dir = ws_root / 'src' / 'lekiwi_ros2' / 'lekiwi_audio' / 'sounds'
        sounds_dir.mkdir(parents=True)
        orphan_name = 'deadbeefdeadbe.wav'
        orphan = sounds_dir / orphan_name
        orphan.write_bytes(b'x')

        build_sounds = ws_root / 'build' / 'lekiwi_audio' / 'sounds'
        build_sounds.mkdir(parents=True)
        (build_sounds / orphan_name).symlink_to(orphan)

        install_sounds = ws_root / 'install' / 'lekiwi_audio' / 'share' / 'lekiwi_audio' / 'sounds'
        install_sounds.mkdir(parents=True)
        (install_sounds / orphan_name).symlink_to(orphan)

        render_phrases._prune_orphans(phrases=set(), sounds_dir=sounds_dir)

        assert not orphan.exists()
        assert not (build_sounds / orphan_name).is_symlink()
        assert not (install_sounds / orphan_name).is_symlink()

    def test_no_workspace_found_is_a_noop(self, tmp_path):
        # tmp_path has no 'src' ancestor - should prune the source orphan without erroring.
        sounds_dir = tmp_path / 'sounds'
        sounds_dir.mkdir()
        orphan = sounds_dir / 'deadbeefdeadbe.wav'
        orphan.write_bytes(b'x')

        render_phrases._prune_orphans(phrases=set(), sounds_dir=sounds_dir)

        assert not orphan.exists()
