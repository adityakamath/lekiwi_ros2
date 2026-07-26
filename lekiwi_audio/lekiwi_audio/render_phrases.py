#!/usr/bin/env python3
"""Build-time tool: keep sounds/ in sync with config/phrases.yaml.

Renders every new/changed phrase to a WAV under sounds/ via Kokoro (tts.Synthesizer) and prunes
orphaned files, so a colcon build always matches the current config. Called automatically from
setup.py, before `data_files` globs sounds/. Filenames are a short hash of the phrase text, so
rendering is incremental (unchanged phrases already have a file) and no separate manifest is
needed - indicator_node.py recomputes the same hash at runtime to find the matching file.
"""

from __future__ import annotations

import hashlib
import sys
import urllib.request
from pathlib import Path

import yaml

_KOKORO_RELEASE_URL = 'https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0'
_KOKORO_VOICE_FILES = ('kokoro-v1.0.int8.onnx', 'voices-v1.0.bin')


def phrase_filename(phrase: str) -> str:
    """Hash a phrase to its rendered filename."""
    return hashlib.sha1(phrase.encode('utf-8')).hexdigest()[:12] + '.wav'


def _ensure_voice_files(voices_dir: Path) -> None:
    """Download the Kokoro model files from their upstream GitHub release if not already cached.

    Not committed to this repo (116 MB combined) - fetched here instead, only when a phrase
    actually needs rendering and no local copy exists yet.
    """
    voices_dir.mkdir(parents=True, exist_ok=True)
    for filename in _KOKORO_VOICE_FILES:
        dest = voices_dir / filename
        if dest.exists():
            continue
        url = f'{_KOKORO_RELEASE_URL}/{filename}'
        print(f'lekiwi_audio: downloading {filename} from {url} ...', file=sys.stderr)
        urllib.request.urlretrieve(url, dest)


def collect_phrases(phrases_path: Path) -> set[str]:
    """Collect every distinct spoken phrase referenced in phrases.yaml."""
    with open(phrases_path) as f:
        data = yaml.safe_load(f)
    phrases = set()
    for state_map in data['services'].values():
        for outcome_map in state_map.values():
            phrases.update(outcome_map.values())
    phrases.update(data.get('nav_goal', {}).values())
    return phrases


def _prune_orphans(phrases: set[str], sounds_dir: Path) -> None:
    """Delete any sounds/*.wav that no longer matches a phrase still in phrases.yaml."""
    wanted = {phrase_filename(p) for p in phrases}
    for wav_path in sounds_dir.glob('*.wav'):
        if wav_path.name not in wanted:
            wav_path.unlink()
            print(f'  removed orphaned {wav_path.name}', file=sys.stderr)


def render_missing(phrases_path: Path | None = None, sounds_dir: Path | None = None) -> None:
    """Render any phrase in phrases.yaml that doesn't already have a WAV, then prune orphans."""
    here = Path(__file__).resolve().parent.parent  # lekiwi_audio/ package root
    phrases_path = phrases_path or here / 'config' / 'phrases.yaml'
    sounds_dir = sounds_dir or here / 'sounds'
    sounds_dir.mkdir(parents=True, exist_ok=True)

    phrases = collect_phrases(phrases_path)
    missing = {
        p: phrase_filename(p) for p in phrases if not (sounds_dir / phrase_filename(p)).exists()
    }

    if not missing:
        print(
            f'lekiwi_audio: all {len(phrases)} phrase(s) already rendered, nothing to do.',
            file=sys.stderr,
        )
    else:
        print(
            f'lekiwi_audio: rendering {len(missing)} new/changed phrase(s) '
            f'of {len(phrases)} total...',
            file=sys.stderr,
        )
        # Deferred: only needed when there's something to render, avoiding the
        # onnxruntime/kokoro import cost on every build where nothing changed.
        from lekiwi_audio.tts import Synthesizer

        voices_dir = here / 'voices' / 'kokoro'
        _ensure_voice_files(voices_dir)
        synth = Synthesizer(
            model_path=str(voices_dir / 'kokoro-v1.0.int8.onnx'),
            voices_path=str(voices_dir / 'voices-v1.0.bin'),
        )
        for phrase, filename in sorted(missing.items()):
            wav_path = synth.synthesize(phrase)
            (sounds_dir / filename).write_bytes(Path(wav_path).read_bytes())
            print(f'  {filename}  <-  {phrase!r}', file=sys.stderr)

        print('lekiwi_audio: done rendering phrases.', file=sys.stderr)

    _prune_orphans(phrases, sounds_dir)


def main() -> None:
    """Render missing phrases and prune orphans, using argv overrides if given."""
    phrases_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    sounds_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    render_missing(phrases_path, sounds_dir)


if __name__ == '__main__':
    main()
