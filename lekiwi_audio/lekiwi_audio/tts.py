#!/usr/bin/env python3
"""Kokoro-82M TTS synthesis for lekiwi_audio - used only at build time, by render_phrases.py.

Voice model files are bundled under this package's own share directory (voices/kokoro/).
"""

from __future__ import annotations

import tempfile
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import onnxruntime as ort
import soundfile as sf
from ament_index_python.packages import get_package_share_directory
from kokoro_onnx import Kokoro


def default_voice_paths() -> tuple[str, str]:
    """Return (model_path, voices_path) for the bundled Kokoro files."""
    voice_dir = Path(get_package_share_directory('lekiwi_audio')) / 'voices' / 'kokoro'
    return (
        str(voice_dir / 'kokoro-v1.0.int8.onnx'),
        str(voice_dir / 'voices-v1.0.bin'),
    )


@contextmanager
def _capped_onnx_threads(n: int):
    """Cap onnxruntime's thread pool for the duration of the block.

    kokoro-onnx constructs its InferenceSession with no sess_options to intercept
    (`rt.InferenceSession(model_path, providers=providers)`), so this patches
    InferenceSession.__init__ itself, injecting a capped sess_options whenever the caller
    didn't supply one. Only matters for the build-time synthesis burst (nothing imports
    Kokoro at runtime), keeping build-time CPU usage from fighting a concurrent colcon build.
    """
    orig_init = ort.InferenceSession.__init__

    def capped_init(self, *args, **kwargs):
        if kwargs.get('sess_options') is None:
            so = ort.SessionOptions()
            so.intra_op_num_threads = n
            so.inter_op_num_threads = 1
            kwargs['sess_options'] = so
        orig_init(self, *args, **kwargs)

    ort.InferenceSession.__init__ = capped_init
    try:
        yield
    finally:
        ort.InferenceSession.__init__ = orig_init


class Synthesizer:
    """Renders text to a gain-boosted WAV file using a bundled Kokoro-82M voice."""

    def __init__(
        self,
        model_path: str | None = None,
        voices_path: str | None = None,
        voice: str = 'af_kore',
        lang: str = 'en-us',
        speed: float = 1.0,
        volume_boost_db: float = 15.0,
        onnx_threads: int = 3,
    ) -> None:
        """Load the Kokoro model/voices, capping onnxruntime's thread pool while doing so."""
        if model_path is None or voices_path is None:
            model_path, voices_path = default_voice_paths()
        with _capped_onnx_threads(onnx_threads):
            self._kokoro = Kokoro(model_path, voices_path)
        self._voice = voice
        self._lang = lang
        self._speed = speed
        self._boost_db = volume_boost_db

    def synthesize(self, text: str) -> str:
        """Return the path to a gain-boosted WAV file ready to play.

        Boost is applied in-process via a numpy gain multiply, not an ffmpeg subprocess.
        """
        samples, sample_rate = self._kokoro.create(
            text, voice=self._voice, speed=self._speed, lang=self._lang
        )

        if self._boost_db > 0:
            gain = 10 ** (self._boost_db / 20)
            samples = np.clip(samples.astype(np.float32) * gain, -1.0, 1.0)

        out_path = tempfile.mktemp(suffix='.wav')
        sf.write(out_path, samples, sample_rate, subtype='PCM_16')
        return out_path
