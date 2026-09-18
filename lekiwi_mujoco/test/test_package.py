"""Package boundaries and explicit data paths without ROS discovery."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lekiwi_mujoco import build_mujoco_models as builder  # noqa: E402


def test_explicit_paths_do_not_require_package_discovery(monkeypatch, tmp_path):
    def unavailable(name):
        raise AssertionError(f'Unexpected package lookup: {name}')
    monkeypatch.setattr(builder, 'package_share', unavailable)
    result = builder.build('base', tmp_path / 'model.xml', absolute=True,
                           control_dir=ROOT.parent / 'lekiwi_control',
                           description_dir=ROOT.parent / 'lekiwi_description')
    assert result.is_file()


def test_dependencies_do_not_point_back_to_control():
    for root in (ROOT, ROOT.parent / 'lekiwi_description'):
        package = ET.parse(root / 'package.xml').getroot()
        assert 'lekiwi_control' not in [entry.text for entry in package if 'depend' in entry.tag]
    assert ET.parse(ROOT / 'package.xml').findtext('export/build_type') == 'ament_python'
