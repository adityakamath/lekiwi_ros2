#!/usr/bin/env python3
"""Measure open-loop body motion in the current models."""
import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np

from lekiwi_mujoco.simulation import Simulation
from lekiwi_mujoco.build_mujoco_models import build


def benchmark(directory):
    rows = []
    commands = [('forward', [.15, 0, 0]), ('reverse', [-.15, 0, 0]),
                ('left', [0, .15, 0]), ('right', [0, -.15, 0]),
                ('ccw', [0, 0, .5]), ('cw', [0, 0, -.5])]
    for variant in ['base', 'pt100', 'pt101']:
        name = 'lekiwi_base.xml' if variant == 'base' else f'lekiwi_{variant}_oakd_s2.xml'
        runtime = Simulation(mujoco.MjModel.from_xml_path(str(directory / name)))
        for motion, command in commands:
            runtime.reset()
            start = runtime.pose()
            runtime.command(command)
            runtime.step(round(2 / runtime.model.opt.timestep))
            measured = runtime.pose() - start
            rows.append({'variant': variant, 'motion': motion, 'command': command,
                         'seconds': 2, 'expected_delta': (np.array(command) * 2).tolist(),
                         'measured_delta': measured.tolist(),
                         'total_mass_kg': float(runtime.model.body_mass.sum()),
                         'max_abs_roll_pitch_qvel': float(np.max(np.abs(runtime.data.qvel[3:5])))})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-package', type=Path)
    parser.add_argument('--description-package', type=Path)
    parser.add_argument('--pt-package', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with TemporaryDirectory(prefix='lekiwi_benchmark_') as directory:
        for variant in ('base', 'pt100', 'pt101'):
            filename = 'lekiwi_base.xml' if variant == 'base' else f'lekiwi_{variant}_oakd_s2.xml'
            build(variant, Path(directory) / filename, absolute=True, control_dir=args.control_package,
                  description_dir=args.description_package, pt_package=args.pt_package)
        result = {'updated': benchmark(Path(directory))}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(args.output)


if __name__ == '__main__':
    main()
