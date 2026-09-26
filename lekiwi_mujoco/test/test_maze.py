"""Check the actual scene geometry leaves a robot-sized route and branch choices."""
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest


def test_maze_clearance_route_dead_ends_and_clearings():
    path = Path(__file__).resolve().parents[1] / 'mjcf/scenes/maze.xml'
    root = ET.parse(path).getroot()
    walls = [(np.fromstring(g.get('pos'), sep=' ')[:2],
              np.fromstring(g.get('size'), sep=' ')[:2])
             for g in root.findall('worldbody/geom')
             if g.get('type') == 'box' and g.get('contype') != '0']

    def clear(a, b):
        # Conservative 0.5 m diameter footprint along the entire segment.
        points = np.linspace(a, b, 101)
        return not any(np.any(np.all(np.abs(points - pos) < size + .25, axis=1))
                       for pos, size in walls)

    cells = {(i, j): np.array([1.4 + i * 1.2, j * 1.2])
             for i in range(7) for j in range(7)}
    graph = {a: [b for b in cells if abs(a[0]-b[0])+abs(a[1]-b[1]) == 1
                 and clear(cells[a], cells[b])] for a in cells}
    visited = {(0, 0)}
    queue = deque(visited)
    while queue:
        for b in graph[queue.popleft()]:
            if b not in visited:
                visited.add(b)
                queue.append(b)
    assert len(visited) == 49
    assert clear([0, 0], cells[0, 0])
    assert clear(cells[6, 6], [10, 7.2])
    assert sum(len(v) == 1 for k, v in graph.items() if k not in [(0, 0), (6, 6)]) >= 3
    for x, y in [(1, 1), (4, 4)]:
        for i in range(x, x + 2):
            assert (i, y + 1) in graph[i, y]
        for j in range(y, y + 2):
            assert (x + 1, j) in graph[x, j]
    mujoco = pytest.importorskip('mujoco')
    mujoco.MjModel.from_xml_path(str(path))
