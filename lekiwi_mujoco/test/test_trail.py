"""Viewer trail is bounded, flat and resettable without changing physics."""
import mujoco

from lekiwi_mujoco.mujoco_preview import Trail


def test_trail_sampling_capacity_and_reset():
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    scene = mujoco.MjvScene(model, maxgeom=3)
    trail = Trail()
    trail.draw(scene, [0, 0, 2])
    trail.draw(scene, [.001, 0, 5])
    assert len(trail.points) == 1
    for i in range(1100):
        trail.draw(scene, [i * .02, 0, 2])
    assert len(trail.points) == 1000
    assert scene.ngeom == 3
    assert all(point[2] == .003 for point in trail.points)
    assert all(geom.type == mujoco.mjtGeom.mjGEOM_LINE for geom in scene.geoms)
    assert model.ngeom == 0
    trail.clear()
    trail.draw(scene, [0, 0, 0])
    assert scene.ngeom == 0
