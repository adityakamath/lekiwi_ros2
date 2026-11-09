# LeKiwi MuJoCo scenes

Select a scene with `--scene <name>` in the standalone tools or `mujoco_scene:=<name>` with ROS. `flat` is the standard floor.

## Arena

`arena` is a 4 × 4 m robot test course with a clear 1.3 × 1.3 m central maneuvering
area. Along the perimeter are three north gates (0.6, 0.8 and 1.0 m clear width,
marked with one, two and three blue floor ticks), an east slalom, a southwest
docking pocket with a flared entrance and a 1 m inner width, and a southern pushing area with a 180 g box
and 60 g ball. The docking pocket opens east. All other obstacles are fixed.
The dock has a centerline, a robot-center stop line, and a contrasting back-wall
target. Push the box east into the orange floor target, then reposition south of
it and push north into the blue target. The lane behind the gates is 0.70 m wide;
the outer slalom lane has at least 0.60 m clearance.
Reset restores both movable objects. Prop masses and friction are illustrative.

Warm walls, colored landmarks, and a bounded sand-colored floor distinguish the
course; the standard blue checkerboard continues outside. Floor markings are
visual only. Select `arena_top` or `arena_overview` for a view of the entire course.

```sh
mjpython -m lekiwi_mujoco.mujoco_preview --variant base --scene arena
```

## Home

Use `--scene home` with the standalone viewer or `mujoco_scene:=home` with ROS.
The 8 × 8 m apartment has a living room, kitchen and dining area, an enlarged bedroom, and
a 2.8 × 2.8 m room in the northwest corner with glass partitions and a 1 m doorway.
The glass partitions are translucent, solid collision geometry. Low cutaway walls expose the layout from above; doorways, table legs,
and furniture provide navigation obstacles. The robot starts in the clear central
hall. `home_overview` and `home_top` are fixed cameras available in the viewer.

Three objects have free joints and can be pushed: a 180 g cardboard box east of
spawn, an 80 g foam block northwest of spawn, and a 60 g toy ball near the dining
area. They use low friction; furniture stays fixed. Reset restores the objects as
well as the robot. The apartment flooring stops at the outer walls, with the standard blue MuJoCo
checkerboard outside. Rugs and floor finishes are visual only, so they introduce no
steps or changes in wheel traction. Prop masses and friction are illustrative,
not calibrated to physical household objects.

```sh
# macOS standalone apartment
mjpython -m lekiwi_mujoco.mujoco_preview --variant base --scene home
```

## Maze

`--scene maze` opens a 7 × 7-cell maze with 1.12 m clear corridors, multiple
dead ends, and two 2 × 2-cell clearings (2.32 m clear width). The robot starts
outside the blue west entrance at `(0, 0)`. Follow the east-pointing entry
chevrons; the orange exit is on the east boundary at `(9.2, 7.2)`.
The maze uses warm off-white walls and the home scene’s wood-colored floor finish.
This finish extends 0.5 m beyond the outer walls; the standard blue
checkerboard floor continues elsewhere. Clearings use the same floor as corridors.
All cells are reachable, with a continuous route through the maze. No route is
painted on the floor. `maze_top` and `maze_overview` provide fixed camera views.

```sh
mjpython -m lekiwi_mujoco.mujoco_preview --variant base --scene maze
```
