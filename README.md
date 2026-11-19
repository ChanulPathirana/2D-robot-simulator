# 2D Robot Simulator

A C++20 and SDL3 simulator for a differential-drive robot navigating a fixed 40 × 30 metre maze. The world uses mathematical coordinates: positive X points right, positive Y points up, and all distances are in metres.

## Features

- A handcrafted, connected maze with 38 permanent wall segments and blocks, long corridors, alternate routes, open turning areas, and dead ends.
- Four deterministic moving rectangles that travel along fixed routes, pause, and reverse. They stop or reverse before crossing a wall, another moving object, or the robot.
- A 0.25 m occupancy grid. Permanent blocked cells are built once; moving obstacles are queried as a temporary layer during planning.
- Eight-direction A* with diagonal corner-cutting prevention and a waypoint-following controller.
- A robot-following camera with zoom, manual pan, and camera-aware mouse targeting.
- Three visible distance rays, footprint-aware emergency stopping, and collision rollback.

The robot has a 0.4 m collision radius. Planning inflates obstacles by the robot radius, 0.3 m safety clearance, and half a grid-cell diagonal so a free cell is safe throughout its area. The emergency stop removes forward wheel translation when straight-line footprint clearance reaches 0.3 m while preserving rotation. Reverse movement remains available.

## Build and run

SDL3 is included as a Git submodule. On Linux with CMake and a C++20 compiler:

```bash
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DSDL_X11_XSCRNSAVER=OFF
cmake --build build -j2
./build/robot_simulator
```

## Controls

| Control | Action |
| --- | --- |
| W / S | Drive forward / reverse |
| A / D | Turn left / right; combine with W or S |
| Left click | Select a navigation destination |
| Mouse wheel | Zoom camera |
| F | Toggle camera follow |
| Arrow keys | Pan while camera follow is off |
| R | Reset camera to robot and resume following |
| Esc | Quit |

Manual driving cancels the current autonomous destination. The window title reports navigation state and zoom. A blue line shows the current A* path; the green marker shows the selected goal. Permanent walls are slate, moving obstacles are orange, and translucent orange marks blocked or unsafe grid areas.

## Navigation and safety

The maze has fixed world bounds of X = −20 to 20 m and Y = −15 to 15 m. The robot starts at (−17.5, −12.5), clear of obstacles. Both collision queries and distance rays include moving obstacles and the world boundary.

The static occupancy map is created once. During A*, cells overlapping the **current** moving-obstacle positions are treated as blocked. The controller checks upcoming path cells and stops when one becomes blocked. It replans after a cooldown, also replanning after prolonged lack of motion. Failed attempts have a longer retry delay; five failed plans or repeated replans without meaningful progress stop navigation. A new click starts a fresh request. A blocked or unreachable destination is reported in the window title and console.

**Planning limitation:** A* sees moving obstacles at their current positions. It does not predict where they will be when the robot arrives. Paths can therefore become invalid and require replanning.

**Motion limitation:** `World::forwardClearance()` models straight-line travel along the current heading. Simultaneous driving and turning follows a curved path. Each physics update also checks the swept segment between short substeps and rolls back unsafe movement; moving obstacles refuse a step that would overlap the robot. These checks provide conservative local protection without predicting an entire future trajectory.

## Code layout

- `World`: maze geometry, bounds, moving obstacles, collision and clearance queries.
- `OccupancyGrid`: cached permanent occupancy plus live moving-obstacle checks.
- `AStar`: eight-direction route search.
- `Navigator`: differential-drive waypoint following.
- `Camera`: follow, pan, zoom and coordinate conversions.
- `Sensor`: ray-based distance readings.
- `Robot`: differential-drive kinematics.
- `main.cpp`: SDL input, fixed-step orchestration and rendering.

The fixed simulation timestep is 1/60 s. The rendering camera does not change world-space physics or planning coordinates.
