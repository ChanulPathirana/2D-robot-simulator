# 2D Robot Simulator

A **C++20 robotics simulator** built with **SDL3** and **CMake** to explore differential-drive motion, obstacle detection, collision avoidance, occupancy grids, and path planning.

The project starts with a manually controlled robot in a 2D environment and is being extended toward **autonomous, goal-directed navigation using A\***.

## Features

### Implemented

- **Differential-drive robot:** Simulates translation and rotation using independent left and right wheel velocities.
- **Keyboard control:** Drive forward, reverse, and turn with `W`, `A`, `S`, and `D`.
- **2D environment:** Rectangular obstacles in world coordinates (metres).
- **Collision detection:** Treats the robot as a circular footprint and prevents penetration of obstacles by rolling back unsafe motion steps.
- **Distance sensors:** Three simulated ray-casting sensors at −45°, 0°, and +45° relative to the robot's heading.
- **Footprint-aware emergency stopping:** Removes forward motion when the robot's circular footprint is within **0.3 m** of an obstacle along its current heading; turning and reverse remain available.
- **Occupancy grid:** Discretizes the known environment into free and blocked cells for path planning. Cells are checked against the robot's radius, not just obstacle interiors.
- **Real-time visualization:** SDL3 rendering for the robot, its heading, obstacles, sensor rays, and blocked grid cells.
- **Fixed-step simulation:** Robot physics runs at **60 updates per second**.

### In progress / planned

- **A\* pathfinding:** Calculate a path from the robot's grid cell to a mouse-selected destination.
- **Path visualization:** Draw the planned route in the simulator window.
- **Waypoint following:** Turn toward and drive between path waypoints automatically.
- **Autonomous navigation:** Combine planning, motion control, and safety checks.
- **Exploration and mapping (future extension):** Discover an initially unknown environment using sensor measurements, rather than reading the simulator's true obstacle map.

> **Project status:** The occupancy grid has been added. A\* and mouse-selected destinations are the next development milestone; do not assume autonomous movement is available in the current build. If you've already added and verified `AStar.cpp`, update this section accordingly.

## Preview

The simulator displays:

- A **blue triangular robot** with a **yellow heading indicator**.
- **Red rectangles** representing obstacles.
- **Green sensor rays** when no obstacle is detected within range, or **red rays** when an obstacle is detected.
- **Translucent orange cells** indicating positions where the robot's **center** would cause its circular body to overlap an obstacle.

_Add a screenshot or GIF here once captured:_

```markdown
![2D Robot Simulator](docs/simulator-preview.png)
```

## Tech stack

| Technology | Purpose |
| --- | --- |
| C++20 | Simulation logic and algorithms |
| SDL3 | Window, input handling, and 2D rendering |
| CMake | Build configuration |
| Git submodules | SDL source dependency under `external/SDL` |

## Getting started

### Prerequisites

- A C++20-capable compiler (such as GCC or Clang)
- CMake
- Git
- SDL3 build dependencies for your platform

The documented commands below are for **Ubuntu/Linux**. The project uses SDL3 as a Git submodule, so the submodule must be available before building.

### Clone

```bash
git clone --recurse-submodules <YOUR_REPOSITORY_URL>
cd <YOUR_REPOSITORY_DIRECTORY>
```

If the repository was already cloned without submodules:

```bash
git submodule update --init --recursive
```

### Configure and build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DSDL_X11_XSCRNSAVER=OFF
cmake --build build -j2
```

### Run

```bash
./build/robot_simulator
```

After making code changes, you normally only need to rebuild and run:

```bash
cmake --build build -j2
./build/robot_simulator
```

## Controls

| Key | Action |
| --- | --- |
| `W` | Move forward |
| `S` | Move backward |
| `A` | Turn left |
| `D` | Turn right |
| `W` + `A` / `D` | Move forward while steering |
| `Esc` | Exit simulator |

**Planned control:** Left-click a free position to choose an A\* destination (once click-to-plan has been implemented and tested).

## How it works

### 1. Differential-drive kinematics

The robot is controlled by its left and right wheel velocities:

\[
v = \frac{v_R + v_L}{2}, \qquad
\omega = \frac{v_R - v_L}{L}
\]

where:

- `v` is forward linear velocity.
- `ω` is angular velocity.
- `vL` and `vR` are left/right wheel velocities.
- `L` is the distance between wheels.

The robot updates its position and heading each fixed physics timestep.

### 2. World and collision detection

Obstacles are axis-aligned rectangles. For safety, the robot is approximated as a circle with a **0.4 m radius**. After each movement update, collision detection checks whether the new pose overlaps any obstacle. If it does, the previous pose is restored.

### 3. Simulated distance sensors

The simulator casts three rays from the robot and checks for intersections with obstacles within a **4.0 m maximum range**. These readings are displayed in real time.

### 4. Footprint-aware emergency stop

Unlike a single forward sensor ray, `World::forwardClearance()` considers the robot's **entire circular footprint**. If an obstacle lies within the **0.3 m forward clearance threshold**, the controller cancels forward translation while preserving the commanded angular velocity.

**Limitation:** The clearance calculation considers straight-line forward travel. Combined driving and turning follows a curved path; the collision rollback remains a separate safeguard. The simulator does not yet provide full trajectory-level collision checking.

### 5. Occupancy grid

The occupancy grid converts the continuous world into cells (currently **0.25 m per cell**). For each cell center, the simulator checks whether the robot's circular footprint would collide with an obstacle.

This creates a grid for A\* search:

- **Free cell:** The robot's center can occupy this position under the collision model.
- **Blocked cell:** Placing the robot's center here would cause a collision, or the cell is outside grid bounds.

The grid is built from **known world obstacles**, not from the robot's sensor observations. A later exploration extension will maintain a separate internal map.

### 6. A\* pathfinding — next milestone

The planned A\* module will search neighboring free cells using:

\[
f(n) = g(n) + h(n)
\]

where `g(n)` is the traveled path cost and `h(n)` estimates the remaining cost to the goal. The intended implementation supports eight-direction grid movement with a diagonal-corner-cutting check.

A\* **plans a path**; it does not command the robot's wheels. Autonomous motion will require a separate waypoint-following controller.

## Project structure

```text
2D-robot-simulator/
├── CMakeLists.txt
├── include/
│   ├── Robot.hpp
│   ├── World.hpp
│   ├── Sensor.hpp
│   └── OccupancyGrid.hpp
├── src/
│   ├── main.cpp
│   ├── Robot.cpp
│   ├── World.cpp
│   ├── Sensor.cpp
│   └── OccupancyGrid.cpp
├── external/
│   └── SDL/                 # Git submodule
└── README.md
```

**Next files:** `include/AStar.hpp` and `src/AStar.cpp`. Later milestones may add a navigation controller and mapping modules.

## Roadmap

- [x] Differential-drive robot motion
- [x] Manual keyboard control
- [x] Rectangular obstacles and collision rollback
- [x] Simulated ray-casting sensors
- [x] Footprint-aware emergency stop
- [x] Occupancy grid generation and visualization
- [ ] A\* route calculation
- [ ] Mouse-selected destination
- [ ] Planned path rendering
- [ ] Autonomous waypoint-following controller
- [ ] Safe replanning when the route becomes invalid
- [ ] Unknown-environment mapping and exploration
- [ ] Localization / SLAM experiments

## Testing

The emergency-stop implementation has been checked with a standalone test harness covering head-on and diagonal approach, center-ray misses, obstacle corners, clear paths, reverse, rotation, collision rollback, tangency, and initially overlapping states. The project has also passed a CMake build and `git diff --check` during that milestone.

**Not yet verified in those checks:** Interactive SDL rendering and any newly added A\* implementation. Test those separately after integration.

## Educational goals

This project is intended to develop practical knowledge of:

- C++ class design and separation of header/source files
- Robot motion models and coordinate transformations
- Real-time simulation loops
- Sensor simulation and collision geometry
- Occupancy grids and graph-search algorithms
- Autonomous navigation architecture

## License

No project license has been specified yet. Add a `LICENSE` file if you intend to make the repository available for reuse.
