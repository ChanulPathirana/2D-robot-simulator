# 2D Robot Simulator

A C++20 and SDL3 simulator for a differential-drive robot navigating a fixed 40 × 30 metre maze. The world uses mathematical coordinates: positive X points right, positive Y points up, and all distances are in metres.

## Features

- A handcrafted, connected maze with 38 permanent wall segments and blocks, long corridors, alternate routes, open turning areas, and dead ends.
- Four deterministic moving rectangles that travel along fixed routes, pause, and reverse. They stop or reverse before crossing a wall, another moving object, or the robot.
- A 0.25 m occupancy grid. Permanent blocked cells are built once; moving obstacles are queried as a temporary layer during planning.
- Eight-direction A* with diagonal corner-cutting prevention and a waypoint-following controller. Planning can use either the complete maze or the robot's LiDAR-discovered map.
- A robot-following camera with zoom, manual pan, and camera-aware mouse targeting.
- A 72-ray, 360° LiDAR scan at about 10 Hz, a separate robot-discovered occupancy map, footprint-aware emergency stopping, and collision rollback.
- Autonomous frontier exploration, plus goal-directed exploration toward a clicked destination that starts outside the observed map.

The robot has a 0.4 m collision radius. Known-map planning inflates obstacles by the robot radius, 0.3 m safety clearance, and half a grid-cell diagonal so a free cell is safe throughout its area. Discovered-map planning uses the same 0.7 m clearance measured between grid-cell squares. The emergency stop removes forward wheel translation when straight-line footprint clearance reaches 0.3 m while preserving rotation. Reverse movement remains available.

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
| Left click | Select a navigation destination; unknown destinations in Discovered Map mode start goal-directed exploration |
| Mouse wheel | Zoom camera |
| F | Toggle camera follow |
| Arrow keys | Pan while camera follow is off |
| R | Reset camera to robot and resume following |
| M | Switch between complete-maze and discovered-map views |
| P | Switch A* planning between Known Map and Discovered Map |
| E | Start or stop independent frontier exploration; starting cancels a goal task and selects Discovered Map planning |
| C | Clear the discovered map; the next scans rebuild it |
| L | Show or hide LiDAR rays; scanning and mapping continue |
| Esc | Quit |

Manual driving cancels the current autonomous destination. The window title reports navigation state, planning mode, and zoom. A blue line shows the current A* path. A green marker identifies an ordinary known-map goal; during goal-directed exploration, a green cross marks the exact requested point and a purple square marks the current temporary frontier viewpoint. Permanent walls are slate, moving obstacles are orange, and translucent orange marks blocked or unsafe grid areas. **M** only changes the view; **P** changes the planner and cancels the previous route.

## LiDAR occupancy mapping

Each discovered cell stores an occupancy probability: the current confidence that something occupies that cell. An unobserved cell starts at **50%** because the robot has no evidence either way. The map also records whether a cell has ever been observed, so an unobserved 50% cell remains distinct from an observed cell whose evidence has balanced back to 50%.

LiDAR updates use log-odds, `L = log(p / (1 - p))`. A detected endpoint adds occupied evidence; cells crossed before it add free-space evidence. A ray with no hit adds only free-space evidence through its reported range. Each update adds the log-odds of its observation and clamps the accumulated value to `[−4, +4]`. Repeated measurements therefore strengthen confidence while contradictory measurements can gradually reverse a classification. These probabilities are initial tuning values, not calibrated physical sensor accuracy.

| Mapping parameter | Initial value |
| --- | ---: |
| Occupied observation probability | 0.70 |
| Free observation probability | 0.30 |
| Log-odds bounds | −4 to +4 |
| Occupied classification | probability ≥ 0.65 |
| Free classification | probability ≤ 0.35 |

Observed cells between the classification thresholds remain **Unknown**. The discovered-map view draws Unknown dark grey, Free light blue-grey, and Occupied red-orange. The first LiDAR scan runs at startup; later scans arrive about every 0.1 s using their recorded robot positions. Drive with W/A/S/D, or choose a destination with left click, then press **M** to watch evidence accumulate. Press **C** to reset confidence and observation history; subsequent scans rebuild the map. **L** changes ray visibility only, so mapping continues when rays are hidden.

The initial values can be tuned through `MappingParameters`. `Mapping::getOccupancyProbability(x, y)` returns 0.5 for an out-of-bounds cell, while `getCell(x, y)` returns `Unknown`.

The complete-maze view shows the physical walls, moving obstacles, planning grid, route, and goal. The discovered-map view shows the robot's occupancy map, robot, and optionally LiDAR rays. It shows only discovered-map routes, the requested goal, and any temporary frontier viewpoint; a known-map route cannot reveal unexplored space. Switching views does not change physics or navigation. The map uses the exact simulated robot pose, so localization uncertainty is not modeled and this is not yet full SLAM. Moving obstacles may leave temporary occupied evidence; later clear observations can lower that confidence, but the map does not track moving objects explicitly.

## Planning with the discovered map

Press **P** for Discovered Map planning, then left-click a destination. A* uses the same eight-direction search as Known Map mode, but its grid is built only from LiDAR evidence. Unknown and Occupied cells are blocked. The planner also requires 0.7 m of Euclidean clearance around each candidate cell from unsafe cells and the world boundary. This includes the 0.4 m robot radius and 0.3 m safety margin. The first scan makes a small safe region around the robot available; driving or navigating reveals more space.

If the clicked cell is safely reachable through known Free space, the robot follows a direct discovered-map route. An Unknown destination, or a known Free destination without a safe connecting route, starts goal-directed exploration. A destination known Occupied at click time or outside the map is rejected with a reason; the robot never silently chooses a nearby substitute. If a scan blocks a direct route, the robot stops, retries safely, and can return to frontier exploration. **C** clears the discovered map and cancels a route or goal task that depended on it. Known Map navigation continues when **C** is pressed, because it uses the separate complete-map grid. Switching planning modes with **P** cancels the current route and destination.

Discovered-map planning uses current observations only. It does not predict moving-obstacle trajectories, and a temporary obstacle can leave map evidence until later clear scans reduce it.

## Goal-directed exploration

In Discovered Map mode, a left click stores the exact clicked world position separately from the temporary frontier viewpoint. The original position remains unchanged as the robot visits frontiers. Viewpoint scoring reuses the independent explorer's information gain, safe path cost, clearance, and failure history, with an additional **0.30 penalty per metre** from a viewpoint to the requested destination. This is a preference toward the goal; reachable alternative routes can still win when direct progress is blocked. No ground-truth maze geometry is used to select a viewpoint or calculate a route.

After changes to the discovered traversability grid, the robot checks for a safe route to the original destination at a bounded rate of at most one check every 0.75 s. Once A* finds a route, it cancels the frontier route and follows the direct route. The Navigator finishes at the exact clicked world point, and success is reported only after the robot physically comes within 0.12 m of it. If a new obstacle invalidates the direct route, the robot stops and can resume frontier exploration. Recovery is bounded to five unsuccessful attempts without meaningful travel; moving more than 0.75 m resets that count. A destination that stays classified Occupied for three seconds, or a task with no safe progress after bounded recovery, ends as **Goal unavailable** with a reason in the console.

Clicking another destination replaces the previous task. WASD, **C**, or switching planning mode with **P** cancels it. Pressing **E** cancels the goal task and starts independent exploration. **M** and **L** remain visualization-only controls. Known Map left-click navigation retains its previous behavior.

## Autonomous frontier exploration

Press **M** to see the robot's discovered map, then **E** to start independent exploration without a requested destination. The explorer uses only the LiDAR map and its safe planning grid; it never reads the complete maze to select a target or route. A frontier is a Free cell next to an in-bounds Unknown cell. Connected frontier cells form regions; isolated one-cell artifacts are ignored by default. Frontier cells are often too close to Unknown space for the 0.7 m robot footprint and safety margin, so the explorer selects a viewpoint set back in known Free space. Every selected viewpoint must be reachable through the discovered grid. A single grid search computes travel costs to candidates, and A* plans only the chosen route.

The score estimates information gain with 24 sample directions out to the 6 m LiDAR range, stopping each direction at the first Unknown or Occupied cell. It combines that estimate with actual reachable path distance and nearby maneuvering room. The default weights are 0.45 per informative direction, −1 per metre of route, and +0.05 per safe neighboring cell. Very short moves receive a small penalty. Recently failed or visited viewpoints are skipped until 25 more cells have been observed or four seconds have passed; older failures carry a penalty. This permits reconsideration after temporary obstacles move. These are planning heuristics, not a guarantee about what the next scan will reveal.

Exploration cycles through selecting a frontier, navigating, observing briefly, and selecting again. LiDAR scans continue during travel, and newly observed cells along the route count as useful progress. The observation pause is at most 0.4 s; the 360° LiDAR does not need a full robot rotation. If a new obstacle invalidates the route, the robot enters **Temporarily blocked**; stalls and unproductive viewpoints enter **Recovering**. Both stop the robot and retry after a cooldown, for at most five attempts. A lack of safe reachable viewpoints leads to **No safe reachable frontier**; repeated motion or observation failures lead to **Stopped after repeated failures**. **Observed map complete** requires every cell to have a conclusive classification. Unknown or inaccessible areas therefore remain reported as incomplete.

Press **E** again, use WASD, click a manual destination, press **C**, or change planning mode with **P** to stop exploration. **M** and **L** affect visualization only. The window title shows exploration state, observed-map coverage, and visited/failed viewpoint counts; state changes in the console also report Unknown, Free, and Occupied counts, frontier regions, reachable candidates, and distance traveled. Observed-map coverage measures cells that have LiDAR evidence. It does not measure how much physical space the robot can reach.

Unknown-space clearance can leave a visible frontier with no safe viewpoint, especially in narrow passages. Moving obstacles can create temporary map evidence and require recovery. Information-gain scoring estimates what a scan may reveal; it cannot see hidden walls or guarantee an entire route will become available. Mapping and exploration both use the exact simulated robot pose, so this is not full SLAM.

## Navigation and safety

The maze has fixed world bounds of X = −20 to 20 m and Y = −15 to 15 m. The robot starts at (−17.5, −12.5), clear of obstacles. Both collision queries and distance rays include moving obstacles and the world boundary.

The static occupancy map is created once. During A*, cells overlapping the **current** moving-obstacle positions are treated as blocked. The controller checks upcoming path cells and stops when one becomes blocked. It replans after a cooldown, also replanning after prolonged lack of motion. Failed attempts have a longer retry delay; five failed plans or repeated replans without meaningful progress stop navigation. A new click starts a fresh request. A blocked or unreachable destination is reported in the window title and console.

**Planning limitation:** A* sees moving obstacles at their current positions. It does not predict where they will be when the robot arrives. Paths can therefore become invalid and require replanning.

**Motion limitation:** `World::forwardClearance()` models straight-line travel along the current heading. Simultaneous driving and turning follows a curved path. Each physics update also checks the swept segment between short substeps and rolls back unsafe movement; moving obstacles refuse a step that would overlap the robot. These checks provide conservative local protection without predicting an entire future trajectory.

## Code layout

- `World`: maze geometry, bounds, moving obstacles, collision and clearance queries.
- `OccupancyGrid`: cached permanent occupancy plus live moving-obstacle checks for Known Map planning.
- `DiscoveredPlanningGrid`: cached robot-footprint traversability derived only from the discovered map.
- `FrontierExplorer`: frontier clustering, safe viewpoint scoring, independent and goal-directed exploration state.
- `AStar`: eight-direction route search.
- `Navigator`: differential-drive waypoint following.
- `Camera`: follow, pan, zoom and coordinate conversions.
- `Lidar` and `Mapping`: 360° measurements and the robot's discovered occupancy cells.
- `Sensor`: ray-based distance readings.
- `Robot`: differential-drive kinematics.
- `main.cpp`: SDL input, fixed-step orchestration and rendering.

The fixed simulation timestep is 1/60 s. The rendering camera does not change world-space physics or planning coordinates.
