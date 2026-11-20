# Milestone 15 evaluation report

## Reproduction and scope

Built in Debug with `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DSDL_X11_XSCRNSAVER=OFF`, then ran `cmake --build build -j2`, `ctest --test-dir build --output-on-failure`, and `./build/simulation_evaluation results/milestone15_results.csv`. The CSV is the measured source for this report. The evaluator uses a fixed 1/60 s step, a 160 × 120 map with 0.25 m cells, 72 LiDAR rays every 0.1 s, and deterministic world and moving-obstacle updates. It calls the production A*, mapping, grid, explorer, Navigator, and the exact motion safety step shared with the SDL loop. The standalone runner accepts an optional scenario ID for a focused reproduction, for example `./build/simulation_evaluation /tmp/new_obstacle.csv new_obstacle`.

Fourteen headless scenarios ran. Ten of the thirteen goal-bearing scenarios physically reached the requested point within 0.12 m. The other three were deliberately occupied, outside the map, or inside an impassable wall and returned `NoSafeRoute`. Thus the measured goal-bearing success rate is **10/13 (76.9%)**, or **10/10** for the tested valid destinations. Average time among reached goals was **17.5817 s**; the worst was **27.3833 s** (`several_turns`). An A* route alone was never counted as arrival.

| Scenario | Outcome | Time (s) | Final error (m) | Replans | Route invalidations |
| --- | --- | ---: | ---: | ---: | ---: |
| short_clear | Reached | 5.7667 | 0.1163 | 0 | 0 |
| long_corridor | Reached | 16.2667 | 0.1171 | 0 | 0 |
| several_turns | Reached | 27.3833 | 0.1186 | 0 | 0 |
| narrow_passage | Reached | 14.5000 | 0.1188 | 0 | 0 |
| initially_unknown | Reached | 18.1667 | 0.1171 | 2 | 0 |
| known_occupied | NoSafeRoute | 0 | 5.5227 | 0 | 0 |
| out_of_bounds | NoSafeRoute | 0 | 40.4784 | 0 | 0 |
| unreachable | NoSafeRoute | 0 | 2.6476 | 0 | 0 |
| frontier_discovery | Reached | 18.1667 | 0.1171 | 2 | 0 |
| new_obstacle | Reached | 13.7667 | 0.1121 | 3 | 2 |
| multi_frontier_goal | Reached | 26.1833 | 0.1133 | 2 | 0 |
| mode_switch | Reached | 18.1667 | 0.1171 | 2 | 0 |
| independent_exploration | TimedOut | 120 | n/a | 15 | 0 |
| dynamic_crossing | Reached | 17.4500 | 0.1134 | 1 | 1 |

## Blue path and planning source

The blue line observed for an unknown clicked destination is an active A* route to a temporary frontier viewpoint. It may be long because the viewpoint is selected by information gain, travel cost, clearance, and goal direction. The original requested world point remains separate. A route log for `initially_unknown` shows `FrontierViewpoint` before `DiscoveredMapGoal`, and all accepted routes report `valid`. The renderer now draws the route directly from Navigator's current path, ending at the purple viewpoint while exploring; the green marker remains at the exact requested point. Switching planning mode or resetting a discovered map stops the prior incompatible route.

Source and regression evidence show that discovered A* cannot use the complete maze: `FrontierExplorer` holds only `Mapping` and `DiscoveredPlanningGrid`, and both frontier and direct routes pass that grid to production `AStar::findPath`. `DiscoveredPlanningGrid::rebuild` marks every non-Free cell blocked and applies the unchanged 0.7 m clearance to candidate cells. A* rejects blocked start and goal cells and blocked diagonal corners. Each accepted route is validated against its actual grid, including every waypoint and diagonal transition. The tests cover Unknown and Occupied cells, footprint clearance, unknown direct goals, safe frontier endpoints, map reset, mode switch, new obstacles, and known-map routing. The evaluator aborts on any unsafe accepted discovered route.

## Exploration, collisions, and recovery

The 120 s independent exploration increased observed coverage from **3.7344% to 40.2917%** while traveling **43.8654 m**. It visited **15 useful viewpoints**; the final map had **11,490 Unknown, 7,285 Free, and 425 Occupied cells**, **28 frontier regions**, and **2,608 reachable viewpoint candidates** at the last selection. Coverage at 30/60/90/120 s was **14.2188/24.1615/33.4844/40.2917%**. It timed out while still making progress, so the maze was not reported complete. Across all scenarios, there were **27 accepted-route replans**, **3 obstacle route invalidations**, **2 recovery entries**, **0 collision rollbacks**, **0 navigation stalls**, **0 detected two-viewpoint oscillations**, and **0 failed frontier viewpoints**. The longest measured no-growth interval was **1.0 s** and the longest stationary interval **0.6167 s**. Collision checking and the emergency stop remained active independently of map classification.

In `new_obstacle`, a deterministic rectangle crossed an already selected discovered route, then moved clear. Two route invalidations and recovery entries occurred before physical arrival. In `dynamic_crossing`, the known-map route invalidated once and recovered. The new CTest cases assert that each stress case both invalidates a route and reaches the goal; they do not accept a route-only result.

## Performance

The following Debug measurements are from the 120 s independent exploration case. Average and maximum timings are wall-clock milliseconds per measured call or update on this machine; they are not portable real-time guarantees.

| Operation | Average (ms) | Maximum (ms) |
| --- | ---: | ---: |
| LiDAR mapping update | 0.1163 | 0.2936 |
| Discovered grid rebuild | 4.6975 | 10.1400 |
| A* call | 0.3232 | 0.4204 |
| Frontier selection and scoring | 26.4355 | 51.9112 |
| Full simulation update | 4.4189 | 72.2381 |

The full run's largest observed mapping, grid, A*, and update calls were **0.3456, 10.1400, 1.1687, and 72.2381 ms**, respectively. The full evaluator process peaked at **4,408 KiB resident memory** according to `/usr/bin/time -v`. Frontier scoring is the main intermittent frame-time bottleneck: a selection can exceed the 16.7 ms fixed step. Repeated samples of the same grid cell along each information ray were removed; selection still has measurable spikes and remains a limitation.

## Defects fixed and verification

- The renderer previously consumed a second path vector. It now consumes Navigator's owned route, so the blue drawing cannot diverge from the active controller route.
- Known-map navigation previously ended at the destination cell center even for an off-center click. Navigator now receives the exact clicked world point for its final leg; a regression test requires physical arrival within tolerance.
- Accepted routes are checked and logged with target identity and relevant requested and temporary points. Invalid candidate routes are rejected.
- The SDL loop and headless evaluator now use one shared emergency-stop and swept-collision motion step.
- Frontier information gain no longer repeats samples within the same grid cell along a ray.

The final Debug build succeeded. All **6 CTest tests passed**. `git diff --check` passed. The SDL application also launched in the available graphical session and ran until an intentional three-second timeout without an immediate startup error. Keyboard, mouse, camera, and visual colors were inspected in source but were not interactively exercised in that smoke test. The input handler keeps `M` and `L` visual-only, `P` cancels incompatible navigation, `C` clears discovered-map tasks, `E` starts/stops independent exploration, and manual WASD cancels autonomous control.

## Remaining limits

The independent explorer had not finished after 120 s; **40.2917% observed** is a bounded coverage result, not complete exploration. Frontier selection still causes occasional frame-time spikes. The headless evaluator reuses production planning, control, sensing, world, and physics, but its outer known-map retry orchestration mirrors `main.cpp` rather than calling the SDL event loop itself. Neither mapping nor planning models pose uncertainty or predicts moving-obstacle trajectories. The three-second graphical launch was a startup smoke test, not a manual interaction test.
