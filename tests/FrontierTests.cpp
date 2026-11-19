#include "FrontierExplorer.hpp"
#include "Lidar.hpp"
#include "PlanningMode.hpp"
#include "SimulationConfig.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace std;

namespace {

void require(bool value, const char* message) {
    if (!value) throw runtime_error(message);
}

void markFree(Mapping& map, int x, int y) {
    map.update({{0.0, 0.4, false}}, x + 0.5, y + 0.5);
}

void markOccupied(Mapping& map, int x, int y) {
    for (int i = 0; i < 2; ++i) {
        map.update({{0.0, 0.1, true}}, x + 0.5, y + 0.5);
    }
}

Mapping corridorMap() {
    Mapping map(0.0, 0.0, 14.0, 12.0, 1.0);
    for (int y = 2; y <= 9; ++y) {
        for (int x = 1; x <= 7; ++x) markFree(map, x, y);
    }
    return map;
}

void testFrontierDetection() {
    Mapping empty(0.0, 0.0, 12.0, 12.0, 1.0);
    require(FrontierExplorer::detectFrontiers(empty).empty(),
            "empty map has frontiers");

    for (int y = 1; y <= 2; ++y) {
        for (int x = 1; x <= 2; ++x) markFree(empty, x, y);
    }
    for (int y = 8; y <= 9; ++y) {
        for (int x = 8; x <= 9; ++x) markFree(empty, x, y);
    }
    const auto regions = FrontierExplorer::detectFrontiers(empty);
    require(regions.size() == 2, "disconnected frontier clusters not separated");
    markOccupied(empty, 1, 1);
    for (const auto& region : FrontierExplorer::detectFrontiers(empty, 1)) {
        for (const auto cell : region.cells) {
            require(empty.getCell(cell.x, cell.y) == CellState::Free,
                    "occupied cell classified as frontier");
        }
    }

    Mapping boundary(0.0, 0.0, 1.0, 1.0, 1.0);
    markFree(boundary, 0, 0);
    require(FrontierExplorer::detectFrontiers(boundary, 1).empty(),
            "outside map boundary counted as unknown frontier");

    Mapping full(0.0, 0.0, 6.0, 6.0, 1.0);
    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 6; ++x) markFree(full, x, y);
    }
    require(FrontierExplorer::detectFrontiers(full).empty(),
            "fully observed free map has frontiers");
}

void testViewpointsAndState() {
    Mapping map = corridorMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot;
    robot.x = 2.5;
    robot.y = 5.5;
    Navigator navigator;
    vector<GridCell> path;
    explorer.start(robot, navigator, path);
    require(explorer.state() == ExplorationState::SelectingFrontier,
            "start did not enter selection");
    const auto initialStats = explorer.statistics();
    require(initialStats.observed == 56 && initialStats.free == 56 &&
            initialStats.unknown == 112 && initialStats.occupied == 0,
            "observed coverage counts are wrong");

    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.state() == ExplorationState::Navigating && navigator.isActive(),
            "no safe reachable viewpoint selected");
    require(explorer.statistics().frontierRegions > 0 &&
            explorer.statistics().reachableCandidates > 0,
            "frontier or candidate counts missing");
    const GridCell goal = *explorer.goal();
    require(!grid.isBlocked(goal.x, goal.y), "viewpoint lacks footprint clearance");
    require(map.getCell(goal.x, goal.y) == CellState::Free,
            "viewpoint is not observed free");
    for (const GridCell cell : path) {
        require(!grid.isBlocked(cell.x, cell.y),
                "exploration path entered unknown or occupied space");
    }
    double pathCost = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        pathCost += map.getCellSize() * hypot(
            path[i].x - path[i - 1].x,
            path[i].y - path[i - 1].y
        );
    }
    require(abs(pathCost - explorer.statistics().selectedTravelCost) < 1e-8,
            "viewpoint score did not use reachable path travel cost");

    Mapping duplicate = corridorMap();
    DiscoveredPlanningGrid duplicateGrid(duplicate, 0.7);
    FrontierExplorer duplicateExplorer(duplicate, duplicateGrid, 6.0);
    Robot duplicateRobot = robot;
    Navigator duplicateNavigator;
    vector<GridCell> duplicatePath;
    duplicateExplorer.start(duplicateRobot, duplicateNavigator, duplicatePath);
    duplicateExplorer.update(0.1, false, duplicateRobot, duplicateNavigator,
                             duplicatePath);
    const GridCell duplicateGoal = *duplicateExplorer.goal();
    require(goal.x == duplicateGoal.x && goal.y == duplicateGoal.y,
            "frontier selection is nondeterministic");

    markFree(map, 8, 5);
    grid.rebuild();
    explorer.update(0.1, true, robot, navigator, path);
    require(explorer.statistics().observed == initialStats.observed + 1 &&
            explorer.statistics().observedPercent > initialStats.observedPercent,
            "new scan did not increase observed coverage");

    markFree(map, 8, 4);
    markFree(map, 8, 6);
    grid.rebuild();
    robot.x = grid.gridToWorldX(goal.x);
    robot.y = grid.gridToWorldY(goal.y);
    path = {goal};
    navigator.setPath(path);
    explorer.update(0.1, true, robot, navigator, path);
    require(explorer.state() == ExplorationState::Observing,
            "arrival did not enter bounded observation");
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.statistics().visitedViewpoints == 1 &&
            explorer.state() == ExplorationState::SelectingFrontier,
            "map gain during travel was not credited to viewpoint");

    explorer.stop(robot, navigator, path);
    require(explorer.state() == ExplorationState::Stopped && !navigator.isActive() &&
            path.empty() && robot.leftSpeed == 0.0 && robot.rightSpeed == 0.0,
            "manual interruption left exploration driving");
}

void testReachabilityAndRecovery() {
    Mapping map(0.0, 0.0, 14.0, 12.0, 1.0);
    for (int y = 2; y <= 9; ++y) {
        for (int x = 1; x <= 4; ++x) markFree(map, x, y);
        for (int x = 9; x <= 12; ++x) markFree(map, x, y);
    }
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot;
    robot.x = 2.5;
    robot.y = 5.5;
    Navigator navigator;
    vector<GridCell> path;
    explorer.start(robot, navigator, path);
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.goal() && explorer.goal()->x <= 4,
            "selected a frontier across unknown space");

    const GridCell target = *explorer.goal();
    markOccupied(map, target.x, target.y);
    grid.rebuild();
    explorer.update(0.1, true, robot, navigator, path);
    require(explorer.state() == ExplorationState::TemporarilyBlocked &&
            explorer.statistics().failedViewpoints == 1 && path.empty() &&
            !navigator.isActive(),
            "new obstacle did not stop and recover from an invalid route");

    explorer.stop(robot, navigator, path);
    map = corridorMap();
    grid.rebuild();
    explorer.start(robot, navigator, path);
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.state() == ExplorationState::Navigating,
            "failed frontier setup has no alternate viewpoint");
    for (int tick = 0; tick < 60 && explorer.state() == ExplorationState::Navigating;
         ++tick) {
        explorer.update(0.1, false, robot, navigator, path);
    }
    require(explorer.state() == ExplorationState::Recovering &&
            explorer.statistics().failedViewpoints == 1,
            "navigation stall was not bounded and recorded");

    PlanningMode mode = PlanningMode::DiscoveredMap;
    optional<GridCell> ordinaryGoal = GridCell{3, 5};
    switchPlanningMode(mode, navigator, robot, path, ordinaryGoal);
    explorer.stop(robot, navigator, path);
    require(mode == PlanningMode::KnownMap && !explorer.isActive() &&
            !ordinaryGoal && path.empty(),
            "planning-mode interruption left exploration active");

    explorer.start(robot, navigator, path);
    map.reset();
    grid.rebuild();
    explorer.onMappingReset(robot, navigator, path);
    require(!explorer.isActive() && map.getCell(2, 5) == CellState::Unknown &&
            grid.isBlocked(2, 5) && explorer.statistics().observed == 0,
            "map reset did not leave exploration stopped and space unknown");
}

void testNoFrontierAndBoundedRetry() {
    Mapping map(0.0, 0.0, 8.0, 8.0, 1.0);
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot;
    robot.x = 3.5;
    robot.y = 3.5;
    Navigator navigator;
    vector<GridCell> path;
    explorer.start(robot, navigator, path);
    for (int tick = 0; tick < 100 && explorer.isActive(); ++tick) {
        explorer.update(0.1, false, robot, navigator, path);
    }
    require(explorer.state() == ExplorationState::Limited &&
            explorer.statistics().unknown == 64,
            "unexplored inaccessible map was incorrectly marked complete");

    Mapping narrow(0.0, 0.0, 3.0, 3.0, 1.0);
    markFree(narrow, 1, 1);
    DiscoveredPlanningGrid narrowGrid(narrow, 0.7);
    FrontierExplorerSettings singleFrontier;
    singleFrontier.minimumFrontierSize = 1;
    FrontierExplorer noView(narrow, narrowGrid, 6.0, singleFrontier);
    robot.x = 1.5;
    robot.y = 1.5;
    noView.start(robot, navigator, path);
    for (int tick = 0; tick < 100 && noView.isActive(); ++tick) {
        noView.update(0.1, false, robot, navigator, path);
    }
    require(noView.state() == ExplorationState::Limited &&
            noView.statistics().unknown > 0,
            "unsafe frontier was treated as explored or reachable");

    Mapping full(0.0, 0.0, 8.0, 8.0, 1.0);
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) markFree(full, x, y);
    }
    DiscoveredPlanningGrid fullGrid(full, 0.7);
    FrontierExplorer complete(full, fullGrid, 6.0);
    complete.start(robot, navigator, path);
    complete.update(0.1, false, robot, navigator, path);
    require(complete.state() == ExplorationState::Completed &&
            complete.statistics().observedPercent == 100.0,
            "fully observed map was not completed");
}

void testUnproductiveObservation() {
    Mapping map = corridorMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot;
    robot.x = 2.5;
    robot.y = 5.5;
    Navigator navigator;
    vector<GridCell> path;
    explorer.start(robot, navigator, path);
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.goal().has_value(), "observation test has no target");
    const GridCell goal = *explorer.goal();
    robot.x = grid.gridToWorldX(goal.x);
    robot.y = grid.gridToWorldY(goal.y);
    path = {goal};
    navigator.setPath(path);
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.state() == ExplorationState::Observing,
            "arrival did not start observation");
    for (int i = 0; i < 5; ++i) {
        explorer.update(0.1, false, robot, navigator, path);
    }
    require(explorer.state() == ExplorationState::Recovering &&
            explorer.statistics().failedViewpoints == 1,
            "unproductive viewpoint did not end bounded observation");
    for (int i = 0; i < 16; ++i) {
        explorer.update(0.1, false, robot, navigator, path);
    }
    if (explorer.state() == ExplorationState::SelectingFrontier) {
        explorer.update(0.1, false, robot, navigator, path);
    }
    require(!explorer.goal() ||
            explorer.goal()->x != goal.x || explorer.goal()->y != goal.y,
            "failed viewpoint was immediately selected again");
}

void testBoundedMotionFailure() {
    Mapping map = corridorMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorerSettings settings;
    settings.maximumRecoveryAttempts = 1;
    FrontierExplorer explorer(map, grid, 6.0, settings);
    Robot robot;
    robot.x = 2.5;
    robot.y = 5.5;
    Navigator navigator;
    vector<GridCell> path;
    explorer.start(robot, navigator, path);
    explorer.update(0.1, false, robot, navigator, path);
    for (int tick = 0; tick < 60 && explorer.isActive(); ++tick) {
        explorer.update(0.1, false, robot, navigator, path);
    }
    require(explorer.state() == ExplorationState::Blocked &&
            !navigator.isActive() && path.empty(),
            "bounded motion failures were mislabeled as no frontier");
}

void testHeadlessExploration() {
    World world;
    Robot robot;
    robot.x = SimConfig::START_X;
    robot.y = SimConfig::START_Y;
    Lidar lidar(72, 6.0);
    Mapping map(World::minX, World::minY, World::maxX, World::maxY,
                SimConfig::GRID_CELL_SIZE);
    lidar.scan(robot, world);
    map.update(lidar.getReadings(), robot.x, robot.y);
    DiscoveredPlanningGrid grid(map, SimConfig::PLANNING_RADIUS);
    FrontierExplorer explorer(map, grid, 6.0);
    Navigator navigator;
    vector<GridCell> path;
    explorer.start(robot, navigator, path);
    const int initialObserved = explorer.statistics().observed;
    bool freshScan = false;

    for (int tick = 0; tick < 1500 && explorer.isActive(); ++tick) {
        world.update(SimConfig::DT, robot.x, robot.y, SimConfig::ROBOT_RADIUS);
        explorer.update(SimConfig::DT, freshScan, robot, navigator, path);
        freshScan = false;

        Robot trial = robot;
        trial.update(SimConfig::DT);
        const double step = hypot(trial.x - robot.x, trial.y - robot.y);
        const bool sweptHit = step > 0.0 && world.forwardClearance(
            robot.x, robot.y, atan2(trial.y - robot.y, trial.x - robot.x),
            SimConfig::ROBOT_RADIUS, step
        ) < step - 1e-9;
        if (!sweptHit && !world.collides(trial.x, trial.y,
                                         SimConfig::ROBOT_RADIUS)) robot = trial;
        require(!world.collides(robot.x, robot.y, SimConfig::ROBOT_RADIUS),
                "headless exploration penetrated obstacle");

        if ((tick + 1) % 6 == 0) {
            lidar.scan(robot, world);
            map.update(lidar.getReadings(), robot.x, robot.y);
            grid.rebuild();
            freshScan = true;
        }
    }
    const auto& stats = explorer.statistics();
    cout << "Headless exploration: initial observed " << initialObserved
         << ", final observed " << stats.observed
         << ", travel " << stats.distanceTravelled
         << " m, visited " << stats.visitedViewpoints << '\n';
    require(stats.observed > initialObserved + 20 &&
            stats.distanceTravelled > 0.5,
            "headless exploration did not expand beyond initial scan");
}

} // namespace

int main() {
    try {
        testFrontierDetection();
        testViewpointsAndState();
        testReachabilityAndRecovery();
        testNoFrontierAndBoundedRetry();
        testUnproductiveObservation();
        testBoundedMotionFailure();
        testHeadlessExploration();
        cout << "Frontier tests passed\n";
    } catch (const exception& error) {
        cerr << "Frontier test failed: " << error.what() << '\n';
        return 1;
    }
}
