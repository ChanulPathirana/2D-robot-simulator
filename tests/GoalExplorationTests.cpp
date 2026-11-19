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

void require(bool condition, const char* message) {
    if (!condition) throw runtime_error(message);
}

void markFree(Mapping& map, int x, int y) {
    map.update({{0.0, 0.4, false}}, x + 0.5, y + 0.5);
}

void markOccupied(Mapping& map, int x, int y) {
    for (int i = 0; i < 2; ++i) {
        map.update({{0.0, 0.1, true}}, x + 0.5, y + 0.5);
    }
}

Mapping corridor(int lastFreeX = 6) {
    Mapping map(0.0, 0.0, 14.0, 12.0, 1.0);
    for (int y = 2; y <= 9; ++y) {
        for (int x = 1; x <= lastFreeX; ++x) markFree(map, x, y);
    }
    return map;
}

Robot startingRobot() {
    Robot robot;
    robot.x = 2.5;
    robot.y = 5.5;
    return robot;
}

void testDirectAndPhysicalArrival() {
    Mapping map = corridor(10);
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot = startingRobot();
    Navigator navigator;
    vector<GridCell> path;
    const WorldPoint requested{5.2, 5.6};
    require(explorer.startGoal(requested, robot, navigator, path) ==
                GoalRequestResult::Direct &&
            explorer.state() == ExplorationState::NavigatingToGoal,
            "safe known destination did not start direct navigation");
    for (int tick = 0; tick < 1200 && explorer.isActive(); ++tick) {
        explorer.update(SimConfig::DT, false, robot, navigator, path);
        robot.update(SimConfig::DT);
    }
    require(explorer.state() == ExplorationState::GoalReached &&
            hypot(robot.x - requested.x, robot.y - requested.y) <=
                Navigator::GOAL_TOLERANCE,
            "goal was reported reached before physical arrival");
    require(explorer.requestedDestination() &&
            explorer.requestedDestination()->x == requested.x,
            "original goal was lost at arrival");
}

void testUnknownToDirectAndPreservedGoal() {
    Mapping map = corridor();
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot = startingRobot();
    Navigator navigator;
    vector<GridCell> path;
    const WorldPoint requested{9.2, 5.6};
    require(map.getCell(9, 5) == CellState::Unknown,
            "test destination was already known");
    require(explorer.startGoal(requested, robot, navigator, path) ==
                GoalRequestResult::Exploring,
            "unknown destination was rejected instead of explored");
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.state() == ExplorationState::Navigating &&
            explorer.goal().has_value(),
            "no frontier viewpoint selected for unknown goal");
    require(explorer.requestedDestination() &&
            explorer.requestedDestination()->x == requested.x &&
            explorer.requestedDestination()->y == requested.y,
            "frontier navigation overwrote original requested world point");
    for (const GridCell cell : path) {
        require(!grid.isBlocked(cell.x, cell.y),
                "frontier path deliberately entered unknown space");
    }

    for (int y = 2; y <= 9; ++y) {
        for (int x = 7; x <= 10; ++x) markFree(map, x, y);
    }
    require(grid.rebuild(), "new observations did not change planning grid");
    explorer.update(0.8, true, robot, navigator, path);
    require(explorer.state() == ExplorationState::NavigatingToGoal &&
            !explorer.goal() && explorer.requestedDestination() &&
            explorer.requestedDestination()->x == requested.x,
            "discovered safe route did not replace temporary frontier route");
    for (const GridCell cell : path) {
        require(!grid.isBlocked(cell.x, cell.y),
                "direct goal path entered unsafe space");
    }
    for (int tick = 0; tick < 1800 && explorer.isActive(); ++tick) {
        explorer.update(SimConfig::DT, false, robot, navigator, path);
        robot.update(SimConfig::DT);
    }
    require(explorer.state() == ExplorationState::GoalReached &&
            hypot(robot.x - requested.x, robot.y - requested.y) <=
                Navigator::GOAL_TOLERANCE,
            "discovered route did not reach original goal position");
}

void testDisconnectedAndInvalidGoals() {
    Mapping map = corridor();
    for (int y = 2; y <= 9; ++y) {
        for (int x = 10; x <= 12; ++x) markFree(map, x, y);
    }
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot = startingRobot();
    Navigator navigator;
    vector<GridCell> path;
    require(map.getCell(11, 5) == CellState::Free &&
            explorer.startGoal({11.2, 5.6}, robot, navigator, path) ==
                GoalRequestResult::Exploring,
            "disconnected known-free destination did not explore");

    markOccupied(map, 11, 5);
    grid.rebuild();
    require(explorer.startGoal({11.2, 5.6}, robot, navigator, path) ==
                GoalRequestResult::Occupied &&
            explorer.state() == ExplorationState::InvalidDestination &&
            explorer.goalFailureReason() == GoalFailureReason::OccupiedAtClick &&
            !navigator.isActive(),
            "known occupied destination was accepted");
    require(explorer.startGoal({14.0, 5.5}, robot, navigator, path) ==
                GoalRequestResult::OutsideMap &&
            explorer.state() == ExplorationState::InvalidDestination &&
            explorer.goalFailureReason() == GoalFailureReason::OutsideMap,
            "out-of-bounds destination was accepted");
}

void testGoalDirectedFrontierBias() {
    Mapping map(0.0, 0.0, 14.0, 12.0, 1.0);
    for (int y = 3; y <= 8; ++y) {
        for (int x = 3; x <= 10; ++x) markFree(map, x, y);
    }
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorerSettings settings;
    settings.goalDistanceWeight = 5.0;
    FrontierExplorer explorer(map, grid, 6.0, settings);
    Robot robot;
    robot.x = 6.5;
    robot.y = 5.5;
    Navigator navigator;
    vector<GridCell> path;
    require(explorer.startGoal({12.5, 5.5}, robot, navigator, path) ==
                GoalRequestResult::Exploring,
            "goal-bias test did not start frontier exploration");
    explorer.update(0.1, false, robot, navigator, path);
    require(explorer.goal() && explorer.goal()->x > 6 &&
            !grid.isBlocked(explorer.goal()->x, explorer.goal()->y),
            "goal-directed score did not prefer safe reachable viewpoint toward goal");
    for (const GridCell cell : path) {
        require(!grid.isBlocked(cell.x, cell.y),
                "goal-directed frontier route entered unknown space");
    }
}

void testReplacementAndCancellation() {
    Mapping map = corridor(10);
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot = startingRobot();
    Navigator navigator;
    vector<GridCell> path;
    explorer.startGoal({8.2, 5.4}, robot, navigator, path);
    require(explorer.startGoal({4.1, 5.7}, robot, navigator, path) ==
                GoalRequestResult::Direct &&
            explorer.requestedDestination()->x == 4.1,
            "new click did not replace requested goal");
    explorer.stop(robot, navigator, path);
    require(!explorer.requestedDestination() && !navigator.isActive() &&
            path.empty() && robot.leftSpeed == 0.0,
            "manual interruption did not cancel goal task");

    explorer.startGoal({9.2, 5.6}, robot, navigator, path);
    explorer.start(robot, navigator, path);
    require(explorer.taskMode() == ExplorationTaskMode::Independent &&
            !explorer.requestedDestination() &&
            explorer.state() == ExplorationState::SelectingFrontier,
            "independent E exploration conflicted with goal task");

    explorer.startGoal({9.2, 5.6}, robot, navigator, path);
    PlanningMode mode = PlanningMode::DiscoveredMap;
    optional<GridCell> ordinaryGoal;
    explorer.stop(robot, navigator, path);
    switchPlanningMode(mode, navigator, robot, path, ordinaryGoal);
    require(mode == PlanningMode::KnownMap && !explorer.requestedDestination(),
            "planning-mode switch retained goal-directed exploration");

    explorer.startGoal({9.2, 5.6}, robot, navigator, path);
    map.reset();
    grid.rebuild();
    explorer.onMappingReset(robot, navigator, path);
    require(!explorer.requestedDestination() &&
            explorer.statistics().observed == 0 && !navigator.isActive(),
            "map reset retained requested goal or motion");
}

void testBlockedRouteAndNoProgress() {
    Mapping map = corridor(10);
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot = startingRobot();
    Navigator navigator;
    vector<GridCell> path;
    require(explorer.startGoal({9.2, 5.6}, robot, navigator, path) ==
                GoalRequestResult::Direct && path.size() > 3,
            "blocked-route test has no direct path");
    const GridCell blocked = path[path.size() / 2];
    markOccupied(map, blocked.x, blocked.y);
    grid.rebuild();
    explorer.update(0.1, true, robot, navigator, path);
    require(explorer.state() == ExplorationState::TemporarilyBlocked &&
            !navigator.isActive() && path.empty() &&
            explorer.requestedDestination(),
            "new obstacle did not safely stop goal route");

    map = corridor(10);
    grid.rebuild();
    explorer.update(0.8, true, robot, navigator, path);
    require(explorer.state() == ExplorationState::NavigatingToGoal &&
            navigator.isActive(),
            "goal route did not replan after obstacle cleared");

    Mapping empty(0.0, 0.0, 8.0, 8.0, 1.0);
    DiscoveredPlanningGrid emptyGrid(empty, 0.7);
    FrontierExplorer noProgress(empty, emptyGrid, 6.0);
    Robot isolated;
    isolated.x = 3.5;
    isolated.y = 3.5;
    Navigator isolatedNavigator;
    vector<GridCell> isolatedPath;
    require(noProgress.startGoal({6.5, 6.5}, isolated,
                                 isolatedNavigator, isolatedPath) ==
                GoalRequestResult::Exploring,
            "unknown goal should initially attempt exploration");
    for (int tick = 0; tick < 100 && noProgress.isActive(); ++tick) {
        noProgress.update(0.1, false, isolated, isolatedNavigator, isolatedPath);
    }
    require(noProgress.state() == ExplorationState::GoalUnavailable &&
            !isolatedNavigator.isActive() && noProgress.requestedDestination() &&
            noProgress.goalFailureReason() == GoalFailureReason::NoSafeProgress,
            "no reachable frontier did not terminate goal task safely");
}

void testGoalBecomesOccupied() {
    Mapping map = corridor();
    DiscoveredPlanningGrid grid(map, 0.7);
    FrontierExplorer explorer(map, grid, 6.0);
    Robot robot = startingRobot();
    Navigator navigator;
    vector<GridCell> path;
    require(explorer.startGoal({9.2, 5.6}, robot, navigator, path) ==
                GoalRequestResult::Exploring,
            "occupied-later test did not start exploration");
    markOccupied(map, 9, 5);
    grid.rebuild();
    for (int tick = 0; tick < 45 && explorer.isActive(); ++tick) {
        explorer.update(0.1, tick == 0, robot, navigator, path);
    }
    require(explorer.state() == ExplorationState::GoalUnavailable &&
            explorer.goalFailureReason() ==
                GoalFailureReason::OccupiedAfterObservation &&
            !navigator.isActive(),
            "goal that stayed occupied did not stop with a clear outcome");
}

void testHeadlessUnknownGoal() {
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
    const WorldPoint requested{-11.0, -12.5};
    const GridCell target = grid.worldToGrid(requested.x, requested.y);
    require(map.getCell(target.x, target.y) == CellState::Unknown,
            "headless goal was not initially unknown");
    require(explorer.startGoal(requested, robot, navigator, path) ==
                GoalRequestResult::Exploring,
            "headless unknown goal was not explored");
    const int initialObserved = explorer.statistics().observed;
    bool freshScan = false;
    int collisionRollbacks = 0;
    for (int tick = 0; tick < 3600 && explorer.isActive(); ++tick) {
        world.update(SimConfig::DT, robot.x, robot.y, SimConfig::ROBOT_RADIUS);
        explorer.update(SimConfig::DT, freshScan, robot, navigator, path);
        freshScan = false;

        const double forward = (robot.leftSpeed + robot.rightSpeed) * 0.5;
        if (forward > 0.0 && world.forwardClearance(
                robot.x, robot.y, robot.theta, SimConfig::ROBOT_RADIUS,
                6.0) <= SimConfig::SAFETY_CLEARANCE) {
            const double omega = (robot.rightSpeed - robot.leftSpeed) /
                                 robot.wheelBase;
            robot.leftSpeed = -omega * robot.wheelBase * 0.5;
            robot.rightSpeed = omega * robot.wheelBase * 0.5;
        }

        Robot trial = robot;
        trial.update(SimConfig::DT);
        const double step = hypot(trial.x - robot.x, trial.y - robot.y);
        const bool sweptHit = step > 0.0 && world.forwardClearance(
            robot.x, robot.y, atan2(trial.y - robot.y, trial.x - robot.x),
            SimConfig::ROBOT_RADIUS, step
        ) < step - 1e-9;
        if (sweptHit || world.collides(trial.x, trial.y,
                                       SimConfig::ROBOT_RADIUS)) {
            ++collisionRollbacks;
        } else {
            robot = trial;
        }
        require(!world.collides(robot.x, robot.y, SimConfig::ROBOT_RADIUS),
                "headless robot penetrated obstacle");

        if ((tick + 1) % 6 == 0) {
            lidar.scan(robot, world);
            map.update(lidar.getReadings(), robot.x, robot.y);
            grid.rebuild();
            freshScan = true;
        }
    }
    const auto& stats = explorer.statistics();
    cout << "Goal headless: observed " << initialObserved << " -> "
         << stats.observed << ", frontier visits " << stats.visitedViewpoints
         << ", travel " << stats.distanceTravelled << " m, rollbacks "
         << collisionRollbacks << ", reached "
         << (explorer.state() == ExplorationState::GoalReached) << '\n';
    require(explorer.state() == ExplorationState::GoalReached &&
            hypot(robot.x - requested.x, robot.y - requested.y) <=
                Navigator::GOAL_TOLERANCE && stats.observed > initialObserved,
            "headless robot did not physically reach originally unknown goal");
}

} // namespace

int main() {
    try {
        testDirectAndPhysicalArrival();
        testUnknownToDirectAndPreservedGoal();
        testDisconnectedAndInvalidGoals();
        testGoalDirectedFrontierBias();
        testReplacementAndCancellation();
        testBlockedRouteAndNoProgress();
        testGoalBecomesOccupied();
        testHeadlessUnknownGoal();
        cout << "Goal exploration tests passed\n";
    } catch (const exception& error) {
        cerr << "Goal exploration test failed: " << error.what() << '\n';
        return 1;
    }
}
