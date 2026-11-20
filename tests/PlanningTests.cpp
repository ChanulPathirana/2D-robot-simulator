#include "AStar.hpp"
#include "DiscoveredPlanningGrid.hpp"
#include "Lidar.hpp"
#include "OccupancyGrid.hpp"
#include "PlanningMode.hpp"
#include "RouteValidation.hpp"
#include "RouteDiagnostics.hpp"
#include "SimulationConfig.hpp"
#include "SimulationStep.hpp"

#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

using namespace std;

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw runtime_error(message);
}

Mapping freeMap() {
    Mapping map(0.0, 0.0, 10.0, 10.0, 1.0);
    for (int y = 0; y < 10; ++y) {
        map.update({{0.0, 9.4, false}}, 0.5, y + 0.5);
    }
    return map;
}

void occupy(Mapping& map, int x, int y) {
    for (int i = 0; i < 2; ++i) {
        map.update({{0.0, static_cast<double>(x), true}}, 0.5, y + 0.5);
    }
    require(map.getCell(x, y) == CellState::Occupied, "test obstacle was not occupied");
}

void testDiscoveredRoutesAndClearance() {
    Mapping map = freeMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    const auto path = AStar::findPath(grid, {2, 4}, {7, 4});
    require(!path.empty(), "known free cells did not route");
    for (const auto cell : path) {
        require(map.getCell(cell.x, cell.y) == CellState::Free,
                "route entered a non-free cell");
        require(!grid.isBlocked(cell.x, cell.y), "route entered inflated obstacle");
    }
    require(grid.isBlocked(0, 4), "world boundary was not inflated");
    require(!grid.isBlocked(1, 4), "boundary inflated beyond clearance radius");

    occupy(map, 5, 5);
    require(grid.rebuild(), "new obstacle did not change traversability");
    require(grid.isBlocked(5, 5), "occupied cell is traversable");
    require(grid.isBlocked(6, 5), "robot radius was not inflated horizontally");
    require(grid.isBlocked(6, 6), "robot radius was not inflated diagonally");
    require(!grid.isBlocked(7, 7), "inflation exceeded Euclidean clearance");

    Mapping partial(0.0, 0.0, 10.0, 10.0, 1.0);
    for (int y = 0; y < 5; ++y) {
        partial.update({{0.0, 9.4, false}}, 0.5, y + 0.5);
    }
    DiscoveredPlanningGrid partialGrid(partial, 0.7);
    require(partialGrid.isBlocked(7, 6), "unknown cell is traversable");
    require(AStar::findPath(partialGrid, {2, 2}, {7, 6}).empty(),
            "unknown goal produced a route");
    require(!AStar::findPath(partialGrid, {2, 2}, {7, 2}).empty(),
            "known free region did not route beside unknown region");
}

void testRouteInvalidationAndController() {
    Mapping map = freeMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    vector<GridCell> path = AStar::findPath(grid, {2, 4}, {7, 4});
    require(!path.empty(), "initial route missing");

    Navigator navigator;
    navigator.setPath(path);
    require(navigator.route() == path && validateRoute(grid, navigator.route()),
            "displayed Navigator route differs from accepted safe route");
    Robot robot;
    robot.x = 2.5;
    robot.y = 4.5;
    robot.theta = 0.0;
    require(!routeBlockedAhead(navigator, grid, path, true), "free route invalid");
    navigator.update(robot, grid);
    require(navigator.isActive() && robot.leftSpeed > 0.0 && robot.rightSpeed > 0.0,
            "navigator did not continue on a safe discovered path");

    for (int y = 0; y < 10; ++y) occupy(map, 5, y);
    grid.rebuild();
    require(routeBlockedAhead(navigator, grid, path, true),
            "newly observed obstacle did not invalidate route");
    require(AStar::findPath(grid, {2, 4}, {7, 4}).empty(),
            "occupied barrier did not block route");
}

class CornerGrid : public NavigationGrid {
public:
    bool isBlocked(int x, int y) const override {
        return x < 0 || y < 0 || x >= 3 || y >= 3 ||
               (x == 1 && y == 0) || (x == 0 && y == 1);
    }
    GridCell worldToGrid(double x, double y) const override {
        return {static_cast<int>(x), static_cast<int>(y)};
    }
    double gridToWorldX(int x) const override { return x + 0.5; }
    double gridToWorldY(int y) const override { return y + 0.5; }
    int getRows() const override { return 3; }
    int getCols() const override { return 3; }
};

void testCornerCuttingAndKnownMap() {
    CornerGrid corner;
    require(AStar::findPath(corner, {0, 0}, {1, 1}).empty(),
            "A* cut through a blocked diagonal corner");
    Navigator diagonalNavigator;
    const vector<GridCell> diagonal{{0, 0}, {1, 1}};
    diagonalNavigator.setPath(diagonal);
    require(routeBlockedAhead(diagonalNavigator, corner, diagonal, true),
            "blocked diagonal corner did not invalidate an existing route");

    World world;
    OccupancyGrid known(
        world, World::maxX - World::minX, World::maxY - World::minY,
        SimConfig::GRID_CELL_SIZE, SimConfig::PLANNING_RADIUS
    );
    const GridCell start = known.worldToGrid(SimConfig::START_X, SimConfig::START_Y);
    const GridCell goal = known.worldToGrid(-16.0, -12.5);
    require(!AStar::findPath(known, start, goal).empty(),
            "existing known-map A* route failed");
}

void testModeSwitchAndReset() {
    Mapping map = freeMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    Navigator navigator;
    Robot robot;
    vector<GridCell> path = AStar::findPath(grid, {2, 4}, {7, 4});
    optional<GridCell> goal = GridCell{7, 4};
    navigator.setPath(path);
    robot.leftSpeed = robot.rightSpeed = 0.5;
    PlanningMode mode = PlanningMode::KnownMap;
    switchPlanningMode(mode, navigator, robot, path, goal);
    require(mode == PlanningMode::DiscoveredMap && !navigator.isActive() &&
            path.empty() && !goal && robot.leftSpeed == 0.0 && robot.rightSpeed == 0.0,
            "mode switch did not cancel old-map navigation");

    path = AStar::findPath(grid, {2, 4}, {7, 4});
    goal = GridCell{7, 4};
    navigator.setPath(path);
    resetDiscoveredMap(map, grid, mode, navigator, robot, path, goal);
    require(map.getCell(4, 4) == CellState::Unknown && grid.isBlocked(4, 4) &&
            !navigator.isActive() && path.empty() && !goal,
            "discovered reset did not invalidate navigation");

    switchPlanningMode(mode, navigator, robot, path, goal);
    require(mode == PlanningMode::KnownMap, "known-map mode was not restored");

    path = {{2, 4}, {3, 4}};
    goal = GridCell{3, 4};
    navigator.setPath(path);
    resetDiscoveredMap(map, grid, mode, navigator, robot, path, goal);
    require(navigator.isActive() && !path.empty() && goal,
            "discovered-map reset cancelled known-map navigation");
}

void testInitialRealScan() {
    World world;
    Robot robot;
    robot.x = SimConfig::START_X;
    robot.y = SimConfig::START_Y;
    Mapping map(
        World::minX, World::minY, World::maxX, World::maxY,
        SimConfig::GRID_CELL_SIZE
    );
    Lidar lidar(72, 6.0);
    lidar.scan(robot, world);
    map.update(lidar.getReadings(), robot.x, robot.y);
    DiscoveredPlanningGrid grid(map, SimConfig::PLANNING_RADIUS);
    const GridCell start = grid.worldToGrid(robot.x, robot.y);
    require(!grid.isBlocked(start.x, start.y),
            "first real LiDAR scan leaves start unsafe for discovered planning");
    bool nearbyRoute = false;
    for (int dy = -4; dy <= 4 && !nearbyRoute; ++dy) {
        for (int dx = -4; dx <= 4; ++dx) {
            const GridCell goal{start.x + dx, start.y + dy};
            if (!grid.isBlocked(goal.x, goal.y) &&
                AStar::findPath(grid, start, goal).size() > 1) {
                nearbyRoute = true;
                break;
            }
        }
    }
    require(nearbyRoute, "first real LiDAR scan has no nearby safe route");
}

void testExactRequestedArrival() {
    Mapping map = freeMap();
    DiscoveredPlanningGrid grid(map, 0.7);
    Robot robot;
    robot.x = 2.5; robot.y = 4.5; robot.theta = 0.0;
    const WorldPoint requested{6.2, 4.7};
    const auto route = AStar::findPath(grid, {2, 4}, {6, 4});
    Navigator navigator;
    navigator.setPath(route, requested);
    for (int tick = 0; tick < 1200 && navigator.isActive(); ++tick) {
        navigator.update(robot, grid);
        robot.update(SimConfig::DT);
    }
    require(!navigator.isActive() &&
            hypot(robot.x - requested.x, robot.y - requested.y) <=
                Navigator::GOAL_TOLERANCE,
            "Navigator reported arrival at cell center instead of exact goal");
}

void testPhysicalSafetyWithoutMapping() {
    World world;
    Robot robot;
    robot.x = -17.0; robot.y = -11.5;
    robot.theta = 1.5707963267948966;
    robot.leftSpeed = robot.rightSpeed = 1.0;
    for (int i = 0; i < 180; ++i) {
        advanceRobotSafely(robot, world, SimConfig::DT);
        require(!world.collides(robot.x, robot.y, SimConfig::ROBOT_RADIUS),
                "physical safety depended on mapping or planner");
    }
    require(robot.y < -10.4, "emergency stop did not prevent wall contact");
}

} // namespace

int main() {
    try {
        testDiscoveredRoutesAndClearance();
        testRouteInvalidationAndController();
        testCornerCuttingAndKnownMap();
        testModeSwitchAndReset();
        testInitialRealScan();
        testExactRequestedArrival();
        testPhysicalSafetyWithoutMapping();
        cout << "Planning tests passed\n";
    } catch (const exception& e) {
        cerr << "Planning test failed: " << e.what() << '\n';
        return 1;
    }
}
