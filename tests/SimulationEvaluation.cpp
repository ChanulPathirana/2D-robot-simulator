#include "AStar.hpp"
#include "DiscoveredPlanningGrid.hpp"
#include "FrontierExplorer.hpp"
#include "Lidar.hpp"
#include "Mapping.hpp"
#include "Navigator.hpp"
#include "OccupancyGrid.hpp"
#include "RouteDiagnostics.hpp"
#include "RouteValidation.hpp"
#include "SimulationConfig.hpp"
#include "SimulationStep.hpp"
#include "World.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;
using Clock = chrono::steady_clock;

enum class Task { Known, Discovered, Independent };
struct Scenario {
    const char* id;
    WorldPoint start;
    double heading;
    optional<WorldPoint> goal;
    Task task;
    double limit;
    bool extraMoving = false;
    bool startKnownThenSwitch = false;
};

struct Samples {
    int count = 0;
    double total = 0.0;
    double maximum = 0.0;
    void add(double ms) { ++count; total += ms; maximum = max(maximum, ms); }
    double average() const { return count ? total / count : 0.0; }
};
double elapsedMs(Clock::time_point started) {
    return chrono::duration<double, milli>(Clock::now() - started).count();
}
double coverage(const Mapping& map) {
    int observed = 0;
    for (int y = 0; y < map.getRows(); ++y)
        for (int x = 0; x < map.getCols(); ++x)
            observed += map.hasObservation(x, y);
    return 100.0 * observed / (map.getRows() * map.getCols());
}
const char* taskName(Task task) {
    return task == Task::Known ? "Known Map" : "Discovered Map";
}
const char* stateName(ExplorationState state) {
    switch (state) {
        case ExplorationState::Completed: return "completed observations";
        case ExplorationState::Limited: return "no reachable frontier";
        case ExplorationState::Blocked: return "repeated failures";
        case ExplorationState::GoalUnavailable: return "goal unavailable";
        case ExplorationState::InvalidDestination: return "invalid destination";
        default: return "bounded duration";
    }
}

struct Result {
    string outcome;
    string reason;
    double duration = 0.0, distance = 0.0, error = 0.0;
    double initialCoverage = 0.0, finalCoverage = 0.0;
    int replans = 0, frontierVisits = 0, stalls = 0, rollbacks = 0;
    int routeInvalidations = 0;
    int viewpointOscillations = 0, recoveryEntries = 0;
    double maxCoveragePlateau = 0.0, maxStationarySeconds = 0.0;
    int unknown = 0, free = 0, occupied = 0, frontierRegions = 0;
    int reachableCandidates = 0, failedFrontiers = 0;
    Samples mapping, rebuild, selecting, update;
    vector<pair<double, double>> coverageHistory;
};

Result run(const Scenario& scenario) {
    World world;
    Robot robot;
    robot.x = scenario.start.x;
    robot.y = scenario.start.y;
    robot.theta = scenario.heading;
    if (world.collides(robot.x, robot.y, SimConfig::ROBOT_RADIUS))
        throw runtime_error(string(scenario.id) + ": invalid starting pose");

    OccupancyGrid known(world, World::maxX - World::minX,
                        World::maxY - World::minY,
                        SimConfig::GRID_CELL_SIZE, SimConfig::PLANNING_RADIUS);
    Mapping map(World::minX, World::minY, World::maxX, World::maxY,
                SimConfig::GRID_CELL_SIZE);
    DiscoveredPlanningGrid discovered(map, SimConfig::PLANNING_RADIUS);
    FrontierExplorer explorer(map, discovered, 6.0);
    Lidar lidar(72, 6.0);
    Navigator navigator;
    vector<GridCell> route;
    Result result;
    auto scan = [&] {
        lidar.scan(robot, world);
        auto started = Clock::now();
        map.update(lidar.getReadings(), robot.x, robot.y);
        result.mapping.add(elapsedMs(started));
        started = Clock::now();
        discovered.rebuild();
        result.rebuild.add(elapsedMs(started));
        explorer.onMapScan();
    };
    scan();
    result.initialCoverage = coverage(map);
    result.coverageHistory.push_back({0.0, result.initialCoverage});

    bool knownTask = scenario.task == Task::Known;
    int acceptedRoutes = 0;
    bool hasRoute = false;
    std::uint64_t observedRouteRevision = 0;
    int attemptsWithoutProgress = 0, failedPlans = 0;
    double progressX = robot.x, progressY = robot.y;
    auto knownPlan = [&] {
        if (!scenario.goal) return false;
        if (++attemptsWithoutProgress > 5) {
            result.outcome = "Incomplete";
            result.reason = "repeated replans without progress";
            return false;
        }
        const GridCell start = known.worldToGrid(robot.x, robot.y);
        const GridCell target = known.worldToGrid(scenario.goal->x, scenario.goal->y);
        route = AStar::findPath(known, start, target);
        if (!validateRoute(known, route)) {
            navigator.stop(); route.clear();
            if (++failedPlans >= 5) {
                result.outcome = "NoSafeRoute";
                result.reason = "five failed plans";
            }
            return false;
        }
        navigator.setPath(route, scenario.goal);
        failedPlans = 0;
        if (acceptedRoutes++) ++result.replans;
        return true;
    };
    if (scenario.startKnownThenSwitch) {
        knownPlan();
        navigator.stop(); route.clear();
        hasRoute = false;
        acceptedRoutes = 0;
        knownTask = false;
    }
    if (knownTask) {
        if (!knownPlan()) {
            result.outcome = "NoSafeRoute";
            result.reason = "known-map A* found no safe route";
        }
    } else if (scenario.task == Task::Independent) {
        explorer.start(robot, navigator, route);
    } else {
        const auto request = explorer.startGoal(*scenario.goal, robot,
                                                navigator, route);
        if (request == GoalRequestResult::Occupied ||
            request == GoalRequestResult::OutsideMap) {
            result.outcome = "NoSafeRoute";
            result.reason = request == GoalRequestResult::Occupied
                ? "occupied destination" : "out-of-bounds destination";
        }
    }

    bool freshScan = false;
    double retry = 0.0, still = 0.0, scanClock = 0.0;
    double stationary = 0.0, lastCoverageGrowth = 0.0;
    int lastObserved = explorer.statistics().observed;
    vector<GridCell> selectedViewpoints;
    ExplorationState previousState = explorer.state();
    for (int tick = 0; result.outcome.empty() &&
         tick < static_cast<int>(scenario.limit / SimConfig::DT); ++tick) {
        auto updateStarted = Clock::now();
        // The new obstacle crosses an already planned route, then parks clear.
        const bool discoveryBlock = string(scenario.id) == "new_obstacle";
        if (scenario.extraMoving && tick == (discoveryBlock ? 90 : 180))
            world.movingObstacles.push_back(
                {{discoveryBlock ? -16.1 : -14.1, -13.1, 0.7, 0.7},
                 false, -13.1, -11.2, 0.6});
        world.update(SimConfig::DT, robot.x, robot.y, SimConfig::ROBOT_RADIUS);
        if (scenario.extraMoving && world.movingObstacles.size() > 4 &&
            world.movingObstacles.back().shape.y >= -11.25)
            world.movingObstacles.back().speed = 0.0;
        if (hypot(robot.x - progressX, robot.y - progressY) > 0.75) {
            progressX = robot.x; progressY = robot.y;
            attemptsWithoutProgress = 0;
        }
        if (knownTask) {
            retry = max(0.0, retry - SimConfig::DT);
            if (navigator.isActive() &&
                routeBlockedAhead(navigator, known, navigator.route(), false)) {
                ++result.routeInvalidations;
                navigator.stop(); route.clear(); retry = 0.75;
            }
            if (!navigator.isActive() && retry <= 0.0 &&
                scenario.goal && result.error > Navigator::GOAL_TOLERANCE)
                if (!knownPlan()) retry = 1.5;
            if (navigator.isActive()) navigator.update(robot, known);
        } else {
            const bool selecting = explorer.state() == ExplorationState::SelectingFrontier;
            auto selectionStarted = Clock::now();
            explorer.update(SimConfig::DT, freshScan, robot, navigator, route);
            const auto state = explorer.state();
            if (state != previousState &&
                (state == ExplorationState::Recovering ||
                 state == ExplorationState::TemporarilyBlocked))
                ++result.recoveryEntries;
            previousState = state;
            if (hasRoute && explorer.state() == ExplorationState::TemporarilyBlocked)
                ++result.routeInvalidations;
            if (selecting) result.selecting.add(elapsedMs(selectionStarted));
        }
        freshScan = false;
        const bool nowHasRoute = navigator.isActive();
        if (!knownTask && nowHasRoute &&
            navigator.routeRevision() != observedRouteRevision) {
            observedRouteRevision = navigator.routeRevision();
            if (acceptedRoutes++) ++result.replans;
            if (!validateRoute(discovered, navigator.route()))
                throw runtime_error(string(scenario.id) + ": unsafe accepted route");
            if (explorer.goal()) {
                const GridCell chosen = *explorer.goal();
                if (selectedViewpoints.size() >= 2 &&
                    chosen == selectedViewpoints[selectedViewpoints.size() - 2])
                    ++result.viewpointOscillations;
                selectedViewpoints.push_back(chosen);
            }
        }
        hasRoute = nowHasRoute;
        const double beforeX = robot.x, beforeY = robot.y, beforeTheta = robot.theta;
        result.rollbacks += advanceRobotSafely(robot, world, SimConfig::DT);
        if (world.collides(robot.x, robot.y, SimConfig::ROBOT_RADIUS))
            throw runtime_error(string(scenario.id) + ": physical collision");
        const double delta = hypot(robot.x - beforeX, robot.y - beforeY);
        result.distance += delta;
        const double motion = delta + 0.2 * abs(remainder(
            robot.theta - beforeTheta, 2.0 * numbers::pi));
        still = nowHasRoute && motion < 1e-5 ? still + SimConfig::DT : 0.0;
        if (still >= 2.5) { ++result.stalls; still = 0.0; }
        stationary = motion < 1e-5 ? stationary + SimConfig::DT : 0.0;
        result.maxStationarySeconds = max(result.maxStationarySeconds, stationary);

        scanClock += SimConfig::DT;
        if (scanClock >= 0.1 - 1e-9) {
            scan(); freshScan = true; scanClock -= 0.1;
            const int observed = explorer.statistics().observed;
            if (observed > lastObserved) {
                result.maxCoveragePlateau = max(result.maxCoveragePlateau,
                    (tick + 1) * SimConfig::DT - lastCoverageGrowth);
                lastCoverageGrowth = (tick + 1) * SimConfig::DT;
                lastObserved = observed;
            }
        }
        result.duration = (tick + 1) * SimConfig::DT;
        if ((tick + 1) % 600 == 0)
            result.coverageHistory.push_back({result.duration, coverage(map)});
        result.error = scenario.goal ? hypot(robot.x - scenario.goal->x,
                                             robot.y - scenario.goal->y) : 0.0;
        if (scenario.goal && result.error <= Navigator::GOAL_TOLERANCE) {
            result.outcome = "Reached";
        } else if (!knownTask && !explorer.isActive()) {
            result.outcome = explorer.state() == ExplorationState::GoalReached
                ? "Reached" : "Incomplete";
            result.reason = stateName(explorer.state());
        }
        result.update.add(elapsedMs(updateStarted));
    }
    if (result.outcome.empty()) {
        result.outcome = "TimedOut";
        result.reason = "bounded simulation duration";
    }
    result.error = scenario.goal ? hypot(robot.x - scenario.goal->x,
                                         robot.y - scenario.goal->y) : 0.0;
    result.finalCoverage = coverage(map);
    result.maxCoveragePlateau = max(result.maxCoveragePlateau,
                                    result.duration - lastCoverageGrowth);
    if (result.coverageHistory.back().first != result.duration)
        result.coverageHistory.push_back({result.duration, result.finalCoverage});
    const auto& stats = explorer.statistics();
    result.frontierVisits = stats.visitedViewpoints;
    result.unknown = stats.unknown;
    result.free = stats.free;
    result.occupied = stats.occupied;
    result.frontierRegions = stats.frontierRegions;
    result.reachableCandidates = stats.reachableCandidates;
    result.failedFrontiers = stats.failedViewpoints;
    if (scenario.extraMoving &&
        (result.outcome != "Reached" || result.routeInvalidations == 0))
        throw runtime_error(string(scenario.id) +
                            ": obstacle recovery did not reach goal after invalidation");
    return result;
}

int main(int argc, char** argv) {
    try {
        if (argc != 2 && argc != 3)
            throw runtime_error("usage: simulation_evaluation OUTPUT.csv [SCENARIO_ID]");
        const vector<Scenario> scenarios{
            {"short_clear", {-17.5,-12.5}, 0, WorldPoint{-16.0,-12.5}, Task::Known, 25},
            {"long_corridor", {-17.5,-12.5}, 0, WorldPoint{-11.5,-12.5}, Task::Known, 60},
            {"several_turns", {-17.5,-12.5}, 0, WorldPoint{-11.5,-6.0}, Task::Known, 90},
            {"narrow_passage", {-12.0,-12.5}, 1.57, WorldPoint{-12.0,-8.0}, Task::Known, 90},
            {"initially_unknown", {-17.5,-12.5}, 0, WorldPoint{-11.0,-12.5}, Task::Discovered, 90},
            {"known_occupied", {-17.5,-12.5}, 0, WorldPoint{-17.0,-7.0}, Task::Known, 5},
            {"out_of_bounds", {-17.5,-12.5}, 0, WorldPoint{21.0,0.0}, Task::Discovered, 5},
            {"unreachable", {-17.5,-12.5}, 0, WorldPoint{-17.0,-9.9}, Task::Known, 5},
            {"frontier_discovery", {-17.5,-12.5}, 0, WorldPoint{-11.0,-12.5}, Task::Discovered, 90},
            {"new_obstacle", {-17.5,-12.5}, 0, WorldPoint{-14.5,-12.5}, Task::Discovered, 90, true},
            {"multi_frontier_goal", {-17.5,-12.5}, 0, WorldPoint{-10.0,-7.5}, Task::Discovered, 120},
            {"mode_switch", {-17.5,-12.5}, 0, WorldPoint{-11.0,-12.5}, Task::Discovered, 90, false, true},
            {"independent_exploration", {-17.5,-12.5}, 0, nullopt, Task::Independent, 120},
            {"dynamic_crossing", {-17.5,-12.5}, 0, WorldPoint{-11.5,-12.5}, Task::Known, 90, true}
        };
        ofstream out(argv[1]);
        if (!out) throw runtime_error("cannot create CSV output");
        out << "Scenario ID,Starting pose,Goal,Planning mode,Outcome,Simulated duration,Travel distance,Final goal error,Initial coverage,Final coverage,Replan count,Frontier visits,Collision rollbacks,Failure reason,Navigation stalls,Route invalidations,Viewpoint oscillations,Recovery entries,Max coverage plateau s,Max stationary s,Unknown,Free,Occupied,Frontier regions,Reachable candidates,Failed frontiers,Mapping avg ms,Mapping max ms,Grid avg ms,Grid max ms,Frontier selection avg ms,Frontier selection max ms,Update avg ms,Update max ms,AStar calls,AStar avg ms,AStar max ms,Coverage timeline\n";
        out << fixed << setprecision(4);
        for (const auto& scenario : scenarios) {
            if (argc == 3 && scenario.id != string(argv[2])) continue;
            AStar::resetTiming();
            const auto result = run(scenario);
            const auto astar = AStar::timing();
            out << scenario.id << ",\"" << scenario.start.x << ' ' << scenario.start.y
                << ' ' << scenario.heading << "\",\"";
            if (scenario.goal) out << scenario.goal->x << ' ' << scenario.goal->y;
            out << "\"," << taskName(scenario.task) << ',' << result.outcome << ','
                << result.duration << ',' << result.distance << ',' << result.error << ','
                << result.initialCoverage << ',' << result.finalCoverage << ','
                << result.replans << ',' << result.frontierVisits << ',' << result.rollbacks
                << ',' << result.reason << ',' << result.stalls << ','
                << result.routeInvalidations << ',' << result.viewpointOscillations
                << ',' << result.recoveryEntries << ','
                << result.maxCoveragePlateau << ',' << result.maxStationarySeconds
                << ',' << result.unknown
                << ',' << result.free << ',' << result.occupied << ','
                << result.frontierRegions << ',' << result.reachableCandidates << ','
                << result.failedFrontiers << ',' << result.mapping.average() << ','
                << result.mapping.maximum << ',' << result.rebuild.average() << ','
                << result.rebuild.maximum << ',' << result.selecting.average() << ','
                << result.selecting.maximum << ',' << result.update.average() << ','
                << result.update.maximum << ',' << astar.calls << ','
                << (astar.calls ? astar.totalMs / astar.calls : 0.0) << ','
                << astar.maxMs << ",\"";
            for (const auto& [time, observed] : result.coverageHistory)
                out << time << ':' << observed << ';';
            out << "\"\n";
            cout << scenario.id << ": " << result.outcome << " in "
                 << result.duration << " s, error " << result.error << " m\n";
        }
    } catch (const exception& error) {
        cerr << "Evaluation failed: " << error.what() << '\n';
        return 1;
    }
}
