#include "FrontierExplorer.hpp"

#include "AStar.hpp"
#include "RouteValidation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>
#include <utility>

using namespace std;

namespace {

constexpr array<GridCell, 4> CARDINAL{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr array<GridCell, 8> NEIGHBORS{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
}};
constexpr int INFORMATION_RAYS = 24;
constexpr size_t HISTORY_LIMIT = 32;

bool inside(const Mapping& map, int x, int y) {
    return x >= 0 && y >= 0 && x < map.getCols() && y < map.getRows();
}

size_t indexOf(int x, int y, int cols) {
    return static_cast<size_t>(y) * cols + x;
}

bool sameCell(GridCell a, GridCell b) {
    return a.x == b.x && a.y == b.y;
}

} // namespace

FrontierExplorer::FrontierExplorer(
    const Mapping& map,
    const DiscoveredPlanningGrid& grid,
    double lidarRange,
    FrontierExplorerSettings settings
) : map(map), grid(grid), lidarRange(lidarRange), settings(settings) {
    if (!isfinite(lidarRange) || lidarRange <= 0.0 ||
        settings.minimumFrontierSize < 1 ||
        settings.maximumRecoveryAttempts < 1 ||
        !isfinite(settings.maximumFrontierOffset) ||
        settings.maximumFrontierOffset <= 0.0 ||
        !isfinite(settings.minimumTravel) ||
        settings.minimumTravel < 0.0 ||
        !isfinite(settings.informationGainWeight) ||
        settings.informationGainWeight < 0.0 ||
        !isfinite(settings.travelCostWeight) ||
        settings.travelCostWeight < 0.0 ||
        !isfinite(settings.clearanceWeight) ||
        settings.clearanceWeight < 0.0 ||
        !isfinite(settings.goalDistanceWeight) ||
        settings.goalDistanceWeight < 0.0 ||
        !isfinite(settings.failurePenalty) ||
        settings.failurePenalty < 0.0 ||
        !isfinite(settings.shortTravelPenaltyWeight) ||
        settings.shortTravelPenaltyWeight < 0.0 ||
        !isfinite(settings.observationSeconds) ||
        settings.observationSeconds <= 0.0 ||
        !isfinite(settings.recoverySeconds) ||
        settings.recoverySeconds <= 0.0 ||
        !isfinite(settings.stallSeconds) ||
        settings.stallSeconds <= 0.0 ||
        settings.minimumObservedGain < 1 ||
        settings.mapChangeRetryCells < 0 ||
        !isfinite(settings.failureRetrySeconds) ||
        settings.failureRetrySeconds <= 0.0 ||
        !isfinite(settings.goalCheckSeconds) ||
        settings.goalCheckSeconds <= 0.0 ||
        !isfinite(settings.occupiedGoalWaitSeconds) ||
        settings.occupiedGoalWaitSeconds <= 0.0) {
        throw invalid_argument("Invalid frontier exploration settings");
    }
    refreshMapStats();
}

vector<FrontierRegion> FrontierExplorer::detectFrontiers(
    const Mapping& map, int minimumRegionSize
) {
    const int cols = map.getCols();
    const int rows = map.getRows();
    vector<uint8_t> frontier(static_cast<size_t>(cols) * rows, 0);
    vector<uint8_t> visited(frontier.size(), 0);
    vector<FrontierRegion> regions;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (map.getCell(x, y) != CellState::Free) continue;
            for (const GridCell step : CARDINAL) {
                const int nx = x + step.x;
                const int ny = y + step.y;
                if (inside(map, nx, ny) &&
                    map.getCell(nx, ny) == CellState::Unknown) {
                    frontier[indexOf(x, y, cols)] = 1;
                    break;
                }
            }
        }
    }

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            const size_t seed = indexOf(x, y, cols);
            if (!frontier[seed] || visited[seed]) continue;

            FrontierRegion region;
            queue<GridCell> open;
            open.push({x, y});
            visited[seed] = 1;
            while (!open.empty()) {
                const GridCell cell = open.front();
                open.pop();
                region.cells.push_back(cell);
                for (const GridCell step : NEIGHBORS) {
                    const int nx = cell.x + step.x;
                    const int ny = cell.y + step.y;
                    if (!inside(map, nx, ny)) continue;
                    const size_t next = indexOf(nx, ny, cols);
                    if (!frontier[next] || visited[next]) continue;
                    visited[next] = 1;
                    open.push({nx, ny});
                }
            }
            if (static_cast<int>(region.cells.size()) >= minimumRegionSize) {
                regions.push_back(move(region));
            }
        }
    }
    return regions;
}

void FrontierExplorer::refreshMapStats() {
    stats.unknown = 0;
    stats.free = 0;
    stats.occupied = 0;
    stats.observed = 0;
    for (int y = 0; y < map.getRows(); ++y) {
        for (int x = 0; x < map.getCols(); ++x) {
            stats.observed += map.hasObservation(x, y);
            switch (map.getCell(x, y)) {
                case CellState::Unknown: ++stats.unknown; break;
                case CellState::Free: ++stats.free; break;
                case CellState::Occupied: ++stats.occupied; break;
            }
        }
    }
    const int total = map.getRows() * map.getCols();
    stats.observedPercent = 100.0 * stats.observed / total;
}

vector<double> FrontierExplorer::travelCosts(GridCell start) const {
    const int cols = grid.getCols();
    const int rows = grid.getRows();
    const double infinity = numeric_limits<double>::infinity();
    vector<double> cost(static_cast<size_t>(cols) * rows, infinity);
    if (grid.isBlocked(start.x, start.y)) return cost;

    using Entry = pair<double, size_t>;
    priority_queue<Entry, vector<Entry>, greater<Entry>> open;
    const size_t first = indexOf(start.x, start.y, cols);
    cost[first] = 0.0;
    open.push({0.0, first});
    while (!open.empty()) {
        const auto [distance, current] = open.top();
        open.pop();
        if (distance > cost[current] + 1e-9) continue;
        const int x = static_cast<int>(current % cols);
        const int y = static_cast<int>(current / cols);
        for (const GridCell step : NEIGHBORS) {
            const int nx = x + step.x;
            const int ny = y + step.y;
            if (grid.isBlocked(nx, ny)) continue;
            if (step.x != 0 && step.y != 0 &&
                (grid.isBlocked(x + step.x, y) ||
                 grid.isBlocked(x, y + step.y))) continue;
            const size_t next = indexOf(nx, ny, cols);
            const double moveCost = map.getCellSize() *
                (step.x != 0 && step.y != 0 ? numbers::sqrt2 : 1.0);
            const double candidate = distance + moveCost;
            if (candidate + 1e-9 < cost[next]) {
                cost[next] = candidate;
                open.push({candidate, next});
            }
        }
    }
    return cost;
}

int FrontierExplorer::estimateInformationGain(GridCell viewpoint) const {
    const double x0 = grid.gridToWorldX(viewpoint.x);
    const double y0 = grid.gridToWorldY(viewpoint.y);
    const double step = map.getCellSize() * 0.5;
    int gain = 0;
    for (int ray = 0; ray < INFORMATION_RAYS; ++ray) {
        const double angle = 2.0 * numbers::pi * ray / INFORMATION_RAYS;
        const double dx = cos(angle);
        const double dy = sin(angle);
        for (double distance = step; distance <= lidarRange; distance += step) {
            const GridCell cell = grid.worldToGrid(x0 + dx * distance,
                                                   y0 + dy * distance);
            if (!inside(map, cell.x, cell.y)) break;
            const CellState state = map.getCell(cell.x, cell.y);
            if (state == CellState::Occupied) break;
            if (state == CellState::Unknown) {
                ++gain;
                break;
            }
        }
    }
    return gain;
}

int FrontierExplorer::nearbyClearCells(GridCell viewpoint) const {
    int clear = 0;
    for (const GridCell step : NEIGHBORS) {
        clear += !grid.isBlocked(viewpoint.x + step.x,
                                 viewpoint.y + step.y);
    }
    return clear;
}

double FrontierExplorer::historyPenalty(GridCell viewpoint) const {
    const double exclusion = settings.minimumTravel;
    double penalty = 0.0;
    for (const auto& item : failed) {
        const double distance = map.getCellSize() *
            hypot(viewpoint.x - item.cell.x, viewpoint.y - item.cell.y);
        if (distance > exclusion) continue;
        if (stats.observed - item.observedAtRecord < settings.mapChangeRetryCells &&
            elapsedSeconds - item.timeAtRecord < settings.failureRetrySeconds) {
            return numeric_limits<double>::infinity();
        }
        penalty += settings.failurePenalty * item.failures;
    }
    for (const auto& item : visited) {
        const double distance = map.getCellSize() *
            hypot(viewpoint.x - item.cell.x, viewpoint.y - item.cell.y);
        if (distance > exclusion) continue;
        if (stats.observed - item.observedAtRecord < settings.mapChangeRetryCells &&
            elapsedSeconds - item.timeAtRecord < settings.failureRetrySeconds) {
            return numeric_limits<double>::infinity();
        }
        penalty += settings.failurePenalty * 0.5;
    }
    return penalty;
}

optional<GridCell> FrontierExplorer::selectViewpoint(const Robot& robot) {
    const auto regions = detectFrontiers(map, settings.minimumFrontierSize);
    stats.frontierRegions = static_cast<int>(regions.size());
    stats.reachableCandidates = 0;
    stats.selectedTravelCost = 0.0;
    if (regions.empty()) return nullopt;

    const int cols = map.getCols();
    const int rows = map.getRows();
    const GridCell start = grid.worldToGrid(robot.x, robot.y);
    const vector<double> costs = travelCosts(start);
    vector<uint8_t> nearFrontier(static_cast<size_t>(cols) * rows, 0);
    const int reach = static_cast<int>(ceil(settings.maximumFrontierOffset /
                                             map.getCellSize()));
    const double reachSquared = settings.maximumFrontierOffset *
                                settings.maximumFrontierOffset;
    for (const auto& region : regions) {
        for (const GridCell frontier : region.cells) {
            for (int dy = -reach; dy <= reach; ++dy) {
                for (int dx = -reach; dx <= reach; ++dx) {
                    const int x = frontier.x + dx;
                    const int y = frontier.y + dy;
                    if (!inside(map, x, y)) continue;
                    const double squared = map.getCellSize() * map.getCellSize() *
                                           (dx * dx + dy * dy);
                    if (squared <= reachSquared) {
                        nearFrontier[indexOf(x, y, cols)] = 1;
                    }
                }
            }
        }
    }

    optional<GridCell> best;
    double bestScore = -numeric_limits<double>::infinity();
    double bestCost = numeric_limits<double>::infinity();
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            const size_t index = indexOf(x, y, cols);
            if (!nearFrontier[index] || grid.isBlocked(x, y) ||
                !isfinite(costs[index]) || costs[index] < map.getCellSize() * 0.5) {
                continue;
            }
            const GridCell candidate{x, y};
            const double penalty = historyPenalty(candidate);
            if (!isfinite(penalty)) continue;
            const int gain = estimateInformationGain(candidate);
            if (gain == 0) continue;
            ++stats.reachableCandidates;
            const double score = settings.informationGainWeight * gain -
                                 settings.travelCostWeight * costs[index] +
                                 settings.clearanceWeight * nearbyClearCells(candidate) -
                                 penalty -
                                 settings.shortTravelPenaltyWeight *
                                     max(0.0, settings.minimumTravel - costs[index]) -
                                 (requestedGoal
                                     ? settings.goalDistanceWeight * hypot(
                                           grid.gridToWorldX(x) - requestedGoal->x,
                                           grid.gridToWorldY(y) - requestedGoal->y)
                                     : 0.0);
            if (score > bestScore + 1e-9 ||
                (abs(score - bestScore) <= 1e-9 && costs[index] < bestCost - 1e-9)) {
                best = candidate;
                bestScore = score;
                bestCost = costs[index];
            }
        }
    }
    if (best) stats.selectedTravelCost = bestCost;
    return best;
}

void FrontierExplorer::recordFailure(GridCell viewpoint) {
    ++stats.failedViewpoints;
    for (auto& item : failed) {
        if (sameCell(item.cell, viewpoint)) {
            ++item.failures;
            item.observedAtRecord = stats.observed;
            item.timeAtRecord = elapsedSeconds;
            return;
        }
    }
    if (failed.size() >= HISTORY_LIMIT) failed.erase(failed.begin());
    failed.push_back({viewpoint, stats.observed, 1, elapsedSeconds});
}

void FrontierExplorer::clearRoute(
    Robot& robot, Navigator& navigator, vector<GridCell>& path
) {
    navigator.stop();
    path.clear();
    currentGoal.reset();
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;
}

void FrontierExplorer::enterRecovery(
    Robot& robot, Navigator& navigator, vector<GridCell>& path,
    RecoveryCause cause
) {
    clearRoute(robot, navigator, path);
    ++recoveryAttempts;
    stallTimer = 0.0;
    if (recoveryAttempts >= settings.maximumRecoveryAttempts) {
        if (currentTask == ExplorationTaskMode::GoalDirected) {
            failureReason = GoalFailureReason::NoSafeProgress;
        }
        currentState = currentTask == ExplorationTaskMode::GoalDirected
            ? ExplorationState::GoalUnavailable
            : (cause == RecoveryCause::NoCandidate
                ? ExplorationState::Limited : ExplorationState::Blocked);
    } else {
        currentState = cause == RecoveryCause::RouteBlocked
            ? ExplorationState::TemporarilyBlocked
            : ExplorationState::Recovering;
        stateTimer = settings.recoverySeconds;
    }
}

void FrontierExplorer::start(
    Robot& robot, Navigator& navigator, vector<GridCell>& path
) {
    clearRoute(robot, navigator, path);
    currentTask = ExplorationTaskMode::Independent;
    requestedGoal.reset();
    failureReason = GoalFailureReason::None;
    stats = {};
    failed.clear();
    visited.clear();
    recoveryAttempts = 0;
    stallTimer = 0.0;
    elapsedSeconds = 0.0;
    goalCheckTimer = 0.0;
    occupiedGoalTimer = 0.0;
    lastGoalCheckRevision = grid.revision();
    lastX = robot.x;
    lastY = robot.y;
    lastTheta = robot.theta;
    progressAnchorX = robot.x;
    progressAnchorY = robot.y;
    refreshMapStats();
    currentState = ExplorationState::SelectingFrontier;
}

GoalRequestResult FrontierExplorer::startGoal(
    WorldPoint requested,
    Robot& robot, Navigator& navigator, vector<GridCell>& path
) {
    start(robot, navigator, path);
    currentTask = ExplorationTaskMode::GoalDirected;
    requestedGoal = requested;
    if (!isfinite(requested.x) || !isfinite(requested.y) ||
        requested.x < map.getMinX() || requested.x >= map.getMaxX() ||
        requested.y < map.getMinY() || requested.y >= map.getMaxY()) {
        currentState = ExplorationState::InvalidDestination;
        failureReason = GoalFailureReason::OutsideMap;
        return GoalRequestResult::OutsideMap;
    }
    const GridCell destination = grid.worldToGrid(requested.x, requested.y);
    if (map.getCell(destination.x, destination.y) == CellState::Occupied) {
        currentState = ExplorationState::InvalidDestination;
        failureReason = GoalFailureReason::OccupiedAtClick;
        return GoalRequestResult::Occupied;
    }
    if (tryPlanRequestedGoal(robot, navigator, path)) {
        return GoalRequestResult::Direct;
    }
    return GoalRequestResult::Exploring;
}

bool FrontierExplorer::tryPlanRequestedGoal(
    Robot& robot, Navigator& navigator, vector<GridCell>& path
) {
    if (!requestedGoal) return false;
    lastGoalCheckRevision = grid.revision();
    goalCheckTimer = settings.goalCheckSeconds;
    const GridCell target = grid.worldToGrid(requestedGoal->x, requestedGoal->y);
    const GridCell start = grid.worldToGrid(robot.x, robot.y);
    if (map.getCell(target.x, target.y) != CellState::Free ||
        grid.isBlocked(target.x, target.y) ||
        grid.isBlocked(start.x, start.y)) return false;
    vector<GridCell> directPath = AStar::findPath(grid, start, target);
    if (directPath.empty()) return false;
    clearRoute(robot, navigator, path);
    path = move(directPath);
    navigator.setPath(path, requestedGoal);
    currentState = ExplorationState::NavigatingToGoal;
    goalCheckTimer = settings.goalCheckSeconds;
    stallTimer = 0.0;
    return true;
}

void FrontierExplorer::stop(
    Robot& robot, Navigator& navigator, vector<GridCell>& path
) {
    clearRoute(robot, navigator, path);
    requestedGoal.reset();
    failureReason = GoalFailureReason::None;
    currentTask = ExplorationTaskMode::Independent;
    currentState = ExplorationState::Stopped;
}

void FrontierExplorer::onMapScan() {
    refreshMapStats();
}

void FrontierExplorer::onMappingReset(
    Robot& robot, Navigator& navigator, vector<GridCell>& path
) {
    if (isActive()) stop(robot, navigator, path);
    failed.clear();
    visited.clear();
    requestedGoal.reset();
    failureReason = GoalFailureReason::None;
    currentTask = ExplorationTaskMode::Independent;
    elapsedSeconds = 0.0;
    stats = {};
    refreshMapStats();
    if (currentState != ExplorationState::Idle) {
        currentState = ExplorationState::Stopped;
    }
}

void FrontierExplorer::update(
    double dt,
    bool freshScan,
    Robot& robot,
    Navigator& navigator,
    vector<GridCell>& path
) {
    if (!isActive() || dt <= 0.0) return;
    elapsedSeconds += dt;
    const double distance = hypot(robot.x - lastX, robot.y - lastY);
    const double rotation = abs(remainder(robot.theta - lastTheta,
                                           2.0 * numbers::pi));
    stats.distanceTravelled += distance;
    lastX = robot.x;
    lastY = robot.y;
    lastTheta = robot.theta;
    if (hypot(robot.x - progressAnchorX,
              robot.y - progressAnchorY) > 0.75) {
        progressAnchorX = robot.x;
        progressAnchorY = robot.y;
        recoveryAttempts = 0;
    }
    if (freshScan) refreshMapStats();
    goalCheckTimer = max(0.0, goalCheckTimer - dt);

    if (currentTask == ExplorationTaskMode::GoalDirected && requestedGoal) {
        const GridCell target = grid.worldToGrid(requestedGoal->x,
                                                  requestedGoal->y);
        occupiedGoalTimer = map.getCell(target.x, target.y) == CellState::Occupied
            ? occupiedGoalTimer + dt : 0.0;
        if (occupiedGoalTimer >= settings.occupiedGoalWaitSeconds) {
            clearRoute(robot, navigator, path);
            failureReason = GoalFailureReason::OccupiedAfterObservation;
            currentState = ExplorationState::GoalUnavailable;
            return;
        }
    }

    if (currentTask == ExplorationTaskMode::GoalDirected &&
        freshScan && goalCheckTimer <= 0.0 &&
        grid.revision() != lastGoalCheckRevision &&
        currentState != ExplorationState::NavigatingToGoal) {
        goalCheckTimer = settings.goalCheckSeconds;
        if (tryPlanRequestedGoal(robot, navigator, path)) return;
    }

    switch (currentState) {
        case ExplorationState::SelectingFrontier: {
            if (currentTask == ExplorationTaskMode::GoalDirected &&
                goalCheckTimer <= 0.0 &&
                grid.revision() != lastGoalCheckRevision &&
                tryPlanRequestedGoal(robot, navigator, path)) break;
            const GridCell startCell = grid.worldToGrid(robot.x, robot.y);
            if (grid.isBlocked(startCell.x, startCell.y)) {
                enterRecovery(robot, navigator, path, RecoveryCause::NoCandidate);
                break;
            }
            const auto viewpoint = selectViewpoint(robot);
            if (!viewpoint) {
                if (stats.unknown == 0) {
                    clearRoute(robot, navigator, path);
                    currentState = currentTask == ExplorationTaskMode::GoalDirected
                        ? ExplorationState::GoalUnavailable
                        : ExplorationState::Completed;
                    if (currentTask == ExplorationTaskMode::GoalDirected) {
                        failureReason = GoalFailureReason::NoSafeProgress;
                    }
                } else {
                    enterRecovery(robot, navigator, path, RecoveryCause::NoCandidate);
                }
                break;
            }
            path = AStar::findPath(grid, startCell, *viewpoint);
            if (path.empty()) {
                recordFailure(*viewpoint);
                enterRecovery(robot, navigator, path, RecoveryCause::MotionFailure);
                break;
            }
            currentGoal = viewpoint;
            observedAtTargetStart = stats.observed;
            navigator.setPath(path);
            currentState = ExplorationState::Navigating;
            stallTimer = 0.0;
            break;
        }
        case ExplorationState::NavigatingToGoal: {
            const GridCell here = grid.worldToGrid(robot.x, robot.y);
            const GridCell target = grid.worldToGrid(requestedGoal->x,
                                                      requestedGoal->y);
            if (grid.isBlocked(here.x, here.y) ||
                grid.isBlocked(target.x, target.y) ||
                routeBlockedAhead(navigator, grid, path, true)) {
                enterRecovery(robot, navigator, path, RecoveryCause::RouteBlocked);
                break;
            }
            navigator.update(robot, grid);
            if (!navigator.isActive()) {
                if (hypot(robot.x - requestedGoal->x,
                          robot.y - requestedGoal->y) <= Navigator::GOAL_TOLERANCE) {
                    clearRoute(robot, navigator, path);
                    currentState = ExplorationState::GoalReached;
                } else {
                    enterRecovery(robot, navigator, path,
                                  RecoveryCause::MotionFailure);
                }
                break;
            }
            stallTimer = distance + 0.2 * rotation < 1e-5
                ? stallTimer + dt : 0.0;
            if (stallTimer >= settings.stallSeconds) {
                enterRecovery(robot, navigator, path, RecoveryCause::MotionFailure);
            }
            break;
        }
        case ExplorationState::Navigating: {
            const GridCell here = grid.worldToGrid(robot.x, robot.y);
            if (grid.isBlocked(here.x, here.y) ||
                routeBlockedAhead(navigator, grid, path, true)) {
                if (currentGoal) recordFailure(*currentGoal);
                enterRecovery(robot, navigator, path, RecoveryCause::RouteBlocked);
                break;
            }
            navigator.update(robot, grid);
            if (!navigator.isActive()) {
                path.clear();
                robot.leftSpeed = robot.rightSpeed = 0.0;
                stateTimer = 0.0;
                currentState = ExplorationState::Observing;
                break;
            }
            stallTimer = distance + 0.2 * rotation < 1e-5
                ? stallTimer + dt : 0.0;
            if (stallTimer >= settings.stallSeconds) {
                if (currentGoal) recordFailure(*currentGoal);
                enterRecovery(robot, navigator, path, RecoveryCause::MotionFailure);
            }
            break;
        }
        case ExplorationState::Observing: {
            robot.leftSpeed = robot.rightSpeed = 0.0;
            stateTimer += dt;
            const int gain = stats.observed - observedAtTargetStart;
            if (gain >= settings.minimumObservedGain ||
                stateTimer >= settings.observationSeconds) {
                if (gain >= settings.minimumObservedGain) {
                    ++stats.visitedViewpoints;
                    if (visited.size() >= HISTORY_LIMIT) visited.erase(visited.begin());
                    visited.push_back({*currentGoal, stats.observed, 0,
                                       elapsedSeconds});
                    recoveryAttempts = 0;
                    clearRoute(robot, navigator, path);
                    currentState = ExplorationState::SelectingFrontier;
                } else {
                    recordFailure(*currentGoal);
                    enterRecovery(robot, navigator, path, RecoveryCause::MotionFailure);
                }
            }
            break;
        }
        case ExplorationState::TemporarilyBlocked:
        case ExplorationState::Recovering:
            robot.leftSpeed = robot.rightSpeed = 0.0;
            stateTimer -= dt;
            if (stateTimer <= 0.0) currentState = ExplorationState::SelectingFrontier;
            break;
        default:
            break;
    }
}

bool FrontierExplorer::isActive() const {
    return currentState == ExplorationState::SelectingFrontier ||
           currentState == ExplorationState::Navigating ||
           currentState == ExplorationState::NavigatingToGoal ||
           currentState == ExplorationState::Observing ||
           currentState == ExplorationState::TemporarilyBlocked ||
           currentState == ExplorationState::Recovering;
}

ExplorationState FrontierExplorer::state() const { return currentState; }
ExplorationTaskMode FrontierExplorer::taskMode() const { return currentTask; }
optional<GridCell> FrontierExplorer::goal() const { return currentGoal; }
optional<WorldPoint> FrontierExplorer::requestedDestination() const {
    return requestedGoal;
}
GoalFailureReason FrontierExplorer::goalFailureReason() const {
    return failureReason;
}
const ExplorationStats& FrontierExplorer::statistics() const { return stats; }
