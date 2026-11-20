#pragma once

#include "DiscoveredPlanningGrid.hpp"
#include "Navigator.hpp"

#include <optional>
#include <array>
#include <cstdint>
#include <vector>

struct FrontierRegion {
    std::vector<GridCell> cells;
};

enum class ExplorationState {
    Idle,
    SelectingFrontier,
    Navigating,
    NavigatingToGoal,
    Observing,
    TemporarilyBlocked,
    Recovering,
    Completed,
    GoalReached,
    InvalidDestination,
    GoalUnavailable,
    Limited,
    Blocked,
    Stopped
};

enum class ExplorationTaskMode { Independent, GoalDirected };

enum class GoalRequestResult {
    Direct,
    Exploring,
    Occupied,
    OutsideMap
};

enum class GoalFailureReason {
    None,
    OutsideMap,
    OccupiedAtClick,
    OccupiedAfterObservation,
    NoSafeProgress
};

struct ExplorationStats {
    int unknown = 0;
    int free = 0;
    int occupied = 0;
    int observed = 0;
    int frontierRegions = 0;
    int reachableCandidates = 0;
    int visitedViewpoints = 0;
    int failedViewpoints = 0;
    double selectedTravelCost = 0.0;
    double distanceTravelled = 0.0;
    double observedPercent = 0.0;
};

struct FrontierExplorerSettings {
    double informationGainWeight = 0.45;
    double travelCostWeight = 1.0;
    double clearanceWeight = 0.05;
    double goalDistanceWeight = 0.30;
    double failurePenalty = 10.0;
    double shortTravelPenaltyWeight = 4.0;
    double minimumTravel = 0.75;
    double maximumFrontierOffset = 3.0;
    double observationSeconds = 0.4;
    double recoverySeconds = 1.5;
    double stallSeconds = 2.5;
    int minimumFrontierSize = 2;
    int minimumObservedGain = 3;
    int mapChangeRetryCells = 25;
    double failureRetrySeconds = 4.0;
    double goalCheckSeconds = 0.75;
    double occupiedGoalWaitSeconds = 3.0;
    int maximumRecoveryAttempts = 5;
};

class FrontierExplorer {
public:
    FrontierExplorer(
        const Mapping& map,
        const DiscoveredPlanningGrid& grid,
        double lidarRange,
        FrontierExplorerSettings settings = {}
    );

    static std::vector<FrontierRegion> detectFrontiers(
        const Mapping& map, int minimumRegionSize = 2
    );

    void start(Robot& robot, Navigator& navigator, std::vector<GridCell>& path);
    GoalRequestResult startGoal(WorldPoint requested,
                                Robot& robot, Navigator& navigator,
                                std::vector<GridCell>& path);
    void stop(Robot& robot, Navigator& navigator, std::vector<GridCell>& path);
    void onMapScan();
    void onMappingReset(Robot& robot, Navigator& navigator,
                        std::vector<GridCell>& path);
    void update(
        double dt,
        bool freshScan,
        Robot& robot,
        Navigator& navigator,
        std::vector<GridCell>& path
    );

    bool isActive() const;
    ExplorationState state() const;
    ExplorationTaskMode taskMode() const;
    std::optional<GridCell> goal() const;
    std::optional<WorldPoint> requestedDestination() const;
    GoalFailureReason goalFailureReason() const;
    const ExplorationStats& statistics() const;

private:
    enum class RecoveryCause { NoCandidate, RouteBlocked, MotionFailure };

    struct ViewpointRecord {
        GridCell cell;
        int observedAtRecord;
        int failures;
        double timeAtRecord;
    };

    const Mapping& map;
    const DiscoveredPlanningGrid& grid;
    double lidarRange;
    FrontierExplorerSettings settings;
    ExplorationState currentState = ExplorationState::Idle;
    ExplorationTaskMode currentTask = ExplorationTaskMode::Independent;
    ExplorationStats stats;
    std::optional<GridCell> currentGoal;
    std::optional<WorldPoint> requestedGoal;
    GoalFailureReason failureReason = GoalFailureReason::None;
    std::vector<ViewpointRecord> failed;
    std::vector<ViewpointRecord> visited;
    std::array<std::vector<GridCell>, 24> informationRayOffsets;
    double lastX = 0.0;
    double lastY = 0.0;
    double lastTheta = 0.0;
    double progressAnchorX = 0.0;
    double progressAnchorY = 0.0;
    double elapsedSeconds = 0.0;
    double stallTimer = 0.0;
    double stateTimer = 0.0;
    double goalCheckTimer = 0.0;
    double occupiedGoalTimer = 0.0;
    std::uint64_t lastGoalCheckRevision = 0;
    int observedAtTargetStart = 0;
    int recoveryAttempts = 0;

    void refreshMapStats();
    std::optional<GridCell> selectViewpoint(const Robot& robot);
    std::vector<double> travelCosts(GridCell start) const;
    int estimateInformationGain(GridCell viewpoint) const;
    int nearbyClearCells(GridCell viewpoint) const;
    double historyPenalty(GridCell viewpoint) const;
    void recordFailure(GridCell viewpoint);
    bool tryPlanRequestedGoal(Robot& robot, Navigator& navigator,
                              std::vector<GridCell>& path);
    void enterRecovery(Robot& robot, Navigator& navigator,
                       std::vector<GridCell>& path, RecoveryCause cause);
    void clearRoute(Robot& robot, Navigator& navigator,
                    std::vector<GridCell>& path);
};
