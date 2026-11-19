#pragma once

#include "DiscoveredPlanningGrid.hpp"
#include "Navigator.hpp"
#include "Robot.hpp"

#include <optional>
#include <vector>

enum class PlanningMode { KnownMap, DiscoveredMap };

inline void cancelNavigation(
    Navigator& navigator,
    Robot& robot,
    std::vector<GridCell>& path,
    std::optional<GridCell>& goal
) {
    navigator.stop();
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;
    path.clear();
    goal.reset();
}

inline void switchPlanningMode(
    PlanningMode& mode,
    Navigator& navigator,
    Robot& robot,
    std::vector<GridCell>& path,
    std::optional<GridCell>& goal
) {
    mode = mode == PlanningMode::KnownMap
        ? PlanningMode::DiscoveredMap : PlanningMode::KnownMap;
    cancelNavigation(navigator, robot, path, goal);
}

inline void resetDiscoveredMap(
    Mapping& map,
    DiscoveredPlanningGrid& grid,
    PlanningMode mode,
    Navigator& navigator,
    Robot& robot,
    std::vector<GridCell>& path,
    std::optional<GridCell>& goal
) {
    map.reset();
    grid.rebuild();
    if (mode == PlanningMode::DiscoveredMap) {
        cancelNavigation(navigator, robot, path, goal);
    }
}
