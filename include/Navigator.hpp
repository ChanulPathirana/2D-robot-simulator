
#pragma once

#include "Robot.hpp"
#include "NavigationGrid.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

class Navigator {
public:
    static constexpr double GOAL_TOLERANCE = 0.12;

    void setPath(const std::vector<GridCell>& newPath,
                 std::optional<WorldPoint> exactGoal = std::nullopt);
    void stop();

    void update(Robot& robot, const NavigationGrid& grid);

    bool isActive() const;
    std::size_t waypointIndex() const;
    const std::vector<GridCell>& route() const;
    std::uint64_t routeRevision() const;

private:
    std::vector<GridCell> path;
    std::optional<WorldPoint> finalGoal;
    std::size_t currentWaypoint = 0;
    bool active = false;
    std::uint64_t revision = 0;
};
