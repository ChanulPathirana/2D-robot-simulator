
#pragma once

#include "Robot.hpp"
#include "OccupancyGrid.hpp"

#include <cstddef>
#include <vector>

class Navigator {
public:
    void setPath(const std::vector<GridCell>& newPath);
    void stop();

    void update(Robot& robot, const OccupancyGrid& grid);

    bool isActive() const;
    std::size_t waypointIndex() const;

private:
    std::vector<GridCell> path;
    std::size_t currentWaypoint = 0;
    bool active = false;
};
