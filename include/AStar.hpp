
#pragma once

#include "OccupancyGrid.hpp"
#include <vector>

class AStar {
public:
    static std::vector<GridCell> findPath(
        const OccupancyGrid& grid,
        GridCell start,
        GridCell goal
    );
};
