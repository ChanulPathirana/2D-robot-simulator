
#pragma once

#include "NavigationGrid.hpp"
#include <vector>

class AStar {
public:
    static std::vector<GridCell> findPath(
        const NavigationGrid& grid,
        GridCell start,
        GridCell goal
    );
};
