
#pragma once

#include "NavigationGrid.hpp"
#include <vector>

class AStar {
public:
    struct Timing {
        std::size_t calls = 0;
        double totalMs = 0.0;
        double maxMs = 0.0;
    };
    static Timing timing();
    static void resetTiming();
    static std::vector<GridCell> findPath(
        const NavigationGrid& grid,
        GridCell start,
        GridCell goal
    );
};
