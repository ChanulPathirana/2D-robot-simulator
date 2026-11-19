#pragma once

#include "Navigator.hpp"

#include <algorithm>
#include <vector>

inline bool routeBlockedAhead(
    const Navigator& navigator,
    const NavigationGrid& grid,
    const std::vector<GridCell>& path,
    bool checkEntireRoute
) {
    const std::size_t from = navigator.waypointIndex();
    const std::size_t to = checkEntireRoute
        ? path.size() : std::min(path.size(), from + 20);

    for (std::size_t i = from; i < to; ++i) {
        if (grid.isBlocked(path[i].x, path[i].y)) return true;
        if (i == 0) continue;

        const int dx = path[i].x - path[i - 1].x;
        const int dy = path[i].y - path[i - 1].y;
        if (dx != 0 && dy != 0 &&
            (grid.isBlocked(path[i - 1].x + dx, path[i - 1].y) ||
             grid.isBlocked(path[i - 1].x, path[i - 1].y + dy))) {
            return true;
        }
    }
    return false;
}
