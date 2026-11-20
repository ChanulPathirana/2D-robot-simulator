#pragma once

#include "NavigationGrid.hpp"
#include "Robot.hpp"

#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

inline bool validateRoute(const NavigationGrid& grid,
                          const std::vector<GridCell>& route) {
    if (route.empty()) return false;
    for (std::size_t i = 0; i < route.size(); ++i) {
        const auto cell = route[i];
        if (grid.isBlocked(cell.x, cell.y)) return false;
        if (i == 0) continue;
        const int dx = cell.x - route[i - 1].x;
        const int dy = cell.y - route[i - 1].y;
        if ((dx == 0 && dy == 0) || std::abs(dx) > 1 || std::abs(dy) > 1)
            return false;
        if (dx && dy &&
            (grid.isBlocked(route[i - 1].x + dx, route[i - 1].y) ||
             grid.isBlocked(route[i - 1].x, route[i - 1].y + dy)))
            return false;
    }
    return true;
}

inline void logAcceptedRoute(const char* planningMode, const char* task,
                             const char* targetType,
                             std::optional<WorldPoint> requested,
                             std::optional<WorldPoint> frontier,
                             const Robot& robot, const NavigationGrid& grid,
                             const std::vector<GridCell>& route) {
    auto printPoint = [](std::optional<WorldPoint> point) {
        if (point) std::cout << '(' << point->x << ", " << point->y << ')';
        else std::cout << "none";
    };
    double length = 0.0;
    double x = robot.x, y = robot.y;
    for (const auto cell : route) {
        const double nextX = grid.gridToWorldX(cell.x);
        const double nextY = grid.gridToWorldY(cell.y);
        length += std::hypot(nextX - x, nextY - y);
        x = nextX; y = nextY;
    }
    if (requested && !frontier)
        length += std::hypot(requested->x - x, requested->y - y);
    std::cout << "Route accepted | Planning mode: " << planningMode
              << " | Navigation task: " << task
              << " | Active target type: " << targetType
              << " | Requested destination: ";
    printPoint(requested);
    std::cout << " | Temporary frontier viewpoint: ";
    printPoint(frontier);
    std::cout << " | Current robot position: (" << robot.x << ", " << robot.y
              << ") | Route waypoint count: " << route.size()
              << " | Route length: " << length
              << " | Route validation result: "
              << (validateRoute(grid, route) ? "valid" : "INVALID") << '\n';
}
