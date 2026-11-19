#include "DiscoveredPlanningGrid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

DiscoveredPlanningGrid::DiscoveredPlanningGrid(
    const Mapping& map, double clearanceRadius
) : map(map), clearanceRadius(clearanceRadius), cellSize(map.getCellSize()) {
    if (!isfinite(clearanceRadius) || clearanceRadius < 0.0) {
        throw invalid_argument("Invalid planning clearance");
    }

    // Minimum Euclidean distance between two cell squares, indexed by their
    // center offset. This protects the entire candidate cell, not just its center.
    const int reach = static_cast<int>(ceil(clearanceRadius / cellSize)) + 1;
    const double radiusSquared = clearanceRadius * clearanceRadius;
    for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
            const double gapX = max(0, abs(dx) - 1) * cellSize;
            const double gapY = max(0, abs(dy) - 1) * cellSize;
            if (gapX * gapX + gapY * gapY <= radiusSquared + 1e-12) {
                inflationOffsets.push_back({dx, dy});
            }
        }
    }

    blocked.assign(static_cast<size_t>(map.getRows()) * map.getCols(), 1);
    rebuild();
}

bool DiscoveredPlanningGrid::rebuild() {
    bool changed = false;
    for (int y = 0; y < map.getRows(); ++y) {
        for (int x = 0; x < map.getCols(); ++x) {
            bool unsafe = map.getCell(x, y) != CellState::Free;
            if (!unsafe) {
                const double left = map.getMinX() + x * cellSize;
                const double bottom = map.getMinY() + y * cellSize;
                unsafe = left - clearanceRadius < map.getMinX() ||
                         left + cellSize + clearanceRadius > map.getMaxX() ||
                         bottom - clearanceRadius < map.getMinY() ||
                         bottom + cellSize + clearanceRadius > map.getMaxY();
            }

            if (!unsafe) {
                for (const GridCell offset : inflationOffsets) {
                    const int otherX = x + offset.x;
                    const int otherY = y + offset.y;
                    if (otherX < 0 || otherY < 0 ||
                        otherX >= map.getCols() || otherY >= map.getRows()) continue;
                    if (map.getCell(otherX, otherY) != CellState::Free) {
                        unsafe = true;
                        break;
                    }
                }
            }

            const size_t index = static_cast<size_t>(y) * map.getCols() + x;
            const std::uint8_t value = unsafe ? 1 : 0;
            changed |= blocked[index] != value;
            blocked[index] = value;
        }
    }
    if (changed) ++gridRevision;
    return changed;
}

std::uint64_t DiscoveredPlanningGrid::revision() const { return gridRevision; }

bool DiscoveredPlanningGrid::isBlocked(int x, int y) const {
    if (x < 0 || y < 0 || x >= map.getCols() || y >= map.getRows()) return true;
    return blocked[static_cast<size_t>(y) * map.getCols() + x] != 0;
}

GridCell DiscoveredPlanningGrid::worldToGrid(double x, double y) const {
    return {
        static_cast<int>(floor((x - map.getMinX()) / cellSize)),
        static_cast<int>(floor((y - map.getMinY()) / cellSize))
    };
}

double DiscoveredPlanningGrid::gridToWorldX(int x) const { return map.gridToWorldX(x); }
double DiscoveredPlanningGrid::gridToWorldY(int y) const { return map.gridToWorldY(y); }
int DiscoveredPlanningGrid::getRows() const { return map.getRows(); }
int DiscoveredPlanningGrid::getCols() const { return map.getCols(); }
