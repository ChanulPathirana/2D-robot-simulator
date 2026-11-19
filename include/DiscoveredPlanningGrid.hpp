#pragma once

#include "Mapping.hpp"
#include "NavigationGrid.hpp"

#include <cstdint>
#include <vector>

// A cached, conservative traversability view of LiDAR evidence only.
class DiscoveredPlanningGrid : public NavigationGrid {
public:
    DiscoveredPlanningGrid(const Mapping& map, double clearanceRadius);

    // Call after a fresh scan or reset. Returns true when traversability changed.
    bool rebuild();

    bool isBlocked(int x, int y) const override;
    GridCell worldToGrid(double x, double y) const override;
    double gridToWorldX(int x) const override;
    double gridToWorldY(int y) const override;
    int getRows() const override;
    int getCols() const override;

private:
    const Mapping& map;
    double clearanceRadius;
    double cellSize;
    std::vector<GridCell> inflationOffsets;
    std::vector<std::uint8_t> blocked;
};
