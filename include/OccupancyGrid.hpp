
#pragma once

#include "NavigationGrid.hpp"
#include "World.hpp"
#include <vector>

using namespace std;

class OccupancyGrid : public NavigationGrid {
public:
    OccupancyGrid(
        const World& world,
        double width,
        double height,
        double cellSize,
        double robotRadius
    );

    bool isBlocked(int x, int y) const override;
    bool isStaticBlocked(int x, int y) const;

    GridCell worldToGrid(double x, double y) const override;

    double gridToWorldX(int x) const override;
    double gridToWorldY(int y) const override;

    int getRows() const override;
    int getCols() const override;

private:
    int rows;
    int cols;

    double cellSize;
    double originX;
    double originY;
    double inflationRadius;
    const World& world;

    vector<vector<bool>> blocked;
};
