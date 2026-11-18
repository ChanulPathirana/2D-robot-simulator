
#pragma once

#include "World.hpp"
#include <vector>

using namespace std;

struct GridCell {
    int x;
    int y;
};

class OccupancyGrid {
public:
    OccupancyGrid(
        const World& world,
        double width,
        double height,
        double cellSize,
        double robotRadius
    );

    bool isBlocked(int x, int y) const;

    GridCell worldToGrid(double x, double y) const;

    double gridToWorldX(int x) const;
    double gridToWorldY(int y) const;

    int getRows() const;
    int getCols() const;

private:
    int rows;
    int cols;

    double cellSize;
    double originX;
    double originY;

    vector<vector<bool>> blocked;
};
