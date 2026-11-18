
#include "OccupancyGrid.hpp"

#include <cmath>
#include <stdexcept>

using namespace std;

OccupancyGrid::OccupancyGrid(
    const World& world,
    double width,
    double height,
    double size,
    double robotRadius
)
    : cellSize(size),
      originX(-width / 2.0),
      originY(-height / 2.0)
{
    if (width <= 0 || height <= 0 || size <= 0) {
        throw invalid_argument("Invalid occupancy grid dimensions");
    }

    cols = static_cast<int>(ceil(width / size));
    rows = static_cast<int>(ceil(height / size));

    blocked.resize(rows, vector<bool>(cols, false));

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {

            double worldX = gridToWorldX(x);
            double worldY = gridToWorldY(y);

            blocked[y][x] = world.collides(
                worldX,
                worldY,
                robotRadius
            );
        }
    }
}

bool OccupancyGrid::isBlocked(int x, int y) const {
    if (x < 0 || y < 0 || x >= cols || y >= rows) {
        return true;
    }

    return blocked[y][x];
}

GridCell OccupancyGrid::worldToGrid(
    double x, double y
) const {
    return {
        static_cast<int>(floor((x - originX) / cellSize)),
        static_cast<int>(floor((y - originY) / cellSize))
    };
}

double OccupancyGrid::gridToWorldX(int x) const {
    return originX + (x + 0.5) * cellSize;
}

double OccupancyGrid::gridToWorldY(int y) const {
    return originY + (y + 0.5) * cellSize;
}

int OccupancyGrid::getRows() const {
    return rows;
}

int OccupancyGrid::getCols() const {
    return cols;
}
