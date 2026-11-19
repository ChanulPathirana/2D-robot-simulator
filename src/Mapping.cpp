
#include "Mapping.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

using namespace std;

Mapping::Mapping(
    double minX,
    double minY,
    double maxX,
    double maxY,
    double size
)
    : cellSize(size),
      originX(minX),
      originY(minY)
{
    if (maxX <= minX ||
        maxY <= minY ||
        size <= 0.0) {
        throw invalid_argument("Invalid map dimensions");
    }

    cols = static_cast<int>(
        ceil((maxX - minX) / cellSize)
    );

    rows = static_cast<int>(
        ceil((maxY - minY) / cellSize)
    );

    // All cells start UNKNOWN.
    cells.assign(
        rows * cols,
        CellState::Unknown
    );
}

CellState Mapping::getCell(int x, int y) const {
    if (x < 0 || y < 0 ||
        x >= cols || y >= rows) {
        return CellState::Unknown;
    }

    return cells[y * cols + x];
}

int Mapping::getRows() const {
    return rows;
}

int Mapping::getCols() const {
    return cols;
}

double Mapping::getCellSize() const {
    return cellSize;
}

double Mapping::gridToWorldX(int x) const {
    return originX + (x + 0.5) * cellSize;
}

double Mapping::gridToWorldY(int y) const {
    return originY + (y + 0.5) * cellSize;
}

void Mapping::reset() {
    fill(
        cells.begin(),
        cells.end(),
        CellState::Unknown
    );
}

bool Mapping::inBounds(int x, int y) const {
    return x >= 0 && x < cols &&
           y >= 0 && y < rows;
}

void Mapping::update(
    const vector<LidarReading>& readings,
    double sensorX,
    double sensorY
) {
    // Process each laser measurement.
    for (const auto& reading : readings) {
        updateRay(reading, sensorX, sensorY);
    }
}

void Mapping::updateRay(
    const LidarReading& reading,
    double sensorX,
    double sensorY
) {
    if (!isfinite(reading.angle) ||
        !isfinite(reading.distance) ||
        reading.distance <= 0.0) {
        return;
    }

    // Direction of the laser in world coordinates.
    const double dx = cos(reading.angle);
    const double dy = sin(reading.angle);

    // Convert sensor position to grid coordinates.
    int x = static_cast<int>(
        floor((sensorX - originX) / cellSize)
    );

    int y = static_cast<int>(
        floor((sensorY - originY) / cellSize)
    );

    // For now, scans must originate inside the map.
    if (!inBounds(x, y)) {
        return;
    }

    // Direction of grid movement.
    const int stepX = (dx > 0) - (dx < 0);
    const int stepY = (dy > 0) - (dy < 0);

    const double infinity =
        numeric_limits<double>::infinity();

    // How far along the ray before crossing
    // the next vertical grid boundary?
    double tMaxX = infinity;
    double tDeltaX = infinity;

    if (abs(dx) > 1e-12) {
        const double boundaryX =
            originX +
            (stepX > 0 ? x + 1 : x) * cellSize;

        tMaxX = max(
            0.0,
            (boundaryX - sensorX) / dx
        );

        tDeltaX = cellSize / abs(dx);
    }

    // How far along the ray before crossing
    // the next horizontal grid boundary?
    double tMaxY = infinity;
    double tDeltaY = infinity;

    if (abs(dy) > 1e-12) {
        const double boundaryY =
            originY +
            (stepY > 0 ? y + 1 : y) * cellSize;

        tMaxY = max(
            0.0,
            (boundaryY - sensorY) / dy
        );

        tDeltaY = cellSize / abs(dy);
    }

    // For a hit, determine which cell contains
    // the detected surface point.
    // A small backward offset avoids assigning
    // a wall to the cell behind its near surface.
    int hitCellX = -1;
    int hitCellY = -1;

    if (reading.hit) {
        const double hitDistance =
            max(0.0, reading.distance - 1e-6);

        const double hitX =
            sensorX + hitDistance * dx;

        const double hitY =
            sensorY + hitDistance * dy;

        hitCellX = static_cast<int>(
            floor((hitX - originX) / cellSize)
        );

        hitCellY = static_cast<int>(
            floor((hitY - originY) / cellSize)
        );
    }

    // Traverse cells intersected by the ray.
    while (inBounds(x, y)) {

        // Stop when the obstacle cell is reached.
        if (reading.hit &&
            x == hitCellX &&
            y == hitCellY) {
            cells[y * cols + x] =
                CellState::Occupied;
            break;
        }

        // Space before the obstacle is free.
        cells[y * cols + x] =
            CellState::Free;

        const double nextT = min(tMaxX, tMaxY);

        if (nextT >= reading.distance) {
            break;
        }

        if (tMaxX < tMaxY) {
            x += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxX) {
            y += stepY;
            tMaxY += tDeltaY;
        } else {
            // Ray crosses a grid corner.
            x += stepX;
            y += stepY;

            tMaxX += tDeltaX;
            tMaxY += tDeltaY;
        }
    }
}
