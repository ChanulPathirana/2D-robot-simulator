
#pragma once

#include <cstdint>
#include <vector>
#include "Lidar.hpp"

enum class CellState : std::uint8_t {
    Unknown,
    Free,
    Occupied
};

class Mapping {
public:
    Mapping(
        double minX,
        double minY,
        double maxX,
        double maxY,
        double cellSize
    );

    CellState getCell(int x, int y) const;

    int getRows() const;
    int getCols() const;
    double getCellSize() const;

    double gridToWorldX(int x) const;
    double gridToWorldY(int y) const;

    void reset();
    // Process a completed LiDAR scan.
    void update(
        const std::vector<LidarReading>& readings,
        double sensorX,
        double sensorY
    );

private:
    int rows;
    int cols;

    double cellSize;
    double originX;
    double originY;

    std::vector<CellState> cells;

    bool inBounds(int x, int y) const;

    void updateRay(
        const LidarReading& reading,
        double sensorX,
        double sensorY
    );
};
