
#pragma once

#include <cstdint>
#include <vector>
#include "Lidar.hpp"

enum class CellState : std::uint8_t {
    Unknown,
    Free,
    Occupied
};

struct MappingParameters {
    double occupiedObservationProbability = 0.70;
    double freeObservationProbability = 0.30;
    double minLogOdds = -4.0;
    double maxLogOdds = 4.0;
    double occupiedThreshold = 0.65;
    double freeThreshold = 0.35;
};

class Mapping {
public:
    Mapping(
        double minX,
        double minY,
        double maxX,
        double maxY,
        double cellSize,
        MappingParameters parameters = {}
    );

    CellState getCell(int x, int y) const;
    bool hasObservation(int x, int y) const;
    // Out-of-bounds cells have the unknown prior probability of 0.5.
    double getOccupancyProbability(int x, int y) const;

    int getRows() const;
    int getCols() const;
    double getCellSize() const;
    double getMinX() const;
    double getMinY() const;
    double getMaxX() const;
    double getMaxY() const;

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
    double maxX;
    double maxY;

    MappingParameters parameters;
    double occupiedIncrement = 0.0;
    double freeIncrement = 0.0;
    double occupiedThresholdLogOdds = 0.0;
    double freeThresholdLogOdds = 0.0;

    std::vector<double> logOdds;
    std::vector<std::uint8_t> observed;

    bool inBounds(int x, int y) const;
    void addEvidence(int x, int y, double increment);

    void updateRay(
        const LidarReading& reading,
        double sensorX,
        double sensorY
    );
};
