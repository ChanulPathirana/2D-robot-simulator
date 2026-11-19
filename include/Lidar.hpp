
#pragma once

#include "Robot.hpp"
#include "World.hpp"

#include <vector>

struct LidarReading {
    double angle;       // Ray direction in world coordinates
    double distance;    // Distance in metres
    bool hit;           // Whether an obstacle was detected
};

class Lidar {
public:
    Lidar(int rayCount = 72, double maxRange = 6.0);

    void scan(const Robot& robot, const World& world);

    const std::vector<LidarReading>& getReadings() const;

private:
    int rayCount;
    double maxRange;

    std::vector<LidarReading> readings;
};
