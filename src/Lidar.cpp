
#include "Lidar.hpp"
#include "Sensor.hpp"

#include <numbers>
#include <stdexcept>

using namespace std;

Lidar::Lidar(int count, double range)
    : rayCount(count), maxRange(range)
{
    if (rayCount <= 0 || maxRange <= 0.0) {
        throw invalid_argument("Invalid LiDAR configuration");
    }

    readings.reserve(rayCount);
}

void Lidar::scan(
    const Robot& robot,
    const World& world
) {
    readings.clear();

    constexpr double TWO_PI = 2.0 * numbers::pi;

    for (int i = 0; i < rayCount; ++i) {

        // Angle relative to the robot.
        double relativeAngle = i * TWO_PI / rayCount;

        // Convert to world-space direction.
        double worldAngle = robot.theta + relativeAngle;

        // Use existing ray casting to detect obstacles.
        RayHit result = Sensor::castRay(
            world,
            robot.x,
            robot.y,
            worldAngle,
            maxRange
        );

        readings.push_back({
            worldAngle,
            result.distance,
            result.hit
        });
    }
}

const vector<LidarReading>& Lidar::getReadings() const {
    return readings;
}
