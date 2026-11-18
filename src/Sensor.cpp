
#include "Sensor.hpp"

#include <algorithm>
#include <cmath>

using namespace std;

RayHit Sensor::castRay(
    const World& world,
    double originX,
    double originY,
    double angle,
    double maxRange
) {
    constexpr double STEP = 0.02;

    if (maxRange <= 0.0) {
        return {0.0, false};
    }

    double dx = cos(angle);
    double dy = sin(angle);

    for (double distance = 0.0;
         distance <= maxRange;
         distance += STEP) {

        double x = originX + distance * dx;
        double y = originY + distance * dy;

        // Check whether this ray point is inside an obstacle
        if (world.collides(x, y, 0.0)) {
            return {distance, true};
        }
    }

    // Check the exact endpoint too
    double endX = originX + maxRange * dx;
    double endY = originY + maxRange * dy;

    if (world.collides(endX, endY, 0.0)) {
        return {maxRange, true};
    }

    return {maxRange, false};
}
