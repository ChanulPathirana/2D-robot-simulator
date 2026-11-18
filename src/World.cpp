
#include "World.hpp"

#include <algorithm>

using namespace std;

World::World() {
    obstacles.push_back({1.5, 1.0, 1.0, 0.5});
    obstacles.push_back({-2.0, -1.0, 0.8, 1.5});
    obstacles.push_back({0.5, -2.0, 2.0, 0.4});
}

bool World::collides(
    double x, double y, double radius
) const {
    for (const auto& obstacle : obstacles) {
        // Closest point on the rectangle to robot center
        double closestX = clamp(
            x, obstacle.x, obstacle.x + obstacle.width
        );

        double closestY = clamp(
            y, obstacle.y, obstacle.y + obstacle.height
        );

        double dx = x - closestX;
        double dy = y - closestY;

        if (dx * dx + dy * dy <= radius * radius) {
            return true;
        }
    }

    return false;
}
