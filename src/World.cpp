
#include "World.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

double World::forwardClearance(
    double x,
    double y,
    double theta,
    double radius,
    double maxDistance
) const {
    if (collides(x, y, radius)) {
        return 0.0;
    }

    if (maxDistance <= 0.0) {
        return 0.0;
    }

    const double footprintRadius = abs(radius);
    const double dx = cos(theta);
    const double dy = sin(theta);
    const double radiusSquared = footprintRadius * footprintRadius;
    double clearance = maxDistance;

    auto consider = [&](double distance) {
        if (distance >= 0.0 && distance < clearance) {
            clearance = distance;
        }
    };

    for (const auto& obstacle : obstacles) {
        const double left = obstacle.x;
        const double right = obstacle.x + obstacle.width;
        const double bottom = obstacle.y;
        const double top = obstacle.y + obstacle.height;

        // The swept center first touches a straight edge at an offset face.
        if (dx != 0.0) {
            for (double faceX : {left - footprintRadius, right + footprintRadius}) {
                const double distance = (faceX - x) / dx;
                if (distance >= 0.0 && distance < clearance) {
                    const double hitY = fma(distance, dy, y);
                    if (hitY >= bottom && hitY <= top) {
                        consider(distance);
                    }
                }
            }
        }

        if (dy != 0.0) {
            for (double faceY : {bottom - footprintRadius, top + footprintRadius}) {
                const double distance = (faceY - y) / dy;
                if (distance >= 0.0 && distance < clearance) {
                    const double hitX = fma(distance, dx, x);
                    if (hitX >= left && hitX <= right) {
                        consider(distance);
                    }
                }
            }
        }

        // Rounded corners of the expanded rectangle are circles of robot radius.
        for (double cornerX : {left, right}) {
            for (double cornerY : {bottom, top}) {
                const double offsetX = cornerX - x;
                const double offsetY = cornerY - y;
                const double along = fma(offsetX, dx, offsetY * dy);
                const double across = fma(offsetX, dy, -offsetY * dx);
                const double acrossSquared = across * across;
                const double tolerance = 32.0 * numeric_limits<double>::epsilon()
                    * max(radiusSquared, acrossSquared);

                if (acrossSquared > radiusSquared + tolerance) {
                    continue;
                }

                const double halfChord = sqrt(max(0.0, radiusSquared - acrossSquared));
                consider(along - halfChord);
            }
        }
    }

    return clearance;
}
