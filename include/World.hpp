
#pragma once

#include <vector>

struct Obstacle {
    double x;
    double y;
    double width;
    double height;
};

class World {
public:
    std::vector<Obstacle> obstacles;

    World();

    bool collides(double x, double y, double radius) const;

    // Straight-line travel distance until the circular footprint touches an obstacle.
    double forwardClearance(
        double x,
        double y,
        double theta,
        double radius,
        double maxDistance
    ) const;
};
