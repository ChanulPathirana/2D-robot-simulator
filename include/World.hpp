
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
};
