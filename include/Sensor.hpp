
#pragma once

#include "World.hpp"

struct RayHit {
    double distance;
    bool hit;
};

class Sensor {
public:
    static RayHit castRay(
        const World& world,
        double originX,
        double originY,
        double angle,
        double maxRange
    );
};
