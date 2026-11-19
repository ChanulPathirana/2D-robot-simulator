
#pragma once

#include "SimulationConfig.hpp"
#include <vector>

struct Obstacle {
    double x;
    double y;
    double width;
    double height;
};

struct MovingObstacle {
    Obstacle shape;
    bool horizontal;
    double minPosition;
    double maxPosition;
    double speed;
    int direction = 1;
    double pauseRemaining = 0.0;
};

class World {
public:
    std::vector<Obstacle> obstacles;
    std::vector<MovingObstacle> movingObstacles;

    static constexpr double minX = SimConfig::WORLD_MIN_X;
    static constexpr double maxX = SimConfig::WORLD_MAX_X;
    static constexpr double minY = SimConfig::WORLD_MIN_Y;
    static constexpr double maxY = SimConfig::WORLD_MAX_Y;

    World();

    void update(double dt, double robotX, double robotY, double robotRadius);
    bool collidesStatic(double x, double y, double radius) const;
    bool collidesDynamic(double x, double y, double radius) const;
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
