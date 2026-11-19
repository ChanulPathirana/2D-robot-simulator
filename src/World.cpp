#include "World.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace std;

namespace {

bool circleTouchesRectangle(double x, double y, double radius, const Obstacle& box) {
    const double nearestX = clamp(x, box.x, box.x + box.width);
    const double nearestY = clamp(y, box.y, box.y + box.height);
    const double dx = x - nearestX;
    const double dy = y - nearestY;
    return dx * dx + dy * dy <= radius * radius;
}

bool rectanglesOverlap(const Obstacle& a, const Obstacle& b) {
    return a.x < b.x + b.width && a.x + a.width > b.x &&
           a.y < b.y + b.height && a.y + a.height > b.y;
}

void addHorizontal(vector<Obstacle>& walls, double y,
                   initializer_list<pair<double, double>> spans) {
    for (const auto& [from, to] : spans) {
        walls.push_back({from, y, to - from, 0.4});
    }
}

void addVertical(vector<Obstacle>& walls, double x,
                 initializer_list<pair<double, double>> spans) {
    for (const auto& [from, to] : spans) {
        walls.push_back({x, from, 0.4, to - from});
    }
}

} // namespace

World::World() {
    // Staggered openings provide alternate east-west routes.
    addHorizontal(obstacles, -10.0, {{-18,-14}, {-11,-3}, {-0.5,7}, {10,18}});
    addHorizontal(obstacles,  -4.0, {{-18,-12}, {-9,-1}, {2,11}, {14,18}});
    addHorizontal(obstacles,   2.0, {{-18,-15}, {-12,-5}, {-2,4}, {7,15}});
    addHorizontal(obstacles,   8.0, {{-18,-10}, {-7,0}, {3,12}, {15,18}});

    // North-south barriers have several breaks, forming a connected network.
    addVertical(obstacles, -13.0, {{-2,0}, {4.5,5.5}, {10.5,12}});
    addVertical(obstacles,  -5.0, {{-8,-6}, {-2,0}, {4.5,5.5}, {10.5,12}});
    addVertical(obstacles,   3.0, {{-8,-6}, {-2,0}, {4.5,5.5}, {10.5,12}});
    addVertical(obstacles,  11.0, {{4.5,5.5}, {10.5,12}});

    // Short spurs and blocks make dead ends without sealing entire regions.
    addHorizontal(obstacles, -7.0, {{-17,-15}, {15,17}});
    addHorizontal(obstacles,  5.0, {{-10,-8}, {8,10}});
    addVertical(obstacles,  16.0, {{-2,0.5}});
    obstacles.push_back({-9.5, -0.5, 1.0, 1.0});
    obstacles.push_back({-1.0, 4.5, 1.0, 1.0});
    obstacles.push_back({7.5, -7.5, 1.0, 1.0});
    obstacles.push_back({15.5, 10.0, 1.0, 1.0});

    // Deterministic corridor traffic pauses and reverses at route endpoints.
    movingObstacles = {
        {{-9.0, -12.5, 0.8, 0.8}, true, -9.0, -6.0, 0.65},
        {{-1.5, -1.0, 0.8, 0.8}, true, -1.5, 1.0, 0.55},
        {{15.0, 4.0, 0.8, 0.8}, false, 4.0, 6.2, 0.5},
        {{-9.0, 10.5, 0.8, 0.8}, true, -9.0, -7.0, 0.6}
    };
}

bool World::collidesStatic(double x, double y, double radius) const {
    if (x - radius <= minX || x + radius >= maxX ||
        y - radius <= minY || y + radius >= maxY) return true;
    for (const auto& box : obstacles) {
        if (circleTouchesRectangle(x, y, radius, box)) return true;
    }
    return false;
}

bool World::collidesDynamic(double x, double y, double radius) const {
    for (const auto& moving : movingObstacles) {
        if (circleTouchesRectangle(x, y, radius, moving.shape)) return true;
    }
    return false;
}

bool World::collides(double x, double y, double radius) const {
    return collidesStatic(x, y, radius) || collidesDynamic(x, y, radius);
}

void World::update(double dt, double robotX, double robotY, double robotRadius) {
    if (dt <= 0.0) return;
    for (size_t i = 0; i < movingObstacles.size(); ++i) {
        auto& moving = movingObstacles[i];
        if (moving.pauseRemaining > 0.0) {
            moving.pauseRemaining = max(0.0, moving.pauseRemaining - dt);
            continue;
        }
        Obstacle next = moving.shape;
        double& position = moving.horizontal ? next.x : next.y;
        const double requested = position + moving.direction * moving.speed * dt;
        position = clamp(requested, moving.minPosition, moving.maxPosition);

        bool blocked = next.x < minX || next.x + next.width > maxX ||
                       next.y < minY || next.y + next.height > maxY ||
                       circleTouchesRectangle(robotX, robotY, robotRadius + 0.05, next);
        for (const auto& wall : obstacles) blocked |= rectanglesOverlap(next, wall);
        for (size_t j = 0; j < movingObstacles.size(); ++j) {
            if (j != i) blocked |= rectanglesOverlap(next, movingObstacles[j].shape);
        }
        if (blocked) {
            moving.direction = -moving.direction;
            moving.pauseRemaining = 0.35;
            continue;
        }

        moving.shape = next;
        if (requested <= moving.minPosition || requested >= moving.maxPosition) {
            moving.direction = -moving.direction;
            moving.pauseRemaining = 0.45;
        }
    }
}

double World::forwardClearance(double x, double y, double theta,
                               double radius, double maxDistance) const {
    if (collides(x, y, radius) || maxDistance <= 0.0) return 0.0;
    const double r = abs(radius);
    const double dx = cos(theta), dy = sin(theta);
    const double r2 = r * r;
    double clearance = maxDistance;
    auto consider = [&](double distance) {
        if (distance >= 0.0 && distance < clearance) clearance = distance;
    };

    // The world boundary is another straight contact surface.
    if (dx > 0.0) consider((maxX - r - x) / dx);
    if (dx < 0.0) consider((minX + r - x) / dx);
    if (dy > 0.0) consider((maxY - r - y) / dy);
    if (dy < 0.0) consider((minY + r - y) / dy);

    auto checkBox = [&](const Obstacle& box) {
        const double left = box.x, right = box.x + box.width;
        const double bottom = box.y, top = box.y + box.height;
        if (dx != 0.0) {
            for (double faceX : {left - r, right + r}) {
                const double distance = (faceX - x) / dx;
                if (distance >= 0.0 && distance < clearance) {
                    const double hitY = fma(distance, dy, y);
                    if (hitY >= bottom && hitY <= top) consider(distance);
                }
            }
        }
        if (dy != 0.0) {
            for (double faceY : {bottom - r, top + r}) {
                const double distance = (faceY - y) / dy;
                if (distance >= 0.0 && distance < clearance) {
                    const double hitX = fma(distance, dx, x);
                    if (hitX >= left && hitX <= right) consider(distance);
                }
            }
        }
        for (double cornerX : {left, right}) {
            for (double cornerY : {bottom, top}) {
                const double ox = cornerX - x, oy = cornerY - y;
                const double along = fma(ox, dx, oy * dy);
                const double across = fma(ox, dy, -oy * dx);
                const double across2 = across * across;
                const double tolerance = 32.0 * numeric_limits<double>::epsilon()
                    * max(r2, across2);
                if (across2 <= r2 + tolerance) {
                    consider(along - sqrt(max(0.0, r2 - across2)));
                }
            }
        }
    };
    for (const auto& box : obstacles) checkBox(box);
    for (const auto& moving : movingObstacles) checkBox(moving.shape);
    return clearance;
}
