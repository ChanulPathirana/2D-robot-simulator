
#include "Navigator.hpp"

#include <algorithm>
#include <cmath>

using namespace std;

namespace {

constexpr double WAYPOINT_TOLERANCE = 0.12;
constexpr double MAX_FORWARD_SPEED = 0.55;
constexpr double MAX_ANGULAR_SPEED = 1.4;
constexpr double ANGULAR_GAIN = 2.5;

// Keep angle error between -pi and pi.
double normalizeAngle(double angle) {
    return atan2(sin(angle), cos(angle));
}

// Convert forward and turning velocities
// into left and right wheel velocities.
void setWheelSpeeds(
    Robot& robot,
    double forwardSpeed,
    double angularSpeed
) {
    double halfWheelbase = robot.wheelBase / 2.0;

    robot.leftSpeed =
        forwardSpeed - angularSpeed * halfWheelbase;

    robot.rightSpeed =
        forwardSpeed + angularSpeed * halfWheelbase;
}

} // namespace

void Navigator::setPath(const vector<GridCell>& newPath) {
    path = newPath;
    currentWaypoint = 0;
    active = !path.empty();
}

void Navigator::stop() {
    active = false;
    currentWaypoint = 0;
    path.clear();
}

bool Navigator::isActive() const {
    return active;
}

size_t Navigator::waypointIndex() const {
    return currentWaypoint;
}

void Navigator::update(
    Robot& robot,
    const OccupancyGrid& grid
) {
    if (!active) {
        setWheelSpeeds(robot, 0.0, 0.0);
        return;
    }

    // Skip waypoints that are already reached.
    while (currentWaypoint < path.size()) {
        GridCell target = path[currentWaypoint];

        double targetX = grid.gridToWorldX(target.x);
        double targetY = grid.gridToWorldY(target.y);

        double distance = hypot(
            targetX - robot.x,
            targetY - robot.y
        );

        if (distance <= WAYPOINT_TOLERANCE) {
            ++currentWaypoint;
        } else {
            break;
        }
    }

    // Stop after reaching the last waypoint.
    if (currentWaypoint >= path.size()) {
        stop();
        setWheelSpeeds(robot, 0.0, 0.0);
        return;
    }

    GridCell target = path[currentWaypoint];

    double targetX = grid.gridToWorldX(target.x);
    double targetY = grid.gridToWorldY(target.y);

    double dx = targetX - robot.x;
    double dy = targetY - robot.y;

    double distance = hypot(dx, dy);
    double desiredAngle = atan2(dy, dx);

    double angleError = normalizeAngle(
        desiredAngle - robot.theta
    );

    // Turn toward the waypoint.
    double omega = clamp(
        ANGULAR_GAIN * angleError,
        -MAX_ANGULAR_SPEED,
        MAX_ANGULAR_SPEED
    );

    // Turn first if the heading error is large.
    double forwardSpeed = 0.0;

    if (abs(angleError) < 0.20) {
        forwardSpeed = min(
            MAX_FORWARD_SPEED,
            2.0 * distance
        );
    }

    setWheelSpeeds(robot, forwardSpeed, omega);
}
