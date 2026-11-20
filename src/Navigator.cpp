
#include "Navigator.hpp"

#include <algorithm>
#include <cmath>

using namespace std;

namespace {

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

void Navigator::setPath(const vector<GridCell>& newPath,
                        optional<WorldPoint> exactGoal) {
    path = newPath;
    ++revision;
    finalGoal = exactGoal;
    currentWaypoint = 0;
    active = !path.empty() || finalGoal.has_value();
}

void Navigator::stop() {
    active = false;
    currentWaypoint = 0;
    path.clear();
    finalGoal.reset();
}

bool Navigator::isActive() const {
    return active;
}

size_t Navigator::waypointIndex() const {
    return currentWaypoint;
}

const vector<GridCell>& Navigator::route() const { return path; }
std::uint64_t Navigator::routeRevision() const { return revision; }

void Navigator::update(
    Robot& robot,
    const NavigationGrid& grid
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

        if (distance <= GOAL_TOLERANCE) {
            ++currentWaypoint;
        } else {
            break;
        }
    }

    // Stop after reaching the last waypoint.
    if (currentWaypoint >= path.size() && !finalGoal) {
        stop();
        setWheelSpeeds(robot, 0.0, 0.0);
        return;
    }

    double targetX;
    double targetY;
    if (currentWaypoint < path.size()) {
        const GridCell target = path[currentWaypoint];
        targetX = grid.gridToWorldX(target.x);
        targetY = grid.gridToWorldY(target.y);
    } else {
        targetX = finalGoal->x;
        targetY = finalGoal->y;
        if (hypot(targetX - robot.x, targetY - robot.y) <= GOAL_TOLERANCE) {
            stop();
            setWheelSpeeds(robot, 0.0, 0.0);
            return;
        }
    }

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
