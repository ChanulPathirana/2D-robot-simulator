#pragma once

#include "Robot.hpp"
#include "World.hpp"
#include "SimulationConfig.hpp"

#include <algorithm>
#include <cmath>

// Shared motion safety step for the SDL loop and deterministic headless runs.
// Returns true when a swept collision or endpoint collision rolled motion back.
inline bool advanceRobotSafely(Robot& robot, const World& world, double dt) {
    const double forward = (robot.leftSpeed + robot.rightSpeed) * 0.5;
    if (forward > 0.0 && world.forwardClearance(
            robot.x, robot.y, robot.theta, SimConfig::ROBOT_RADIUS,
            SimConfig::SENSOR_RANGE) <= SimConfig::SAFETY_CLEARANCE) {
        const double omega = (robot.rightSpeed - robot.leftSpeed) / robot.wheelBase;
        robot.leftSpeed = -omega * robot.wheelBase * 0.5;
        robot.rightSpeed = omega * robot.wheelBase * 0.5;
    }

    const Robot oldPose = robot;
    const double speed = (robot.leftSpeed + robot.rightSpeed) * 0.5;
    const int steps = std::max(1, static_cast<int>(
        std::ceil(std::abs(speed) * dt / 0.10)));
    for (int i = 0; i < steps; ++i) {
        Robot trial = robot;
        trial.update(dt / steps);
        const double dx = trial.x - robot.x;
        const double dy = trial.y - robot.y;
        const double distance = std::hypot(dx, dy);
        const bool sweptHit = distance > 0.0 && world.forwardClearance(
            robot.x, robot.y, std::atan2(dy, dx), SimConfig::ROBOT_RADIUS,
            distance) < distance - 1e-9;
        if (sweptHit || world.collides(trial.x, trial.y,
                                       SimConfig::ROBOT_RADIUS)) {
            robot = oldPose;
            return true;
        }
        robot = trial;
    }
    return false;
}
