
#include "Robot.hpp"

#include <cmath>
#include <numbers>

using namespace std;

void Robot::update(double dt) {
    if (dt <= 0.0 || wheelBase <= 0.0) {
        return;
    }

    // Calculate linear and angular velocities
    double v = (leftSpeed + rightSpeed) / 2.0;
    double omega = (rightSpeed - leftSpeed) / wheelBase;

    // Midpoint integration
    double midTheta = theta + omega * dt / 2.0;

    // Update robot position
    x += v * cos(midTheta) * dt;
    y += v * sin(midTheta) * dt;

    // Update orientation
    theta = remainder(
        theta + omega * dt,
        2.0 * numbers::pi
    );
}
