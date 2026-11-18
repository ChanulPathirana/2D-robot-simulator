
#pragma once

class Robot {
public:
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double wheelBase = 0.5;
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    void update(double dt);
};
