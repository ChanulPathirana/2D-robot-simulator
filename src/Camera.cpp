#include "Camera.hpp"
#include "SimulationConfig.hpp"

#include <algorithm>
#include <cmath>

using namespace std;

Camera::Camera(int width, int height) : width(width), height(height) {}

ScreenPoint Camera::toScreen(double worldX, double worldY) const {
    return {
        static_cast<float>(width * 0.5 + (worldX - centerX) * pixelsPerMetre),
        static_cast<float>(height * 0.5 - (worldY - centerY) * pixelsPerMetre)
    };
}

pair<double, double> Camera::screenToWorld(double screenX, double screenY) const {
    return {
        centerX + (screenX - width * 0.5) / pixelsPerMetre,
        centerY - (screenY - height * 0.5) / pixelsPerMetre
    };
}

void Camera::clampToWorld() {
    const double halfW = width / (2.0 * pixelsPerMetre);
    const double halfH = height / (2.0 * pixelsPerMetre);
    centerX = halfW * 2.0 >= SimConfig::WORLD_MAX_X - SimConfig::WORLD_MIN_X
        ? (SimConfig::WORLD_MIN_X + SimConfig::WORLD_MAX_X) * 0.5
        : clamp(centerX, SimConfig::WORLD_MIN_X + halfW, SimConfig::WORLD_MAX_X - halfW);
    centerY = halfH * 2.0 >= SimConfig::WORLD_MAX_Y - SimConfig::WORLD_MIN_Y
        ? (SimConfig::WORLD_MIN_Y + SimConfig::WORLD_MAX_Y) * 0.5
        : clamp(centerY, SimConfig::WORLD_MIN_Y + halfH, SimConfig::WORLD_MAX_Y - halfH);
}

void Camera::update(double dt, double robotX, double robotY) {
    if (following) {
        const double blend = 1.0 - exp(-6.0 * max(0.0, dt));
        centerX += (robotX - centerX) * blend;
        centerY += (robotY - centerY) * blend;
    }
    clampToWorld();
}

void Camera::pan(double dx, double dy) {
    centerX += dx;
    centerY += dy;
    clampToWorld();
}

void Camera::zoomBy(double wheelSteps) {
    pixelsPerMetre = clamp(pixelsPerMetre * pow(1.15, wheelSteps), 20.0, 125.0);
    clampToWorld();
}

void Camera::toggleFollow() { following = !following; }

void Camera::reset(double robotX, double robotY) {
    following = true;
    centerX = robotX;
    centerY = robotY;
    clampToWorld();
}
