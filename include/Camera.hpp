#pragma once

#include <utility>

struct ScreenPoint {
    float x;
    float y;
};

class Camera {
public:
    Camera(int width, int height);

    ScreenPoint toScreen(double worldX, double worldY) const;
    std::pair<double, double> screenToWorld(double screenX, double screenY) const;
    void update(double dt, double robotX, double robotY);
    void pan(double dx, double dy);
    void zoomBy(double wheelSteps);
    void toggleFollow();
    void reset(double robotX, double robotY);

    double scale() const { return pixelsPerMetre; }
    bool followsRobot() const { return following; }
    int viewportWidth() const { return width; }
    int viewportHeight() const { return height; }

private:
    void clampToWorld();

    int width;
    int height;
    double centerX = 0.0;
    double centerY = 0.0;
    double pixelsPerMetre = 60.0;
    bool following = true;
};
