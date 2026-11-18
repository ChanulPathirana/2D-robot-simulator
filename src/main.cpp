
#include "Robot.hpp"
#include "World.hpp"
#include "Sensor.hpp"
#include "OccupancyGrid.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>

using namespace std;

// -------------------------------------
// Simulation settings
// -------------------------------------

constexpr int WIDTH = 900;
constexpr int HEIGHT = 650;

constexpr double SCALE = 60.0;
constexpr double DT = 1.0 / 60.0;

constexpr double ROBOT_RADIUS = 0.4;
constexpr double SENSOR_RANGE = 4.0;
constexpr double SAFETY_CLEARANCE = 0.3;

constexpr double DRIVE_SPEED = 1.0;
constexpr double TURN_SPEED = 0.5;

constexpr double GRID_CELL_SIZE = 0.25;

struct Point {
    float x;
    float y;
};

// -------------------------------------
// World coordinates -> screen pixels
// -------------------------------------

Point toScreen(double x, double y) {
    return {
        static_cast<float>(
            WIDTH / 2.0 + x * SCALE
        ),
        static_cast<float>(
            HEIGHT / 2.0 - y * SCALE
        )
    };
}

// -------------------------------------
// Draw occupancy grid
// -------------------------------------

void drawGrid(
    SDL_Renderer* renderer,
    const OccupancyGrid& grid
) {
    SDL_SetRenderDrawBlendMode(
        renderer,
        SDL_BLENDMODE_BLEND
    );

    for (int y = 0; y < grid.getRows(); ++y) {
        for (int x = 0; x < grid.getCols(); ++x) {

            if (!grid.isBlocked(x, y)) {
                continue;
            }

            double wx = grid.gridToWorldX(x);
            double wy = grid.gridToWorldY(y);

            Point topLeft = toScreen(
                wx - GRID_CELL_SIZE / 2.0,
                wy + GRID_CELL_SIZE / 2.0
            );

            SDL_FRect rect = {
                topLeft.x,
                topLeft.y,
                static_cast<float>(
                    GRID_CELL_SIZE * SCALE
                ),
                static_cast<float>(
                    GRID_CELL_SIZE * SCALE
                )
            };

            // Transparent orange blocked cell
            SDL_SetRenderDrawColor(
                renderer,
                255, 170, 40, 100
            );

            SDL_RenderFillRect(renderer, &rect);
        }
    }

    // Restore normal opaque rendering
    SDL_SetRenderDrawBlendMode(
        renderer,
        SDL_BLENDMODE_NONE
    );
}

// -------------------------------------
// Draw obstacles
// -------------------------------------

void drawWorld(
    SDL_Renderer* renderer,
    const World& world
) {
    SDL_SetRenderDrawColor(
        renderer,
        200, 80, 80, 255
    );

    for (const auto& obstacle : world.obstacles) {

        Point topLeft = toScreen(
            obstacle.x,
            obstacle.y + obstacle.height
        );

        SDL_FRect rect = {
            topLeft.x,
            topLeft.y,
            static_cast<float>(
                obstacle.width * SCALE
            ),
            static_cast<float>(
                obstacle.height * SCALE
            )
        };

        SDL_RenderFillRect(renderer, &rect);
    }
}

// -------------------------------------
// Draw robot
// -------------------------------------

void drawRobot(
    SDL_Renderer* renderer,
    const Robot& robot
) {
    double c = cos(robot.theta);
    double s = sin(robot.theta);

    // Transform local robot coordinates
    // into world coordinates, then pixels.
    auto transform = [&](double lx, double ly) {
        return toScreen(
            robot.x + lx * c - ly * s,
            robot.y + lx * s + ly * c
        );
    };

    Point front = transform(0.35, 0.0);
    Point rearLeft = transform(-0.25, 0.23);
    Point rearRight = transform(-0.25, -0.23);

    SDL_SetRenderDrawColor(
        renderer,
        50, 130, 255, 255
    );

    // Blue triangle
    SDL_RenderLine(
        renderer,
        front.x, front.y,
        rearLeft.x, rearLeft.y
    );

    SDL_RenderLine(
        renderer,
        rearLeft.x, rearLeft.y,
        rearRight.x, rearRight.y
    );

    SDL_RenderLine(
        renderer,
        rearRight.x, rearRight.y,
        front.x, front.y
    );

    // Yellow heading indicator
    Point center = transform(0.0, 0.0);
    Point heading = transform(0.55, 0.0);

    SDL_SetRenderDrawColor(
        renderer,
        255, 220, 50, 255
    );

    SDL_RenderLine(
        renderer,
        center.x, center.y,
        heading.x, heading.y
    );
}

// -------------------------------------
// Draw distance sensors
// -------------------------------------

void drawSensors(
    SDL_Renderer* renderer,
    const Robot& robot,
    const World& world
) {
    constexpr double DEG_TO_RAD =
        numbers::pi / 180.0;

    const double offsets[] = {
        -45.0,
        0.0,
        45.0
    };

    Point start = toScreen(
        robot.x,
        robot.y
    );

    for (double offset : offsets) {

        double angle =
            robot.theta + offset * DEG_TO_RAD;

        RayHit reading = Sensor::castRay(
            world,
            robot.x,
            robot.y,
            angle,
            SENSOR_RANGE
        );

        Point end = toScreen(
            robot.x + reading.distance * cos(angle),
            robot.y + reading.distance * sin(angle)
        );

        if (reading.hit) {
            // Red: detected obstacle
            SDL_SetRenderDrawColor(
                renderer,
                255, 90, 90, 255
            );
        } else {
            // Green: no detected obstacle
            SDL_SetRenderDrawColor(
                renderer,
                50, 220, 100, 255
            );
        }

        SDL_RenderLine(
            renderer,
            start.x, start.y,
            end.x, end.y
        );
    }
}

// -------------------------------------
// Keyboard controls
// -------------------------------------

void handleInput(
    Robot& robot,
    const bool* keys
) {
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;

    // Forward
    if (keys[SDL_SCANCODE_W]) {
        robot.leftSpeed += DRIVE_SPEED;
        robot.rightSpeed += DRIVE_SPEED;
    }

    // Backward
    if (keys[SDL_SCANCODE_S]) {
        robot.leftSpeed -= DRIVE_SPEED;
        robot.rightSpeed -= DRIVE_SPEED;
    }

    // Rotate left
    if (keys[SDL_SCANCODE_A]) {
        robot.leftSpeed -= TURN_SPEED;
        robot.rightSpeed += TURN_SPEED;
    }

    // Rotate right
    if (keys[SDL_SCANCODE_D]) {
        robot.leftSpeed += TURN_SPEED;
        robot.rightSpeed -= TURN_SPEED;
    }
}

// -------------------------------------
// Footprint-aware emergency stop
// -------------------------------------

void applyEmergencyStop(
    Robot& robot,
    const World& world
) {
    double forwardSpeed =
        (robot.leftSpeed + robot.rightSpeed) / 2.0;

    // Do not block reverse or stationary motion
    if (forwardSpeed <= 0.0) {
        return;
    }

    // Check forward clearance of entire circular body
    double clearance = world.forwardClearance(
        robot.x,
        robot.y,
        robot.theta,
        ROBOT_RADIUS,
        SENSOR_RANGE
    );

    if (clearance <= SAFETY_CLEARANCE) {

        // Preserve angular velocity
        double omega =
            (robot.rightSpeed - robot.leftSpeed)
            / robot.wheelBase;

        // Remove forward translation
        robot.leftSpeed =
            -omega * robot.wheelBase / 2.0;

        robot.rightSpeed =
            omega * robot.wheelBase / 2.0;
    }
}

// -------------------------------------
// Main application
// -------------------------------------

int main(int argc, char* argv[]) {

    // 1. Initialize SDL3
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        cerr << "SDL initialization failed: "
             << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "2D Robot Simulator",
        WIDTH,
        HEIGHT,
        0
    );

    if (!window) {
        cerr << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer =
        SDL_CreateRenderer(window, nullptr);

    if (!renderer) {
        cerr << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // 2. Create robot and world
    Robot robot;
    World world;

    // 3. Create occupancy grid once
    OccupancyGrid grid(
        world,
        WIDTH / SCALE,
        HEIGHT / SCALE,
        GRID_CELL_SIZE,
        ROBOT_RADIUS
    );

    bool running = true;

    Uint64 previous = SDL_GetTicksNS();
    double accumulator = 0.0;

    // ---------------------------------
    // Main simulation loop
    // ---------------------------------

    while (running) {

        // Process SDL events
        SDL_Event event;

        while (SDL_PollEvent(&event)) {

            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }

            if (
                event.type == SDL_EVENT_KEY_DOWN &&
                event.key.scancode == SDL_SCANCODE_ESCAPE
            ) {
                running = false;
            }
        }

        const bool* keys =
            SDL_GetKeyboardState(nullptr);

        // Calculate elapsed time
        Uint64 now = SDL_GetTicksNS();

        double elapsed =
            static_cast<double>(
                now - previous
            ) / 1e9;

        previous = now;

        elapsed = min(elapsed, 0.05);
        accumulator += elapsed;

        // ---------------------------------
        // Fixed physics update
        // ---------------------------------

        while (accumulator >= DT) {

            // Read keyboard commands
            handleInput(robot, keys);

            // Stop unsafe forward translation
            applyEmergencyStop(robot, world);

            // Save previous pose
            double oldX = robot.x;
            double oldY = robot.y;
            double oldTheta = robot.theta;

            // Update differential-drive movement
            robot.update(DT);

            // Prevent obstacle penetration
            if (world.collides(
                robot.x,
                robot.y,
                ROBOT_RADIUS
            )) {
                robot.x = oldX;
                robot.y = oldY;
                robot.theta = oldTheta;
            }

            accumulator -= DT;
        }

        // ---------------------------------
        // Render simulation
        // ---------------------------------

        SDL_SetRenderDrawColor(
            renderer,
            20, 22, 32, 255
        );

        SDL_RenderClear(renderer);

        // NEW: Display blocked grid cells
        drawGrid(renderer, grid);

        // Draw obstacles, sensor rays and robot
        drawWorld(renderer, world);
        drawSensors(renderer, robot, world);
        drawRobot(renderer, robot);

        SDL_RenderPresent(renderer);

        SDL_Delay(1);
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
