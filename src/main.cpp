
#include "Robot.hpp"
#include "World.hpp"
#include "Sensor.hpp"

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

struct Point {
    float x;
    float y;
};

// -------------------------------------
// World coordinates -> screen pixels
// -------------------------------------

Point toScreen(double x, double y) {
    return {
        static_cast<float>(WIDTH / 2.0 + x * SCALE),
        static_cast<float>(HEIGHT / 2.0 - y * SCALE)
    };
}

// -------------------------------------
// Draw obstacles
// -------------------------------------

void drawWorld(SDL_Renderer* renderer, const World& world) {
    SDL_SetRenderDrawColor(renderer, 200, 80, 80, 255);

    for (const auto& obstacle : world.obstacles) {
        Point topLeft = toScreen(
            obstacle.x,
            obstacle.y + obstacle.height
        );

        SDL_FRect rect = {
            topLeft.x,
            topLeft.y,
            static_cast<float>(obstacle.width * SCALE),
            static_cast<float>(obstacle.height * SCALE)
        };

        SDL_RenderFillRect(renderer, &rect);
    }
}

// -------------------------------------
// Draw robot
// -------------------------------------

void drawRobot(SDL_Renderer* renderer, const Robot& robot) {
    double c = cos(robot.theta);
    double s = sin(robot.theta);

    auto transform = [&](double lx, double ly) {
        return toScreen(
            robot.x + lx * c - ly * s,
            robot.y + lx * s + ly * c
        );
    };

    Point front = transform(0.35, 0.0);
    Point rearLeft = transform(-0.25, 0.23);
    Point rearRight = transform(-0.25, -0.23);

    SDL_SetRenderDrawColor(renderer, 50, 130, 255, 255);

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

    SDL_SetRenderDrawColor(renderer, 255, 220, 50, 255);

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
    constexpr double DEG_TO_RAD = numbers::pi / 180.0;

    const double offsets[] = {
        -45.0,
        0.0,
        45.0
    };

    Point start = toScreen(robot.x, robot.y);

    for (double offset : offsets) {
        double angle = robot.theta + offset * DEG_TO_RAD;

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
            SDL_SetRenderDrawColor(
                renderer, 255, 90, 90, 255
            );
        } else {
            SDL_SetRenderDrawColor(
                renderer, 50, 220, 100, 255
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

void handleInput(Robot& robot, const bool* keys) {
    robot.leftSpeed = 0.0;
    robot.rightSpeed = 0.0;

    if (keys[SDL_SCANCODE_W]) {
        robot.leftSpeed += DRIVE_SPEED;
        robot.rightSpeed += DRIVE_SPEED;
    }

    if (keys[SDL_SCANCODE_S]) {
        robot.leftSpeed -= DRIVE_SPEED;
        robot.rightSpeed -= DRIVE_SPEED;
    }

    if (keys[SDL_SCANCODE_A]) {
        robot.leftSpeed -= TURN_SPEED;
        robot.rightSpeed += TURN_SPEED;
    }

    if (keys[SDL_SCANCODE_D]) {
        robot.leftSpeed += TURN_SPEED;
        robot.rightSpeed -= TURN_SPEED;
    }
}

// -------------------------------------
// Footprint-based emergency stop
// -------------------------------------

void applyEmergencyStop(Robot& robot, const World& world) {
    // Robot's forward linear velocity
    double forwardSpeed =
        (robot.leftSpeed + robot.rightSpeed) / 2.0;

    if (forwardSpeed <= 0.0) {
        return;
    }

    // Clearance assumes straight travel; collision rollback still guards curved steps.
    double clearance = world.forwardClearance(
        robot.x,
        robot.y,
        robot.theta,
        ROBOT_RADIUS,
        SENSOR_RANGE
    );

    if (clearance <= SAFETY_CLEARANCE) {

        // Remove forward translation while
        // keeping differential steering possible.
        double omega =
            (robot.rightSpeed - robot.leftSpeed)
            / robot.wheelBase;

        robot.leftSpeed = -omega * robot.wheelBase / 2.0;
        robot.rightSpeed = omega * robot.wheelBase / 2.0;
    }
}

// -------------------------------------
// Main application
// -------------------------------------

int main(int argc, char* argv[]) {
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

    Robot robot;
    World world;

    bool running = true;

    Uint64 previous = SDL_GetTicksNS();
    double accumulator = 0.0;

    while (running) {
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }

            if (event.type == SDL_EVENT_KEY_DOWN &&
                event.key.scancode == SDL_SCANCODE_ESCAPE) {
                running = false;
            }
        }

        const bool* keys = SDL_GetKeyboardState(nullptr);

        Uint64 now = SDL_GetTicksNS();

        double elapsed =
            static_cast<double>(now - previous) / 1e9;

        previous = now;

        elapsed = min(elapsed, 0.05);
        accumulator += elapsed;

        // ---------------------------------
        // Fixed physics update
        // ---------------------------------

        while (accumulator >= DT) {

            // 1. Read keyboard commands
            handleInput(robot, keys);

            // 2. Check footprint clearance and restrict forward motion
            applyEmergencyStop(robot, world);

            // 3. Save previous pose
            double oldX = robot.x;
            double oldY = robot.y;
            double oldTheta = robot.theta;

            // 4. Update robot movement
            robot.update(DT);

            // 5. Prevent obstacle penetration
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
        // Rendering
        // ---------------------------------

        SDL_SetRenderDrawColor(
            renderer, 20, 22, 32, 255
        );

        SDL_RenderClear(renderer);

        drawWorld(renderer, world);
        drawSensors(renderer, robot, world);
        drawRobot(renderer, robot);

        SDL_RenderPresent(renderer);
        SDL_Delay(1);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
