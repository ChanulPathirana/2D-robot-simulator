
#include "Robot.hpp"
#include "World.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;

constexpr int WIDTH = 900;
constexpr int HEIGHT = 650;
constexpr double SCALE = 60.0;
constexpr double DT = 1.0 / 60.0;

struct Point {
    float x, y;
};

// Convert robot world coordinates to screen pixels
Point toScreen(double x, double y) {
    return {
        static_cast<float>(WIDTH / 2.0 + x * SCALE),
        static_cast<float>(HEIGHT / 2.0 - y * SCALE)
    };
}

// Draw the robot and its heading
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

    SDL_RenderLine(renderer,
        front.x, front.y, rearLeft.x, rearLeft.y);

    SDL_RenderLine(renderer,
        rearLeft.x, rearLeft.y, rearRight.x, rearRight.y);

    SDL_RenderLine(renderer,
        rearRight.x, rearRight.y, front.x, front.y);

    // Yellow line indicates robot direction
    Point center = transform(0.0, 0.0);
    Point heading = transform(0.55, 0.0);

    SDL_SetRenderDrawColor(renderer, 255, 220, 50, 255);

    SDL_RenderLine(renderer,
        center.x, center.y, heading.x, heading.y);
}

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


int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        cerr << "SDL initialization failed: "
             << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "2D Robot Simulator", WIDTH, HEIGHT, 0
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

        robot.leftSpeed = 0.0;
        robot.rightSpeed = 0.0;

        // Forward
        if (keys[SDL_SCANCODE_W]) {
            robot.leftSpeed += 1.0;
            robot.rightSpeed += 1.0;
        }

        // Backward
        if (keys[SDL_SCANCODE_S]) {
            robot.leftSpeed -= 1.0;
            robot.rightSpeed -= 1.0;
        }

        // Turn left
        if (keys[SDL_SCANCODE_A]) {
            robot.leftSpeed -= 0.5;
            robot.rightSpeed += 0.5;
        }

        // Turn right
        if (keys[SDL_SCANCODE_D]) {
            robot.leftSpeed += 0.5;
            robot.rightSpeed -= 0.5;
        }

        Uint64 now = SDL_GetTicksNS();

        double elapsed =
            static_cast<double>(now - previous) / 1e9;

        previous = now;

        elapsed = min(elapsed, 0.05);
        accumulator += elapsed;

        // Update robot physics at fixed intervals
        // while (accumulator >= DT) {
        //     robot.update(DT);
        //     accumulator -= DT;
        // }

        while (accumulator >= DT) {

            // 1. Save robot's previous position
            double oldX = robot.x;
            double oldY = robot.y;
            double oldTheta = robot.theta;

            // 2. Update robot movement
            robot.update(DT);

            // 3. Check collision
            constexpr double ROBOT_RADIUS = 0.4;

            if (world.collides(robot.x, robot.y, ROBOT_RADIUS)) {

                // 4. Restore previous pose if collision occurs
                robot.x = oldX;
                robot.y = oldY;
                robot.theta = oldTheta;
            }

            accumulator -= DT;
        }


        // Clear background
        SDL_SetRenderDrawColor(renderer, 20, 22, 32, 255);
        SDL_RenderClear(renderer);
        drawWorld(renderer, world);

        // Render robot
        drawRobot(renderer, robot);

        SDL_RenderPresent(renderer);
        SDL_Delay(1);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
