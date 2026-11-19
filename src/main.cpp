
#include "AStar.hpp"
#include "Camera.hpp"
#include "Lidar.hpp"
#include "Mapping.hpp"
#include "Navigator.hpp"
#include "OccupancyGrid.hpp"
#include "Robot.hpp"
#include "Sensor.hpp"
#include "SimulationConfig.hpp"
#include "World.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

using namespace std;

namespace {

constexpr int WIDTH = 900;
constexpr int HEIGHT = 650;

constexpr double DRIVE_SPEED = 1.0;
constexpr double TURN_SPEED = 0.5;

constexpr double REPLAN_COOLDOWN = 0.75;
constexpr double RETRY_DELAY = 1.5;
constexpr int MAX_FAILED_REPLANS = 5;

// LiDAR configuration
constexpr int LIDAR_RAYS = 72;
constexpr double LIDAR_RANGE = 6.0;
constexpr double LIDAR_INTERVAL = 0.1;  // 10 Hz

enum class NavigationState {
    Idle,
    Navigating,
    Waiting,
    NoPath,
    Arrived
};

const char* stateName(NavigationState state) {
    switch (state) {
        case NavigationState::Idle:
            return "Manual";
        case NavigationState::Navigating:
            return "Navigating";
        case NavigationState::Waiting:
            return "Waiting / replanning";
        case NavigationState::NoPath:
            return "No route";
        case NavigationState::Arrived:
            return "Arrived";
    }

    return "Unknown";
}

// -------------------------------------
// Screen geometry helpers
// -------------------------------------

SDL_FRect screenRect(
    const Camera& camera,
    const Obstacle& box
) {
    const ScreenPoint topLeft = camera.toScreen(
        box.x,
        box.y + box.height
    );

    return {
        topLeft.x,
        topLeft.y,
        static_cast<float>(box.width * camera.scale()),
        static_cast<float>(box.height * camera.scale())
    };
}

bool visible(const SDL_FRect& rect) {
    return rect.x + rect.w >= 0 &&
           rect.y + rect.h >= 0 &&
           rect.x <= WIDTH &&
           rect.y <= HEIGHT;
}

// -------------------------------------
// Draw floor and world boundaries
// -------------------------------------

void drawFloor(
    SDL_Renderer* renderer,
    const Camera& camera
) {
    const auto [left, top] =
        camera.screenToWorld(0, 0);

    const auto [right, bottom] =
        camera.screenToWorld(WIDTH, HEIGHT);

    SDL_SetRenderDrawColor(
        renderer, 34, 42, 55, 255
    );

    for (
        int x = static_cast<int>(ceil(left));
        x <= static_cast<int>(floor(right));
        ++x
    ) {
        if (x < World::minX || x > World::maxX) {
            continue;
        }

        const auto a = camera.toScreen(x, World::minY);
        const auto b = camera.toScreen(x, World::maxY);

        SDL_RenderLine(
            renderer, a.x, a.y, b.x, b.y
        );
    }

    for (
        int y = static_cast<int>(ceil(bottom));
        y <= static_cast<int>(floor(top));
        ++y
    ) {
        if (y < World::minY || y > World::maxY) {
            continue;
        }

        const auto a = camera.toScreen(World::minX, y);
        const auto b = camera.toScreen(World::maxX, y);

        SDL_RenderLine(
            renderer, a.x, a.y, b.x, b.y
        );
    }

    SDL_SetRenderDrawColor(
        renderer, 100, 125, 148, 255
    );

    const SDL_FRect border = screenRect(
        camera,
        {
            World::minX,
            World::minY,
            World::maxX - World::minX,
            World::maxY - World::minY
        }
    );

    SDL_RenderRect(renderer, &border);
}

// -------------------------------------
// Draw occupancy grid
// -------------------------------------

void drawGrid(
    SDL_Renderer* renderer,
    const Camera& camera,
    const OccupancyGrid& grid,
    const World& world
) {
    const auto [left, top] =
        camera.screenToWorld(0, 0);

    const auto [right, bottom] =
        camera.screenToWorld(WIDTH, HEIGHT);

    const GridCell low =
        grid.worldToGrid(left, bottom);

    const GridCell high =
        grid.worldToGrid(right, top);

    const int firstX = max(0, low.x);
    const int lastX = min(grid.getCols() - 1, high.x);

    const int firstY = max(0, low.y);
    const int lastY = min(grid.getRows() - 1, high.y);

    SDL_SetRenderDrawBlendMode(
        renderer, SDL_BLENDMODE_BLEND
    );

    SDL_SetRenderDrawColor(
        renderer, 255, 156, 56, 54
    );

    for (int y = firstY; y <= lastY; ++y) {
        for (int x = firstX; x <= lastX; ++x) {

            if (!grid.isStaticBlocked(x, y)) {
                continue;
            }

            const Obstacle cell{
                grid.gridToWorldX(x) -
                    SimConfig::GRID_CELL_SIZE * 0.5,
                grid.gridToWorldY(y) -
                    SimConfig::GRID_CELL_SIZE * 0.5,
                SimConfig::GRID_CELL_SIZE,
                SimConfig::GRID_CELL_SIZE
            };

            const SDL_FRect rect =
                screenRect(camera, cell);

            SDL_RenderFillRect(renderer, &rect);
        }
    }

    // Moving obstacle safety regions
    SDL_SetRenderDrawColor(
        renderer, 255, 145, 44, 70
    );

    const double margin =
        SimConfig::PLANNING_RADIUS +
        SimConfig::GRID_CELL_SIZE * sqrt(2.0) * 0.5;

    for (const auto& moving : world.movingObstacles) {
        const auto& box = moving.shape;

        const SDL_FRect unsafe = screenRect(
            camera,
            {
                box.x - margin,
                box.y - margin,
                box.width + 2 * margin,
                box.height + 2 * margin
            }
        );

        if (visible(unsafe)) {
            SDL_RenderFillRect(renderer, &unsafe);
        }
    }

    SDL_SetRenderDrawBlendMode(
        renderer, SDL_BLENDMODE_NONE
    );
}

// -------------------------------------
// Draw only the robot's discovered occupancy map
// -------------------------------------

void drawDiscoveredMap(
    SDL_Renderer* renderer,
    const Camera& camera,
    const Mapping& discoveredMap
) {
    const double cellSize = discoveredMap.getCellSize();
    const double originX = discoveredMap.gridToWorldX(0) - cellSize * 0.5;
    const double originY = discoveredMap.gridToWorldY(0) - cellSize * 0.5;
    const double pixelSize = cellSize * camera.scale();

    const ScreenPoint mapTopLeft = camera.toScreen(
        originX,
        originY + discoveredMap.getRows() * cellSize
    );
    const SDL_FRect mapRect{
        mapTopLeft.x,
        mapTopLeft.y,
        static_cast<float>(discoveredMap.getCols() * pixelSize),
        static_cast<float>(discoveredMap.getRows() * pixelSize)
    };
    if (!visible(mapRect)) {
        return;
    }

    // One background fill represents all unknown cells without drawing
    // thousands of identical rectangles every frame.
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(renderer, 45, 51, 61, 255);
    SDL_RenderFillRect(renderer, &mapRect);

    const auto [left, top] = camera.screenToWorld(0, 0);
    const auto [right, bottom] = camera.screenToWorld(WIDTH, HEIGHT);
    const int firstX = max(0, static_cast<int>(floor((left - originX) / cellSize)));
    const int lastX = min(discoveredMap.getCols() - 1,
        static_cast<int>(floor((right - originX) / cellSize)));
    const int firstY = max(0, static_cast<int>(floor((bottom - originY) / cellSize)));
    const int lastY = min(discoveredMap.getRows() - 1,
        static_cast<int>(floor((top - originY) / cellSize)));

    for (int y = firstY; y <= lastY; ++y) {
        for (int x = firstX; x <= lastX; ++x) {
            const CellState state = discoveredMap.getCell(x, y);
            if (state == CellState::Unknown) {
                continue;
            }
            if (state == CellState::Free) {
                SDL_SetRenderDrawColor(renderer, 151, 177, 193, 255);
            } else {
                SDL_SetRenderDrawColor(renderer, 235, 96, 65, 255);
            }

            const ScreenPoint topLeft = camera.toScreen(
                discoveredMap.gridToWorldX(x) - cellSize * 0.5,
                discoveredMap.gridToWorldY(y) + cellSize * 0.5
            );
            const SDL_FRect cell{
                topLeft.x,
                topLeft.y,
                static_cast<float>(pixelSize),
                static_cast<float>(pixelSize)
            };
            SDL_RenderFillRect(renderer, &cell);
        }
    }
}

// -------------------------------------
// Draw static and dynamic obstacles
// -------------------------------------

void drawWorld(
    SDL_Renderer* renderer,
    const Camera& camera,
    const World& world
) {
    for (const auto& box : world.obstacles) {
        const SDL_FRect rect =
            screenRect(camera, box);

        if (!visible(rect)) {
            continue;
        }

        SDL_SetRenderDrawColor(
            renderer, 88, 111, 137, 255
        );

        SDL_RenderFillRect(renderer, &rect);

        SDL_SetRenderDrawColor(
            renderer, 145, 169, 191, 255
        );

        SDL_RenderRect(renderer, &rect);
    }

    for (const auto& moving : world.movingObstacles) {
        const SDL_FRect rect =
            screenRect(camera, moving.shape);

        if (!visible(rect)) {
            continue;
        }

        SDL_SetRenderDrawColor(
            renderer, 224, 105, 72, 255
        );

        SDL_RenderFillRect(renderer, &rect);

        SDL_SetRenderDrawColor(
            renderer, 255, 185, 94, 255
        );

        SDL_RenderRect(renderer, &rect);
    }
}

// -------------------------------------
// Draw robot
// -------------------------------------

void drawRobot(
    SDL_Renderer* renderer,
    const Camera& camera,
    const Robot& robot
) {
    const double c = cos(robot.theta);
    const double s = sin(robot.theta);

    const auto point = [&](double lx, double ly) {
        return camera.toScreen(
            robot.x + lx * c - ly * s,
            robot.y + lx * s + ly * c
        );
    };

    const auto front = point(0.35, 0);
    const auto rearLeft = point(-0.25, 0.23);
    const auto rearRight = point(-0.25, -0.23);

    SDL_SetRenderDrawColor(
        renderer, 50, 130, 255, 255
    );

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

    // Robot heading indicator
    const auto center = point(0, 0);
    const auto heading = point(0.55, 0);

    SDL_SetRenderDrawColor(
        renderer, 255, 220, 50, 255
    );

    SDL_RenderLine(
        renderer,
        center.x, center.y,
        heading.x, heading.y
    );
}

// -------------------------------------
// NEW: Draw 360-degree LiDAR
// -------------------------------------

void drawLidar(
    SDL_Renderer* renderer,
    const Camera& camera,
    const Lidar& lidar,
    double scanX,
    double scanY
) {
    const auto& readings = lidar.getReadings();

    if (readings.empty()) {
        return;
    }

    SDL_SetRenderDrawBlendMode(
        renderer, SDL_BLENDMODE_BLEND
    );

    // Position at which this scan was captured
    const ScreenPoint start =
        camera.toScreen(scanX, scanY);

    for (const auto& reading : readings) {

        // Compute LiDAR ray endpoint
        const double endX =
            scanX +
            reading.distance * cos(reading.angle);

        const double endY =
            scanY +
            reading.distance * sin(reading.angle);

        // Convert world coordinates to screen pixels
        const ScreenPoint end =
            camera.toScreen(endX, endY);

        if (reading.hit) {
            // Red = detected obstacle
            SDL_SetRenderDrawColor(
                renderer, 255, 80, 80, 150
            );
        } else {
            // Green = no obstacle within sensor range
            SDL_SetRenderDrawColor(
                renderer, 50, 220, 100, 65
            );
        }

        SDL_RenderLine(
            renderer,
            start.x, start.y,
            end.x, end.y
        );
    }

    SDL_SetRenderDrawBlendMode(
        renderer, SDL_BLENDMODE_NONE
    );
}

// -------------------------------------
// Draw A* path
// -------------------------------------

void drawPath(
    SDL_Renderer* renderer,
    const Camera& camera,
    const OccupancyGrid& grid,
    const vector<GridCell>& path
) {
    SDL_SetRenderDrawColor(
        renderer, 50, 160, 255, 255
    );

    for (size_t i = 1; i < path.size(); ++i) {

        const auto a = camera.toScreen(
            grid.gridToWorldX(path[i - 1].x),
            grid.gridToWorldY(path[i - 1].y)
        );

        const auto b = camera.toScreen(
            grid.gridToWorldX(path[i].x),
            grid.gridToWorldY(path[i].y)
        );

        SDL_RenderLine(
            renderer,
            a.x, a.y,
            b.x, b.y
        );

        SDL_RenderLine(
            renderer,
            a.x + 1, a.y,
            b.x + 1, b.y
        );

        SDL_RenderLine(
            renderer,
            a.x, a.y + 1,
            b.x, b.y + 1
        );
    }
}

// -------------------------------------
// Draw destination marker
// -------------------------------------

void drawGoal(
    SDL_Renderer* renderer,
    const Camera& camera,
    const OccupancyGrid& grid,
    optional<GridCell> goal
) {
    if (!goal) {
        return;
    }

    const auto center = camera.toScreen(
        grid.gridToWorldX(goal->x),
        grid.gridToWorldY(goal->y)
    );

    const float size = clamp(
        static_cast<float>(camera.scale() * 0.22),
        6.0f,
        15.0f
    );

    const SDL_FRect marker{
        center.x - size * 0.5f,
        center.y - size * 0.5f,
        size,
        size
    };

    SDL_SetRenderDrawColor(
        renderer, 50, 220, 100, 255
    );

    SDL_RenderFillRect(renderer, &marker);
}

// -------------------------------------
// Manual keyboard controls
// -------------------------------------

void handleInput(
    Robot& robot,
    const bool* keys
) {
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
// Emergency stopping
// -------------------------------------

void applyEmergencyStop(
    Robot& robot,
    const World& world
) {
    const double forward =
        (robot.leftSpeed + robot.rightSpeed) * 0.5;

    if (forward <= 0.0) {
        return;
    }

    const double clearance = world.forwardClearance(
        robot.x,
        robot.y,
        robot.theta,
        SimConfig::ROBOT_RADIUS,
        SimConfig::SENSOR_RANGE
    );

    if (clearance <= SimConfig::SAFETY_CLEARANCE) {

        const double omega =
            (robot.rightSpeed - robot.leftSpeed)
            / robot.wheelBase;

        // Stop translation but preserve rotation
        robot.leftSpeed =
            -omega * robot.wheelBase * 0.5;

        robot.rightSpeed =
            omega * robot.wheelBase * 0.5;
    }
}

// -------------------------------------
// Check whether route is obstructed
// -------------------------------------

bool routeBlockedAhead(
    const Navigator& navigator,
    const OccupancyGrid& grid,
    const vector<GridCell>& path
) {
    const size_t from = navigator.waypointIndex();
    const size_t to = min(path.size(), from + 20);

    for (size_t i = from; i < to; ++i) {
        if (grid.isBlocked(path[i].x, path[i].y)) {
            return true;
        }
    }

    return false;
}

} // namespace

// =====================================
// MAIN APPLICATION
// =====================================

int main(int argc, char* argv[]) {

    // ---------------------------------
    // SDL initialization
    // ---------------------------------

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

    // ---------------------------------
    // Simulation components
    // ---------------------------------

    World world;

    Robot robot;
    robot.x = SimConfig::START_X;
    robot.y = SimConfig::START_Y;

    Camera camera(WIDTH, HEIGHT);
    camera.reset(robot.x, robot.y);

    OccupancyGrid grid(
        world,
        World::maxX - World::minX,
        World::maxY - World::minY,
        SimConfig::GRID_CELL_SIZE,
        SimConfig::PLANNING_RADIUS
    );

    Navigator navigator;

    vector<GridCell> path;
    optional<GridCell> goal;

    NavigationState state = NavigationState::Idle;

    double replanTimer = 0.0;
    double stallTimer = 0.0;

    int failedReplans = 0;
    int replansWithoutProgress = 0;

    double progressAnchorX = robot.x;
    double progressAnchorY = robot.y;

    // ---------------------------------
    // NEW: LiDAR setup
    // ---------------------------------

    Lidar lidar(LIDAR_RAYS, LIDAR_RANGE);

    // Robot-owned map: starts entirely unknown; no obstacle geometry is shared.
    Mapping discoveredMap(
        World::minX, World::minY,
        World::maxX, World::maxY,
        SimConfig::GRID_CELL_SIZE
    );

    double lidarAccumulator = 0.0;

    // Store the position where the scan was taken.
    double lidarScanX = robot.x;
    double lidarScanY = robot.y;

    bool showLidar = true;
    bool showDiscoveredMap = false;

    // Initial scan, before any movement.
    lidar.scan(robot, world);

    // Discover cells using the position of this initial scan.
    discoveredMap.update(
        lidar.getReadings(), lidarScanX, lidarScanY
    );

    // ---------------------------------
    // Navigation path planner
    // ---------------------------------

    auto planToGoal = [&] {

        if (!goal) {
            return;
        }

        if (++replansWithoutProgress > MAX_FAILED_REPLANS) {
            navigator.stop();
            path.clear();

            state = NavigationState::NoPath;

            cout
                << "Repeated replans made no progress; "
                << "navigation stopped.\n";

            return;
        }

        path = AStar::findPath(
            grid,
            grid.worldToGrid(robot.x, robot.y),
            *goal
        );

        if (path.empty()) {

            navigator.stop();

            if (++failedReplans >= MAX_FAILED_REPLANS) {
                state = NavigationState::NoPath;

                cout
                    << "No route to destination; "
                    << "navigation stopped.\n";
            } else {
                state = NavigationState::Waiting;
                replanTimer = RETRY_DELAY;
            }

        } else {

            navigator.setPath(path);

            state = NavigationState::Navigating;

            failedReplans = 0;
            replanTimer = REPLAN_COOLDOWN;
            stallTimer = 0.0;
        }
    };

    // ---------------------------------
    // Application state
    // ---------------------------------

    bool running = true;

    Uint64 previous = SDL_GetTicksNS();
    double accumulator = 0.0;

    NavigationState lastTitleState =
        NavigationState::Arrived;

    int lastZoom = -1;
    bool lastTitleDiscoveredMap = true;

    // =================================
    // Main simulation loop
    // =================================

    while (running) {

        SDL_Event event;

        // -----------------------------
        // SDL event handling
        // -----------------------------

        while (SDL_PollEvent(&event)) {

            if (
                event.type == SDL_EVENT_QUIT ||
                (
                    event.type == SDL_EVENT_KEY_DOWN &&
                    event.key.scancode == SDL_SCANCODE_ESCAPE
                )
            ) {
                running = false;
            }

            if (
                event.type == SDL_EVENT_KEY_DOWN &&
                !event.key.repeat
            ) {

                if (event.key.scancode == SDL_SCANCODE_F) {
                    camera.toggleFollow();
                }

                if (event.key.scancode == SDL_SCANCODE_R) {
                    camera.reset(robot.x, robot.y);
                }

                if (event.key.scancode == SDL_SCANCODE_M) {
                    showDiscoveredMap = !showDiscoveredMap;
                }

                if (event.key.scancode == SDL_SCANCODE_C) {
                    discoveredMap.reset();
                    lidarAccumulator = 0.0;
                }

                // NEW: Toggle LiDAR visualization
                if (event.key.scancode == SDL_SCANCODE_L) {
                    showLidar = !showLidar;

                    cout << "LiDAR display: "
                         << (showLidar ? "ON" : "OFF")
                         << '\n';
                }
            }

            if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                camera.zoomBy(event.wheel.y);
            }

            // Left-click destination
            if (
                event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
                event.button.button == SDL_BUTTON_LEFT
            ) {

                const auto [targetX, targetY] =
                    camera.screenToWorld(
                        event.button.x,
                        event.button.y
                    );

                goal = grid.worldToGrid(
                    targetX,
                    targetY
                );

                navigator.stop();
                path.clear();

                failedReplans = 0;
                replansWithoutProgress = 0;

                progressAnchorX = robot.x;
                progressAnchorY = robot.y;

                replanTimer = 0.0;

                if (grid.isStaticBlocked(
                    goal->x,
                    goal->y
                )) {
                    state = NavigationState::NoPath;

                    cout
                        << "Destination is outside "
                        << "the navigable maze.\n";
                } else {
                    planToGoal();
                }
            }
        }

        const bool* keys =
            SDL_GetKeyboardState(nullptr);

        // -----------------------------
        // Timing
        // -----------------------------

        const Uint64 now = SDL_GetTicksNS();

        accumulator += min(
            static_cast<double>(now - previous) / 1e9,
            0.05
        );

        previous = now;

        // =============================
        // Fixed physics update
        // =============================

        while (accumulator >= SimConfig::DT) {

            // Update moving obstacles
            world.update(
                SimConfig::DT,
                robot.x,
                robot.y,
                SimConfig::ROBOT_RADIUS
            );

            // Track navigation progress
            if (
                hypot(
                    robot.x - progressAnchorX,
                    robot.y - progressAnchorY
                ) > 0.75
            ) {
                progressAnchorX = robot.x;
                progressAnchorY = robot.y;

                replansWithoutProgress = 0;
            }

            replanTimer = max(
                0.0,
                replanTimer - SimConfig::DT
            );

            const bool manual =
                keys[SDL_SCANCODE_W] ||
                keys[SDL_SCANCODE_A] ||
                keys[SDL_SCANCODE_S] ||
                keys[SDL_SCANCODE_D];

            if (manual) {

                if (goal) {
                    navigator.stop();
                    path.clear();
                    goal.reset();

                    state = NavigationState::Idle;
                }

                handleInput(robot, keys);

            } else {

                if (
                    state == NavigationState::Waiting &&
                    replanTimer <= 0.0
                ) {
                    planToGoal();
                }

                if (
                    navigator.isActive() &&
                    routeBlockedAhead(
                        navigator,
                        grid,
                        path
                    )
                ) {
                    navigator.stop();
                    path.clear();

                    state = NavigationState::Waiting;

                    replanTimer = max(
                        replanTimer,
                        REPLAN_COOLDOWN
                    );
                }

                if (navigator.isActive()) {

                    navigator.update(robot, grid);

                    if (!navigator.isActive()) {
                        path.clear();
                        state = NavigationState::Arrived;
                    }

                } else {

                    robot.leftSpeed = 0.0;
                    robot.rightSpeed = 0.0;
                }
            }

            // -------------------------
            // Safety and robot physics
            // -------------------------

            applyEmergencyStop(robot, world);

            const Robot oldPose = robot;

            const double forward =
                (robot.leftSpeed + robot.rightSpeed) * 0.5;

            const int steps = max(
                1,
                static_cast<int>(
                    ceil(
                        abs(forward) *
                        SimConfig::DT / 0.10
                    )
                )
            );

            bool collided = false;

            for (int i = 0; i < steps; ++i) {

                Robot trial = robot;
                trial.update(SimConfig::DT / steps);

                const double dx = trial.x - robot.x;
                const double dy = trial.y - robot.y;

                const double distance = hypot(dx, dy);

                const bool sweptHit =
                    distance > 0.0 &&
                    world.forwardClearance(
                        robot.x,
                        robot.y,
                        atan2(dy, dx),
                        SimConfig::ROBOT_RADIUS,
                        distance
                    ) < distance - 1e-9;

                if (
                    sweptHit ||
                    world.collides(
                        trial.x,
                        trial.y,
                        SimConfig::ROBOT_RADIUS
                    )
                ) {
                    collided = true;
                    break;
                }

                robot = trial;
            }

            if (collided) {
                robot = oldPose;
            }

            // -------------------------
            // Navigation stall detection
            // -------------------------

            if (state == NavigationState::Navigating) {

                const double motion =
                    hypot(
                        robot.x - oldPose.x,
                        robot.y - oldPose.y
                    ) +
                    0.2 * abs(
                        remainder(
                            robot.theta - oldPose.theta,
                            2.0 * numbers::pi
                        )
                    );

                stallTimer =
                    motion < 1e-5
                    ? stallTimer + SimConfig::DT
                    : 0.0;

                if (stallTimer > 2.5) {

                    navigator.stop();
                    path.clear();

                    state = NavigationState::Waiting;

                    replanTimer = max(
                        replanTimer,
                        REPLAN_COOLDOWN
                    );

                    stallTimer = 0.0;
                }
            }

            // -------------------------
            // Camera controls
            // -------------------------

            if (!camera.followsRobot()) {

                const double panSpeed =
                    8.0 * SimConfig::DT;

                camera.pan(
                    (
                        keys[SDL_SCANCODE_RIGHT] -
                        keys[SDL_SCANCODE_LEFT]
                    ) * panSpeed,

                    (
                        keys[SDL_SCANCODE_UP] -
                        keys[SDL_SCANCODE_DOWN]
                    ) * panSpeed
                );
            }

            camera.update(
                SimConfig::DT,
                robot.x,
                robot.y
            );

            // -------------------------
            // NEW: LiDAR scan update
            // -------------------------

            lidarAccumulator += SimConfig::DT;

            if (lidarAccumulator >= LIDAR_INTERVAL) {

                // Capture location at scan time.
                lidarScanX = robot.x;
                lidarScanY = robot.y;

                // Simulate a full 360-degree scan.
                lidar.scan(robot, world);

                // Update only when a fresh scan is available.
                // Scan position and measurement angles belong to the same pose.
                discoveredMap.update(
                    lidar.getReadings(), lidarScanX, lidarScanY
                );

                lidarAccumulator -= LIDAR_INTERVAL;
            }

            accumulator -= SimConfig::DT;
        }

        // -----------------------------
        // Window title
        // -----------------------------

        const int zoom =
            static_cast<int>(round(camera.scale()));

        if (
            state != lastTitleState ||
            zoom != lastZoom ||
            showDiscoveredMap != lastTitleDiscoveredMap
        ) {

            const string title =
                string("2D Robot Simulator | ") +
                stateName(state) +
                " | Zoom " +
                to_string(zoom) +
                " px/m | " +
                (showDiscoveredMap ? "Discovered map" : "Complete maze");

            SDL_SetWindowTitle(
                window,
                title.c_str()
            );

            lastTitleState = state;
            lastZoom = zoom;
            lastTitleDiscoveredMap = showDiscoveredMap;
        }

        // =============================
        // Rendering
        // =============================

        SDL_SetRenderDrawColor(
            renderer, 20, 25, 35, 255
        );

        SDL_RenderClear(renderer);

        if (showDiscoveredMap) {
            drawDiscoveredMap(renderer, camera, discoveredMap);
        } else {
            drawFloor(renderer, camera);

            drawGrid(
                renderer,
                camera,
                grid,
                world
            );

            drawWorld(
                renderer,
                camera,
                world
            );

            drawPath(
                renderer,
                camera,
                grid,
                path
            );

            drawGoal(
                renderer,
                camera,
                grid,
                goal
            );
        }

        // NEW: Display 360-degree LiDAR
        if (showLidar) {

            drawLidar(
                renderer,
                camera,
                lidar,
                lidarScanX,
                lidarScanY
            );
        }

        drawRobot(
            renderer,
            camera,
            robot
        );

        SDL_RenderPresent(renderer);
        SDL_Delay(1);
    }

    // ---------------------------------
    // Cleanup
    // ---------------------------------

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
