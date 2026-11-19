#pragma once

namespace SimConfig {
inline constexpr double WORLD_MIN_X = -20.0;
inline constexpr double WORLD_MAX_X = 20.0;
inline constexpr double WORLD_MIN_Y = -15.0;
inline constexpr double WORLD_MAX_Y = 15.0;
inline constexpr double START_X = -17.5;
inline constexpr double START_Y = -12.5;
inline constexpr double ROBOT_RADIUS = 0.4;
inline constexpr double SAFETY_CLEARANCE = 0.3;
inline constexpr double PLANNING_RADIUS = ROBOT_RADIUS + SAFETY_CLEARANCE;
inline constexpr double GRID_CELL_SIZE = 0.25;
inline constexpr double SENSOR_RANGE = 4.0;
inline constexpr double DT = 1.0 / 60.0;
}
