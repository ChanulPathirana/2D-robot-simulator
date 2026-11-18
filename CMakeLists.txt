
cmake_minimum_required(VERSION 3.16)

project(RobotSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SDL3 dependency
add_subdirectory(external/SDL EXCLUDE_FROM_ALL)

# Our simulator source files
add_executable(robot_simulator
    src/main.cpp
    src/Robot.cpp
    src/World.cpp
    src/Sensor.cpp
)

# Header directory
target_include_directories(robot_simulator
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# SDL3 library
target_link_libraries(robot_simulator
    PRIVATE SDL3::SDL3
)
