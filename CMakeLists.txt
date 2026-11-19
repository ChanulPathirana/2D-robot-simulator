
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
    src/OccupancyGrid.cpp
    src/AStar.cpp
    src/Navigator.cpp
    src/Camera.cpp
    src/Lidar.cpp
    src/Mapping.cpp
    src/DiscoveredPlanningGrid.cpp
    src/FrontierExplorer.cpp
    
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

include(CTest)
if(BUILD_TESTING)
    add_executable(mapping_tests
        tests/MappingTests.cpp
        src/Mapping.cpp
    )
    target_include_directories(mapping_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME mapping_tests COMMAND mapping_tests)

    add_executable(planning_tests
        tests/PlanningTests.cpp
        src/AStar.cpp
        src/DiscoveredPlanningGrid.cpp
        src/Lidar.cpp
        src/Mapping.cpp
        src/Navigator.cpp
        src/OccupancyGrid.cpp
        src/Robot.cpp
        src/Sensor.cpp
        src/World.cpp
    )
    target_include_directories(planning_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME planning_tests COMMAND planning_tests)

    add_executable(frontier_tests
        tests/FrontierTests.cpp
        src/AStar.cpp
        src/DiscoveredPlanningGrid.cpp
        src/FrontierExplorer.cpp
        src/Lidar.cpp
        src/Mapping.cpp
        src/Navigator.cpp
        src/Robot.cpp
        src/Sensor.cpp
        src/World.cpp
    )
    target_include_directories(frontier_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME frontier_tests COMMAND frontier_tests)

    add_executable(goal_exploration_tests
        tests/GoalExplorationTests.cpp
        src/AStar.cpp
        src/DiscoveredPlanningGrid.cpp
        src/FrontierExplorer.cpp
        src/Lidar.cpp
        src/Mapping.cpp
        src/Navigator.cpp
        src/Robot.cpp
        src/Sensor.cpp
        src/World.cpp
    )
    target_include_directories(goal_exploration_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME goal_exploration_tests COMMAND goal_exploration_tests)
endif()
