cmake_minimum_required (VERSION 2.6)

project(TD_CMake)

set(CMAKE_BUILD_TYPE Debug)

add_library(vibes STATIC vibes.cpp)

add_library(interval STATIC interval.cpp)


add_executable(robot_simu main.cpp robot.cpp controller.cpp)
target_link_libraries(robot_simu vibes interval)

install(TARGETS robot_simu DESTINATION bin)
install(TARGETS vibes DESTINATION lib)
