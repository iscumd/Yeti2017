cmake_minimum_required(VERSION 2.8.3)
project(test_package)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp joy)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(relay src/relay.cpp)
target_link_libraries(relay ${catkin_LIBRARIES})
