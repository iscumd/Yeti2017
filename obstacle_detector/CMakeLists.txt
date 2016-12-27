cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SegmentObstacle.msg
  Obstacles.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES obstacle_detector
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  # DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
target_link_libraries(obstacle_detection_node ${catkin_LIBRARIES})
add_dependencies(obstacle_detection_node ${PROJECT_NAME}_gencpp)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_obstacle_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

