cmake_minimum_required(VERSION 3.5)
project(paper_benchmarks)


set (CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

add_executable( benchmark_baseline
                src/benchmark_baseline.cpp
                src/scene.cpp
                src/primitive_pick_and_place.cpp
                )

## Specify libraries to link a library or executable target against
ament_target_dependencies(benchmark_baseline 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
)

add_executable( benchmark_synchronous 
                src/benchmark_synchronous.cpp
                src/scene.cpp
                src/primitive_pick_and_place.cpp
                )

## Specify libraries to link a library or executable target against
ament_target_dependencies(benchmark_synchronous 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
)

add_executable( benchmark_asynchronous 
                src/benchmark_asynchronous.cpp
                src/scene.cpp
                src/primitive_pick_and_place.cpp
                )

## Specify libraries to link a library or executable target against
ament_target_dependencies(benchmark_asynchronous 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
)

add_executable( create_scene 
                src/create_scene.cpp
                src/scene.cpp
                )

## Specify libraries to link a library or executable target against
ament_target_dependencies(create_scene 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
)



#############
## Install ##
#############
install(TARGETS benchmark_asynchronous benchmark_synchronous benchmark_baseline create_scene
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
