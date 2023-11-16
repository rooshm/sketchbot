cmake_minimum_required(VERSION 3.5)
project(sketch_controller)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_common
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  rviz_visual_tools
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_executable(sketch_controller_node src/move_group_interface_tutorial.cpp)
ament_target_dependencies(sketch_controller_node
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS sketch_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()