cmake_minimum_required(VERSION 2.8.3)
project(interactive_local_planner_simulated_obstacle)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_msgs
  rospy
  tf2
  tf2_ros
  geometry_msgs
)

catkin_python_setup()

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS nodes/obstacle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

