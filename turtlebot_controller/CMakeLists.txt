cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_controller)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  message_runtime
  rospy
  turtlebot_gazebo_room
)

## http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

add_action_files(
  FILES
    TurtleBotMovement.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_controller
#  CATKIN_DEPENDS actionlib geometry_msgs message_generation message_runtime rospy
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  nodes/controller_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
