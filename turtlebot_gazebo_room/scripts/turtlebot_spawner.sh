#! /bin/bash
# this cannot be a roslaunch file, since env variables
# do not affect $(optenv ...) substitution args
export ROBOT_INITIAL_POSE="-x 1.0 -y 3.1"
roslaunch `rospack find turtlebot_gazebo`/launch/includes/kobuki.launch.xml\
  base:=kobuki stacks:=hexagons 3d_sensor:=asus_xtion_pro
