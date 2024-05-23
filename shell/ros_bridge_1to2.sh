#!/bin/bash

function ros1_bridge_1to2(){
    . /opt/ros/noetic/setup.bash
    . ~/ros1_bridge_ws/devel/setup.bash
    rosrun ros1_bridge ros1_bridge_1to2.py 
}
function ros2_bridge_1to2(){
    . /opt/ros/galactic/setup.bash
    . ~/ros2_bridge_ws/install/setup.bash
    ros2 run ros2_bridge ros2_bridge_1to2 
}
trap "kill 0" EXIT
  ros1_bridge_1to2 &
  ros2_bridge_1to2 &
  wait