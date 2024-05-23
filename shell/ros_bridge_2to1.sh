#!/bin/bash

function ros1_bridge_2to1(){
    . /opt/ros/noetic/setup.bash
    . ~/ros1_bridge_ws/devel/setup.bash
    rosrun ros1_bridge ros1_bridge_2to1.py 
}
function ros2_bridge_2to1(){
    . /opt/ros/galactic/setup.bash
    . ~/ros2_bridge_ws/install/setup.bash
    ros2 run ros2_bridge ros2_bridge_2to1 
}
trap "kill 0" EXIT
  ros1_bridge_2to1 &
  ros2_bridge_2to1 &
  wait