
function trap_fnc(){
    docker exec ros1_ros2_bridge /bin/bash -c "bash shell/ros_bridge_2to1_kill.sh"
}
trap "trap_fnc" 2
  docker exec ros1_ros2_bridge /bin/bash -c "bash shell/ros_bridge_2to1.sh" &
  wait