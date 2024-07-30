
function trap_fnc(){
    docker exec ros1_ros2_bridge${1:-0} /bin/bash -c "bash shell/ros_bridge_1to2_kill.sh"
}
trap "trap_fnc" 2
  docker exec ros1_ros2_bridge${1:-0} /bin/bash -c "bash shell/ros_bridge_1to2.sh" &
  wait