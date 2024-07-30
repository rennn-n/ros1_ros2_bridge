docker rm -f ros1_ros2_bridge_${2:-0} &>/dev/null

cd  $(dirname $0)/..

docker run -itd --net=host \
    --name="ros1_ros2_bridge${2:-0}" \
    -v ./${1:-config.yaml}:/config.yaml \
    --shm-size=1024m \
    ros1_ros2_bridge