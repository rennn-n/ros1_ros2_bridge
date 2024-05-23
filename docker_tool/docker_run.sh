# ROS1 
ROS_IP=127.0.0.1
ROS_MASTER_URI=http://127.0.0.1:11311
#ROS2
ROS_DOMAIN_ID=1
ROS_LOCALHOST_ONLY=0



docker rm -f ros1_ros2_bridge &>/dev/null

cd  $(dirname $0)/..

docker run -itd --net=host \
    --name="ros1_ros2_bridge" \
    -v ./config.yaml:/config.yaml \
    -e ROS_IP=$ROS_IP \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -e ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY \
    --shm-size=1024m \
    ros1_ros2_bridge