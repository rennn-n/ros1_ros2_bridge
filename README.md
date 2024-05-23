# ROS1_ROS2_bridge
## Usage
build (Run only once at the beginning)
```bash
bash docker_tool/docker_build.sh
```
run docker image (Run only once)
```bash
bash docker_tool/docker_run.sh
```
ros1 to ros2 
```bash
bash docker_tool/run_1to2.sh
```
ros2 to ros1 
```bash
bash docker_tool/run_2to1.sh
```
## Configuration

If you want to change the topic you want to bridge, change the config.yaml
```yaml
rate: 0.1 #refresh rate cycle
ros1_to_ros2:
  - name: /chatter # topic name
    ros1_type: std_msgs/String # ros1 msg type 
    ros2_type: std_msgs/msg/String # ros2 msg type 

  - name: /points_raw
    ros1_type: sensor_msgs/PointCloud2.msg
    ros2_type:sensor_msgs/msg/PointCloud2.msg
ros2_to_ros1:
  - name: /chatter
    ros1_type: std_msgs/String
    ros2_type: std_msgs/msg/String

  - name: /points_raw
    ros1_type: sensor_msgs/PointCloud2.msg
    ros2_type: sensor_msgs/msg/PointCloud2.msg
```


If you want to change ROS_MASTER_URI for ROS1 and ROS_DOMAIN_ID for ROS2, edit docker_tool/docker_run.sh
```sh
# ROS1 
ROS_IP=127.0.0.1
ROS_MASTER_URI=http://127.0.0.1:11311 #<-
#ROS2
ROS_DOMAIN_ID=1 #<-
ROS_LOCALHOST_ONLY=0
```

