# ROS1_ROS2_bridge
## Usage
build (Run only once at the beginning)
```bash
bash docker_tool/docker_build.sh
```
run docker image (Run only once)
```bash
bash docker_tool/docker_run.sh (config:ocnfig.yaml) (name:0)
```
ros1 to ros2 
```bash
bash docker_tool/run_1to2.sh (name:0)
```
ros2 to ros1 
```bash
bash docker_tool/run_2to1.sh (name:0)
```
## Configuration

If you want to change the topic you want to bridge, change the config.yaml
```yaml
rate: 0.1 #refresh rate cycle
ros1_to_ros2:
  - ros1_name: /chatter # ros1 topic name
    ros1_type: std_msgs/String # ros1 msg type 
    ros2_name: /chatter_ros2 # remap topic name
    ros2_type: std_msgs/msg/String # ros2 msg type 
~~~~~~
```


If you want to change ROS_MASTER_URI for ROS1 and ROS_DOMAIN_ID for ROS2, edit docker_tool/docker_run.sh
```sh
# ROS1 
ROS_IP=127.0.0.1
ROS_MASTER_URI=http://127.0.0.1:11311 #<-
#ROS2
ROS_DOMAIN_ID=1 #<-
ROS_LOCALHOST_ONLY=0
~~~~~
```

## custom msg
When you use custom messages, put pkg in ros1_custom_msg,ros2_custom_msg and do ```bash docker_tool/docker_build.sh```