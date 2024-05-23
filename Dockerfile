FROM ubuntu:20.04

# install noetic 
RUN apt-get update && apt-get install -y curl  lsb-release gnupg tzdata && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list &&\
    curl -s  https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |  apt-key add -

ENV TZ Asia/Tokyo

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-catkin-tools \
    ros-noetic-rospy-message-converter \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update


# install galactic
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-galactic-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-galactic-rclpy-message-converter \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros1_custom_msg /root/ros1_bridge_ws/src
COPY ./ros1_bridge_ws/src/ros1_bridge /root/ros1_bridge_ws/src/ros1_bridge
WORKDIR /root/ros1_bridge_ws
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin build' 

COPY ./ros2_custom_msg /root/ros2_bridge_ws/src
COPY ./ros2_bridge_ws/src/ros2_bridge /root/ros2_bridge_ws/src/ros2_bridge
WORKDIR /root/ros2_bridge_ws
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash' &&\
    colcon build --cmake-clean-cache

COPY ./shell /root/shell
RUN cat  /root/shell/.add_bashrc >> /root/.bashrc


WORKDIR /root  
