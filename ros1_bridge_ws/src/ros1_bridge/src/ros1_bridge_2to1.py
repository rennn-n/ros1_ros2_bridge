#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import datetime as dt
import time
import os
import yaml

import rospy
import roslib
from rospy_message_converter import json_message_converter


class Json2MsgNode():
    def __init__(self,config) -> None:
        self.sleep_time = config["rate"]

        self.topic_num = len(config["ros2_to_ros1"])
        self.topic_names = []
        self.msg_types = []
        self.bridge_publishers = []
        self.json_paths = []
        self.pre_modified = []
        for i,topic in enumerate(config["ros2_to_ros1"]):
            self.topic_names.append(topic["ros1_name"])
            self.msg_types.append(topic["ros1_type"])
            message_class = roslib.message.get_message_class(topic["ros1_type"])
            self.bridge_publishers.append(rospy.Publisher(topic["ros1_name"],message_class,queue_size=1))

            json_path = f"/dev/shm/bridge_2to1_{i}.json"
            self.json_paths.append(json_path)
            with open(json_path, 'w'):
                pass
            self.pre_modified.append(rtn_modified_time(json_path))
            rospy.Timer(rospy.Duration(self.sleep_time),  lambda event ,i=i: self.callback(event, i))
        rospy.loginfo(f"[ros1_bridge_2to1] rate={self.sleep_time}")
        rospy.loginfo(f"[ros1_bridge_2to1] {' '.join(self.topic_names)}")
        rospy.loginfo("[ros1_bridge_2to1] init")

    def callback(self,event,i) -> None:
        if self.pre_modified[i] != rtn_modified_time(self.json_paths[i]):
            try:
                json_open = open(self.json_paths[i], 'r')
                try:
                    json_load = json.load(json_open)
                    ros_msg = json_message_converter.convert_json_to_ros_message(self.msg_types[i], json_load)
                    self.bridge_publishers[i].publish(ros_msg)
                    rospy.loginfo(f"[ros1_bridge_2to1] <ROS2->ROS1> publish {self.topic_names[i]}")
                    
                except json.JSONDecodeError as e:
                    rospy.logerr(f"[ros1_bridge_2to1] <ROS2->ROS1> JSONDecodeError {self.topic_names[i]}")
                    rospy.logerr(e)
                self.pre_modified[i] = rtn_modified_time(self.json_paths[i])
            except OSError as e:
                rospy.logerr(f"[ros1_bridge_2to1] <ROS2->ROS1> can not open json!!! {self.topic_names[i]}")
                rospy.logerr(e)




def rtn_modified_time(file_path: str) -> dt.datetime:
    file_info = os.stat(file_path)
    modified_time = dt.datetime.fromtimestamp(file_info.st_mtime)
    return modified_time

        

def main(args=None):
    try:
        conf_path = "/config.yaml"
        with open(conf_path) as file:
            config = yaml.safe_load(file.read())
        
        os.environ['ROS_IP'] = config["ROS_IP"]
        os.environ['ROS_MASTER_URI'] = config["ROS_MASTER_URI"]
        
        rospy.init_node("ros1_bridge_2to1",disable_signals=True)
        detector = Json2MsgNode(config)
        rospy.spin()
    except rospy.ROSInterruptException:
        return

if __name__ == '__main__':
    main()