#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os
import yaml

import rospy
import roslib
from rospy_message_converter import json_message_converter


class Msg2JsonNode():
    def __init__(self) -> None:
        conf_path = "/config.yaml"
        with open(conf_path) as file:
            config = yaml.safe_load(file.read())
        
        self.topic_names = []

        self.bridge_subscribers = []
        self.json_paths = []
        for i,topic in enumerate(config["ros1_to_ros2"]):
            topic_name = topic["ros1_name"]
            self.topic_names.append(topic_name)
            message_class = roslib.message.get_message_class(topic["ros1_type"])
            self.bridge_subscribers.append(rospy.Subscriber(topic_name, message_class, self.callback, callback_args=i))
            
            json_path = f"/dev/shm/bridge_1to2_{i}.json"
            self.json_paths.append(json_path)
            with open(json_path, 'w'):
                pass
        rospy.loginfo(f"[ros1_bridge_1to2] {' '.join(self.topic_names)}")
        rospy.loginfo("[ros1_bridge_1to2] init")
    def callback(self,msg,i) -> None:
        json_str = json_message_converter.convert_ros_message_to_json(msg)
        rospy.loginfo(f"[ros1_bridge_1to2] <ROS1->ROS2> subscrib {self.topic_names[i]}")
        with open(self.json_paths[i], mode="w", encoding="utf-8") as f:
            json.dump(json_str, f)


           
        

def main(args=None):
    try:
        rospy.init_node("ros1_bridge_1to2",disable_signals=True)
        detector = Msg2JsonNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        return

if __name__ == '__main__':
    main()