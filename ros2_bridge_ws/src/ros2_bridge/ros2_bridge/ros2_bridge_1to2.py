import json
import datetime as dt
import time
import os
import yaml


import rclpy
from rclpy.node import Node
from rclpy_message_converter import json_message_converter

from rosidl_runtime_py.utilities import get_message


class Json2MsgNode(Node):
    def __init__(self,config) -> None:
        super().__init__("ros2_bridge_1to2")

        
        self.sleep_time = config["rate"]

        self.topic_names = []
        self.bridge_publishers = []
        self.json_paths = []
        self.msg_types = []
        self.pre_modified = []

        for i,topic in enumerate(config["ros1_to_ros2"]): 
            self.topic_names.append(topic["ros2_name"])
            message_class = get_message(topic["ros2_type"])
            self.msg_types.append(topic["ros2_type"])
            self.bridge_publishers.append(self.create_publisher(message_class, topic["ros2_name"],1))
            json_path = f"/dev/shm/bridge_1to2_{i}.json"
            self.json_paths.append(json_path)
            with open(json_path, 'w'):
                pass
            self.pre_modified.append(rtn_modified_time(json_path))
            self.create_timer(self.sleep_time, lambda i=i: self.callback(i))
        self.get_logger().info("init")
        self.get_logger().info(f"rate={self.sleep_time}")


    def callback(self,i) -> None:
        if self.pre_modified[i] != rtn_modified_time(self.json_paths[i]):
            try:
                json_open = open(self.json_paths[i], 'r')
                try:
                    json_load = json.load(json_open)
                    ros_msg = json_message_converter.convert_json_to_ros_message(self.msg_types[i], json_load,strict_mode=False)
                    self.bridge_publishers[i].publish(ros_msg)
                    self.get_logger().info(f"<ROS1->ROS2> publish {self.topic_names[i]}")
                except json.JSONDecodeError as e:
                    self.get_logger().error("JSONDecodeError")
                self.pre_modified[i] = rtn_modified_time(self.json_paths[i])

            except OSError as e:
                self.get_logger().error("can not open json!!!")


def rtn_modified_time(file_path: str) -> dt.datetime:
    file_info = os.stat(file_path)
    modified_time = dt.datetime.fromtimestamp(file_info.st_mtime)
    return modified_time


def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        conf_path = "/config.yaml"
        with open(conf_path) as file:
            config = yaml.safe_load(file.read())
                
        os.environ['ROS_DOMAIN_ID'] = str(config["ROS_DOMAIN_ID"])
        os.environ['ROS_LOCALHOST_ONLY'] = str(config["ROS_LOCALHOST_ONLY"])

        rclpy.init(args=args)
        node = Json2MsgNode(config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
