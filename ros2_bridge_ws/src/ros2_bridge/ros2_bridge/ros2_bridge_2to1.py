import json
import yaml

import rclpy
from rclpy.node import Node
from rclpy_message_converter import json_message_converter

from rosidl_runtime_py.utilities import get_message

class Msg2JsonNode(Node):
    def __init__(self) -> None:
        super().__init__("ros2_bridge_2to1")

        conf_path = "/config.yaml"
        with open(conf_path) as file:
            config = yaml.safe_load(file.read())
        
        self.topic_names = []
        self.json_paths = []

        for i,topic in enumerate(config["ros2_to_ros1"]):
            self.topic_names.append(topic["name"])
            message_class = get_message(topic["ros2_type"])
            self.create_subscription(message_class, topic["name"], lambda msg ,i=i: self.callback(msg, i),rclpy.qos.qos_profile_sensor_data)
            json_path = f"/dev/shm/bridge_2to1_{i}.json"
            self.json_paths.append(json_path)
            with open(json_path, 'w'):
                pass
        self.get_logger().info("init")

    def callback(self,msg,i) -> None:
        json_str = json_message_converter.convert_ros_message_to_json(msg)
        self.get_logger().info(f"<ROS2->ROS1>subscrib {self.topic_names[i]}")
        with open(self.json_paths[i], mode="w", encoding="utf-8") as f:
            json.dump(json_str, f)


def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        rclpy.init(args=args)
        node = Msg2JsonNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
