import socket
import json
import time
from dora import Node, Ros2Context, Ros2NodeOptions, Ros2QosPolicies
import pyarrow as pa
from typing import List

node = Node()


class UdpCmdReceiver:
    def __init__(self, port=9991):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(1.0)
        self.ros2_context = Ros2Context()
        self.ros2_node = self.ros2_context.new_node(
            "dora_custom_publisher_test", "/dora", Ros2NodeOptions(rosout=True)
        )
        self.topic_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
        self.custom_topic = self.ros2_node.create_topic(
            "topic", "std_msgs::String", self.topic_qos
        )
        self.cmd_publisher = self.ros2_node.create_publisher(self.custom_topic)

    def _parse_packet(self, data):
        try:
            # cmd_dict = json.loads(data.decode("utf-8"))
            dora_msg = data.decode("utf-8")
            return dora_msg
        except Exception as e:
            print(f"解析错误: {type(e).__name__} - {str(e)}")
            return None

    def start(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                msg = self._parse_packet(data)
                message = {"data": msg}
                self.cmd_publisher.publish(message)
            except socket.timeout:
                continue
            time.sleep(0.05)


if __name__ == "__main__":
    receiver = UdpCmdReceiver()
    receiver.start()
