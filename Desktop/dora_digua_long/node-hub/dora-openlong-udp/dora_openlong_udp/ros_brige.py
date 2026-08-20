import socket
import json
from dora import Node

node=Node()

class UdpCmdReceiver:
    def __init__(self, port=8891):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.settimeout(1.0)
        ros2_context = dora.Ros2Context()
        ros2_node = ros2_context.new_node(
            "dora_custom_publisher", "/dora", dora.Ros2NodeOptions(rosout=True)
        )
        topic_qos = dora.Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
        custom_topic = ros2_node.create_topic("topic", "std_msgs::String", topic_qos)
        self.cmd_publisher = ros2_node.create_publisher(custom_topic)


    def _parse_packet(self, data):
        try:
            cmd_dict = json.loads(data.decode('utf-8'))
            return cmd_dict
        except Exception as e:
            print(f"解析错误: {type(e).__name__} - {str(e)}")
            return None

    def start(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                parsed = self._parse_packet(data)
                if parsed:
                    data = parsed
                    message = {"data": data}
                    self.cmd_publisher.publish(message)
                    print(parsed)
            except socket.timeout:
                continue
            time.sleep(0.05)

def main():
    receiver = UdpCmdReceiver()
    receiver.start()  # ✅ 传入 node 实例
