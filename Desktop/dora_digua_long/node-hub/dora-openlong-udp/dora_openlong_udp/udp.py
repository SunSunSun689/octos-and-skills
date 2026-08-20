import socket
import json
from dora import Node

node=Node()

class UdpCmdReceiver:
    def __init__(self, port=8891):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.settimeout(1.0)

    def _parse_packet(self, data):
        try:
            cmd_dict = json.loads(data.decode('utf-8'))
            return {
                'in_charge': cmd_dict['in_charge'],
                'filt_level': cmd_dict['filt_level'],
                'arm_mode': cmd_dict['arm_mode'],
                'finger_mode': cmd_dict['finger_mode'],
                'neck_mode': cmd_dict['neck_mode'],
                'arx_pos_left': [round(float(x), 4) for x in cmd_dict['arx_pos_left']],
                'arx_pos_right': [round(float(x), 4) for x in cmd_dict['arx_pos_right']],
                'arm_fm_left': [round(float(x), 4) for x in cmd_dict['arm_fm_left']],
                'arm_fm_right': [round(float(x), 4) for x in cmd_dict['arm_fm_right']],
                'hand_q_left': round(float(cmd_dict['hand_q_left']), 4),
                'hand_q_right': round(float(cmd_dict['hand_q_right']), 4),
                'neck_q':cmd_dict['neck_q'],
                'waist_q': cmd_dict['waist_q'],
                # 'timestamp': cmd_dict['timestamp'],
                # 'sequence': cmd_dict['sequence']
            }
        except Exception as e:
            print(f"解析错误: {type(e).__name__} - {str(e)}")
            return None

    def start(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                parsed = self._parse_packet(data)
                if parsed:
                    node.send_output("arm_state", json.dumps(parsed).encode('utf-8'))  # 转为bytes
                    # print(parsed)
                    # print(f"收到指令 seq:{parsed['sequence']} 时间戳:{parsed['timestamp']}")
            except socket.timeout:
                continue

def main():
    receiver = UdpCmdReceiver()
    receiver.start()  # ✅ 传入 node 实例
    node.run()