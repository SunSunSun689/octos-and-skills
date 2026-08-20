import socket
import json
import time


def gen_cmd(seq):
    return {
        "in_charge": 1,
        "filt_level": 1,
        "arm_mode": 3,
        "finger_mode": 3,
        "neck_mode": 3,
        "arx_pos_left": [0.0] * 7,
        "arx_pos_right": [0.0] * 7,
        "arm_fm_left": [0.0] * 6,
        "arm_fm_right": [0.0] * 6,
        "hand_q_left": 0.0,
        "hand_q_right": 0.0,
        "neck_q": [0.05, 0.0],
        "waist_q": [0.1, 0.0, 0.0],
    }


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = ("127.0.0.1", 9991)
    seq = 0
    while True:
        cmd = gen_cmd(seq)
        msg = json.dumps(cmd).encode("utf-8")
        sock.sendto(msg, addr)
        print(msg)
        seq += 1
        time.sleep(0.04)  # 25Hz


if __name__ == "__main__":
    main()
