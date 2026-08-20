# filepath: /root/dora_digua_long/node-hub/dora-openlong/dora_openlong/control.py
import dora
from dora import Node
import time
import sys
import os
import json
import threading
import numpy as np

os.chdir(os.path.dirname(__file__))
sys.path.append("..")
from sdk.loong_mani_sdk.loong_mani_sdk_udp import (
    maniSdkCtrlDataClass,
    maniSdkClass,
    maniSdkSensDataClass,
)


# 机器人关节配置
jntNum = 19
armDof = 7
fingerDofLeft = 1
fingerDofRight = 1
neckDof = 2
lumbarDof = 3

class RobotController:
    def __init__(self):
        # 初始化控制数据类
        self.ctrl = maniSdkCtrlDataClass(armDof, fingerDofLeft, fingerDofRight, neckDof, lumbarDof)
        # 初始化SDK通信
        self.sdk = maniSdkClass("192.168.1.201", 8003, jntNum, fingerDofLeft, fingerDofRight)
        
        # 线程安全锁
        self.lock = threading.Lock()
        
        # 初始化默认控制参数
        self._init_default_params()
        
        # 启动发送线程
        self.send_thread = threading.Thread(target=self._send_loop)
        self.running = True
        self.send_thread.start()

    def _init_default_params(self):
        """初始化默认控制参数"""
        with self.lock:
            self.ctrl.inCharge = 1
            self.ctrl.filtLevel = 1
            self.ctrl.armMode = 4
            self.ctrl.fingerMode = 3
            self.ctrl.neckMode = 5
            self.ctrl.lumbarMode = 0
            
            # 初始化手臂位置（笛卡尔坐标系）
            self.ctrl.armCmd = np.array(
                [[0.4, 0.4, 0.1, 0, 0, 0, 0.5], 
                 [0.2, -0.4, 0.1, 0, 0, 0, 0.5]], 
                np.float32
            )
            
            # 初始化其他控制指令为零
            self.ctrl.armFM = np.zeros((2, 6), np.float32)
            self.ctrl.fingerLeft = np.zeros(fingerDofLeft, np.float32)
            self.ctrl.fingerRight = np.zeros(fingerDofRight, np.float32)
            self.ctrl.neckCmd = np.zeros(2, np.float32)
            self.ctrl.lumbarCmd = np.zeros(3, np.float32)

    def _send_loop(self):
        """持续发送控制指令的线程"""
        dT = 0.02  # 20ms 发送间隔
        while self.running:
            start_time = time.time()
            
            # 发送控制指令
            with self.lock:
                self.sdk.send(self.ctrl)
                
            # 接收传感器反馈（可选）
            sens = self.sdk.recv()
            if sens:
                sens.print()
                
            # 控制发送频率
            elapsed = time.time() - start_time
            if elapsed < dT:
                time.sleep(dT - elapsed)

    def update_control_params(self, data):
        """更新控制参数"""
        with self.lock:
            try:
                self.lumbarMode = 0  # 重置腰部模式
                # 更新充电状态
                if 'in_charge' in data:
                    self.ctrl.inCharge = int(data['in_charge'])
                
                # 更新滤波等级
                if 'filt_level' in data:
                    self.ctrl.filtLevel = int(data['filt_level'])
                
                # 更新臂部模式
                if 'arm_mode' in data:
                    self.ctrl.armMode = int(data['arm_mode'])
                
                # 更新手指模式
                if 'finger_mode' in data:
                    self.ctrl.fingerMode = int(data['finger_mode'])
                
                # 更新颈部模式
                if 'neck_mode' in data:
                    self.ctrl.neckMode = int(data['neck_mode'])
                
                # 更新臂部位置
                if 'arx_pos_left' in data:
                    self.ctrl.armCmd[0] = np.array(data['arx_pos_left'], dtype=np.float32)
                
                if 'arx_pos_right' in data:
                    self.ctrl.armCmd[1] = np.array(data['arx_pos_right'], dtype=np.float32)
                
                # 更新手指姿态
                if 'hand_q_left' in data:
                    self.ctrl.fingerLeft = np.array(data['hand_q_left'], dtype=np.float32)
                
                if 'hand_q_right' in data:
                    self.ctrl.fingerRight = np.array(data['hand_q_right'], dtype=np.float32)
                
                # 更新颈部角度
                if 'neck_q' in data:
                    self.ctrl.neckCmd = np.array(data['neck_q'], dtype=np.float32)
                
                # 更新腰部角度
                if 'waist_q' in data:
                    self.ctrl.lumbarCmd = np.array(data['waist_q'], dtype=np.float32)
                
                print("✅ 控制参数已更新")
                
            except Exception as e:
                print(f"❌ 更新控制参数失败: {e}")

    def stop(self):
        """停止发送线程"""
        self.running = False
        self.send_thread.join()


class CmdDataReceiver:
    def __init__(self):
        self.node = Node()
        self.robot_ctrl = RobotController()
        
    def on_event(self):
        for event in self.node:
            if event["type"] == "INPUT" and event["id"] == "arm_state":
                try:
                    # 将 pyarrow 数组转换为字节对象
                    raw_bytes = bytes(event["value"].to_pylist())
                    data = json.loads(raw_bytes.decode("utf-8"))
                    print("📥 接收到新的控制指令：")
                    
                    # 打印调试信息
                    print(f"  序列号: {data.get('sequence')}")
                    print(f"  时间戳: {data.get('timestamp')}")
                    print(f"  是否在充电: {data.get('in_charge')}")
                    print(f"  滤波等级: {data.get('filt_level')}")
                    print(f"  臂部模式: {data.get('arm_mode')}")
                    print(f"  手指模式: {data.get('finger_mode')}")
                    print(f"  脖子模式: {data.get('neck_mode')}")
                    print(f"  左臂位置: {data.get('arx_pos_left')}")
                    print(f"  右臂位置: {data.get('arx_pos_right')}")
                    print(f"  左手姿态: {data.get('hand_q_left')}")
                    print(f"  右手姿态: {data.get('hand_q_right')}")
                    print(f"  脖子角度: {data.get('neck_q')}")
                    print(f"  腰部角度: {data.get('waist_q')}")
                    print("-" * 40)
                    
                    # 更新机器人控制参数
                    self.robot_ctrl.update_control_params(data)
                    
                except Exception as e:
                    print(f"❌ 处理输入数据时出错: {e}")

def main():
    print("🚀 Dora 控制节点已启动，等待接收控制指令...")
    try:
        receiver = CmdDataReceiver()
        receiver.on_event()
    except KeyboardInterrupt:
        print("\n🛑 正在停止控制节点...")
        receiver.robot_ctrl.stop()
        print("✅ 程序已安全退出")

if __name__ == "__main__":
    main()