# Octos控制so101机械臂运动

## 硬件准备
- 连接nano电源和机械臂电源
- 将机械臂直连电脑，串口号为**/dev/ttyACM0**

## 启动dora bridge
```bash
cd /home/dora/.octos/skills/skills/so101-pick-cube
python3 minimal_bridge.py    # 显示[bridge] listening on http://127.0.0.1:8768，即bridge启动成功
```

## 另起一个新终端，在根目录下启动octos chat

```bash
octos chat
```
输入如下自然语言指令

```bash
ADora去 p1（0.20,0.10,-0.04）抓方块,然后保持夹爪闭合，再运动到p2（0.20,-0.10,0.10）,再运动到p3(0.20,-0.10,-0.04)去放下放方块

```
## 注意事项
- (0.20,0.10,-0.04）分别代表x,y,z，单位是米（m）
- 注意检查bridge是否建立
- 完成后，需给机械臂断电才可以重新启动下一轮，

