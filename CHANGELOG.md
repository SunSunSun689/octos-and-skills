# robot-multi Changelog

## v0.1.8 — 2026-06-18

**实测：throttle 值与实际速度对应关系（robot2，192.168.110.30）**

| throttle | 实际速度 |
|----------|----------|
| 20       | 0.025 m/s |
| 40       | 0.225 m/s |
| 60       | 0.415 m/s |
| 80       | 0.410 m/s |
| 100      | 0.410 m/s |
| 120      | 0.410 m/s |
| 127      | 0.410 m/s |

**结论：**
- throttle 60 是速度饱和点，约 0.41 m/s
- 60~127 速度几乎相同，继续增大无效
- 20~60 之间近似线性增长
- 默认值 80 已在饱和区，等效于最大速度

---

## v0.1.7 — 2026-06-18

**修复：timeout_secs 从 30 提升到 90，ws_hold 时长从 60s 改为 80s**

- `pickup_tennis` 调用 `ws_hold` 保持 WebSocket 80 秒，让 tennis demo 有足够时间完成
- `manifest.json` 中 `timeout_secs` 改为 90（原来 30s 导致进程被 octos 强制 Killed）
- 留 10 秒余量：ws_hold 80s + demo 启动延时 < timeout 90s

---

## v0.1.6 — 2026-06-18

**修复：捡球功能正确实现**

**根因排查**
- tennis demo 启动后 motor target 始终为 0，小车不动
- 通过 tcpdump 抓包发现：demo 程序通过 WebSocket 广播 `[0xBB, left_i16_le, right_i16_le]` 电机指令
- **必须有 WebSocket 客户端连接存在，demo 的指令才会被服务器执行**
- 之前 skill 只调用 HTTP 启动 demo 就返回，没有保持 WebSocket 连接，所以小车不动

**额外发现**
- robot1（192.168.110.225）的 tennis demo 本身不工作：连接 WebSocket 后 demo 15 秒内无任何电机指令输出，可能是固件/相机配置问题
- robot2（192.168.110.30）正常

**修复内容**
- `pickup_tennis` 动作：先停止已有 demo，再启动 tennis demo，然后调用新增的 `ws_hold()` 保持 WebSocket 连接 60 秒
- 新增 `ws_hold(ip, hold_ms)`：纯连接保持函数，只读取服务器消息不发送，让 demo 能持续驱动电机

---

## v0.1.5 — 2026-06-18

**重大修复：电机驱动改用 WebSocket，修正消息格式**

**根因排查过程**
- 之前三种方式（HTTP motor/direct、HTTP motor/status、tungstenite 发送后立刻 close）都不能让小车动
- 通过 tcpdump 对比 Python（能动）和 Rust（不能动）的流量，发现差异：
  - Python 发完消息后持续 2 秒读取服务器心跳，最后才关闭
  - Rust 发完消息后立刻发 WebSocket close 帧（28字节），服务器收到 close 立刻断开，指令被丢弃
- 服务器每 ~400ms 发一个 29 字节心跳包，必须保持连接读取心跳，指令才会被处理

**消息格式发现**
- 之前错误实现：`[0xAA, left_motor, right_motor]`
- 正确格式：`[0xAA, steering_i8, throttle_i8]`
  - steering: 负数左转，正数右转，0 直行
  - throttle: 正数前进，负数后退，0 停止
- 从 JS 源码分析：`sendJoystick(x, y)` 中 x 是水平轴（转向），y 是纵轴（油门）

**修复内容**
- 新增 `ws_drive(ip, steering, throttle, hold_ms)` 函数：建立 WSS 连接、发送指令、持续读取心跳直到 hold_ms 结束、发送停止指令、再短暂 drain 后断开
- `action_to_joystick()` 替换原 `action_to_motors()`，映射为正确的 steering/throttle 格式
- 添加 `tungstenite` + `native-tls` 依赖

---

## v0.1.4 — 2026-06-18

**新功能：捡网球**

- 新增 `pickup_tennis` / `捡球` / `捡网球` 动作：调用小车内置 tennis demo（`POST /api/demo/init {"name":"tennis"}`），小车自主寻球并捡起
- 新增 `stop_demo` / `停止捡球` 动作：停止正在运行的 demo（`POST /api/demo/stop`）
- 两个动作同样支持广播（`robots: ["all"]`）或指定具体机器人
- 新增 `json` feature 到 reqwest 依赖，支持 POST JSON body

---

## v0.1.3 — 2026-06-18

**问题排查与修复**

**根因发现：`/api/control` 接口不驱动电机**
- `/api/control?action=up&speed=N&time=N` 会返回 `{"status":"success"}` 但电机实际不转
- 通过 `/api/motor/status` 确认：`left_target` / `right_target` 有值，但 `left_speed` / `right_speed` 始终为 0
- 真正驱动电机的接口是 `/api/motor/direct?left=N&right=N`，调用后 speed 字段有实际读数

**修复：改用 `/api/motor/direct` 接口**
- 将 `build_url` 改为生成 `/api/motor/direct?left=N&right=N` 请求
- 新增 `action_to_motors(action, speed)` 函数，将动作映射为左右轮速度：
  - 前进：`(speed, speed)`，后退：`(-speed, -speed)`
  - 左转：`(-speed, speed)`，右转：`(speed, -speed)`，停止：`(0, 0)`
- 新增定时停止逻辑：`time` 参数指定后，等待对应毫秒后自动发送停止指令
- `speed` 默认值改为 30（原来无默认值）

**其他修复**
- reqwest 客户端加 `.no_proxy()`：系统 `https_proxy=127.0.0.1:7890` 会拦截局域网请求导致失败
- reqwest 客户端加 `.danger_accept_invalid_certs(true)`：小车使用自签名证书

**skill 安装**
- 通过 `octos skills install /home/dora/.octos/skills/robot-multi` 完成安装
- octos 将 skill 注册到 `.octos/skills/skills/robot-multi/` 目录

---

## v0.1.2 — 2026-06-17

**配置变更**
- 更新 `robots.json`：robot2 IP 由 `192.168.110.226` 改为 `192.168.110.30`

**当前小车 IP 配置**
| 名称   | IP              |
|--------|-----------------|
| robot1 | 192.168.110.225 |
| robot2 | 192.168.110.30  |

---

## v0.1.1 — 2026-06-17

- 添加 `.gitignore`

---

## v0.1.0 — 2026-06-17

**首次发布**

功能：
- 通过 `robot_multi_control` tool 同时控制多台 AKA-00 小车
- 支持广播（`robots: ["all"]`）或指定具体机器人名称
- 并行发送 HTTP 请求，汇总每台小车的执行结果
- 支持中英文动作名：前进/up、后退/down、左转/left、右转/right、停止/stop、抓取/grab、释放/release
- 可选参数：`speed`（0-50）、`time`（毫秒）
- 机器人 IP 配置从 `robots.json` 读取，修改即生效无需重新编译

实现：
- Rust 编写，使用 `reqwest` 做并行 HTTP 请求（多线程）
- 读取 `robots.json` 解析机器人名称→IP 映射
- 未知机器人名称立即失败，网络错误逐台记录不中断其他请求
