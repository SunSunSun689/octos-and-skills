---
name: robot-api
description: Sends robot control commands (前进/后退/左转/右转/停止/抓取/释放) directly to the AKA-00 car via HTTP GET. Triggers: 辰龙机器人, robot, 前进, 后退, 左转, 右转, 停止, 抓取, 释放
version: 2.0.0
author: octos
always: false
---

# Robot API Skill v2

Translates natural-language robot control commands into HTTP GET requests and sends them directly to the AKA-00 smart car. Requires the car's IP address.

## Tools

### robot_control

Constructs a full URL `http://{ip}/api/control?action=xxx&speed=xxx&time=xxx` and sends a GET request using `reqwest`. Returns the actual HTTP response from the car.

**Actions mapping:**
- 前进 / up → `action=up`
- 后退 / down → `action=down`
- 左转 / left → `action=left`
- 右转 / right → `action=right`
- 停止 / stop → `action=stop`
- 抓取 / grab → `action=grab`
- 释放 / release → `action=release`

**Examples:**
```json
{"ip": "192.168.0.107", "action": "前进", "speed": 50, "time": 10000}
```
→ GET `http://192.168.0.107/api/control?action=up&speed=50&time=10000`
→ `{"message":"up scheduled for 10000.0ms","status":"success"}`

**Parameters:**
- `ip` (required): IP address of the car, e.g. `192.168.0.107`
- `action` (required): The robot action. Accepts Chinese or English.
- `speed` (optional): Speed value 0-50.
- `time` (optional): Duration in milliseconds.
