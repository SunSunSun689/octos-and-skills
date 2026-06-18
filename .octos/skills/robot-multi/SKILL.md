---
name: robot-multi
description: Controls multiple AKA-00 robots simultaneously. Triggers: 多个机器人, 所有机器人, 同时控制, 广播, robot1, robot2, 编队, 协同, 多台小车
version: 0.1.0
author: octos
always: false
hardware_requirements: AKA-00 cars reachable via HTTPS on local network
---

# Robot Multi Skill

Controls multiple AKA-00 robots simultaneously via a single natural-language command.
Reads robot name→IP mappings from `robots.json` in the skill directory.

## Tools

### robot_multi_control

Sends a motion command to one or more robots in parallel. Returns a per-robot success/failure summary.

**Broadcast to all robots:**
```json
{"robots": ["all"], "action": "前进", "speed": 50, "time": 2000}
```

**Target specific robots:**
```json
{"robots": ["robot1", "robot2"], "action": "左转", "time": 1000}
```

**Actions mapping:**
- 前进 / up → `action=up`
- 后退 / down → `action=down`
- 左转 / left → `action=left`
- 右转 / right → `action=right`
- 停止 / stop → `action=stop`
- 抓取 / grab → `action=grab`
- 释放 / release → `action=release`

**Parameters:**
- `robots` (required): `["all"]` or list of robot names defined in `robots.json`
- `action` (required): Chinese or English action name
- `speed` (optional): 0-50
- `time` (optional): milliseconds

## Error Codes

| Situation | Behavior |
|---|---|
| Robot name not in robots.json | That robot marked failed, others continue |
| HTTP request failed | That robot marked failed, error message recorded |
| robots.json missing | Entire skill fails with error message |
| Invalid action | Entire skill fails with valid action list |

## Configuring Robots

Edit `robots.json` in the skill directory (changes take effect immediately on next call):

```json
{
  "robots": [
    {"name": "robot1", "ip": "192.168.110.225"},
    {"name": "robot2", "ip": "192.168.110.30"}
  ]
}
```
