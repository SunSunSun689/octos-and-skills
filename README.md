# so101-pick-cube — 自包含 Skill 包

此 skill 已整合所有外部依赖，**无需安装 dorobot2、skill_pack 或额外 manifest 仓库**即可使用。

## 当前模式：无相机

Cube 的 (x, y, z) 坐标由调用方（LLM / 上游系统）直接提供。不依赖 Orbbec 相机。

## 工具

| 工具 | 输入 | 说明 |
|------|------|------|
| `get_dropoff_position` | *(无)* | 返回放置盘 (x, y) |
| `pick_cube_at` | `{"x": 0.15, "y": 0.05, "z": 0.02}` | 在指定基座坐标抓取方块 |
| `place_cube_at` | `{"x": 0.25, "y": 0.0}` | 放置已抓取的方块 |

## 目录结构

```
so101-pick-cube/
├── main                        # octos 工具入口
├── manifest.json               # 工具清单（暴露给 LLM）
├── SKILL.md                    # 技能描述
├── arm_skills.py               # IK 求解 + 运动规划
├── manifest.py                 # manifest 加载器
├── manifests/
│   ├── so101.json              # 仿真 manifest
│   └── so101-hw.json           # 硬件 manifest（arm_skills + bridge 配置，已合并）
├── mjcf/so101/
│   ├── so101_new_calib.xml              # 机械臂运动学模型（hw 模式直接使用）
│   ├── so101_new_calib_nocompiler.xml   # 同上但无 compiler（供 wrapper 引用）
│   └── assets/                          # STL 网格（13 个文件）
├── so101_pickplace_hw.xml      # MuJoCo wrapper（sim 测试用）
│
├── vision_service.py           # [保留] Orbbec 相机驱动（不启动）
├── find_cube.py                # [保留] 方块检测算法（不调用）
├── calibrate_extrinsics.py     # [保留] 手眼标定工具
├── calibrate_hand_eye.py       # [保留]
├── auto_calibrate.py           # [保留]
├── calibrate.sh                # [保留]
├── start_vision.sh             # [保留]
└── ee_T_camera*.npy            # [保留] 标定结果
```

## 外部依赖

| 依赖 | 说明 |
|------|------|
| MuJoCo (`pip install mujoco`) | IK 求解 |
| numpy | 数值计算 |
| dora bridge (`:8768`) | 硬件通信桥接（由 octos-dora-bridge 提供，已预装） |

注意：`vision_service.py` 等相机相关源码已保留，如需启用需额外安装 pyorbbecsdk 1.10.22。

## 环境变量

所有路径默认指向 skill 自身目录，无需额外配置：

| 变量 | 默认值 |
|------|--------|
| `SKILL_PACK` | `<skill-dir>` |
| `MODEL_NAME` | `<skill-dir>/mjcf/so101/so101_new_calib.xml` |
| `ROBOT_MANIFEST` | `<skill-dir>/manifests/so101-hw.json` |
| `ARM_BRIDGE_URL` | `http://127.0.0.1:8768` |

## 来源

整合自以下仓库的文件：

- `arm_skills.py`, `manifest.py`, `manifests/so101.json` — [moveit-arm-dora-node](https://github.com/dorarobotics/moveit-arm-dora-node)
- `manifests/so101-hw.json` — [rebot-hw-dora-node](https://github.com/dorarobotics/rebot-hw-dora-node)
- `mjcf/so101/` — dorobot2 (Onshape CAD 导出)
