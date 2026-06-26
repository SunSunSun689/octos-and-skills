# so101-pick-cube 开发进度记录

更新日期: 2026-06-25

## 总览

| 阶段 | 状态 | 日期 |
|------|------|------|
| 1. SDK 升级 (Orbbec 1.10.22) | ✅ 完成 | 2026-06-23 |
| 2. 视觉服务 (vision_service.py) | ✅ 完成 | 2026-06-23 |
| 3. 手眼标定 (CAMERA_TO_BASE) | ⚠️ 1-point seed | 2026-06-24 |
| 4. IK 求解器修复 | ✅ 完成 | 2026-06-24 |
| 5. Skill 自包含整合 | ✅ 完成 | 2026-06-25 |
| 6. 无相机模式切换 | ✅ 完成 | 2026-06-25 |
| 7. Adora 机械臂支持 | ✅ 完成 | 2026-06-25 |
| 8. dora bridge 仿真 | ✅ 完成 | 2026-06-25 |
| 9. octos 注册 | ✅ 完成 | 2026-06-25 |
| 10. move_to 工具 (v0.4.0) | ✅ 完成 | 2026-06-25 |
| 11. 端到端验收 | ⏳ 待用户 | - |

---

## 阶段 1: SDK 升级 (✅ 完成)

pyorbbecsdk 1.10.16 在 Orbbec Gemini 335 上返回全 0 深度 buffer。已升级到 1.10.22:
- 切到 main 分支 (v1.x)
- 替换 sdk/lib/linux_x64/ 为 OrbbecViewer 自带的 1.10.22
- 重新编译，产物在 `/home/dora/SDK/pyorbbecsdk/install/lib/pyorbbecsdk.cpython-310-x86_64-linux-gnu.so`

详见内存: [[orbbec-gemini-335-sdk]]

## 阶段 2: 视觉服务 (✅ 完成)

`vision_service.py` 稳定工作:
- Pipeline: 640x400 @ 30fps Y16 深度流, Orbbec SN=CP1E542000CJ
- 几何过滤检测方块: 模态深度 → 紧 band → connected components → 最大 cluster
- 端点: `GET /ball`, `GET /bbox`, `GET /healthz`
- Cube detection fix: `np.histogram` → `np.median()` 使检测 11× 更稳定

详见内存: [[so101-pick-vision-ik-progress]]

### 已知问题
- Orbbec Gemini 335 不支持进程重启，每次需 USB 重新插拔
- Gemini 335 不支持深度+彩色同时流，彩色检测代码已写但未启用
- 相机内参 fx/fy 是粗估值 (~457 px/m)，3-point 标定可吸收残差

## 阶段 3: 手眼标定 (⚠️ 1-point seed)

`CAMERA_TO_BASE` = 1-point seed:
- R = [[0,0,1],[1,0,0],[0,-1,0]] (假设光轴指向 base +x)
- t = [-0.26, 0, 0]
- 用户测量: 方块在 base 中心正前方 29cm (0.29, 0, 0), base 原点高于桌面 0.115m

工具: `calibrate_extrinsics.py` (Kabsch SVD), `auto_calibrate.py`, `calibrate_hand_eye.py`

**待完成**: 收集 ≥3 个非共线点跑 Kabsch，覆盖 1-point seed

详见内存: [[so101-pick-cube-calibration]]

## 阶段 4: IK 求解器修复 (✅ 完成)

`arm_skills.py` 修改:
- Position-only IK: 去掉不稳定的 approach-axis 旋转约束，纯位置 IK 收敛到 0cm 误差
- Gripper qpos fix: `_d.qpos[:] = 0` 防止 gripper 污染
- Damping: 1e-4 → 1e-2
- Joint limit widening: wrist/shoulder/elbow 扩到 ±170°
- `has_limits` guard: 处理空 jnt_range
- Manifest 修复: arm_home 改为实际弧度值, jnt_range_start=0
- `pick_at` 简化为 5 步: home → open → grasp → close → home
- `_ball()` 返回 None 时 b['z'] TypeError 修复

详见内存: [[so101-pick-vision-ik-progress]]

## 阶段 5: Skill 自包含整合 (✅ 完成)

将所有外部依赖复制到 skill 目录内:
- `arm_skills.py`, `manifest.py` ← skill_pack
- `manifests/so101.json`, `manifests/so101-hw.json`
- `mjcf/so101/` (MuJoCo XML + 13 个 STL assets) ← dorobot2

路径修复:
- `main`: SKILL_PACK/MODEL_NAME/ROBOT_MANIFEST 默认指向本地
- `SKILL.md`: init 钩子用 `SKILL_DIR=$(pwd)`
- `so101_pickplace_hw.xml`: include 改为相对路径 + 自提供 compiler meshdir

MuJoCo XML 验证: 直接加载 6 joints ✅, Wrapper 加载 12 joints ✅

详见内存: [[so101-pick-cube-skill-consolidation]]

## 阶段 6: 无相机模式切换 (✅ 完成)

- `get_cube_position` 工具删除
- `pick_cube_at` 改为需要 `x`, `y`, `z` 三个必填参数
- vision_service.py 等相机文件保留但不启动
- `manifest.json` 版本升至 0.2.0
- Cube 坐标由调用方 (LLM / 上游系统) 直接提供

详见内存: [[so101-pick-cube-skill-consolidation]]

## 阶段 7: Adora 机械臂支持 (✅ 完成)

Adora = SO-101 的 1.15× 放大版:
- 新增 `mjcf/adora/adora.xml` (所有 body/geom pos ×1.15)
- 新增 `manifests/adora.json` (高度/抓取参数 ×1.15)
- 新增 `manifests/adora-hw.json` (硬件参数 ×1.15)
- `main --robot so101|adora` 切换机械臂，状态文件按机器人隔离
- `arm_skills.py`: `pick_at` 硬编码改 manifest `GRASP_BIAS`/`LIFT_OK_Z`
- `manifest.json` 版本升至 0.3.0

Review 修复:
1. `--robot` 放在工具名后报错 (不静默忽略)
2. 状态文件: `octos_grasp_so101-pick-cube_{robot}.json` (按机器人隔离)
3. 未知 `--robot` 名称报错

详见内存: [[so101-pick-cube-octos-deploy-progress]]

## 阶段 8: dora bridge 仿真 (✅ 完成)

版本:
- dora CLI: 0.4.1 (系统)
- venv dora Python: 0.4.1
- moveit-arm-dora-node: pip 约束 <0.3 但运行时兼容 0.4.1

关键配置:
- venv-python launcher: `/home/dora/.octos/skills/skills/octos-dora-bridge/venv-python`
- dataflow: `so101-mujoco-bridge-resolved.yaml` (路径已解析，8 节点全部正常)

启动命令:
```bash
cd /home/dora/.octos/skills/skills/octos-dora-bridge
dora up
dora start so101-mujoco-bridge-resolved.yaml
```

已验证:
- `main get_dropoff_position` → SO-101: x=0.250 / Adora: x=0.287 ✅
- `main --robot adora pick_cube_at` → IK 计算 + bridge 通信 ✅
- Bridge `move_to_named("home")` → ok ✅

详见内存: [[so101-pick-cube-octos-deploy-progress]]

## 阶段 9: octos 注册 (✅ 完成)

- mujoco 3.10.0 已安装 (系统 Python)
- `octos skills install` 注册成功
- `octos skills list` 显示 `so101-pick-cube v0.3.0 [3 tools]`
- 3 个 tool: `get_dropoff_position`, `pick_cube_at`, `place_cube_at`

## 阶段 10: move_to 工具 (✅ 完成)

v0.4.0 新增 `move_to` 纯移动工具:
- IK 求解 → bridge `move_to_joint_state`，不操作夹爪
- 输入: `{"x": 0.20, "y": 0.10, "z": 0.05}` (米)
- `manifest.json` 升至 0.4.0, 4 tools

## 阶段 11: 端到端验收 (⏳ 待用户)

```bash
export MODEL_NAME=/home/dora/so101-sim/dora-moveit2/examples/move_group_demo/models/so101_pickplace.xml
export ROBOT_MANIFEST=/home/dora/.octos/skills/skills/so101-pick-cube/manifests/adora.json
octos serve    # 终端1
octos chat     # 终端2
```

---

## 文件清单

### 核心文件 (自包含)
```
so101-pick-cube/
├── main                    # octos 工具入口 (158 行)
├── manifest.json           # 工具清单 v0.3.0 [3 tools]
├── SKILL.md                # 技能描述 (167 行)
├── arm_skills.py           # IK 求解 + 运动规划 (261 行)
├── manifest.py             # manifest 加载器
├── manifests/
│   ├── so101.json          # 仿真 manifest
│   ├── so101-hw.json       # 硬件 manifest (合并 arm_skills + bridge)
│   ├── adora.json          # Adora 仿真 manifest (1.15×)
│   └── adora-hw.json       # Adora 硬件 manifest
├── mjcf/
│   ├── so101/              # SO-101 MuJoCo 模型 + 13 STL assets
│   └── adora/              # Adora MuJoCo 模型 (STL → so101/assets symlink)
└── so101_pickplace_hw.xml  # MuJoCo wrapper (仿真测试用)
```

### 保留文件 (相机相关，当前不启用)
```
├── vision_service.py       # Orbbec 相机驱动
├── find_cube.py            # 方块检测算法
├── calibrate_extrinsics.py # 手眼标定 (Kabsch SVD)
├── calibrate_hand_eye.py   # 手眼标定 (完整流程)
├── auto_calibrate.py       # 自动标定
├── calibrate.sh            # 标定脚本
├── start_vision.sh         # 视觉服务启动
├── minimal_bridge.py       # 简易硬件桥
└── ee_T_camera*.npy        # 标定矩阵
```

---

## 相关内存

- [[orbbec-gemini-335-sdk]] — Orbbec SDK 1.10.22 升级
- [[so101-pick-vision-ik-progress]] — 视觉检测 + IK 修复
- [[so101-pick-cube-calibration]] — 手眼标定 (1-point seed)
- [[so101-pick-cube-skill-consolidation]] — Skill 自包含整合 + 无相机模式
- [[so101-pick-cube-octos-deploy-progress]] — octos 部署 + Adora + dora bridge
- [[octos-so101-skill-architecture]] — arm_skills.py manifest-driven 架构
- [[new-arm-measurement-guide]] — 新机械臂 MuJoCo XML 测量指南
