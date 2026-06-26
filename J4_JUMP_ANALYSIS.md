# SO101 J4 (wrist_flex) 抓取跳变分析

## 现象

`pick_at` 执行时，机械臂从 approach（方块上方 5cm）下降到 grasp（方块表面），J4（wrist_flex）关节从 ~68° 猛跳到 ~163°，变化 95°，导致夹爪姿态突变、抓取失败。

## 根因

### 1. IK 种子点 `arm_home` 单位混淆

**文件**：`manifests/so101-hw.json`

```json
"arm_home": [0.0, -1.0, 1.0, 0.0, 0.0],
"use_degrees": true
```

`arm_skills.py` 通过 `cfg.vec("arm_home", ...)` 读取该值，**不进行单位转换**，直接当作弧度传入 IK solver：

```
manifest 意图（度）:  [ 0°,  -1°,   1°,   0°,   0°]
实际传入 IK（弧度）: [ 0,   -1,    1,    0,    0  ]  ← 错!
实际 home（弧度）:   [-0.185, -0.058, 0.275, 1.213, -0.002]
```

后果：IK solver 从完全错误的种子点出发，在零空间中漂移到完全不同的 wrist 配置。

**修复**：将 `arm_home` 改为实测弧度值：
```json
"arm_home": [-0.185, -0.058, 0.275, 1.213, -0.002]
```

### 2. 关节限位索引错位 (jnt_range vs qpos offset)

**文件**：`arm_skills.py`

```python
# arm_skills 从 MuJoCo 模型取关节限位
lo = _m.jnt_range[JNT_RANGE_START : JNT_RANGE_START + NUM_JOINTS, 0]
hi = _m.jnt_range[JNT_RANGE_START : JNT_RANGE_START + NUM_JOINTS, 1]
```

模型 `so101_new_calib.xml` 有 6 个关节 (qposadr 0~5)：

| 索引 | 关节 |
|------|------|
| 0 | shoulder_pan |
| 1 | shoulder_lift |
| 2 | elbow_flex |
| 3 | wrist_flex |
| 4 | wrist_roll |
| 5 | gripper |

但 `JNT_RANGE_START` 默认为 1（非 0），且 manifest 中 `arm_qpos_start=0`，两者不对齐：

```
jnt_range[1:6] → [shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]  (5个)
qpos[0:5]      → [shoulder_pan,  shoulder_lift, elbow_flex, wrist_flex, wrist_roll] (5个)
```

索引偏移 1 位 → shoulder_pan 无限位，其他关节限位全错。

**修复**：manifest 中显式设置 `"jnt_range_start": 0`，使限位与 qpos 对齐。

### 3. IK 零空间自由度

SO101 是 5-DOF 臂。`_hold()` 对 5-DOF 臂返回 `None`，IK 仅约束 approach 轴（ee_x）指向下方（2-DOF 旋转约束）。绕 approach 轴的旋转在零空间中完全自由。

```
约束系统: 3 位置 + 2 旋转 = 5 DOF 约束
机械臂:   5 DOF
零空间:   1 DOF（绕夹爪轴的旋转）
```

同一点位有无限多组关节解，IK solver 在 approach→grasp 时沿零空间漂移，找到完全不同的 wrist_flex 配置。

**解决方案**：修复种子点和关限后，DLS solver 自然保持解在种子附近，零空间漂移被抑制。

## 修复后效果

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| J4 最大阶跃 | 95° | 30° |
| 关节执行误差 | max_err≈66° | max_err≈4° |
| IK 收敛 | 不稳定 | 所有阶段关节角一致 |

## 修改文件清单

| 文件 | 改动 |
|------|------|
| `manifests/so101-hw.json` | `arm_home` 改为实测弧度值；添加 `jnt_range_start: 0` |
| `skill_pack/arm_skills.py` | `_solve` 添加 `has_limits` 空限位保护；`pick_at` 支持传入 bz |
| `so101-pick-cube/main` | `pick_cube_at` 自动获取 bz 传入 `pick_at` |
| `so101-pick-cube/ee_T_camera.npy` | 旋转矩阵正交化，平移 tz 标定修正 |

## 遗留问题

SO101 工作空间限制：方块在 bx=0.25m 时，shoulder_lift、elbow_flex、wrist_flex 均到限位，末端无法完全降到方块表面（grasp z 差 ~8cm）。需将方块移近或缩短标定的有效臂长。
