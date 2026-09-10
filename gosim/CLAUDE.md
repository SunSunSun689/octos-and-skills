# gosim 项目记忆 — 香橙派 20T 大模型部署

> 详细文档：`so101-pick-cube-架构.md`（skill 源码结构 + 调用链）、`过程记录.md`（部署时间线）

## 硬件环境（香橙派 20T）

- **SSH**：`HwHiAiUser@192.168.1.32`，密码 `dora123`（本机用 sshpass；旧 IP 192.168.1.20 / 192.168.1.27 已失效；2026-09-05 曾短暂切到 192.168.110.189 网段后又切回）
- **主机名**：orangepiaipro-20t，Ubuntu 22.04.3 LTS，aarch64，glibc 2.35
- **NPU**：Atlas 310B1（昇腾），`npu-smi` 在 `/usr/local/sbin/`（免 sudo）
- **CPU**：4 核中仅 **3 核在线**（`nproc`=3，核 3 恒 0%，规划资源按 3 核算）
- **内存**：23G；**磁盘**：235G（eMMC）
- ⚠️ NPU Health 显示 **Alarm**（部署前建议排查，温度 55°C 正常）
- ⚠️⚠️ **当前板子状态（09-01）**：华为通用 SoC 驱动/固件升级写挂板子，启动失败，**需重烧 base 镜像恢复**（详见过程记录阶段 13）

## gosim conda 环境（Python 3.11）

- 路径：`/home/HwHiAiUser/.conda/envs/gosim`（**注意**：板子 conda 二进制在 `/usr/local/miniconda3/bin/conda`，`~/.conda` 是用户侧环境目录）
- **dora-rs 1.0.0rc4** + **dora-cli 1.0.0-rc.4**（用户指定版本，2026-07-22 发布）
- dora-rs 1.0.0rc4 的 wheel 是 `cp311-abi3` → 需要 Python ≥ 3.11（系统自带 3.10 不行）
- 其他：fastapi、uvicorn、httpx、numpy 2.4.6、pyarrow 25.0.1
- **pip 必须用清华源**（直连 PyPI 极慢）：`-i https://pypi.tuna.tsinghua.edu.cn/simple`

## octos 2.0.3-rc.9（源码编译安装）

- **官方二进制不能用**：全部用 ubuntu-24.04-arm 构建，需 glibc 2.38/2.39，板子只有 2.35（所有历史 release 都不兼容）
- **解决方案：源码编译**。板子上装了 Rust 1.98（rustup，中科大镜像 `RUSTUP_DIST_SERVER=https://mirrors.ustc.edu.cn/rust-static`），crates.io 走 USTC sparse 镜像
- 编译：`cargo build --release -p octos-cli --features api,telegram,discord,feishu,twilio,wecom,wecom-bot,audio_mp3 -p octos-sandbox -p <9个skill crate>`，**43 分钟**，0 错误
- 安装位置：`~/.octos/bin/`（11 个二进制 + model_catalog.json，已入 PATH）
- 源码留在 `~/.octos-deploy`？不——源码在 **`/tmp/octos-src` 已随 target 删除**；本机 `/tmp` 有 clone。升级时重新 clone + 编译（`~/.cargo` 1.6G 缓存保留可免下载）
- 配置：`~/.config/octos/`（XDG 规范，**不是** ~/.octos——~/.octos 是运行时数据+安装目录）；API key 已替换为占位符 **sk-xx**
- ⚠️ **沙箱**：默认沙箱开启（`sandbox.enabled=true, allow_network=false`），LLM 的 bash 和 skill 执行都在沙箱里——看不到 gosim 环境、网络、串口。**config.json 需显式关闭**：`"sandbox": {"enabled": false, "allow_network": true}`（已配，机器人控制必需）
- **真机验证通过**（08-28）：octos chat 自然语言 → pick_cube_at(0.25,0,0.15) → 真实机械臂执行完整抓取周期（home→开夹→IK 运动到点→闭夹→回 home）。校准文件在 `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json`（按电机名 key 的 MotorCalibration 格式）
- 真机环境补充：gosim 装了 torch 2.10 + lerobot 0.4.4 + **feetech-servo-sdk**（飞特电机驱动，注意 PyPI 包名是 feetech-servo-sdk 而模块名是 scservo_sdk）；串口 /dev/ttyACM0（HwHiAiUser 已加 dialout 组）
- 已装 7 个 robot skill：so101、rebot（各 4 tools）+ agibot-a2、ff-navi、nav-base、unitree-g1、ur5e（文档型）

## octos-dora-bridge（机器人桥接）

- 源码：`~/octos_skills_source/octos-dora-bridge/`（带 .git）
- 装在 gosim 环境：`pip install --no-deps -e bridge/`（绕过 pyproject 的 `dora-rs<0.5` 约束，实测 dora **1.0.0rc4 兼容**）
- 兼容性验证：Node 类、`event["type"]`/`"INPUT"` 字典、`send_output` 签名均与 bridge 代码匹配；dora up + 最小 dataflow + HTTP `/healthz` 端到端跑通
- **实物栈已全部装齐**（GitHub 拉取，`~/octos_skills_source/`，原 octos-deploy，2026-09-01 重命名）：
  - dora-moveit2（bobdingAI，feat/so101 分支）
  - moveit-arm-dora-node（dorarobotics，main）
  - rebot-hw-dora-node（dorarobotics，main）
  - gosim 里 `--no-deps -e` 装了 dora_moveit / dora-mujoco / moveit-arm-dora-node / rebot-hw-dora-node / move-group-demo（全部绕过 dora-rs 版本约束）
  - 板子版 dataflow：`so101-hw-bridge-resolved-board.yaml`（路径已解析；`venv-python` wrapper 指 gosim python3.11）
  - **7 节点端到端跑通**：healthz=`ok`、/tools 返回完整清单、`pick_cube_at` 走完全流程（当前 ROBOT_BACKEND=loopback 假后端）
  - 接实物：yaml 改 `ROBOT_BACKEND: lerobot` + 装 lerobot（拉 torch）+ 串口 /dev/ttyACM0
- 已装 so101-pick-cube skill（v0.3.0，3 tools）到 `~/.octos/skills/`：main shebang 已改 gosim python3.11（系统 3.10 无 numpy/mujoco）；octos skills install 曾 panic 留半成品，最终手动复制完整目录解决

### vendor adapter 机制（大白话）

- **vendor adapter = 每种机械臂的"专属翻译官"**：把 bridge 的统一命令（SPEC-VENDOR-NODE-V1 标准）翻译成具体品牌的控制指令
- **不是提前定义的**：bridge 代码里没有任何机械臂名单
- **换臂靠"报到"机制**：
  1. dataflow yaml 的 vendor 节点 `args: -m <adapter模块>` 指定用哪个 adapter（人改配置）
  2. adapter 启动后主动广播能力清单（capabilities advert："我是 SO-101，我会这些动作"）
  3. bridge 订阅 `capabilities` 输入，收到后缓存——HTTP `/tools` 的清单来自这份广播
- **换臂三步**：pip 装新 adapter → yaml 改一行 → 启动自动报到，skill/bridge/octos 零改动
- `healthz` 返回 `advert_pending` = bridge 已就绪但 adapter 还没来报到（板子现状：adapter 未装）

## board-monitor（Rust 监控工具）

- 源码：本机 `/tmp/board-monitor/`（单文件 `src/main.rs`，零外部依赖）
- 已装板子 `~/.local/bin/board-monitor`；CSV 日志默认 `~/monitor-logs/`
- 用法：`board-monitor [--interval 秒] [--log-dir 目录] [--no-tui]`
- 坑：npu-smi 表格里 Power/Temp/Hugepages 三列之间**没有竖线分隔**，解析要按空格 token 取

## valve-detect（阀门识别，千问 Qwen2.5-VL）

- 需求：识别照片中的阀门，**评委可视化标注 + 坐标/状态 JSON 给机械臂**；单目 RGB 相机（板子 video4）
- **路线 1（正式方案，已跑通，v10 定稿）**：本机 ollama `qwen2.5vl:7b`（6GB Q4）+ FastAPI 服务（`~/gosim/valve-detect/valve_server.py`，端口 **8789**）+ 板子客户端 `~/valve_client.py`（拍照 video4 → POST → 坐标）。warm 后 0.6 秒/张；31/31 批量检测成功
  - 双类检测：阀门（红框）+ **蓝色手柄**（黄框+黄点=重心=机械臂操作位置）；右上角评委面板
  - 输出：bbox、state、visibility、handle_orientation、centroid_px、handle_px_length
  - **领域规则**（用户提供，已写进 prompt）：right=closed、down=open、右下之间=half_open，状态朝向互相校验——写入后 31/31 稳定
  - 已知边界：侧面视角框标大/朝向漂（img_020）；VLM 只能给框对角当端点（细粒度几何到顶）；伪像素坐标（无 box token）
- **机械臂定位（待办）**：单目估距（导航有误差，homography 不可取）：距离 = 焦距(像素) × 7.5cm(手柄长) ÷ handle_px_length；**相机标定拿焦距还没做**；再配手眼标定转基座坐标
- **valve-detect octos skill（09-03 已部署到 20T）**：本机源码在 `~/.octos/skills/valve-detect/`；20T 运行时目录 `~/.octos/skills/valve-detect/`，源码目录 `~/octos-deploy/octos-dora-bridge/skills/valve-detect/`。20T 版 `main` shebang 指向 `/home/HwHiAiUser/.conda/envs/gosim/bin/python`，默认 `VALVE_SERVER=http://192.168.1.19:8789`，`ARM_BRIDGE_URL=http://127.0.0.1:8768`。20T 已安装 `requests/opencv-python-headless/mujoco/pyorbbecsdk`，`pyorbbecsdk` 从本机 `/home/dora/SDK/pyorbbecsdk` 源码用 arm64 OrbbecSDK 编译安装；udev 规则已安装。20T 调本机 VLM 的 `valve_detect` 纯图片链路已验证通过；RGB-D 入口已能加载 SDK，目前因 20T 未连接 Gemini335 返回 `No Orbbec Gemini device found`。
- **09-05 大更新**：
  - **重新标定**（旧标定零位错误是执行链异常根因）；校准文件迁出缓存 → 20T `~/octos-deploy/rebot-hw-dora-node/calibration/so101.json`（manifest `calibration_dir` 指定，backend 3 行补丁）；本机 `~/gosim/calibration.json` 同步；旧缓存改名 `.pre-20260905-recalib`
  - **detect_pose_deg = [0, -85, 0, 85, 0]**（新标定口径，真机视野已人工确认）；`vlm_server` 进 valve_config.yaml（env > 配置 > 默认）
  - **闭环全自动拉起**：`valve_operate_loop` 自动起 bridge + 8791（dora run 隔离模式 + bash 中间层）；octos 白名单已补该工具
  - **bridge 自愈**：healthz + 状态新鲜度双检、CONTROLLER_BUSY 自动重启（实测生效）
  - **待修**：Gemini335 SDK 彩色流灰帧（UVC 正常）；固定相机 video 编号漂移（现 video0）
  - 一键环境脚本：20T `~/start_valve_env.sh` / `~/stop_valve_env.sh`（版本源在 valve-detect-skill/scripts/）
  - **octos 跑闭环指令模板**：`octos chat --profile coding-full "直接调用 valve_operate_loop 工具执行阀门操作闭环，参数：target_state=open、decision=rule、max_attempts=2、dry_run=arm。不要探索环境，不要自己写 bash 脚本，直接调用工具并等它跑完报告结果。"`（先 arm 后 none；工具自动拉起+自愈，详见 valve-detect-skill/README.md）
- **09-10 排查 move_to_pose 可达性（4 个 bug 全修复，点位真机验证通过）**：
  - 修复顺序：①geometry FK qpos 错位（pick-place 模型 qpos[0:7] 是物体 freejoint，臂 hinge 在 qpos[7:12]）②手眼标定参考系错配（T_cam_in_wrist 语义是"相机在 **gripper** 系"，geometry 原用 wrist body）③20T moveit config 占位版（placeholder LINK_TRANSFORMS + EE_OFFSET=[0,0,0.09] → DE IK 用错误 FK 模型；正确版在本机 so101-sim 仓库，与 mjcf pinch site 数值一致，已替换并删除占位备份，bridge 重启生效）④闭环 joints 提取路径错误（data.joint_positions 应为 data.stream.joint_positions，恒得默认零位 → FK 恒零位姿态）
  - 真机验证：闭环 3D 点 **[0.4414,-0.0051,0.0969]**（径向 0.46m 可达），move_to_pose 成功（修复前 DE error=0.082），运动轨迹扫过把手，用户现场确认点位准确
  - 测试 137 → **140 passed**；geometry.py/orchestration.py 已部署 20T；bridge + 8791 均重启加载新代码
  - **待办**：dry_run=none 完整闭环真压（按压行程/方向可能还需现场标定）
- **VLM 控制阀门可行性试验（09-02）**：`valve-detect` skill 新增 `valve_move_so101_to_handle`（Orbbec 335 RGB-D → 手柄 3D 点 → SO-101 预接近点）和 `valve_vlm_control_trial`（VLM 只选择 move_to_handle/nudge/retreat/stop 离散动作，step_m 内部限制 ≤0.02m，默认 execute=false）。PC 服务新增 `/plan-valve-action`，用于让 Qwen 根据图像+检测结果给受限动作建议。⚠️ 实验模式，不是正式闭环；VLM 不接管 3D 坐标/关节角/轨迹，真实运动仍由深度几何、`gemini335_hand_eye.json` eye-in-hand 标定、当前关节 FK 和 bridge 执行。
- **阀门检测初始姿态（09-02，本机 /dev/ttyACM0 实测后按现场观察修正）**：shoulder_pan=-7.516°、shoulder_lift=-74.066°、elbow_flex=45.0°、wrist_flex=90.0°、wrist_roll=-4.615°、gripper=80.934。已写入 `valve-detect/main` 的 `VALVE_INITIAL_POSE_DEG`；Orbbec 阀门定位/试验控制工具默认先运动到该初始检测位再采集，这是阀门检测 skill 的入口基石，`move_to_initial=false` 仅用于调试跳过。`execute` 只控制后续是否运动到手柄/执行 VLM 小步动作。bridge 只接收前 5 轴弧度 joint-state，gripper 目前仅记录
- **路线 2（20T 本地推理，失败）**：CANN 7.1 无配套 torch_npu；华为 310B 驱动分 EP/SoC 形态，SoC 包写入固件后板子起不来（板级适配缺失）→ 只能等香橙派官方新镜像
- 备选（未试）：板子自带 mxVision 5.0 跑 YOLO 类检测模型（不用千问、不用升驱动）

## base 镜像

- 文件：**本机 `/home/dora/orangepi-base.img.gz`（18.5G）**，从 256G SSD（`/dev/sda`）整盘 dd + gzip 导出，已 `gzip -t` 校验
- 恢复：`gunzip -c orangepi-base.img.gz | sudo dd of=/dev/sdX bs=4M status=progress`
- 目标盘 ≥ 238.5G；容量不一致时用 gdisk 修复 GPT（x → e → w → y）
- 镜像内敏感数据未完全清理（SSH host keys、machine-id、bash_history 都在），仅限内部使用

## 踩过的坑（重要）

1. **dd 导出数据流污染**：`dd status=progress 2>&1 | gzip` 会把进度文本混进压缩流导致镜像损坏。正确：`2>/tmp/dd-progress.log` 单独重定向
2. **SSH 非交互 shell 的 PATH**：板子 `.bashrc` 开头有交互检查提前 return，SSH 远程执行命令时 `source ~/.bashrc` 无效，要用完整路径或手动 export
3. **ports.ubuntu.com TLS 被干扰**：下载 Ubuntu 包用清华镜像
4. **`rm -rf dir/*` 不匹配隐藏文件**：清理要用 `find dir -mindepth 1 -delete`
5. **octos 工具协议**：工具名走 argv[1]，参数 JSON 走 stdin（`echo '{"city":"Beijing"}' | weather get_weather`）
6. **dataflow 老化**：dora 1.0.0-rc.4 的 zenoh 通信层长跑（小时级）后消息链路失效——运动命令 BRIDGE_TIMEOUT 但 healthz 仍 ok（假健康，bridge 缓存的能力广播）。修复 = 重启 dataflow（destroy → up → start）
7. **octos 沙箱默认开启**：LLM 的 bash/skill 执行被隔离（看不到 gosim/网络/串口），机器人控制必须 config.json 关闭：`"sandbox": {"enabled": false, "allow_network": true}`（已配）
8. **octos chat 指令技巧**：明确说"直接调用 XX 工具，不要探索环境"能大幅减少迭代次数；探索式自由发挥会耗尽 20 次迭代预算（budget exhausted）
9. **SOUL.md/AGENTS.md 行为准则**（08-28 已改）：`~/.config/octos/` 下
   - AGENTS.md 写入机械臂场景准则：环境始终就绪勿探索、直接调 so101-pick-cube 工具、失败原样报告勿诊断、运动慢是正常
   - SOUL.md 的 "diagnose before retrying" 加了机器人场景例外
   - 实测效果：探索从 7 次迭代降到 4 次（部分生效）；LLM 仍倾向自己写 bash 脚本调 bridge 而非直接调 skill 工具——后续方向：profile 权限砍掉 bash 工具（釜底抽薪）
10. **机械臂物理断连教训**：电机 0/6 响应 = 机械臂电源/数据线问题（舵机供电与 USB 供电独立）；串口时间戳变化 = USB 重连（权限 666 会丢，需重 chmod）；板子离线后 SSH 不通要等网络恢复
11. **octos 2.0.3 `--profile` 机制**（09-01 实测）：`octos chat` 默认 profile=`coding`（精简工具面：文件/shell/搜索/内存，**不含 skills**）；要注入 skill 工具必须 **`--profile coding-full`**（web/research/pipelines/bundled skills）。`~/.octos/skills` 已被标 legacy global（警告：建议迁到 `<data_dir>/skills/` per-profile install；目前仍会加载，未来版本可能停扫）
12. **skill 更新无需重装**（09-01 实测）：改 `manifest.json`（增删工具）→ 下次 `octos chat` 启动即生效（实测改名 pick_cube_at→pick_cube_at_hot 后立即出现在工具清单）；改 `main`/`arm_skills.py` → 工具每次新起进程执行，下次调用即生效。**唯一例外**：octos serve 常驻进程启动时缓存工具清单（fd 里无 skill 文件保持打开），走 serve App UI 会话时改 manifest.json 需重启 serve
13. **dora 热重载**（09-01 实测）：`dora start --hot-reload`（Python only）运行中改 Python 节点源码自动重启该节点进程；`dora restart` 用存储的 descriptor 快速 stop+start；dataflow YAML 改动**不**热重载（需重新 start），zenoh 老化（坑 6）热重载也无效
14. **昇腾驱动分形态，第三方板只认厂商包**（09-01 血泪）：华为下载页 310B 驱动/固件分 **EP（PCIe 卡）/SoC** 形态；EP 包形态检查失败不写入（安全），**SoC 包写入后板子起不来且固件不可逆**。根因：板级层（驱动/固件/设备树）各板不同，华为通用包只保证自家 Atlas 产品线，香橙派必须用官方渠道的适配包/新镜像。类比 VBIOS 不能跨厂刷
15. **bash 变量名吞下划线**：`$i_annotated` 会被整体解析为变量名（空），5 张图写进同一文件互相覆盖——文件名拼接用 `${i}_annotated`
16. **octos tool_policy 白名单 = LLM 可见工具准入口**（09-05）：新增 skill 工具必须加 `~/.config/octos/config.json` 的 `tool_policy.allow`，否则 LLM 完全看不到（日志 `tools=N` 可对账）
17. **dora run 父进程退出即自杀**（09-05）：工具进程退出 → 子 dora run 的 dataflow 被 AllInputsClosed 拆掉（约 5 秒全灭）。对策：`bash -c` 中间层（不用 exec），bash 成孤儿但存活、dora run 父进程一直在。另：dora CLI 只能连 `dora up` 同会话的 daemon，跨 SSH 会话用 `dora run` 隔离模式（自带 coordinator）
18. **valve /status 未跑闭环时返回 404**（09-05）：`{"error":"no run yet"}` 带 404 状态码，urlopen 会抛异常——就绪探测必须把任何 HTTP 响应（含 4xx）视为就绪，连接拒绝/超时才算没起来
19. **Gemini335 SDK 彩色流黑/灰帧**（09-05 未解决）：auto exposure 属性设置静默失败（wrapper 吞异常），手动 `OCTOS_CAMERA_EXPOSURE=3000` 后从全黑变均匀灰 76（恒定值帧）；UVC 直连 ffmpeg 却是真画面（传感器正常，SDK 彩色流路径问题）。USB 插拔后 video 编号会漂移：固定相机已从 video8 变 video0
19b. **Gemini335 过曝根因（09-08 已修）**：过曝真凶是**增益满格**——设备读回 `gain=16`（最大值）、exposure=156µs，曝光再小也过亮；叠加两个放大器：①曝光/增益属性设置在 `pipeline.start()` **之后**会被固件静默忽略（官方示例是 start 前设），②wrapper 静默吞异常导致问题藏了 3 天。修复：属性设置挪到 start 前 + 失败打日志；`OCTOS_CAMERA_GAIN: "4"` 钉死增益、恢复自动曝光。效果：过曝 26.6%→4.1%，mean 184→89。微调：偏暗调大 GAIN（2-8），再不够配 OCTOS_CAMERA_EXPOSURE 手动曝光（微秒）
22. **夹爪开合命令被读回覆盖**（09-08 已修，真机验证）：`LeRobotBackend.read_state()` 每 tick 用实测夹爪宽度覆盖 `_gripper_w`（命令宽度）——开夹命令发出后下一个 tick（20ms）读到的实测仍是闭合（舵机未动完），命令被拉回闭合，**夹爪永远打不开**。手臂关节无此问题（`_target_sim` 不被读回覆盖），只有夹爪踩坑。排查路径值得记：命令链路日志全通（bridge 返回 ok）→ 物理层用 lerobot API 直接探针验证正常（0.7%↔99.4%）→ 锁定 backend 自身反馈覆盖。修复：命令宽度权威，观测同步只在 start()/断电恢复时
20. **bridge 假健康自愈**（09-05）：healthz ok 但 get_state 的 `data.stale`/`data.last_age_s` 陈旧 = zenoh 老化（坑 6）；CONTROLLER_BUSY = motion 完成消息丢失卡死控制器。工具内已实现双检 + 自动重启重试
21. **部署后闭环不生效 = 常驻进程跑旧代码**（09-08，进程管理问题）：octos chat 无需重启（每次调工具新起进程，坑 12），但 **8791 闭环 dataflow 常驻**——部署只换磁盘文件，orchestration 进程内存里还是旧代码；`valve_operate_loop` 的"8791 没起才拉起"被旧进程假健康骗过（进程活着 ≠ 状态正确，与坑 6/20 同源）。相关事实：`dora run` 隔离模式**不支持热重载**（`--hot-reload` 仅 `dora start`、Python only，坑 13）；`dora list`/`dora stop` 对隔离模式 dataflow 无效（找不到 UUID，stop_valve_env.sh 的停止逻辑实测失效）；杀 `dora run` 进程组后**节点进程孤儿化继续跑**（8791 仍响应）。正确停止：`pkill -f 'valve_task_nodes'` + `pkill -f 'dora run.*valve-task.yaml'`，再 nohup dora run 重启。**待办**：deploy 脚本加"部署即重启"、stop 脚本改用 pkill
23. **mjcf pick-place 模型 qpos 布局**（09-10）：qpos[0:7] 是物体 freejoint，臂 hinge 在 qpos[7:12]。FK 关节角必须按 hinge 的 jnt_qposadr 写入——`data.qpos[:n]=joints` 会把臂关节写进物体自由关节，FK 恒默认姿态（3D 点错 0.26m）。对纯臂模型 hinge 地址从 0 起，该写法两种模型都对
24. **手眼标定 T_cam_in_wrist 语义是 gripper 系**（09-10）：标定脚本 so101-depth-pick/main 的 forward_kinematics 默认返回 **gripper body** 位姿（不是 wrist body）。geometry 的 FK 必须用 gripper body，用 wrist 差 ~0.17m 且方向全变。rpy 是 ZYX 分解，与 _rpy_to_rot 互逆（这层没问题）
25. **moveit config 有占位版**（09-10）：20T 的 move_group_demo/config/so101.py 曾是 placeholder（LINK_TRANSFORMS 注释明说、EE_OFFSET=[0,0,0.09]）——DE IK 用它的 FK 模型判可达性，**占位版会把真实可达点判成 DE failed**（joint-space 运动不受影响，所以 detect_pose/home 一直正常）。正确版在本机 so101-sim 仓库（与 mjcf pinch site 数值一致，已数值验证）。config 由 load_config **每进程缓存**，替换文件必须重启 moveit 节点进程（bridge）
26. **bridge get_state 的 joint_positions 在 data.stream 层**（09-10）：`data.stream.joint_positions`（不是 data.joint_positions）。orchestration 直接 .get 链取值恒得默认 [0]*5 → FK 恒零位姿态，3D 点差 0.27m。正确：用 `_extract_joint_positions`（搜 payload/data/data.stream/stream 四层）。⚠️ 教训：**单测的 fake 响应结构必须与真实 bridge 一致**，假结构是错的则测试全绿也拦不住这种 bug（已把 FakeTransport 改成真实 stream 结构 + 回归测试）
27. **DE IK 对 wrist_roll 无约束 → 按压中途第 5 关节旋转**（09-10 真机观察，方案 A 已实现）：wrist_roll 绕接近轴旋转，对 pinch 位置影响 ≤1.5cm → DE 位置目标对它无梯度 → approach/press 两次独立求解解出的 wrist_roll 不同 → 夹爪扭转、手柄压不到位。修复（方案A）：按压改关节空间——`geometry.solve_press_joints` 锁定 wrist_roll、L-BFGS-B 解前 4 关节使 pinch 到目标，`move_to_joint_state` 执行（approach 仍用 move_to_pose）。文件已部署 20T，8791 待重启后真压验证
