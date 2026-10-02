# RoboCup 多旋翼集群搜索 — 机载感知节点（双目版）

把双目相机的图像变成**目标在世界坐标系里的位置**，并同时喂给两路下游：
**官方裁判**（决定得分）和**团队协同核心**（决定任务分配）。

---

## 1. 它解决什么 — 在本仓库里的位置

本仓库分两部分，**感知节点就是这个 `perception/` 目录**：

| 目录 | 负责 | 产物 |
|---|---|---|
| 仓库根（`weights/` `config/` `scripts/` `dist/`） | **训练** | 权重 `best_yolo11n_bino_v1.pt`、数据集、训练脚本、逐轮指标 |
| **`perception/`（本目录）** | **部署** | 能跑的 ROS 感知节点：图像 → 世界坐标 → 双路发布 |

根目录的 `scripts/infer_demo.py` 只输出像素框 `xyxy`。它**没有**世界坐标、
**没有**话题发布、**没有**官方判据要求的连续跟踪逻辑。本目录补的正是这一段。

> ✅ **权重已在仓库里**：`../weights/best_yolo11n_bino_v1.pt`，节点会自动找到它，
> clone 完不用拷任何文件（见 §3.3）。

> ⚠️ 类序必须与权重一致：`[red, green, blue, white, brown]`。
> 本目录的 `CLASSES` 与权重的 `data.yaml` 对齐，改用别的权重前先核对。

---

## 2. 数据流

```
 双目相机左目  ──►  YOLOv11 推理  ──►  track 关联/外推  ──►  世界坐标 (x, y)
 /<uav>/stereo_camera/left/image_raw                                    │
                                        ┌──────────────────────────────┴───────────┐
                                        ▼                                          ▼
                     /actor_{green,blue,brown,white,red1,red2}_info    /coordination/target_report
                     （ActorInfo，给官方裁判判分）                        （String，给协同核心派任务）

                     另有 /perception/debug_snapshot（String，调试用，不参与判分）
```

**三路输出缺一不可**：少第一路就没有分数，少第二路协同核心收不到目标。

---

## 3. 前置依赖

### 3.1 系统 / ROS

- **ROS Noetic**（Ubuntu 20.04）
- **XTDrone + PX4 SITL + MAVROS**，且仿真世界已能正常起飞
- **已编译的 catkin 工作空间**，需包含：
  - `ros_actor_cmd_pose_plugin_msgs`（提供 `ActorInfo`，本节点和官方裁判都用它）
  - `gazebo_msgs`、`sensor_msgs`、`std_msgs`、`mavros_msgs`
  - `cv_bridge`
- **虚拟显示**：Gazebo 相机传感器需要 GL 上下文才能真正渲染。
  没有 `DISPLAY` 时相机话题**根本不会建出来**，表现为"无人机和相机都在，就是没有图像话题"。
  `scripts/env_robocup.sh` 会自动拉起 `Xvfb :95`。

### 3.2 Python

```bash
pip3 install -r requirements.txt      # ultralytics / opencv-python / numpy
```

> 国内网络：加 `-i https://mirrors.aliyun.com/pypi/simple`

### 3.3 权重

**仓库自带，不需要额外操作。** 节点按顺序查找：

1. `../weights/best_yolo11n_bino_v1.pt` —— 与 `perception/` 同级，**clone 即有**
2. `~/catkin_ws/src/yolov11_ros/weights/best.pt` —— 部署进 catkin 工作空间后的常规位置

启动时会打印实际加载的路径（`[pr] 加载权重 <path>`），**先确认这行指向你要用的权重**。

要指定别的权重（例如自己重训的）：

```bash
export PR_WEIGHTS=/abs/path/to/best.pt
```

### 3.4 上机前置：给无人机装双目相机（**必须做，否则收不到图**）

感知节点订阅 `/<uav>/stereo_camera/left/image_raw`，而**PX4 原厂的
typhoon_h480 只有云台相机，没有 stereo_camera** ⇒ 不打这个 patch
话题根本不存在，现象是"节点在跑、但一条检测都没有"。

```bash
python3 patch/add_stereo_camera.py --check          # 只看状态
python3 patch/add_stereo_camera.py --px4-root ~/PX4_Firmware
```

⚠️ 这里有本机踩过的坑：官方 `stereo_camera` 用的是 `multicamera` 插件，
**话题会 advertise 出来、但永远收不到帧**。所以 patch 改成两只独立单目
（光学参数照抄官方，一个数没改）。验证时必须看**有没有帧**，不能只看话题在不在：

```bash
rostopic hz /typhoon_h480_0/stereo_camera/left/image_raw     # 应 ≈30 Hz（RTF 低时 6~10 Hz 也算正常，但不能是 0）
```

详见 `patch/README.md`。

### 3.5 官方资产依赖（不在本仓库内）

| 资产 | 来自 | 用在哪 |
|---|---|---|
| `walker/walk_0..5.dae` | XTDrone `sitl_config/models/` | 6 个目标的**外观** —— 数据集就是拿它们采的 |
| `stereo_camera` | 同上 | 相机标定的来源（fx=fy=376, cx=376, cy=240, 基线 0.12 m） |
| `robocup/base.world` | XTDrone `robocup/` | 6 个 `<actor>` 的 skin 与初始位姿 |

获取：`git clone https://gitee.com/robin_shaun/XTDrone`（比赛包）

🔴 **不要替换这些模型。** 换成非官方版本（自己造的人、改过色的 dae）之后，
训练数据的外观就不再等于比赛时的外观，模型会**静默掉点** —— 这种错在赛场上
很难当场发现。有一条命令可以核对：

```bash
python3 scripts/verify_official_assets.py --gt ~/robocup_dataset_gt_v3/gt.json
```

它会读 `walk_N.dae` 里的**实际色值**反推颜色（不听信文件名），对照官方
`score_cal.py` 的 `actor_id_dict = {'green':[0],'blue':[1],'brown':[2],'white':[3],'red':[4,5]}`，
再查相机内参、`base.world` 的 skin、以及数据集 `gt.json` 的标注映射。全 PASS 才算对齐。

---

## 4. 目录

```
perception_real.py               ★ 感知主节点（v4.0 双目）
requirements.txt                 非 ROS 的 Python 依赖
launch/
  world_min.launch               轻量世界（绕开 base.world 段错误，见 §8.2）
  world_only.launch              只起 gzserver
  uav_only.launch                只起 typhoon_h480_0（PX4 + MAVROS）
  robocup_1uav.launch            合成：世界 + 单机
patch/
  add_stereo_camera.py           ★ 给无人机装双目相机（上机第一步，见 §3.4）
  README.md                      为什么要打、怎么验证「有帧」
scripts/
  restart_stack_fly.sh           ★ 一键全栈：世界 → 无人机 → actor → 感知
  env_robocup.sh                 ★ 环境变量（ROS / Gazebo / Xvfb）
  run_actors.sh                  起 6 个 actor 行为 AI（仿真验证用）
  control_actor_m2.py            actor 行为控制（run_actors.sh 调用）
  proc_util.sh                   进程管理小工具
  check_target_report_schema.py  用**队友的校验器**自检契约形状
  verify_official_assets.py      ★ 核对模型/相机/标注是否都是官方的（见 §3.5）
  rec_snap.py / analyze_snap.py  采集与分析检测快照（离线复盘）
  judge_official_15s.py          ★ 按**官方 15s 判据**给 PASS/FAIL（复核成绩用这个）
  guide_patrol.py / guide_actor.py  定速引导 actor 往复（2 m/s 压测用）
  probe_actor_speed.py           量 actor **真实**速度（确认引导指令被精确执行，别信标称值）
  snapshot_cam.py                抓一帧相机图像
  fly_takeoff.py                 起飞脚本
  replay_target_report.py        离线重放 target_report（不上仿真也能验契约）
worlds/
  actor_min.world                从官方 base.world 截出的 actor-only 世界
docs/
  部署检查清单.md                 逐项自检
```

---

## 5. 快速开始

### 5.1 一键全栈（推荐）

```bash
bash scripts/restart_stack_fly.sh
```

脚本按顺序做：停旧组件 → 起世界 → 起无人机并等 MAVROS 连上 → 起 6 个 actor → 起感知 → 相机话题自检。

结束打印 `STACK_FLY_OK` 表示飞控栈就绪；`STACK_FLY_FAIL*` 表示失败（日志在 `/tmp/`）。

**先设环境变量**（如果你的目录布局和默认不同）：

```bash
export ROBOCUP_XTDRONE=$HOME/XTDrone/robocup          # XTDrone 的 robocup 目录
export ROBOCUP_WS_SETUP=$HOME/catkin_ws/devel/setup.bash
export ROBOCUP_ROS_SETUP=/opt/ros/noetic/setup.bash
```

### 5.2 只跑感知节点

前提：Gazebo 世界已起来（`/gazebo/get_link_state` 服务可用）。

```bash
cd perception                   # 本目录（perception_real.py 所在地）
source scripts/env_robocup.sh   # 起 Xvfb + 设 ROS / Gazebo 环境
python3 -u perception_real.py
```

> 节点内部只用绝对路径，**不依赖当前工作目录**；`cd` 只是让上面的相对路径调用成立。

> ⚠️ **不要**同时启动独立的 `yolov11_ros` / `yolo_v11.py` 节点。
> 本节点自带 YOLO 推理，两套 YOLO 互抢 CPU 会把主循环从 ~5 Hz 拖到 0.75~1.6 Hz，
> 而官方判据要求**广播间隔 < 1 s** —— 会直接判不出发现。

---

## 6. 接口

### 6.1 输入

| 项 | 值 |
|---|---|
| 图像话题 | `/<PR_UAV>/stereo_camera/left/image_raw`（`sensor_msgs/Image`，默认 `typhoon_h480_0`） |
| 相机位姿 | 服务 `/gazebo/get_link_state`，link = `<PR_UAV>::base_link` |
| 内参 | `FX=FY=376.0`、`CX,CY=376.0,240.0`、图像 `752×480` |

> 相机位姿取的是**无人机自身**位置（合法，不算目标真值）。
> 迁移到真机时，把这一处换成 MAVROS 里程计即可。

### 6.2 输出 A — 给官方裁判（决定得分）

| 话题 | 类型 |
|---|---|
| `/actor_green_info`、`/actor_blue_info`、`/actor_brown_info`、`/actor_white_info` | `ros_actor_cmd_pose_plugin_msgs/ActorInfo` |
| `/actor_red1_info`、`/actor_red2_info` | 同上（红色有**两个**目标） |

载荷：`ActorInfo(cls=<类名>, x=<世界X>, y=<世界Y>)`

> 📌 **判分是两段式**（官方 `score_cal_master.py` 实现，**不是**"必须连续 15 s 才有分"）：
> **① 单帧报对 = +50 分**（误差 <1 m、与上一帧间隔 ≤1 s；中断后不回退）；
> **② 自首次命中起连续 15 s 无中断 = 再 +100 分**。
> ⇒ "报得准"和"报得久"是两件事：前者几乎白拿（6 目标 = 300 分），
> **后者才是真难点**（600 分），而它唯一的敌人是**目标速度 × 链路滞后**（见 §10）。
>
> 🔴 **红色双流是硬约束**：两个红衣人必须**各自锁定一条流并持久绑定**。
> 若逐帧按分数重排（谁分高谁发 red1），流与人的对应会在帧间互换，
> 裁判的连续计时反复清零 —— 那 15 s 就永远累不到（丢的是 100 分的消除分，不是零分）。

### 6.3 输出 B — 给协同核心（决定任务分配）

话题 `/coordination/target_report`，类型 `std_msgs/String`，
**payload 是裸 data 对象**（不是带 envelope 的完整消息），**每个目标一条**：

```json
{"target_id": "green", "frame_id": "world_enu", "xyz": [12.34, -5.67, 1.25],
 "confidence": 0.87, "observation_id": "obs-green-42"}
```

- `target_id`：用**官方目标身份**（`green` / `blue` / `white` / `brown` / `red1` / `red2`），跨帧稳定。
  **不能**用 track 的整数 id —— 那玩意会因重建而变。
- `xyz`：恰 3 维。其中 `z` 是显式补的（默认 1.25 m，官方 actor 体心高度），
  因为感知算出的 `(x, y)` 是**射线与 z=0 平面的交点**（地面点），不是体心。
- 字段集合**必须严格等于**这 5 个。多一个字段都会被核心拒掉。

> 🔴 **契约是 fail-closed 的**：核心校验不过时**只打一条 `logwarn` 就丢弃**，
> 不崩溃、不报错退出、话题上照样有消息在流动。
> 现象就是"核心什么都收不到、什么任务都不分配"，极难从表面看出来。
> **上线前务必跑一次** `python3 scripts/check_target_report_schema.py`。

#### ⚠️ 6.3.1 对接前必读：`confidence` 语义缺口

**当前状态：形状通过，语义会被忽略。**

协同核心 `coordination/core.py` 的 `_target()` 第一句是：

```python
if d['confidence'] < 1.:
    return   # v1 consumes confirmed reports; upstream owns fusion.
```

也就是说核心的约定是：**上游（感知）负责融合与确认判定，只有"已确认目标"才发 `confidence = 1.0`**。

而本节点目前发的是 **YOLO 原始置信度**（实测 0.42~0.91）。

**后果**：消息能 100% 通过 schema 校验，但核心会直接 `return`，
**任务坐标永远不会被更新**。表现与"没接上"几乎一样。

两种解法，**需要和协同核心负责人拍板选一个**：

| 方案 | 做法 | 影响面 |
|---|---|---|
| **A（推荐）** | 感知侧对"已确认目标"发 `confidence = 1.0`，未确认的照旧发真实置信度 | 只改本仓库；核心不动 |
| **B** | 核心侧放宽阈值（承认连续 N 帧一致即视为确认） | 改核心，需重跑其 34 项测试 |

选定前，若要用核心跑端到端，可先用 `PR_COORD_CONF_ONE=1` 临时强制（需本仓库支持该开关）。

### 6.4 输出 C — 调试（不参与判分）

`/perception/debug_snapshot`，`std_msgs/String`，**一帧一条**、含全部候选 track 的分数/命中数/外推帧数等，
供 `rec_snap.py` + `analyze_snap.py` 离线复盘。

---

## 7. 实测性能（真飞，VM 8vCPU / 8G / 无 GPU / 软件渲染）

| 指标 | 实测 |
|---|---|
| 精确率 | **99.2 ~ 100%**（459 条真实观测仅 2 条误报） |
| 类别准确率 | **100%**（无颜色混淆） |
| 定位误差 · 8–15 m | 0.43 ~ 0.54 m |
| 定位误差 · 15–25 m | 0.78 ~ 1.13 m |
| 定位误差 · 25–40 m | 1.10 ~ 1.36 m |
| 召回（含 track 外推） | 50 ~ 61% |
| 有效检测上限 | **40 ~ 50 m** |
| 主循环频率 | 1.6 ~ 5 Hz（取决于 VM 负载；**必须保证发布间隔 < 1 s**） |

**结论**：官方判据是误差 < 1 m，所以**必须把目标压进 12~15 m** 才谈得上稳定达标。

> 📌 **时间滞后补偿已实现**（见 §9 的 `PR_LAG_COMP`）。误差的**横向项正比于目标速度** ——
> 这是链路的相位滞后（真飞实测隐含 **0.352 s 仿真时间**；RTF≈0.42，折墙钟约 0.85 s，
> 约 2 帧，像多级流水线积压，而不只是 YOLO 单次推理）。发布前按 track 速度前推可拿回一部分。
>
> 🔴 **但真飞实测：当前实现几乎不起作用，别指望它**（完整数据见 §10.1）。
> 2 m/s 下 `PR_LAG_COMP=0.35` 与 `=0` 是同版本单变量对照：单帧 <1m 从 64.5% 到 65.6%
> （**+1.1 个百分点**），最长连续反而从 4.1 s 退到 3.2 s。
> 根因：**感知自己的速度估计只有真值的 31%**（中位比值 0.31），补出来的量只有应有值的 1/3。
> 即便换成真值速度（天花板），最优也只到 10.0 s，**仍过不了 15 s**。
>
> ⇒ 默认值仍是 `0.3`（无害），但**真正的钱在"修速度估计"和"削沿向噪声"上**，不在这个开关上。

---

## 8. 已知坑（按踩坑频率排序）

### 8.1 ARM 被拒 / MAVROS 永远 `connected=False`
`gzserver` 里残留旧的 `typhoon_h480_0` 模型时，新 PX4 spawn 会报
`entity already exists`，随后卡在
`Waiting for simulator to accept connection on TCP port 4560`。

**只重启 PX4 救不了**（残留模型仍在世界里）。必须重启 gzserver，并且要显式
`pkill -9 -x gzserver`（只 `pkill` launch 文件拦不住它）。
`scripts/restart_stack_fly.sh` 已把这个顺序固化。

### 8.2 官方 `base.world` 加载段错误
本机实测：完整 `base.world`（7682 行）加载必现 `Segmentation fault`，
而空世界和 actor-only 世界都正常。

`launch/world_min.launch` 用的是从 `base.world` 截出的 `worlds/actor_min.world`（319 行），
只保留世界头 + `ground_plane` + 6 个 actor。

> 🔴 **比赛必须用完整 `base.world`** —— 这是尚未解决的阻塞项
> （怀疑 8 G 内存 + 软件渲染撑不住城市场景）。重新生成 `actor_min.world` 的方法见其文件头注释。

### 8.3 目标掉出视野
相机水平朝前安装但装在机身下方，**有效视野下缘约 −20°**。
悬停 5 m 时，目标需要 **> ~11 m** 才进画面（9.5 m 检不到，11.6 m 命中率约 47%）。

### 8.4 相机话题不存在 / 存在但收不到帧

两种故障现象很像，但原因完全不同，先分清是哪一种：

| 现象 | 原因 | 处理 |
|---|---|---|
| `rostopic list` 里**没有** `stereo_camera/...` | ① `DISPLAY` 未设置，Gazebo 不渲染（见 §3.1）<br>② 压根没装双目相机 | ① 用 `env_robocup.sh` 拉起 Xvfb<br>② 打 `patch/add_stereo_camera.py`（§3.4） |
| 话题**在**，但 `rostopic hz` 一直 `no new messages` | 装成了官方的 `multicamera` 版本 —— 该插件在本机能 advertise 话题，却永远不出帧 | 用 `patch/add_stereo_camera.py` 重打（它改成两只单目） |

🔴 第二种最坑：**话题存在**会让人以为已经对了，然后一直往检测阈值上找原因。
排查相机链路时**永远先 `rostopic hz`，不要只 `rostopic list`**。

### 8.5 目标速度 / 逃逸机制 —— 2 m/s 逃逸的触发条件在这里

**触发条件（代码事实，`control_actor_m2.py` 逃逸判定段）**：

```python
if (uav_twist.x)**2 + (uav_twist.y)**2 > 1.0:      # ① UAV 水平速度 > 1 m/s
    dis = 到该 UAV 的水平距离
    if dis < 20.0 and catching_flag == 0:          # ② 距离 < 20 m
        tracking_flag[i] += 1
        if tracking_flag[i] > 20:                  # ③ 连续 > 20 帧（≈2 s @10Hz）
            catching_flag = 1                      # → 进入"被追"状态，开始躲逃
```

三条**全部满足**才逃。⇒ **悬停（速度 ≈ 0）时第 ① 条不成立，整段逃逸判定被跳过。**

> 🔑 **这是最重要的战术事实**：慢慢贴近 / 悬停，目标**不会**进入躲逃状态；
> 高速掠过才会。一旦触发，actor 会朝背离 UAV 的方向跑，最坏情况下狂奔到地图边界卡死。

**速度值：逃逸速度已确定为 2 m/s，但官方代码尚未跟上**

| | 平时 | 被 UAV 感知 / 追上后 |
|---|---|---|
| **官方规则（已定）** | **1 m/s** 随机走动 | **2 m/s 躲逃**，并向同伴广播无人机位置 |
| 官方代码现状（gitee `robocup/control_actor.py`，最后改动 2024-05-02） | 1 m/s | **仍是 1 m/s（尚未更新）** |

代码在该处是**无条件** `self.avoid.v = 1`，上一行注释写着 `#reduce difficulty`；
再往上有一段被三引号注释掉的原始设计（`catching_flag` → 3 m/s，否则 → 2 m/s）——
即**官方主动降过难度，2 m/s 还没落到代码里**。

⇒ 结论：**现在按 1 m/s 打，但要按 2 m/s 准备**（怎么做见 §10）。

> ⚠️ 顺带一个坑：被注释掉的那段改的是 `self.target_motion.v`，而**实际发布的是
> `self.avoid`**（`cmd_pub.publish(self.avoid)`）⇒ 即便取消注释**也不生效**。
> 真要改成 2 m/s，必须写 `self.avoid.v`。

**本地压测开关**（见 §9）：`ACTOR_V=2 ./scripts/restart_stack_fly.sh` 让目标全程 2 m/s，
可在官方更新前先验证我们的感知扛不扛得住。默认 `ACTOR_V=1`，与官方行为逐字等价。

---

## 9. 环境变量参考

| 变量 | 默认 | 说明 |
|---|---|---|
| `PR_WEIGHTS` | 自动查找（仓库自带优先，见 §3.3） | 权重路径；留空即走自动查找 |
| `PR_UAV` | `typhoon_h480_0` | 无人机命名空间 |
| `PR_CAM_TOPIC` | `/<PR_UAV>/stereo_camera/left/image_raw` | 图像话题 |
| `PR_CAM_LINK` | `<PR_UAV>::base_link` | 取位姿用的 link |
| `PR_CONF` | `0.40` | YOLO 置信度阈值 |
| `PR_COORD_ON` | `1` | 是否发布 `target_report` |
| `PR_COORD_HZ` | `2.0` | `target_report` 发布频率（对齐核心 tick，灌太快会积压） |
| `PR_TARGET_Z` | `1.25` | 上报的 z（官方 actor 体心高度） |
| `PR_DEBUG_TOPIC` | `/perception/debug_snapshot` | 调试快照话题 |
| `PR_MAX_RANGE` | `60` | 最大作用距离 |
| `PR_LAG_COMP` | `0.3` | **发布前按 track 速度前推的秒数（时间滞后补偿）**，见 §7 与 §10；`0` = 关闭，行为回到改动前 |

其余跟踪参数（`PR_H_MIN` / `PR_GATE` / `PR_RED_STICKY_R` 等）见 `perception_real.py` 顶部参数区，
每一项都有实测依据的注释，**改之前先读**。

环境/路径类：

| 变量 | 默认 |
|---|---|
| `ROBOCUP_PERCEPT_DIR` | `perception/` 目录（自动推断） |
| `ROBOCUP_XTDRONE` | `$HOME/XTDrone/robocup` |
| `ROBOCUP_WS_SETUP` | `$HOME/catkin_ws/devel/setup.bash` |
| `ROBOCUP_ROS_SETUP` | `/opt/ros/noetic/setup.bash` |
| `ROBO_REPO_COORD` | 自动查找队友 `robocup_repo` 的 `coordination` 包 |

actor 压测开关（`scripts/control_actor_m2.py`，见 §8.5 / §10）：

| 变量 | 默认 | 说明 |
|---|---|---|
| `ACTOR_V` | `1` | 目标基础速度 (m/s)。设 `2` 即"最坏情况"全程 2 m/s 压测 |
| `ACTOR_ESCAPE_V` | `0` | `>0` 时复现规则：被追（`catching_flag`）后改用该速度。`0` = 关闭 |

> 用法：`ACTOR_V=2 ./scripts/restart_stack_fly.sh`（前缀赋值会随环境传给子进程）。
> **默认值下与官方 `control_actor.py` 行为逐字等价**，不改任何现状。

---

## 10. 逃逸速度确定为 2 m/s —— 我们怎么办

官方规则已明确：被 UAV 感知到后，目标**以 2 m/s 躲逃**。官方 `control_actor.py`
目前仍是无条件 1 m/s，**只是尚未更新**（截至 2026-09-15 最后一次提交）。
本节把"2 m/s 到来之后"的账算清楚。压测开关见 §9 的 `ACTOR_V`。

### 10.1 2 m/s 会怎样 —— 真飞实测（本节全部数据来自真飞，非离线推算）

> **复现方法**：`world_min` 轻量世界，UAV 真起飞悬停 5 m，把**同一个目标**（`actor_3`）
> 钉在视野正中的 **14~15 m** 处往复，**只改速度一个变量**，每轮录 60 s 仿真。
> 轮前先用 `scripts/probe_actor_speed.py` 单独量速度，确认引导指令被精确执行
> （实测 1.000 / 1.971 m/s，不是"以为跑了 2 m/s"）。

| 轮次 | 目标实测速度 | 误差中位 | 单帧 <1m | 最长连续（含 track 外推） | 官方 15 s |
|---|---|---|---|---|---|
| 基线 | **1.000 m/s** | 0.57 m | **93.1%** | **16.4 s** | ✅ **PASS** |
| 2 m/s | 1.971 m/s | 0.70 m | 72.2% | 8.6 s | ❌ FAIL |
| 2 m/s + `PR_LAG_COMP=0` | 1.961 m/s | 0.78 m | 64.5% | 4.1 s | ❌ FAIL |
| 2 m/s + `PR_LAG_COMP=0.35` | 1.966 m/s | 0.77 m | 65.6% | 3.2 s | ❌ FAIL |

**三条读数**：

1. **误差 ≈ `目标速度 × 0.352 s`（横向项）+ 0.47 m（沿向噪声）。**
   实测分解（2 m/s、122 条真实观测）：横向 0.55 m / 沿向 0.47 m，**横向占 57%**。
   2 m/s 时横向项就有 0.69 m，再叠加沿向项正好压在 1 m 线上抖 ——
   这就是 1 m/s 稳过、2 m/s 擦线的原因。
2. **补偿不是解药**（第 5 行 vs 第 4 行是同版本单变量对照，见 §10.1.1）。
3. **15 s 的门槛比看起来高得多。** 它要求连续约 91 帧（15 s @ 6.1 Hz 仿真）
   **一次都不许断**：`0.92^100 ≈ 0.0002` ⇒ 单帧达标率得推到 **99.5%+** 才有戏，72% 离得很远。

#### 10.1.1 补偿为什么没生效 —— 速度估计只有真值的 31%

快照里 `xyz` 是**真正发出去**的值、`xyz_est` 是未补偿的滤波值，两者相减即实际补偿量。实测某帧：

```
xyz=[8.40, -1.05]   xyz_est=[8.41, -0.86]   ⇒ 只补了 0.19 m
应有补偿 = LAG_COMP 0.35 s × 目标速度 1.97 m/s = 0.69 m
```

差在感知自己的速度估计：

| 量 | 实测 |
|---|---|
| 速度估计 / 真值 的中位比值 | **0.31** |
| 速度估计绝对偏差中位 | 1.39 m/s（真值中位 1.97 m/s） |

⇒ **方向对、量只有三分之一。** 所以顺序是：**先修速度估计，再谈补偿**；
否则这个开关开了等于没开。

#### 10.1.2 天花板：换成真值速度也过不了

用**目标真值速度**做补偿（任何估计器都超不过的上限）扫描 `k`：

| 补偿 s | 误差中位 | 单帧 <1m | 最长连续 | 官方 15 s |
|---|---|---|---|---|
| 0.00 | 0.81 m | 66.7% | 3.0 s | FAIL |
| 0.15 | 0.65 m | 79.8% | 7.0 s | FAIL |
| **0.35** | **0.59 m** | **91.9%** | 7.0 s | FAIL |
| 0.50 | 0.62 m | 88.9% | **10.0 s** | FAIL |
| 0.70 | 0.74 m | 69.7% | 2.9 s | FAIL |

最优 k=0.50 ⇒ 最长连续 **10.0 s**，**离 15 s 仍差 5 s**。
原因是那条 0.47 m 沿向噪声**不随速度增长**（是测距/几何噪声），补偿消不掉它。
⇒ **单机在 2 m/s 下过 15 s 判据，是结构性的做不到，不是调参能救的。**

#### 10.1.3 一个白送的决定性对照：会动的挂、不动的满分

同一轮 2 m/s 数据里，另一个 actor（red）恰好**静止**站在路径端点上：

| 目标状态 | 误差中位 | 单帧 <1m | 最长连续 | 官方 15 s |
|---|---|---|---|---|
| 运动（2 m/s，目标 white） | 0.70 m | 72.2% | 8.6 s | FAIL |
| **静止（同一时刻同一节点）** | **0.47 m** | **98.4%** | **49.8 s** | ✅ **PASS** |

**同一个感知节点、同一段录像**，唯一差别是目标动没动。
⇒ 感知本身没问题，**是"目标速度 × 链路滞后"这一项在拖后腿**。

### 10.2 能翻盘的路 A：多机接力（威力最大，但要协同层配合）

官方 `score_cal.py` 里 `topic_arrive_time[actor_id]` / `count_flag` 是**按 actor 共用**的，
不是按 UAV 各算各的 —— **任意一架报中同一个目标，都能把 15 s 计时往前续**。
所以不必一架盯死，6 架轮着看到就行。

下表用**本次真飞实测的单帧达标率**做锚点（不是估算），计算"15 s 全程不断"的概率：

| 单机单帧 <1m（实测来源） | 单机 | 3 架 | **6 架** |
|---|---|---|---|
| 65.6%（2 m/s + 补偿 0.35，本节点当前版本） | 0.0000% | 2.3% | **86.0%** |
| 72.2%（2 m/s，旧版最好一轮） | 0.0000% | 13.9% | **95.9%** |
| 91.9%（2 m/s + 真值速度补偿，天花板） | 0.05% | 95.3% | **99.997%** |
| **93.1%（1 m/s 实测）** | 0.15% | 97.1% | **99.999%** |
| 98.4%（静止目标实测） | 23.0% | 99.96% | **100%** |

> 算法：单帧 6 架联合达标率 `1-(1-p)^6`，再取 91 帧次方（15 s @ 6.1 Hz 仿真）。
> ⚠️ **单帧独立假设，实际偏乐观** —— 真飞误差是时间相关的（目标横穿视线时误差大、
> 正对时小），不会像独立抛硬币那样均匀分布在 6 架之间。**把它当"上界"读。**
>
> 但方向是硬的：**2 m/s 下单机是 0.0000%，6 架接力能到 86~96%** ——
> 这是唯一能把 15 s 从"没戏"变成"有戏"的手段。代价是要队友在协同层配合。

### 10.3 能翻盘的路 B：把 track 外推用足（感知侧自己就能做）

官方判据只认**发出去的坐标误差 < 1 m**，不关心这个坐标是 YOLO 当场检到的、
还是 track 滤波/外推出来的。实测这一条**价值很大**：

| 轮次 | 仅真实观测的最长连续 | **含 track 外推** | 判定变化 |
|---|---|---|---|
| 1.0 m/s（white） | 10.9 s | **16.4 s** | ❌ FAIL → ✅ **PASS** |
| 2.0 m/s（white） | 3.0 s | 8.6 s | FAIL → FAIL（量不够） |

⇒ **本节点 1 m/s 那一轮的 PASS，是外推救回来的。** 外推期间坐标仍满足 <1 m，
说明 track 的 alpha-beta 预测在目标匀速直行段足够准。可做的优化：
拉长外推存活时间、在目标被遮挡/掉出视野时保持预测（但**不许无中生有**：
外推必须建立在最近一次真实观测上，否则会变成盲报，误差陡增反而打断连续计时）。

### 10.4 可执行的改动（按性价比排序）

1. **战术上别追 —— 悬停 / 慢速贴近，把目标压进 12~15 m。** 这条**最确定**，
   而且不需要改任何代码：§8.5 的逃逸触发要求"UAV 水平速度 > 1 m/s"，
   **悬停时第 ① 条不成立，整段逃逸判定被跳过** ⇒ 慢慢贴上去，目标根本不会跑。
   实测 **1 m/s 悬停档：93.1% / 16.4 s / PASS**；2 m/s 档：72.2% / 8.6 s / FAIL。
   > ⚠️ 代价是"追不上要跑的目标"，但那是**协同层**该决定的事 ——
   > 感知侧只需要在被观测时把坐标报准。
2. **多机接力**（§10.2）—— **官方机制允许的、威力最大的一条**。
   2 m/s 单机 0.0000%，6 架接力 86~96%。需要队友在协同层配合，让 6 架轮流指向同一目标。
3. **把 track 外推用足**（§10.3）—— 实测把 1 m/s 的连续时长从 10.9 s 救到 16.4 s，
   **是本轮 PASS 的真正来源**。属于感知节点内部就能做的改动。
4. **先修速度估计，再谈补偿**（§10.1.1）—— 当前估计只有真值的 31%，
   所以 `PR_LAG_COMP` 开与不开几乎无差别。修好它，补偿才有意义
   （天花板见 §10.1.2：k=0.50 时 91.9%，但最长连续仍只有 10.0 s）。
5. **降链路延迟本身 / 削那 0.47 m 沿向噪声** —— 这是唯一能把 92% 推向 **99.5%**
   （单机过 15 s 的门槛）的通道。滞后误差 = 目标速度 × 延迟，延迟砍半则该项直接砍半；
   而沿向噪声与速度无关，只能从测距/时标对齐上抠。**这是独立的一条工作线，收益最大也最难。**

> 🔴 **一句话结论**：**2 m/s 追上 15 s，单机做不到，别在这上面调参**；
> 要么「悬停 + 不触发逃逸」保持 1 m/s，要么「多机接力」。
> 本节点已把能做的都做了（补偿、外推、开关、契约），**剩下的缺口在协同层和链路延迟上**。

### 10.5 压测怎么跑（§10.1 那张表就是这套流程跑出来的）

```bash
# 最坏情况：全程 2 m/s
ACTOR_V=2 ./scripts/restart_stack_fly.sh

# 复现规则：平时 1 m/s，被追（catching_flag）后 2 m/s
ACTOR_V=1 ACTOR_ESCAPE_V=2 ./scripts/restart_stack_fly.sh
```

**复现 §10.1 的完整步骤**（关键在"只改速度一个变量"）：

```bash
# 1) 先单独量目标速度，确认引导指令真的生效 —— 别信标称值
python3 scripts/probe_actor_speed.py 3 20

# 2) 把目标钉在视野正中 14~15 m 处往复（起终点按现场几何改）
python3 scripts/guide_patrol.py 3 8 -2 8 10 2.0 0   # actor_id x1 y1 x2 y2 速度 模式

# 3) 该 actor 原来的随机游走控制器必须先杀掉
#    否则它的 10 Hz 会盖过引导的 5 Hz：目标"看起来在动"，速度却完全不受控（已踩）
kill <control_actor_m2.py <actor_id> 的 PID>

# 4) 录快照（60 s 仿真 ≈ 140 s 墙钟，RTF≈0.42）
REC_OUT=/tmp/snap.jsonl REC_DUR=60 python3 scripts/rec_snap.py

# 5) 按官方判据出结论
python3 scripts/judge_official_15s.py /tmp/snap.jsonl --target a3 --speed 2.0
```

> 📌 **`pgrep -f` 自匹配坑（踩过，废了一轮数据）**：把 `pgrep -f guide_patrol` 与真正的
> `python3 ... guide_patrol.py` 启动串写在**同一条 ssh 命令**里时，`pgrep` 会匹配到自己
> 所在的 bash 进程从而自杀，后续命令全部不执行 —— 表现为"提示启动成功但进程不存在"。
> **kill 与启动必须拆成两条独立命令。**

快照里 `xyz` 是**真正发出**的值、`xyz_est` 是未补偿的滤波值，两者都留 ——
可直接比对补偿前后的效果（见 §6.4）。

> ⚠️ **实测的轮间方差很大**（同一条件重跑，最长连续可差 2~3 倍）。
> 结论请写成"**N 轮里的最好/最差**"，不要拿单轮当定论。

---

## 11. 验证清单

```bash
# 0) 上机前置：双目相机（只读检查，不改任何文件）
python3 patch/add_stereo_camera.py --check

# 0b) 官方资产核对：模型 / 相机内参 / 标注映射是否都与官方一致
python3 scripts/verify_official_assets.py --gt ~/robocup_dataset_gt_v3/gt.json

# 1) 契约形状（不需要 ROS / 不需要仿真）
python3 scripts/check_target_report_schema.py

# 2) 离线重放（不上仿真也能验发布端逻辑）
python3 scripts/replay_target_report.py

# 3) 全栈起来后，看两路输出是否都在动
rostopic hz /coordination/target_report
rostopic echo -n1 /actor_green_info

# 4) 录一段快照，按官方判据出结论（这是唯一能说明"能不能拿分"的一步）
REC_OUT=/tmp/snap.jsonl REC_DUR=60 python3 scripts/rec_snap.py
python3 scripts/judge_official_15s.py /tmp/snap.jsonl --target a3 --speed 2.0
```

**预期数值**（本机 8vCPU / 8G / 无 GPU 条件下，供对照判断"是不是坏了"）：

| 条件 | 误差中位 | 单帧 <1m | 最长连续 | 官方 15s |
|---|---|---|---|---|
| 1 m/s 悬停观测 | ~0.6 m | 90% 以上 | 15 s 上下 | 能 PASS（1 轮里） |
| 2 m/s | ~0.7 m | 70% 上下 | 10 s 以内 | **必然 FAIL** |

明显低于这个区间 → 先查**发布频率**（`rostopic hz /actor_green_info`，掉到 1 Hz 以下
多是同时起了独立 YOLO 节点抢 CPU），别急着调检测阈值。

详见 `docs/部署检查清单.md`。

---

## 12. 合规提醒

赛事有**代码查重**规则：与他人代码相似度 >30% 扣总分 50%、>50% 取消资格。
本目录是团队自有实现，但**请勿把它、或仓库根的训练/示例脚本直接当作提交物模板**，
提交前确认所有外部依赖的许可证（`ultralytics` 为 AGPL-3.0，注意你们的提交方式是否触发其约束）。
