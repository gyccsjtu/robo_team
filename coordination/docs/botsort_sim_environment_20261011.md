# BoT-SORT 开发仿真环境与对照方法

2026-10-11。目标是对 `997cfa006a41d60e58eff724c5762ca13a3d9cc3` 的人物识别后追踪替换进行开发对比。本文记录环境准备及主控试跑中发现的环境问题，不是追踪效果或比赛验收结论。环境准备脚本和隔离 fixture 不启动 Gazebo/PX4；主控实际启动的结果见文末。

## 本轮实测环境

Windows 上指定 `wsl -d Ubuntu-20.04`；默认发行版 Ubuntu-26.04 不适用。开始探测时三个 WSL 发行版均停止，Docker Desktop engine 未运行。进入 Ubuntu-20.04 仅唤醒发行版，进程清单没有旧 ROS/Gazebo/PX4/追踪节点，ROS11345、Gazebo11346和PX4常用端口没有旧仿真占用。

| 项目 | 本轮确认 |
|---|---|
| OS / ROS / Gazebo | Ubuntu20.04.6 / ROS1 Noetic / Gazebo11.15.1 |
| Python | 3.8.10 |
| 原 CPU 感知环境 | `/root/robo_team_build/vision_env`：Ultralytics8.3.0、NumPy1.24.4、OpenCV4.8.1.78、SciPy1.10.1、torch2.4.1+cpu |
| 原 CUDA 感知环境 | `/root/robo_team_build/vision_cuda_20261003`：torch2.4.1+cu121，其余包引用上述 CPU 环境 |
| 本任务新环境 | `/root/robo_team_build/botsort_env_20261011`：只读引用原包，新安装 lap0.5.12 |
| GPU | NVIDIA GeForce RTX4060 Laptop GPU；单进程2×2 CUDA小张量求和实测4.0。六个感知进程并发时有3个报device busy，现默认CPU |
| 显示 | WSLg `DISPLAY=:0`，X0 socket存在；未安装Xvfb |
| 权重 | 本工作树 `weights/best_yolo11n_bino_v1.pt`，5,471,059字节；未更换检测权重 |
| 运行插件 | `/root/robocup_runtime/stereo_20261002T140417Z`，Gazebo/GPS兼容/actor插件只读复用 |

新 venv 的 `.pth` 使用 `site.addsitedir` 引用旧目录，处理原 Matplotlib 的 namespace 配置，避免落到 Ubuntu 不兼容的 `mpl_toolkits`。没有升级 Ultralytics，也没有修改原环境。

WSL直连PyPI超时后，从Windows取得固定cp38 Linux wheel并离线安装：`lap==0.5.12`，SHA256 `72237a267e0245512a58a23a604f1a2590a52cfe43695e1ad84d69d1f51b1a0e`。wheel仅为开发依赖，不是检测模型。

`ImageMOT(('red','blue'),'u0')` 在真实环境中构造成功，空图/空框和下一张单red框均成功更新；单red框取得图像身份，GMC失败计数0。`ROBOCUP_REAL_MOT=1` 的17个图像追踪/世界绑定边界用例通过；这些是库兼容与边界验证，不是真实视频质量或飞行证据。结果见 `botsort_environment_verified_20261011.json`。

## 隔离资源

开发资源根为 `/root/robo_team_build/botsort_sim_20261011/environment_v2`：

- `PX4_Firmware/`：读取官方已编译的PX4；模型搜索目录为独立overlay，增加本任务的 `typhoon_h480_laser`。`build/px4_sitl_default/etc` 是物理私有副本，包含六实例端口修正；已编译二进制和库保持只读引用。
- `XTDrone/`：其它资产只读引用；`robocup/` 的官方脚本与数据为物理副本，供运行写日志，脚本哈希保持原样。
- `PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world`：本轮固定世界的物理副本。
- 新模型的GPS/MAVLink插件引用已验证兼容库；CPU ray和双目光学参数经项目现有 `check_official_sensor_spec.py` 与实际XTDrone原生模型比较通过。
- 同名PX4/XTDrone模型逐字相同。模型SHA256 `915b40a5019d0d492ea69574c333c823a6be2f832eea50626162a606f1593c90`；世界SHA256 `7723c7af704fe37117dfc5f2b08cfe66af4aef570716760f439e494269384ac1`。详情见 `botsort_runtime_manifest_20261011.json`。

`environment/` 是首次准备时因PX4只存jinja模板而失败的目录，保留为诊断；有效入口是 `environment_v2/`。原始官方树 `/root/third_party_official/{PX4-Autopilot,XTDrone_official}` 未修改。资产完整性和传感器参数检查通过，不代表GPS兼容层或整个模型已经与正式比赛环境等价。

本快照的 ROS1 消息和导航包在 `/root/robo_team_build/botsort_messages_20261011` 独立构建成功。没有沿用历史授权v2的生成消息。Windows检出时 `coordination/src/CMakeLists.txt` 是链接目标文本，构建脚本只在新构建区恢复catkin顶层链接。

## 准备和激活命令

本机现已准备好。overlay 脚本首次生成时自动复制私有 `etc` 并隔离六实例端口，不再需要额外端口步骤。已有完整 overlay 会只读验证和复用，不覆盖；已有不完整或哈希不一致的目录会报错并保留现场。venv 的 `setup` 仍拒绝覆盖已有环境。首次准备入口如下，均不启动仿真：

```bash
REPO=/mnt/d/a/.robocup/robo_team_botsort_20261011
python3 "$REPO/coordination/docs/setup_botsort_env_20261011.py" setup
python3 "$REPO/coordination/scripts/container/prepare_botsort_sim_runtime.py" \
  --output /root/robo_team_build/botsort_sim_20261011/environment_v2
bash "$REPO/coordination/scripts/container/build_botsort_messages.sh"
```

已有环境的明确只读检查：

```bash
python3 "$REPO/coordination/scripts/container/prepare_botsort_sim_runtime.py" \
  --output /root/robo_team_build/botsort_sim_20261011/environment_v2 --verify-only
```

旧 `--isolate-px4-ports` 入口保留，用于尚未复制私有 `etc` 的旧 overlay；对已修正配置只读验证。`probe_botsort_runtime_rebuild_20261011.py` 在 `/tmp` 独立 fixture 中使用官方三份启动文件、原生传感器模型及世界的只读副本，实测首次生成自动端口修正；重复调用、`--verify-only` 和兼容端口入口后34项文件/目录的哈希与修改时间均未改变；不完整目录报错且未改写，官方输入哈希未变。结果见 `botsort_runtime_rebuild_verified_20261011.json`。此核验没有访问运行中的 overlay，也没有启动任何仿真进程。

准备后的环境脚本：

```bash
source /mnt/d/a/.robocup/robo_team_botsort_20261011/coordination/scripts/container/botsort_sim_env.sh
```

它指定 `ROBOCUP_ROS_SETUP=/opt/ros/noetic/setup.bash`、`ROBOCUP_WS_SETUP=/root/robocup/robocup_ws/devel/setup.bash`、上述新 `COORD_WS_SETUP`、overlay的 `PX4_ROOT/XTDRONE_DIR`、新venv的PATH、实际actor运行库。默认 `PR_DEVICE=cpu`、`PR_CUDA_VISIBLE_DEVICES` 为空；用户预设值可覆盖默认。单进程CUDA探测成功不能证明六路CUDA并发成功。默认ROS11345、Gazebo11346；六机PX4/MAVROS按下表隔离，禁止与另一套飞控仿真并跑。

按完整环境source和 `run_match.ros_env` 最后的搜索根恢复步骤检查，`rospack find px4` 实测指向本任务 `environment_v2/PX4_Firmware`，`rospack find mavlink_sitl_gazebo` 指向其 `Tools/sitl_gazebo`。没有误用旧官方树来加载新机型。

本轮 preflight 曾遇到两个具体环境问题：Windows shell行尾CRLF；actor插件检查硬编码 `$HOME/catkin_ws/devel/lib`。主控已修复行尾与实际运行库路径检查。初始和overlay检查原始输出保留在 `botsort_preflight_existing_runtime_20261011.log`、`botsort_preflight_overlay_20261011.log`；后者是主控路径修复之前的记录，其最后一项失败不能冒称最终检查通过。官方actor/judge字节核验在两轮均通过，固定版本2783bdb。

六机开发启动由主控在最新preflight通过后执行；沿用严格检查，无正式条件旁路：

```bash
source /mnt/d/a/.robocup/robo_team_botsort_20261011/coordination/scripts/container/botsort_sim_env.sh
bash "$ROBOCUP_TRACKING_WORKTREE/run_match.sh" preflight
bash "$ROBOCUP_TRACKING_WORKTREE/run_match.sh" start --no-gui
```

`start --gui` 需要更多渲染资源；WSLg X0存在只证明显示基础可用，真实相机是否出帧仍须查原图时间戳和频率。停止只能使用本轮组件注册表对应的 `run_match.sh stop`，不得宽泛pkill。

旧 `run_single_swarm_product.py` 的单机停稳用例位于另一历史开发分支，使用授权v2和开发静态地图，不能当作997快照的目标识别/追飞启动入口。若六机资源不足，使用新独立单机调试编排、同一世界/传感器/权重/产品脚本；不能直接用该旧原型的“到达通过”替代本轮追踪效果。

## 旧新对照与数据

先录制同一相机的真实图像及原始时间戳、内参、MAVROS位姿/姿态；旧997算法与新追踪器使用同一批输入、同一检测权重和阈值。记录帧数、实际有效帧率、图像ID变更、空帧期间输出、重捕获时间和拒绝原因。YOLO推理和GMC耗时分开计时，首次CUDA初始化时间单列。

真实飞行另记录每机MAVROS状态、估计轨迹、setpoint、目标报告、桥绑定、目标状态、分配日志；独立审计进程记录Gazebo轨迹/actor真值及contacts，真值不回灌感知或控制。报告原始样本缺失和最大时间间隙，比较目标水平定位误差分位数、追踪持续时间、丢失后重获、最终距离和异常终止原因。需要真实人物匹配标注才可称ID切换率；红衣人匹配任意一个的官方口径不能被本地固定red1/red2标识覆盖。

`logs/latest` 下的感知/桥/manager/agent文本日志及 `source_manifest.json` 可作源码/事件索引；裁判成功必须另核对真实位置、时间与证据。碰撞完整证据缺失仍ABSTAIN，不能用tracker预测框或旧样本续播凑持续观测。

同世界不保证两轮飞行收到完全相同图像，actor随机策略、时钟RTF及控制路线都会变化，因此“同图像离线对照”和“端到端仿真”分开报告。一次仿真变化只能是局部效果证据，不能宣称六机比赛已交付。

## 实际启动后发现的端口配置缺口

主控首次六机开发试跑的 `01_scene.log` 中，六个Gazebo/PX4 TCP链路按4560..4565连接成功，但MAVROS为0/6 connected。PX4已有官方编译配置的Onboard链路仍按local14580+i / remote14540+i启动，仓库launch却配置MAVROS为local24540+i / remote34580+i。另 `6011_typhoon_h480.post` 固定从14558发送到14530，第2..6实例出现 `bind failed: Address already in use`；ctrlalloc版本post文件有同样的固定值。这是开发环境配置不匹配，尚未到YOLO追踪效果验证阶段。

主控停止该失败试跑后，准备脚本将本任务overlay的 `build` 链接替换为真实私有目录。整个 `px4_sitl_default/etc` 物理复制，`bin`、已编译Gazebo库等继续只读引用。现已将此步骤合并入首次准备，私有配置按六机改为：

| 链路 | PX4本地UDP | 接收端UDP |
|---|---|---|
| MAVROS/Offboard | 34580+i | 24540+i |
| payload | 14280+i | 14030+i |
| gimbal | 13030+i | 13280+i |
| GCS | 18570+i | 14550，允许共享接收端 |
| Typhoon post额外链路 | 14558+i | 14530+i |

30个PX4本地UDP端口在六实例中无重复；标准模拟器TCP仍4560+i。新 `px4-rc.mavlink` 与Typhoon post通过 `sh -n`，原官方三份启动文件的SHA256前后完全相同。详细表与官方哈希保存在 `botsort_px4_port_manifest_20261011.json`。只生成开发配置，没有由环境子任务启动进程。

主控的六个spawn include已逐机注入 `udp_gimbal_port=13030+i`。只读核对当前官方源码：`mavlink_interface.cpp` 第38行附近仅在 `hil_mode_` 分支创建SDK/QGC UDP socket，`gazebo_mavlink_interface.cpp` 第581行附近仅在HIL分支调用 `pollFromQgcAndSdk`。当前SDF为 `hil_mode=false`、`use_tcp=true`，固定SDK14540没有用于此轮SITL链路，无须额外修改。原固定14560是UDP备用值，TCP模拟器链路为4560+i。

## 主控实际试跑状态

第二轮重启后，6/6 MAVROS连接和6/6相机出帧已实测成功；六个CUDA感知进程并发时有3个报 `device busy`，该轮由主控停止。此前单CUDA张量及追踪器构造验证不能覆盖这一并发失败。

第三轮 `183813` 使用CPU默认，启动检查通过；Gazebo在仿真时间1790.892附近发生SIGSEGV，独立observer刚启动、事件数为0。该轮缺少可用的目标追踪/定位/碰撞结果，无法判断BoT-SORT实际追踪效果，不能将启动检查或相机出帧认定为视觉效果通过。后续开发试跑若调整actor插件，须单独记录环境差异与用途。

## Gazebo 崩溃与独立视觉采集备选

只读内核日志显示18:39:48故障指令在 `libOgreMain.so.1.9.0`，按内核提供的ELF地址0x1da094查询本机库符号为 `Ogre::EdgeListBuilder::buildTrianglesEdges(...) + 916`。故障发生在OGRE几何处理代码；没有core或完整backtrace，不能认定ActorCollisionsPlugin是已经证实的根因。有限深度检查 `/root`、`/tmp`、`/mnt/wslg` 未发现core，未见OOM kill。诊断记录为 `botsort_gazebo_crash_20261011.json`。

主控授权后准备的 `/root/robo_team_build/botsort_sim_20261011/environment_tracking_dev` 是独立开发场景，世界中仅移除了六个actor的 `libActorCollisionsPlugin.so`。XML结构核验确认其余元素全部保留，包括skin、animation、actor_cmd_pose、建筑；飞机、传感器、检测权重和私有六机启动端口保持一致。原 `environment_v2` 和官方树未改。新世界SHA256为 `7c7726083bf40cc1d5e52e43dc85ff63548eefcd8012d385e5b1175599c89584`；`tracking_dev_manifest.json` 明确 `formal_match=false`，该场景只用于运动人物视觉数据采集，不用于碰撞、导航或正式比赛验收。

准备或只读复查命令（均不启动仿真）：

```bash
REPO=/mnt/d/a/.robocup/robo_team_botsort_20261011
python3 "$REPO/coordination/scripts/container/prepare_botsort_tracking_dev.py"
python3 "$REPO/coordination/scripts/container/prepare_botsort_tracking_dev.py" --verify-only
```

主控启动前在新shell选择该场景，并用独立日志目录：

```bash
export PX4_ROOT=/root/robo_team_build/botsort_sim_20261011/environment_tracking_dev/PX4_Firmware
export XTDRONE_DIR=/root/robo_team_build/botsort_sim_20261011/environment_tracking_dev/XTDrone
export RUN_DIR=/root/robo_team_build/botsort_sim_20261011/logs_tracking_dev
export PR_DEVICE=cpu
export PR_CUDA_VISIBLE_DEVICES=
source /mnt/d/a/.robocup/robo_team_botsort_20261011/coordination/scripts/container/botsort_sim_env.sh
```

这些设置不等于场景已经稳定或相机已经录制。是否解决崩溃，及人物运动图像、检测框/时间戳和旧新算法的实际差异，仍须由后续采集确认。

## 第五轮备选：只关闭场景阴影

第四轮移除actor碰撞插件后仍在sim1801.596发生SIGSEGV；稀疏采样78帧中没有目标，不能用于人物追踪质量对比。因此移除碰撞插件没有解决已观测的崩溃。

只读确认原 `environment_v2` 世界的 `scene/shadows=1`、sun的 `cast_shadows=1` 后，准备了独立 `environment_tracking_no_shadows`。它以原v2为基准，只将 `world/scene/shadows` 改为 `false`，保留六个ActorCollisionsPlugin；灯光的投影标志、颜色、强度和位置未修改。XML结构对比确认除这个阴影设置外全部相同，传感器模型和私有六机端口也保持一致。新的世界SHA256为 `9f25040d4d31086dc35010dce55e705fbfd937083f4d9175e791171e506e6422`，差异记录为 `botsort_tracking_no_shadows_manifest_20261011.json`。

这是针对已观测OGRE几何故障的开发隔离试验，仍需实际运行确认稳定性。阴影渲染与正式场景不同，会影响相机图像与检测结果；旧新追踪器必须用该场景的同一段录像比较，结果不可推广为正式光照下的效果或比赛验收。

```bash
REPO=/mnt/d/a/.robocup/robo_team_botsort_20261011
python3 "$REPO/coordination/scripts/container/prepare_botsort_no_shadows.py"
python3 "$REPO/coordination/scripts/container/prepare_botsort_no_shadows.py" --verify-only
export PX4_ROOT=/root/robo_team_build/botsort_sim_20261011/environment_tracking_no_shadows/PX4_Firmware
export XTDRONE_DIR=/root/robo_team_build/botsort_sim_20261011/environment_tracking_no_shadows/XTDrone
export RUN_DIR=/root/robo_team_build/botsort_sim_20261011/logs_tracking_no_shadows
export PR_DEVICE=cpu
export PR_CUDA_VISIBLE_DEVICES=
source "$REPO/coordination/scripts/container/botsort_sim_env.sh"
```

以上命令仅准备和激活环境，不启动或停止仿真；原v2及前一去碰撞插件场景均保留。
