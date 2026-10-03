# 给队友的代码交接：六机雷达协同与共享 GPU

更新时间：2026-10-04（北京时间）。仓库 `gyccsjtu/robo_team`，分支 `codex/radar-coordination-20261003`。用户要求暂停仿真后上传，本轮仿真及取证进程已经停止。

**03:00定时恢复后的增量见[队友雷达整合记录](teammate_radar_integration_20261004.md)。**
下文保留暂停上传时的结果；当前已取队友`uav-single-link-20261003`修复并恢复共享GPU实飞。
启动快照v1.15已跑满600秒，绿色、红色和蓝色3/6消除，也有快餐店机体接触，
完整证据见[run10归档](validation/shared_gpu_run10_teammate_v115/README.md)。仓库另修定位时效竞态、
脚下净空保留、红色剩余槽误删与追踪时序，并准备局部2.2m高度试验。
低高度短轮v1.18已消除两个红色并越过旧卡点，167秒主动结束后修白色激活阈值；
见[短轮归档](validation/shared_gpu_run11_v118_short/README.md)。
v1.19在340.660秒消除白色，352.312秒为修丢画面接替主动结束，最终1/6；
见[run12归档](validation/shared_gpu_run12_v119_camera_loss/README.md)。当前代码397项测试通过，
v1.20新增实际观察者优先、停稳后接替和每张原图仅上报一次，但307.932秒仍0/6。
现场确认机头转向不能带动云台：插件固定ID/端口与六机不符。v1.21独立覆盖库已构建，
启动连线schema2见[云台接口](radar_gimbal_wiring_v2.md)，最新402项测试通过。
首次新库起栈在ROS初始化报ENOMEM、未起飞，已自动收尾；限制客户端线程池后继续验证。
完整六目标与云台实际响应仍未完成，不把构建通过视为实飞修好。

**这是一版已经能启动六机、运行协同搜索、视觉上报和开发裁判的联调代码；还不是“不撞墙、六目标全部消除”的完成版。** 本次上传保留失败证据，不把途中运动、测试通过或目标跟踪成功当作比赛完成。

## 1. 已完成哪些代码

| 内容 | 当前实现及入口 | 验证范围 |
|---|---|---|
| 六机实际执行链 | `swarm_manager.py`、`swarm_agent.py`、`swarm_task.py`；逻辑/ROS编号 `uav_1..6`，Gazebo模型 `typhoon_h480_0..5` | 已多次实际启动六路PX4/MAVROS、解锁起飞并移动；不是仅合成数据测试 |
| 任务授权和交接 | 新执行话题 `/swarm/authorized_assignment`；统一新 `ROBOCUP_RUN_ID`，授权代次、ACK、停稳交接和唯一目标持有者 | 已接manager/agent及边界测试。旧 `/swarm/assignment` 为诊断，不作为执行依据 |
| 雷达在线规划 | `radar_observed_map.py`、`online_radar_planner.py`，未知区域不直接当自由区；观察前沿、路线授权及拐角直达检查 | 修过拐角捷径造成的卡死，旧卡点有真实越过记录；其他卡点未全部解决 |
| 雷达最终速度约束 | `radar_velocity_guard.py`，检查请求方向、实测惯性、时效及制动距离 | 保持agent唯一setpoint发布者；二维扫描约束不能证明顶棚/屋檐可通过 |
| 追踪接近与追逃 | 新鲜视觉目标允许从15m以外接近；移动盘旋目标更新及路线连续性；逃跑追踪上限2.6m/s | 已接执行链。尚未证明所有目标都能连续定位15秒 |
| 共享GPU推理 | `perception/shared_inference_service.py`、`shared_inference_client.py`、`perception_real.py` | 一个GPU服务供六客户端；正常客户端不加载CUDA。共享服务故障时本地CPU回退；真实正负帧7项测试通过 |
| 绿色误检过滤 | `perception/person_verifier.py`，原图上绿色框须与COCO人体框重合；共享/本地执行一致 | 已拒绝取证垃圾箱框；样本真人保留。本轮绿色正例不足，不能称全场准确 |
| 红色赛事组口径 | 对外统一 `/actor_red_info`、`cls=red`及坐标；开发裁判匹配任一剩余红色真值 | 实际确认过单个红色消除。内部red1/red2是几何轨迹槽，不是官方身份 |
| 平台副本修复与取证 | 演员瞬态服务失败重试、30秒等待循环让出CPU；原生机体contacts观察、真实轨迹、原图时间对齐精度和相机帧保存 | 原官方文件不直接改；隔离副本和源码SHA记录在scene/flight清单 |

控制纯Python部分和ROS接入均在本仓库，没有把早期误选的 `radar_product` 当作成果。单机用例与六机合成自测也没有代替后面的真实六机记录。

## 2. 当前默认试验配置与实际效果

- 城市巡航基础/目标上限为MAVROS局部 **3.0m（v1.13）**；硬高度护栏4.1/4.3/4.5m保持。局部高度不等于真实世界高度。
- 在线地图 **0.25m、720×480（v1.14）**，保留1.2m雷达保护半径、制动参数，以及2.5m路线净空/机间配置。对角量化回放能解开一个起点，但不能证明现场所有起点已解卡。
- 单EKF **v1.12** 在隔离PX4启动副本初始化前配置；六机解锁前读回四参数。它减少了多实例选择的变量，**没有解决碰撞后的失控**。
- 绿色同帧人体核验 **v1.11**，权重 `weights/yolo11n_person.pt`；其余颜色仍用原颜色模型和原输出判决。
- 短时官方上报预测窗口 **v1.10** 使用既有1.5秒coast，原图时间不刷新；超期上报不延长，未更改裁判的连续15秒判据。
- 物理更新率当前实跑显式40Hz、步长0.004s；这是开发仿真减速，传感器参数未改。Gazebo仍软件渲染，GPU仅负责共享推理。

**最新实飞明确反证“已经不撞墙”：** 3.0m巡航+0.25m地图一轮运行87.604仿真秒、最高真实高度4.139939m、0/6消除；6号机仍有3条与加油站的非地面机体接触。首次接触时机体中心高度3.309375m。当前高度试验不是已完成的顶棚修复。

## 3. 可核对的实测与测试

完整核心测试最近一次 **370项通过**，包括新增对角障碍量化回归，近于保护半径的障碍仍阻塞。真实共享推理测试 **7/7通过，无skip**，覆盖正/负真实帧、本地共享数值一致、六客户端及客户端无CUDA。测试不证明飞行无碰撞。

| 证据目录（本页旁 `validation/`） | 实际结果 |
|---|---|
| `shared_gpu_run07_green_false_positive_ekf_failure/` | 397.236秒，1/6红色消除；6号机加油站碰撞及后续定位异常；最高4.143483m |
| `shared_gpu_run08_single_ekf_canopy_failure/` | 301.536秒，1/6红色消除，消除用时30.856秒；1号机再次碰加油站及失控保护；最高4.344350m |
| `shared_gpu_run09_fine_grid_paused/` | 用户要求暂停时87.604秒，0/6；降低高度和细化地图后仍有6号机接触，原始中断结果保留 |
| `shared_gpu_single_ekf_v112_checkpoint/` | 约186秒途中检查，后来同轮发生碰撞；不能单取这份途中证据宣称成功 |

原始 `judge.log` 的 `actor_N is OK` 和剩余目标反馈才作为消除证据；`tracking success`、本地确认进度或“六机移动”不是消除。`result.json` 的原始中断字段没有改写为PASS；人工物理汇总单独记录。接触记录含起飞前落地腿接触，汇总须与轨迹时段及实际碰撞体对齐。

## 4. 尚未完成，队友优先看这里

1. **屋檐/顶棚等三维盲区。** 机载雷达扫描面高于机体中心0.2m，已有扫描越过约4m顶棚而机体碰边缘的证据；新低高度轮也接触了模型。需要从感知及飞行包络真正处理，不能仅扩大水平半径或降低一次高度就宣布修好。
2. **部分飞机长时间卡在起点净空/未知空间。** 细栅格已实装，仍需核查近盲区证书、量化、位姿/扫描时效和执行段。遇到未知不能直接清成自由格。
3. **正确目标区分、坐标准确度与持续上报。** 绿色本轮没有足够真人正例；蓝色某些可见人形上报与对应裁判真值相差约142m。人体框不是对应赛事目标身份的证明，不应无证取消运动筛选。白色等目标曾多次被发现，却没有完成持续消除。
4. **完整六机六目标、600秒结果。** 尚无一轮完整6/6；本次最后一轮0/6。中断后的1/6或六机起飞不是成品验收。
5. **定位异常与恢复。** 大坐标跳变时旧补偿会累加偏移，缺少独立确认；单EKF不保证碰撞后估计器健康。
6. **正式比赛适用性。** 自身相机姿态仍取Gazebo link pose，开发出生净空检查为近似；完整三维路线证明、官方环境等价性和此自身姿态来源的允许性未完成核实，`formal_competition_pass`保持false。

下一步建议先解决实际机体接触和卡点，再解决正确目标的<1m持续15秒上报，最后跑完整六机600秒并对齐裁判、轨迹、接触和执行源码。暂停期间无需继续仿真。

## 5. 接口与规则依据

任务授权见 [task_authority_v2.md](task_authority_v2.md)，确认视觉消息为schema_version=2。全系统使用同一个新运行ID，不能混用旧授权或仅靠超时判旧机已停。路线/目标交接仍依停稳与退出证据；接口草案不当作冻结版。

官方规则核验来源：[2026赛事PDF](https://rcccaa.drct-caa.org.cn/image/file/20260911/1789106295983914.pdf)，已核对本地SHA256：`622bb59b4aea12870be38f9f86d56ef742c210d001ff2264fa509f4fbc86dd41`。六机、六目标、600秒、限高6m和坐标误差/连续15秒按核验记录执行。红色任一真值匹配是**用户转述赛事组说明**，详见 [red_matching_organizer_20261003.md](red_matching_organizer_20261003.md)，不声称已有公开新版裁判。本地开发裁判适配与原文件SHA均记录。

## 6. 取代码、构建与启动

```bash
git fetch origin
git switch codex/radar-coordination-20261003
git pull --ff-only origin codex/radar-coordination-20261003
```

该分支为联调版，不覆盖主干。当前机器的环境与共享GPU启动命令见 [city_swarm_quickstart.md](city_swarm_quickstart.md)。这份说明不自动安装另一台机器的ROS/Gazebo/PX4及资源。

在已有ROS Noetic机器、仓库根目录构建：

```bash
TEAM_BUILD_DIR=/root/robo_team_build/codex_authority_v2 bash coordination/scripts/build_isolated_swarm.sh
source /root/robo_team_build/codex_authority_v2/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s coordination/tests
```

构建脚本需要官方ActorInfo消息包，默认路径或 `ACTOR_MSG_SOURCE`见脚本。当前运行依赖完整runtime `/root/robocup_runtime/stereo_20261002T140417Z`、CPU环境 `/root/robo_team_build/vision_env`、CUDA环境 `/root/robo_team_build/vision_cuda_20261003`及官方资源。换机器先核准这些依赖；不要照抄缺失路径或同时启动会全局清场的脚本。

详细修改过程见 [implementation_progress_20261003.md](implementation_progress_20261003.md)；其中“当时正在运行/待验”的历史文字应结合本交接和最终失败归档阅读，不能当最新完成证明。
