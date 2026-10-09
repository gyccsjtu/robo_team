# 替换模块清单与迁移接线

这份交接将主干已发现的问题对应到可复用的开发模块，供接手者在主干上逐步修改。候选文件集中在 `candidate/`；它们是同一仓库开发代码的快照，已经放入仓库供下载和比对，**尚未接入本分支默认运行链路**。

- 迁移基线：主干 `d4976a2cb4f4dc8212e094839d31f019a0a762d8`。
- 候选来源：远端开发提交 `f4050badc03373329c03923903a640b7859aee16`。
- 本文所称“可复用”指可以提取模块及接线；不代表覆盖大文件后即可运行，也不代表六机飞行验收完成。

`candidate/coordination/src/robocup_swarm/scripts/swarm_agent.py`、`swarm_manager.py`、`yolo_target_bridge.py` 和 `candidate/perception/perception_real.py` 是接线参考。它们包含不同阶段的授权、地图、机型和感知策略。**不要整文件覆盖主干的 agent、manager、perception 或 bridge。** 应按下列组别提取纯模块、保留主干需要的行为，并分别完成默认入口验证。

## 推荐顺序

| 阶段 | 迁移内容 | 完成条件 |
|---|---|---|
| 第一批，三个独立改动 | 雷达最终速度保护；最终执行输出和停车锚点；相机几何及跟踪时间 | 默认 `run_match.sh` 的真实 agent/perception 链路执行新逻辑；受保护结果直接到达唯一发布者 |
| 第二批，授权与证据链 | 任务授权 v2、官方消除、真实图像采样时间、报告适配、搜索证据 | 六机共享新 run_id，旧授权和旧帧不能执行；TTL/失联不清占用 |
| 第三批，地图与多机运动 | 观测地图、在线规划、机队运动、路线预约 | 未知/过期空间阻断，地图和变换版本受约束，全部机队运动证据新鲜 |
| 最后，有界脱困 | 停稳证据、退出候选、授权与真实运动预算 | 不降低全局保护标准；退出只在授权、地图、友机检查全部成立时执行 |

唯一发布进程锁可随第一批加入 Linux 入口。主干的日志目录和旧 watcher 生命周期问题适合直接修小段；开发场景 runner 不能整段替换比赛入口。

## 1. 雷达最终速度保护：优先做小范围接线

**主干问题：** 内置雷达守卫忽略部分后方角度，可能将后退请求重建为前进，旧扫描透传，EMA 稀释原始近障。默认运行使用 agent 内置逻辑，修改未启用的独立 radar 节点不能修复这条链路。

**可复用文件：** [radar_velocity_guard.py](candidate/coordination/src/robocup_swarm/scripts/radar_velocity_guard.py)。`guard_velocity()` 只依赖标准库，输入请求速度、实测速度、yaw、原始激光束和采样时间，输出 `velocity_xy/reason/clearance_m`。请求与实测速度均为世界 ENU 的二维有向向量；算法只缩小原请求，不重造一个朝前方向。除了请求方向，还检查实测动量的停车走廊。

**接线要求：**

1. 新增 `mavros/local_position/velocity_local` 的实测有向速度和原始 `header.stamp`；检查当前位姿和变换证据。标量估速、请求速度或上一条命令不能充当实测速度。
2. 激光束使用原始 `LaserScan.header.stamp`。接收 now 不得刷新旧扫描；检查全周角度覆盖、安装方向、`range_min/range_max` 和坐标系。无帧、旧帧、未来帧或非法数据不能按自由空间放行。
3. 友机/边界改向、速度限幅与加速度限制完成后，再做**最终雷达守卫**，随后立即编码、发布。后级不得把 STOP 恢复为非零。
4. 更新命令历史时记录最终命令。水平 STOP 不证明飞机物理停稳；垂直动作由独立限高、爬升锁、紧急保护决定。

依赖是现有 ROS 传感器适配与时间/坐标证据；纯守卫本身不依赖 ROS。接线参考：[候选 swarm_agent.py](candidate/coordination/src/robocup_swarm/scripts/swarm_agent.py)，重点搜索 `_radar_guard_velocity` 和 `_send_vel`。

**测试：** [test_radar_velocity_guard.py](candidate/coordination/tests/test_radar_velocity_guard.py)，重点是侧飞/后退、旧/未来扫描、实际前进时改成后退、边界恢复及 slew limiter 后 STOP 仍保持。开发默认半径、延迟和制动参数需要按当前机型标定；二维扫描不会自动覆盖扫描面外障碍、起降净空或横滚俯仰。

## 2. 最终执行输出与停车保持：编码器可复用，输出链需统一

**主干问题：** 默认位置输出传入未保护的原始高度/垂直速度，绕过最终爬升锁和恢复超时；固定姿态也丢失偏航。每帧重设当前位置为零速目标可能使停车锚点漂移。

**可复用文件：** [position_brake.py](candidate/coordination/src/robocup_swarm/scripts/position_brake.py)，接口和掩码说明见 [position_brake_v1.md](candidate/coordination/docs/position_brake_v1.md)。`PositionBrake.encode()` 把最终三维速度、yaw rate、当前局部位置和变换 ID 编码为标准 MAVROS `PositionTarget` 字段；停车锚点固定，明确垂直动作优先于高度保持。

迁移粒度是最终输出协议与入口统一，不能把编码器当 `_publish_pos_output` 同签名替换。预热、起飞、移动、停车和降落必须共用**唯一 `/mavros/setpoint_raw/local` 发布者**。需要 `mavros_msgs/PositionTarget`，并同步修改订阅记录脚本、话题检查和启动配置。原有爬升锁、恢复超时、限高和紧急垂直决策应先得到最终 `vz`，再交编码器；不能从未保护的 `target_alt` 重新生成高度命令。

传入新鲜有效的局部位置和变换 ID，失定位或变换变化不能保持旧锚点。编码器不验证定位新鲜性，也不证明飞机已经停稳。接线参考为 [候选 agent](candidate/coordination/src/robocup_swarm/scripts/swarm_agent.py) 的 `_publish_command`。

**测试：** [test_position_brake.py](candidate/coordination/tests/test_position_brake.py)，覆盖停车后漂移不移动锚点、真实 ROS adapter 的 ENU/mask、紧急垂直动作、缺定位时禁止错误高度保持。

## 3. 相机几何与跟踪时间：适合局部修正

**主干问题：** 图像位姿补偿重复除以时间差；离轴人体高度使用错误几何；丢观测后的跟踪时间不推进，累计产生超量漂移。

**可复用文件：** [camera_geometry.py](candidate/perception/camera_geometry.py)，以及 [perception_real.py](candidate/perception/perception_real.py) 中 `pose_at_stamp`、`Track.coast`、`Track.update` 的对应实现。提取正确时间插值、垂直射线求交和状态时间推进，不替换整份 perception。

`aligned_translation()` 接收带时间的两份位置采样，按图像时刻插值；`vertical_extent()` 用顶端射线与脚点上方竖线求交。调用方必须提供当前相机内参、光学坐标变换、尺寸和正确的地面假设；不可盲用开发相机标定。预测推进更新状态时间，独立保留 `observed_s`，预测不能冒充新观测。

**测试：** [test_camera_geometry.py](candidate/coordination/tests/test_camera_geometry.py)、[test_camera_track_evidence.py](candidate/coordination/tests/test_camera_track_evidence.py)。这组纯函数最适合先迁移，随后在当前相机安装与实际图像上核验坐标误差。

## 4. 任务授权、撤销与租约：整套迁入 v2

**主干问题：** TTL 到期清 owner 后重新分派；取消消息使用 `uint8=-1`，可能未发送便清内部状态。旧飞机仍在执行时会产生双重授权。

**可复用文件：** [task_authority.py](candidate/coordination/src/robocup_swarm/scripts/task_authority.py)、[search_occupancy.py](candidate/coordination/src/robocup_swarm/scripts/search_occupancy.py)，以及 [swarm_task.py](candidate/coordination/src/robocup_swarm/scripts/swarm_task.py) 的 LeaseManager 保留占用逻辑。接口按 [task_authority_v2.md](candidate/coordination/docs/task_authority_v2.md) 和 [search_lease_contract_v2.md](candidate/coordination/docs/search_lease_contract_v2.md) 接入，不能只复制一个类。

`TaskAuthority` 管理 offer/withdraw/refresh/ack/tick；`TaskGate` 在实际执行前检查授权。GRANT/STOP 的 schema 为 2，包含 run_id、机号、seq、generation、expires_s 和任务。ACK 含原始 sample_s、实际 xyz、速度和停止状态。授权代次不是地图 epoch。

必须成套迁入 manager 发授权和接 ACK、agent 收授权及最终执行门、真实停稳 ACK、搜索占用消费。每轮六机与 manager 共用非空的新 `ROBOCUP_RUN_ID`；旧 `/swarm/assignment` 仅诊断，新授权话题控制执行。

**TTL、失联、发出 STOP 都不释放锁或通道占用。** 核心连续核验新鲜实测速，而非相信单条 STOPPED 或已发送零速。候选 agent 结合 `velocity_local` 与已接受位姿差分的较大三维速度；依赖 [pose_rate.py](candidate/coordination/src/robocup_swarm/scripts/pose_rate.py)、[pose_quality.py](candidate/coordination/src/robocup_swarm/scripts/pose_quality.py) 和变换检查。停稳后旧目标锁仍保留，直到当前代次实际退出旧区域满足条件；异机不得通过到期抢占。

候选停止阈值、时间和退出半径是开发参数，不是官方安全标准。接口变更须更新版本及迁移说明。

**测试：** [test_task_authority_v2.py](candidate/coordination/tests/test_task_authority_v2.py)、[test_search_lease_v2.py](candidate/coordination/tests/test_search_lease_v2.py)、[test_authority_elimination.py](candidate/coordination/tests/test_authority_elimination.py)、[test_search_occupancy.py](candidate/coordination/tests/test_search_occupancy.py)。默认入口还需注入旧 run、旧代次、丢 ACK、TTL 到期、停稳但未退出等场景。

## 5. 图像新鲜性、跨机融合与裁判报告：证据端到端接入

**主干问题：** 重复推理同一张旧图、把 bridge 接收时间作为观测时间，再持续重发，会制造“新证据”。多机不同采样时刻直接平均也会混入目标运动误差；自定义 ActorInfo 与实际裁判消息包可能不兼容。

**可复用文件：** [visual_observation.py](candidate/coordination/src/robocup_swarm/scripts/visual_observation.py)、[fresh_person.py](candidate/perception/fresh_person.py)、[source_aligned_fusion.py](candidate/coordination/src/robocup_swarm/scripts/source_aligned_fusion.py)、[official_report_sources.py](candidate/coordination/src/robocup_swarm/scripts/official_report_sources.py)、[report_readiness.py](candidate/coordination/src/robocup_swarm/scripts/report_readiness.py)。接线参考：[perception](candidate/perception/perception_real.py) 与 [bridge](candidate/coordination/src/robocup_swarm/scripts/yolo_target_bridge.py)。接口见 [visual_observation_v2.md](candidate/coordination/docs/visual_observation_v2.md)、[visual_observation_v3.md](candidate/coordination/docs/visual_observation_v3.md)。

迁移粒度是 perception → bridge → manager/agent 的完整证据链。原图 `sample_s`、run_id、seq 和唯一 observation_id 必须一路保留；同帧重复处理、旧局重放、过期或未来时间须拒绝。仅加入 `VisualEvidence.receive()`，同时继续填接收 now，不构成修复。预测或补偿的位置应保留原观测时刻与身份；融合按来源时序处理，报告准入不能借用另一台相机的已就绪状态。

先迁必要的时间/去重与报告适配，再按当前目标与传感器验证准入阈值。候选增强人体验证依赖权重和机型，不应随大文件覆盖无意启用。对外报告以实际裁判包 `ros_actor_cmd_pose_plugin_msgs/ActorInfo` 为准；原始官方裁判保持只读，不能以本地改过裁判替代它来声称兼容。

**测试：** [test_visual_report_frame_dedup.py](candidate/coordination/tests/test_visual_report_frame_dedup.py)、[test_visual_observation.py](candidate/coordination/tests/test_visual_observation.py)、[test_official_judge_transport.py](candidate/coordination/tests/test_official_judge_transport.py)、[test_bridge_acceptance.py](candidate/coordination/tests/test_bridge_acceptance.py)、[test_official_report_sources.py](candidate/coordination/tests/test_official_report_sources.py)、[test_official_report_coast.py](candidate/coordination/tests/test_official_report_coast.py)、[test_blue_source_alignment.py](candidate/coordination/tests/test_blue_source_alignment.py)。

## 6. 红人槽与官方消除：局部逻辑加授权 STOP

**主干问题：** 内部红槽按几何发现顺序分配，却拿官方 actor 身份删除；内部计时判定消除后不再上报，官方可能仍要求观察。

**可复用文件：** [red_observations.py](candidate/coordination/src/robocup_swarm/scripts/red_observations.py)、[cooperative_tracker.py](candidate/coordination/src/robocup_swarm/scripts/cooperative_tracker.py)。显式设置 `CooperativeTracker(..., official_only=True)`；默认值不是比赛模式。manager 只按已核验的官方剩余清单消除目标，不能把内部确认时长当官方消除。

依用户提供的赛事组口径，只报告红色与坐标，不要求参赛代码区分官方 red1/red2。来源记录见 [red_matching_organizer_20261003.md](candidate/coordination/docs/red_matching_organizer_20261003.md)；它不等于官方发布新版。任一官方红人仍存在时，保留两个内部红槽；仅凭几何槽号不能认定某个官方身份消失。实际撤销飞行任务接 `TaskAuthority.eliminate_target()`，仍执行停稳和占用规则。

**测试：** [test_red_observations.py](candidate/coordination/tests/test_red_observations.py)、[test_official_tracker.py](candidate/coordination/tests/test_official_tracker.py)、[test_authority_elimination.py](candidate/coordination/tests/test_authority_elimination.py)。

## 7. 搜索覆盖与完成：纯完成函数可先用，覆盖账本成套接入

**主干问题：** 覆盖时间混用系统/仿真时钟；到点或派单被当已观察；缺官方清单、无任务或空 tracker 被误当比赛完成。

**可复用文件：** [search_completion.py](candidate/coordination/src/robocup_swarm/scripts/search_completion.py)、[search_observation.py](candidate/coordination/src/robocup_swarm/scripts/search_observation.py)、[camera_task.py](candidate/perception/camera_task.py)。接口见 [search_observation_v1.md](candidate/coordination/docs/search_observation_v1.md)。

`parse_actor_list/completion_state` 是可局部接入的纯函数，区分官方证据缺失与真的全部消除。简单时钟缺陷可先统一仿真时钟，无须为此导入整个覆盖机制。

完整覆盖迁移要把 processed_camera_frame 的 run/generation/cell/seq/image_s/sample_s、推理完成标志和几何证据，绑定当时任务授权；agent 发 search_feedback，manager 用独立原图缓存验证并 ACK。到点、收到授权、重复旧帧不能刷新覆盖，回执不释放已执行或退役任务占用。地面采样点及二维视线只证明那些采样观察，不证明整格无目标或三维视线清空。任务完成后的垂直降落仍需独立净空。

**测试：** [test_search_completion.py](candidate/coordination/tests/test_search_completion.py)、[test_search_observation.py](candidate/coordination/tests/test_search_observation.py)、[test_search_occupancy.py](candidate/coordination/tests/test_search_occupancy.py)。

## 8. 雷达观测地图和在线规划：成组迁移

**主干问题：** 未知空间当自由，历史自由证据没有寿命，规划可能从被移动的起点开始，已规划路线不再符合最新扫描。

**可复用文件：** [radar_observed_map.py](candidate/coordination/src/robocup_swarm/scripts/radar_observed_map.py)、[online_radar_planner.py](candidate/coordination/src/robocup_swarm/scripts/online_radar_planner.py)，依赖 [astar.py](candidate/coordination/src/robocup_navigation/src/robocup_navigation/astar.py)。接口：[radar_observed_map_v1.md](candidate/coordination/docs/radar_observed_map_v1.md)、[online_radar_planning_v1.md](candidate/coordination/docs/online_radar_planning_v1.md)。

ObservedMap 保留未知、自由和占用及原始证据时间；不把雷达近场盲区填成自由圆盘，不允许长自由射线立刻抹掉新近命中，证据过期返回未知。OnlinePlanner 仅从真实起点经连通观测区规划，复核连接段和最终命令停车走廊。

它们不是主干 `SlamGrid` 的同 API 替换。agent 地图和锁、不可变异步快照、manager 地图格式、搜索前沿语义与最终路线校验要一起调整。主干历史占用可作为额外阻断，不能转成自由空间证明。地图/变换 epoch 改变需要清旧证据；六机相同地图字符串不证明共同坐标已成立。

候选 `RADAR_START_CLEARANCE_FILE/fixture_seed` 来自开发场景启动净空验证。**开发地图与初始自由补丁不能用作正式随机地图先验**；未知启动净空未取得有效证明就不能填自由来恢复规划活性。当前仍是二维/yaw 投影，缺完整姿态、高度分层和扫描面外净空。不得删掉主干的倾斜处理后声称功能等价，也不能把丢扫描当自由。

**测试：** [test_radar_observed_map.py](candidate/coordination/tests/test_radar_observed_map.py)、[test_online_radar_planner.py](candidate/coordination/tests/test_online_radar_planner.py)。包括旧/未来/回放、扫描和位姿不对齐、近盲区、新命中保持、过期未知、连接前沿以及冻结计划不能回滚新证据。

## 9. 机队运动与路线预约：授权、地图、执行门共同接入

**主干问题：** 局部避让使用缺失或旧友机状态，任务授权未约束实际通道，多机路线可能互穿。

**可复用文件：** [fleet_motion_guard.py](candidate/coordination/src/robocup_swarm/scripts/fleet_motion_guard.py)、[route_reservation.py](candidate/coordination/src/robocup_swarm/scripts/route_reservation.py)、[allocation_geometry.py](candidate/coordination/src/robocup_swarm/scripts/allocation_geometry.py)。接口：[fleet_motion_contract_v1.md](candidate/coordination/docs/fleet_motion_contract_v1.md)、[route_reservation_v1.md](candidate/coordination/docs/route_reservation_v1.md)。

MotionCache 接 schema1、run_id、原始 sample_s、seq、world_enu_xy 位姿与实测速；全部成员新鲜才允许最终保护判断。只复制 protect 函数却输入旧 UavStatus 或命令速度，没有相同效力。

RouteAuthority 接受真实折线 offer，grant 绑定任务 generation 和 offer_id；RouteGate 核验连接段、实测停止段和最终命令。整个任务授权 v2、运动证据、在线地图和唯一执行器需一起成立。路线到期只禁执行，保留折线占用；核验物理停稳后才可退役旧折线，并以该机实测位置保留占用。目标锁仍等实际退出，不因路线退役释放。

候选默认间隔、时限、制动及起点误差都是开发参数。路线 wire 尚不携带地图 epoch，执行端还须约束地图/变换版本。等待或让行点必须有候选净空、连接段及有效时间证据，再检查其他预约；空集合不能猜安全点。集中调度与分布式一致性是另一个设计决策，换通信传输不等于分布式锁安全。

**测试：** [test_fleet_motion_guard.py](candidate/coordination/tests/test_fleet_motion_guard.py)、[test_route_reservation.py](candidate/coordination/tests/test_route_reservation.py)、[test_allocation_geometry.py](candidate/coordination/tests/test_allocation_geometry.py)。重点缺成员、旧局/重放、零请求下真实接近、TTL/重规划保留占用，以及停稳路线退役后仍由当前位置阻断。

## 10. 有界脱困：最后迁移，不直接缩小雷达保护

**主干问题：** 堵死后盲退、把起点移到自由格，或永久降低保护距离，可能进入未经证明区域。

**可复用文件：** [bounded_escape.py](candidate/coordination/src/robocup_swarm/scripts/bounded_escape.py)，以及 OnlinePlanner 的退出候选实现。接线说明：[bounded_escape_integration_20261007.md](candidate/coordination/docs/bounded_escape_integration_20261007.md)。

RestEvidence 独立核验 velocity_local 与位姿差分两种新鲜三维实测速、连续停稳、授权与时间单调性。choose_exit 使用同一观测与友机排除的正常/恢复两份网格，提出从真实起点持续远离阻塞命中的短连接路线；它只提出候选，不授予执行权。

依赖前述地图、机队运动、任务授权和路线授权。最终要有 TaskGate、RouteGate、真实位移预算与时限；续约不能重置预算，新近障碍覆盖旧退出承诺。不得仅永久将 guard 半径改小、删未知阻挡，或凭已发零速触发退出。

**测试：** [test_bounded_escape.py](candidate/coordination/tests/test_bounded_escape.py)、[test_bounded_escape_agent.py](candidate/coordination/tests/test_bounded_escape_agent.py)。重点未知/友机堵起点、无 grant 不执行、新命中取消窄通道、续约不能重置实测运动预算。

## 11. 唯一控制进程与其余入口问题

[publisher_authority.py](candidate/coordination/src/robocup_swarm/scripts/publisher_authority.py) 用模型名派生 Linux `fcntl.flock` 进程锁，可独立加到每机控制入口；[test_publisher_authority.py](candidate/coordination/tests/test_publisher_authority.py) 验证同模型拒绝重复、不同模型隔离。所有合作发布进程必须使用它，外部任意 ROS 节点不受该锁约束，所以仍需实际话题发布者检查。Windows 跳过 flock 用例，必须在目标 Linux 验证。

主干首次启动目录缺失、旧 hardcap watcher 读到下一轮 registry、裁判统计解析等问题应按当前入口局部修正。候选中的开发 runner、旧备份、场景文件和裁判适配记录是上下文材料，不应一并启用或覆盖正式裁判。

## 交付验证边界

本包提供源码、接口文档及测试，便于另一位开发者从同一仓库开始迁移。候选测试有纯模块、ROS 替身和提取的 adapter 方法；通过数量以本交接 runner 的实际输出为准。它们不能替代当前主干默认入口的集成运行，也不能替代 PX4/Gazebo 六机物理轨迹、接触和距离证据。

每阶段应在主干默认链路注入后障、旧扫描、丢定位、TTL、旧代次、旧局、重复图像和缺成员，验证最终输出确实遵守保护，再进入实际仿真。验收要分别记录碰撞、间距违规、死锁、超时、落点偏差、缺失结果和证据不足；缺证不能判 PASS。**二维避障、二维预约及开发启动证明均不构成三维起降或正式赛场安全验收。**
