# 协同代码修复进度（2026-10-03）

仓库：gyccsjtu/robo_team，基线 86eb484，开发分支 codex/radar-coordination-20261003。

## 当前检查点（以下旧里程碑为历史记录）

run19在当前软件渲染与确认观测接入后的六箱场景复验120.096s，通过开发观察：最小机间距6.90745m、最高5.27698m、最小箱体距离1.61875m，无机体碰箱，2303条轨迹、最大间隔0.132s；六机均提交路线并有真实raw命令证据。执行源码随结果存档validation/six_obstacles_run19，仍不代表演员追踪、完整碰撞、正式城市或比赛通过。

长时复验run17在120秒中出现uav_6真实机体碰箱，原始失败及离线轨迹/估计位姿对比保留。停止前位姿估计误差不足约0.1m，连续零水平速度仍缓慢漂向建筑；因此执行器改为单一raw setpoint发布者，零水平运动时锁定停止位置，使用PX4位置环保持。接口见position_brake_v1.md。run18同样120秒六箱复验通过原型观察：最小机间距6.7710m、最高5.2925m、最小箱体距离1.53085m，无机体碰箱，2310条真值采样、最大间隔0.112s。每机原始raw命令及实测运动另有executor_telemetry记录。路线刷新先STOP且实测停稳，新代次再获原任务，本轮11次刷新、15次停稳交接；不能凭路线TTL撤占用。

在线雷达地图已实际驱动未知阻塞A*，规划包含友机实测位置和保留路线；manager串行授权实际折线，agent只提交匹配当前代次与规划票号的获准路线。非空六箱场景run14/run15通过原型观察，run15最小采样间距7.3948m、最高5.3299m、最小箱体净空1.6015m、六机路线提交36/35/30/41/29/37次，1154条真值采样、最大间隔0.164s、无提前完成。结果和执行源码独立存档；全球碰撞仍ABSTAIN，正式比赛false。run13失败原始记录保留，声呐感测虚拟碰撞解释见其followup_audit，不改写旧结果。

视觉观测接口v2已接实际感知/桥/manager：保留相机原始时间和置信度，拒绝旧运行/重放/预测冒充新图像；修复red1→t5、red2→t4、官方cls=red。visual_ros_run04使用真实ROS消息与合成观测通过，不能证明真实YOLO命中。官方非空清单→空清单立即封闭忙碌机队；不再凭本地15秒推断消除、25秒推断瞬移或缺少误差证据判达标。在线绕飞逐段走规划和预约，不再直接给未获准圆周速度。

WSL完整coordination unittest 277条通过，旧运动测试已修复过时mock并检查实际终点。catkin三包构建成功，官方ActorInfo使用原消息包而非不兼容自定义类型。相机使用真实CameraInfo，零图像时间不伪造，原时间插值的量纲错误已修复。根目录新增run_radar_swarm.sh可直接执行当前隔离开发产品；旧入口移除宽泛pkill和他人/home路径。

真实相机开发链run21已得到人物像素、实际感知观测、实际桥检测/官方坐标及真实manager目标登记，正常退出；无飞行agent，不代表追踪授权或15s裁判消除。run20退出线程错误保留，新增ROS关闭流程后run21未复现。修复WSL D3D12人物不可见（软件渲染）、历史位姿采样、重复图像跟踪、人物AR门限及根启动身份/相机选择。manager/agent生产接入v2.1 confirmed_visual_observation，复验run/序号/原图时间，旧runless目标话题仅诊断；跨机延迟观测不能覆盖更新目标。最新286条coordination单测与桥自测通过，visual_ros_run05验证实际管理器新接入（合成输入，不是实体飞行）。

继续项：真实相机六机飞行/演员/裁判集成、安全等待与失败接替、旧3m出生队形兼容、正式随机城市长时覆盖和三次评估。不得把开发六箱场景当正式成品验收。

## 六机接入里程碑

run08实际六机agent在线射线地图发布已核验，保留未知与过期信息；最小采样间距5.43m、最高4.99m。地图尚未驱动实际规划，真实障碍/姿态高度层仍需继续，接口见 `docs/radar_observed_map_v1.md`。

已将几何候选筛查接入实际TaskAllocator/manager，并修复无法分配/缺裁判反馈时的提前完成广播；严格解析官方清单，错误文本不能变成空结果。run07六机持续搜索60s，最小采样间距5.4074m、最高4.9971m，无提前完成消息；证据在 `docs/validation/six_search_run07/`。筛查不是最终路径预约，仍须继续完整地图/视觉/通行接入。

已补充实测机队运动接口v1与最终友机制动保护，并在正常manager拍卖前检查完整机队新鲜状态。run04/run05六机真实搜索原型观察均通过，最小采样间距5.3617m/4.8477m、最高5.0095m/5.0673m；保护确实触发，完整证据已存档。仍不能宣称几何通行、完整搜索或正式比赛验收完成，详见 `docs/six_swarm_search_progress.md`。

后续六机正常拍卖飞行已真实运行60s，六机均起飞并执行授权搜索，但最小采样间距2.0797m，结果失败；详情与证据见 `docs/six_swarm_search_progress.md`。已修复MAVROS参数列表未就绪导致飞控设置静默失败，新增pull/set/get读回流程。下一阶段必须修正实际分配与几何通行，不能将起飞成功替代协同验收。

新增由实际PX4构建产物派生的机队接线和六份雷达模型生成，解决旧fleet端口与当前环境不一致。`scripts/six_radar_connectivity.py` 已在独立空世界验证六个真实PX4/MAVROS、六路定位及512束雷达，逐机定位/传感器帧隔离；不解锁、不起飞。run06完整执行源码/模型/启动日志证据位于 `docs/validation/six_radar_connectivity_run06/`，复现命令见 `docs/six_radar_connectivity_quickstart.md`。这不是六机协同飞行验收；实际manager/agent六机搜索、在线地图和几何预约仍须继续。

## 本次实现

1. 实际 swarm 搜索租约 v2：超时报警保留 owner/ASSIGNED 和 manager 活跃任务记录，禁止直接抢占超时格。未收到新状态不会续租；原持有者恢复新鲜状态可延长原租约。普通拍卖能继续给其他空闲机分配其他格。
2. 实际 swarm_agent 最终雷达速度保护 v1：纯 Python 核心，不新增 setpoint 发布者。对请求和实测运动方向检查走廊，按制动距离限制速度；无帧、过期、非法距离/配置、扫描覆盖不足、位姿或实测速度过期时水平零速。使用消息采样时间，不以接收时间掩盖过期。
3. 雷达检查放在网格、边界恢复、普通加速度限幅之后，最终受限指令同步写入持续发布缓存。垂直控制保持原实现，不能据此宣称起降三维安全。
4. run_match 无雷达模式默认关闭雷达保护；雷达模式默认打开。根目录带雷达的一键脚本显式 RADAR_GUARD=1，保持该模式的缺帧停机语义。

接口见 radar_velocity_contract_v1.md、search_lease_contract_v2.md。尚未更改 SearchAssignment 的 ROS 字段或旧 navigation JSON schema1。

## 验证

- 新增 12 条雷达单测（含执行实际 agent 薄接入与最终发布方法）与 3 条真实 swarm 搜索租约单测，均通过。
- WSL Ubuntu-20.04，source /opt/ros/noetic/setup.bash 后，全套 coordination/tests 184 条：181 通过，3 条错误。独立导出未修改的 86eb484 到 /tmp/codex_robo_team_baseline_86eb484 复测 169 条：166 通过，同样 3 条错误。错误均为旧 test_uav_motion 尝试 mock 当前节点不存在的 replan_route；不是本修复引入。不能称全套绿灯。
- Windows 可运行纯 Python 新测试；全套受 rospy 缺失限制。swarm_agent/manager 的 py_compile 通过。run_match.sh 的 WSL bash -n 通过。
- swarm_task 内置 __main__ 自测原有旧栅格尺寸断言与当前 GRID_SIZE_M=7 不一致，首项即失败；本次没有借此修改生产搜索格尺寸。
- 此仓库尚未启动飞行，本修复未获单机/六机实飞验收。旧 radar_product 的到达证据不可作为本仓库飞行证据。

可复现命令（WSL）：

```bash
source /opt/ros/noetic/setup.bash
cd /mnt/d/a/.robocup/robo_team
python3 -m unittest discover -s coordination/tests -p test_radar_velocity_guard.py -v
python3 -m unittest discover -s coordination/tests -p test_search_lease_v2.py -v
python3 -m unittest discover -s coordination/tests -p 'test_*.py'
bash -n run_match.sh
```

## 未完成且必须继续

SearchAssignment 的运行标识/授权代次、目标唯一锁、停止确认、失败接替和几何通道占用尚未统一。覆盖标记和转追踪任务仍能绕过搜索格 owner 生命周期，不能把局部租约保留称为完整安全预约。新雷达制动参数须用本仓库实测标定；本版输出零速不证明已物理停止。

下阶段先做任务/目标授权接口及生命周期边界测试，再薄接入 manager/agent；随后用隔离且可移植的启动入口跑本仓库代码。旧根目录一键脚本仍有硬编码路径和宽泛进程清理，当前不得直接在共享 WSL 环境执行。正式比赛、碰撞和完整六机搜索均未判 PASS。

## 后续实现：Swarm 任务授权 v2

已将纯 Python TaskAuthority/TaskGate 接入上述实际 manager/agent：新 JSON 话题带运行 ID、逐机序号和授权代次；旧 SearchAssignment 话题仅诊断，不能直接驱动 agent。同一目标只获一份执行授权；已持锁的目标不会再派盘旋备份机，也不会由搜索 agent 私自启动盘旋。

换任务必须STOP，并由核心独立累计新鲜速度样本连续1s停稳，再接收STOPPED；停止后旧锁保留，直到原持有者新鲜位置证明离开旧任务点。TTL、单条声称停稳、失联与其他飞机检测均不能替代这些证据。该机制已接入ROS，几何通道和三维路径证明仍未完成。

规划请求增加代次/请求序号保护，任务切换或旧规划迟到不能提交旧路径；相同在途请求合并，避免频繁请求使后台规划无法提交。任务完成广播后仍持续发布停止设定点，避免在空中直接退出OFFBOARD发布循环。

构建命令（WSL，源码只读，输出独立）：

```bash
TEAM_BUILD_DIR=/root/robo_team_build/codex_authority_v2 bash /mnt/d/a/.robocup/robo_team/coordination/scripts/build_isolated_swarm.sh
source /root/robo_team_build/codex_authority_v2/devel/setup.bash
python3 /mnt/d/a/.robocup/robo_team/coordination/scripts/authority_ros_smoke.py --output /tmp/robo_team_authority_smoke_new_run
```

已在当前WSL完成 catkin 两包构建。ROS自测实例化本仓库真实manager和六个agent，并走真实授权/ACK话题；验证六个搜索授权、停稳换目标、拒绝目标双持有者、拒绝旧运行指令。它使用合成位姿/速度数据，不启动Gazebo或PX4，不能作六机物理飞行证明。输出目录必须新建，不覆盖旧证据；脚本只终止自己启动的独立roscore进程组。实际控制源码SHA写入result.json。

新增任务授权16条边界测试通过。最后全套200条中197通过，3条错误仍为原基线 replan_route 缺失。ROS复验run03通过，证据位于工作区根目录 tmp/evidence_archive/robo_team_authority_ros_20261003_run03/result.json 和 authority_events.jsonl，带本轮控制源码SHA；没有残留本轮roscore或仿真进程。后续继续本仓库实际飞行、命名空间/端口审计和几何预约接入。

## 后续真实飞行：单机两段任务

本仓库 SwarmAgent + manager任务授权的单机仿真已经跑通两轮（run02/run03）。修复真实暴露的末段缺陷：A*栅格中心不同于任务点，只有同一已知自由格内才追加精确目标连接；不能向占据格或别的格猜连接段。

补充显式逻辑ID→MAVROS/scan映射、按模型隔离的setpoint进程锁、单机runner锁及共享PX4端口占用拒绝。单机例程仍是任务用例，未启动正常全覆盖拍卖和视觉搜索；六机正常搜索仍是后续目标。

任务本地停止后，STOPPED状态不能不断续原代次；需走协调STOP/停稳确认和新代次恢复，避免异常被旧续约永久卡住。

run03独立最终误差0.27484m、末速0.06122m/s、最高高度4.82664m，STOPPED_ACK一次。原型到达验证true，碰撞ABSTAIN、正式比赛false。可执行步骤见 single_swarm_product_quickstart.md，完整证据与执行源码快照已入 docs/validation/single_swarm_run03。全套207条中204通过，3条基线错误仍在；快照SHA全部匹配，端口占用拒绝实测通过。
