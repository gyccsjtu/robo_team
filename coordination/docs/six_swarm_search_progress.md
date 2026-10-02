# 六机正常协同链首次飞行验证

使用本仓库真实 `swarm_manager.py` 的正常拍卖循环与六个 `swarm_agent.py`，没有手写任务替代拍卖。继承已经跑通的六机雷达连接，场景是明确生成的无障碍开发场景及对应存储地图，尚未接在线建图或视觉目标识别。

## 已实现和已核验

- 新入口 `six_radar_connectivity.py --flight-seconds 60` 在六机连接成功后运行实际manager/agent，共享运行ID，保持唯一发布者锁，记录授权事件和独立Gazebo真值。执行的swarm源码复制到运行目录，后续本地编辑不会改变运行中的核心。
- 修正启动时 `/use_sim_time` 设置晚于 `rospy.init_node` 的问题，记录器现使用仿真时钟。
- 真实六机暴露 agent `_configure_fcu` 在MAVROS参数列表未就绪时调用set，被拒绝后静默继续的问题。新纯逻辑配置流程先pull，再设置并get读回；三轮失败后终止启动，不继续解锁。`NAV_RCL_ACT=0`、`COM_RCL_EXCEPT=4`沿用既有自主飞行配置，没有新增绕过飞控健康检查的参数。
- `6011_typhoon_h480.post` 固定相机端口14558导致PX4间绑定冲突。现在仅在每轮私有etc副本中改为14600+instance/14630+instance，原构建文件保持原样。这是开发接入补丁；仍有插件的硬编码云台目的端口需要核对，不能称正式环境等价。

## 首次六机真实飞行结果：失败

run03六机均进入OFFBOARD、解锁、爬升超过4m、获得正常拍卖授权并执行水平搜索。观察60.076s仿真时间、1123条真值样本，最高真实高度5.0653m。但最小采样机间距只有 **2.0797m**，违反本用例3m保护判据，因此结果为 `SIX_SEARCH_FLIGHT_INCOMPLETE`，`prototype_search_verified=false`、正式比赛false，碰撞证据仍为ABSTAIN。

证据位于 `validation/six_search_run03/`，压缩包包含实际执行源码、生成模型、私有airframe补丁、地图/world/launch、全部控制/授权日志和真值轨迹。原结果中的 `armed=false` 是连接阶段字段遗留，并非飞行阶段状态；agent/PX4日志与真值证明已经起飞。当前入口已把该字段改为 `connectivity_armed`，并单独记录各机解锁观测。当前结果还增加明确失败原因和轨迹采样缺口检查。

run01未起飞，证明参数设置时序问题；run02为诊断过程中手动向uav_1设置参数的干预用例，不能作为自主成功证据。run03未进行人工控制干预，独立复现了六机正常搜索中的间距违规。

## 当前机器复现命令

输出目录必须不存在，复用单机quickstart的真实依赖以及已经构建的isolated catkin工作空间。

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'source /root/robo_team_build/codex_authority_v2/devel/setup.bash; python3 /mnt/d/a/.robocup/robo_team/coordination/scripts/six_radar_connectivity.py --output /tmp/robo_team_six_search_01 --flight-seconds 60'
```

脚本只清理本次启动的进程组，run03完成后未发现残留仿真进程。不得在真实飞机或随机比赛世界使用这个空场景启动入口。

## 下一步必须修正的协同问题

拍卖中的旧角落先验奖励可把右侧飞机派到左下角，搜索路径交叉；二维雷达和当前友机避碰仍不能保证约定间距。应修改实际TaskAllocator/manager的分配与几何通行协调，保留任务授权v2的停稳与占用语义，再重复六机真实轨迹验收。不能仅通过降低验收距离或只展示六机起飞来宣称成功。

配置流程3项测试通过，覆盖未加载列表重试、set失败和读回不一致。全套218项测试为215通过、3项原基线错误（旧test_uav_motion对不存在的replan_route进行mock），无新增错误。
