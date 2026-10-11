# 未起飞与飞行中失效复盘

本次读取本地 agent 日志及 PX4 ULog；没有重启当前比赛，没有修改 PX4、演员或裁判。工具环境不能建立 ROS XMLRPC 服务，因此没有取得实时 ROS 状态订阅。以下区分已证实事实和待验证原因。

## 02:15:58 轮：3 号未完成爬升

- `logs/logs_20261011_021558/06_swarm_agent_3.log`：已解锁，但没有爬升到 2.5 米记录；控制 CSV 持续输出 vz=0.5，高度仍在地面附近。
- 飞控记录：`~/.ros/sitl_typhoon_h480_3/log/2026-10-10/18_16_16.ulg`。31.180 秒解锁；31.680 秒 landed 变 false；32.184 秒起飞状态变为 READY_FOR_TAKEOFF，此后没有进入 RAMPUP。
- 此期间导航状态为 OFFBOARD，保持解锁。上游 trajectory_setpoint 已有约 -0.4 米的 NED 高度目标，但下游位置控制输出 acceleration.z=100、thrust.z≈-0.001，实际高度不增长。
- 本机 PX4 `MulticopterPositionControl.cpp` 在 landed=true 时才允许 offboard 的 want_takeoff；未进入 rampup 则强制低推力。日志显示“误判已离地”和“等待起飞”互相卡住；landed 提前变 false 的初始诱因仍需复验，不能归因为未发送爬升指令或直接归咎于预热时间。

## 02:19:40 轮：六机都起飞，0 号随后失效

- `logs/logs_20261011_021940/06_swarm_agent_0.log` 至 `_5.log` 均有爬升到 2.5 米记录。
- 0 号在 ROS 2509.484 秒锁定定位异常，2510.716 秒请求 AUTO.LAND。
- `~/.ros/sitl_typhoon_h480_0/log/2026-10-10/18_19_49.ulg` 含 invalid setpoints、accel fault、no local position，以及大量位置/高度估计重置；这不是普通未解锁。碰撞与估计失效的先后关系尚未证明。

## 本次代码修改与范围

- `swarm_agent.py`：任务可用状态要求完成起飞、已解锁、处于 OFFBOARD，且没有降落/定位/坠机故障。此前单凭连接就可能占用搜索资源。
- 增加 FCU_STATE 状态变化记录；爬升 10 秒没有取得 0.3 米进展时报告 TAKEOFF_STALLED，含模式、解锁、高度和最后位置设定点。此诊断不自动解锁、不关闭保护。
- 增加离线用例，覆盖地面、锁定、降落模式、正常飞行及起飞无进展。已有管理器不可用任务释放回归一并运行。

这些修复解决“失效飞机仍被当作可用资源”和诊断缺失；尚未声称修复 PX4 起飞卡死或完成 6/6。运行中的 Python 进程不会自动加载这些修改。
