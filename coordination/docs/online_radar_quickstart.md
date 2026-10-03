# 当前机器：六机在线雷达协同开发场景

雷达平面高度规则v1.1：在线二维雷达模式保持原搜索高度目标；不得因追踪、进入障碍附近而自动降低1m/0.5m。单个水平扫描面不证明下方体积可通行，也不授权RTL/降落；垂直通道需另有证明接口。本修复不改变限高护栏或初始起飞逻辑。

使用本仓库实际manager和六个agent的正常任务拍卖，在线射线地图、实际路径预约与最终速度保护全部接入。依赖沿用 `single_swarm_product_quickstart.md`；尚未迁机验证，不能使用旧README中的他人/home路径。

Windows PowerShell命令，输出目录必须不存在：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'bash /mnt/d/a/.robocup/robo_team/run_radar_swarm.sh --output /tmp/robo_team_online_new_run --flight-seconds 60 --obstacle-fixture'
```

入口创建独立ROS/Gazebo master、六个PX4实例、逐机命名空间/端口、8m出生间隔，使用已验证runtime；只清理本轮进程组。6路雷达数据→在线地图→含友机预约障碍的A*候选→manager串行路线授权→最终速度门，缺证停止水平运动。仿真真值只供独立观察器，不反馈给控制。无人机雷达、相机等原传感器参数不变，箱体接触传感器只用于开发观察。

当前执行器唯一发布setpoint_raw/local，停止时锁定首次停止的MAVROS局部XY，通过PX4位置环保持；不再同时发布旧setpoint_velocity/cmd_vel。每30秒可先停稳再刷新一机任务代次/保留路线，缺停稳证据不撤占用。run17长时碰撞失败与run18修复后120秒观察通过同时保存，不能只引用短轮通过。接口见position_brake_v1.md与route_reservation_v1.md。

输出：result.json、six_truth.jsonl、fixture_contacts.jsonl、algorithm/route_events.jsonl与authority_events.jsonl、全部控制日志、模型/world、执行源码与SHA。只有明确的prototype_search_verified=true表明当前观察门槛通过；formal_competition_pass始终false。无官方裁判时不得因超时或全部飞机起飞宣布比赛完成。

空场景去掉 `--obstacle-fixture`。实体视觉开发用例另加`--flight-actor-probe`，按flight_actor_probe_v1.md执行；当前机器须有已验证的/root/robo_team_build/vision_env及仓库现有权重。六机连接后慢速物理步进保证软件渲染能提供新鲜原图，120秒仿真约需10分钟墙钟，传感器参数不变。演员插入、图像、桥和执行均自动进行，启动后无需人工控制；该场景保留8m出生队形，不是正式随机城市。

记录器现在对并发写入加锁，并逐行核验JSONL；文件缺失/损坏不能通过。run19及实体视觉run04/run05存在旧记录器损坏，新增followup_stream_audit.json并保留原汇总，不能借其汇总声称完整证据PASS。完整等待/接替、旧3m出生队形兼容、正式裁判和随机城市验证仍需继续。
