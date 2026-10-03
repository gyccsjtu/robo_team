# 当前机器：六机在线雷达协同开发场景

使用本仓库实际manager和六个agent的正常任务拍卖，在线射线地图、实际路径预约与最终速度保护全部接入。依赖沿用 `single_swarm_product_quickstart.md`；尚未迁机验证，不能使用旧README中的他人/home路径。

Windows PowerShell命令，输出目录必须不存在：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'bash /mnt/d/a/.robocup/robo_team/run_radar_swarm.sh --output /tmp/robo_team_online_new_run --flight-seconds 60 --obstacle-fixture'
```

入口创建独立ROS/Gazebo master、六个PX4实例、逐机命名空间/端口、8m出生间隔，使用已验证runtime；只清理本轮进程组。6路雷达数据→在线地图→含友机预约障碍的A*候选→manager串行路线授权→最终速度门，缺证停止水平运动。仿真真值只供独立观察器，不反馈给控制。无人机雷达、相机等原传感器参数不变，箱体接触传感器只用于开发观察。

当前执行器唯一发布setpoint_raw/local，停止时锁定首次停止的MAVROS局部XY，通过PX4位置环保持；不再同时发布旧setpoint_velocity/cmd_vel。每30秒可先停稳再刷新一机任务代次/保留路线，缺停稳证据不撤占用。run17长时碰撞失败与run18修复后120秒观察通过同时保存，不能只引用短轮通过。接口见position_brake_v1.md与route_reservation_v1.md。

输出：result.json、six_truth.jsonl、fixture_contacts.jsonl、algorithm/route_events.jsonl与authority_events.jsonl、全部控制日志、模型/world、执行源码与SHA。只有明确的prototype_search_verified=true表明当前观察门槛通过；formal_competition_pass始终false。无官方裁判时不得因超时或全部飞机起飞宣布比赛完成。

空场景去掉 `--obstacle-fixture`。本入口还未包含演员、YOLO和官方裁判，六个障碍箱不是官方比赛城市；旧根入口3m出生队形不满足当前保守预约配置，不能套用本用例通过结果。视觉证据v2、完整等待/接替和正式环境迁移继续实施。
