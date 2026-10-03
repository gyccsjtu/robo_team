# RoboCup 队友协同算法：雷达迁移分支

实际修改的是 `coordination/src/robocup_swarm/scripts/swarm_task.py`、`swarm_manager.py`、`swarm_agent.py` 所在执行链。开发分支 `codex/radar-coordination-20261003`；不修改主干。

当前机器已有可执行的六机雷达开发场景。Windows PowerShell：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'bash /mnt/d/a/.robocup/robo_team/run_radar_swarm.sh --output /tmp/robo_team_new_run --flight-seconds 60 --obstacle-fixture'
```

输出目录必须不存在。六机真实PX4/MAVROS使用独立命名空间、端口和进程锁；在线雷达地图保留未知，实际路径获准后执行，保护层仍由agent唯一发布速度。入口只停止自己创建的进程组。完整依赖、证据路径与限制见 [六机运行说明](coordination/docs/online_radar_quickstart.md)。

此入口是非空障碍开发场景，尚未包含真实演员、YOLO和官方裁判；不能把 `prototype_search_verified` 当正式比赛通过。旧 `run_match.sh` 使用另一套3m出生队形和存储地图，尚未完成当前保守预约配置的迁移。接口与修复进度见 [实施记录](coordination/docs/implementation_progress_20261003.md)。
