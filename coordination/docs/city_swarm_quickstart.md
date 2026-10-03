# 六机城市协同入口

显示性能：城市生成模型关闭ray传感器的visualize调试线束；这是GUI显示开关，扫描频率、范围、分辨率、光束数和相机参数保持原值，模型清单单列gui_ray_visualizations_disabled。默认小场景保留原显示开关。当前CPU视觉环境为vision_env；GPU环境验证完成前，不宣称已启用GPU推理或提升帧率。

当前配置语义v1.2：manager与agent共享3m/s最大水平速度，雷达/机间约束可继续减速；视觉融合时序为v2.4。启动器等待城市世界时钟实际推进才插入飞机。已验证城市六机运动和真实视觉派遣，完整六目标连续15秒消除仍待实飞结果，不能用“六机接通”判比赛完成。

代码来自本仓库当前工作分支，复用 WSL Ubuntu-20.04、既有 runtime、独立 catkin 构建及 vision_env。控制在线接收雷达，六机物理模型为 typhoon_h480_0..5，ROS/逻辑编号为 uav_1..6。没有启用独立 radar_avoid 控制发布者。

PowerShell 启动完整 600 秒任务：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'bash /mnt/d/a/.robocup/robo_team/run_city_swarm.sh'
```

针对具体运动问题调试 90 秒：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'CITY_SECONDS=90 bash /mnt/d/a/.robocup/robo_team/run_city_swarm.sh'
```

默认输出 `/root/robocup_runs/city_<时间>_<pid>/`，每次独立目录。`CITY_SEED` 指定地图种子，默认17；同一问题复用种子，不做无目的随机批跑。地图来自当前机器平台资源的隔离副本；生成后检查原六机出生区域，若被建筑或其他障碍矩形覆盖，重新生成，最多12次，仍不合格则明确失败。地图、目标控制及裁判文件的来源哈希和适配记录见 scene/scene_manifest.json。目标控制的25秒瞬移改为规则要求的30秒，修改只在副本。

六路相机使用原模型 CameraInfo；城市障碍/目标真值不进入控制。初始化仅传出生位置和2m局部初始化区域。自身相机姿态暂使用 Gazebo link pose，正式允许性未确认；启动检查依赖生成器矩形近似，并非完整三维几何证明。当前入口是官方资源城市联调入口，不能称正式比赛环境等价或通过。

六路软件渲染较慢，PX4先以250Hz初始化，再将开发仿真的物理更新率设为20Hz，传感器参数不改。CITY_PHYSICS_RATE可覆盖开发更新率；旧50Hz城市运行图像延迟约1.3–1.8秒。20Hz下600秒仿真理论约125分钟墙钟时间，这是本机开发限制。`flight/result.json` 分别记录六机实际运动、目标消除数和裁判分数；formal_competition_pass保持false。`city_trajectory.jsonl` 为独立采样，`city_events.jsonl` 记录视觉和裁判剩余目标，逐机日志用于定位具体运动问题。

不要同时运行会使用全局 pkill 的其他试跑脚本。启动器只清理自己启动的进程组，不删除其他代码、缓存或结果。默认旧小场景入口 run_radar_swarm.sh 保留；城市入口请使用本页命令。
