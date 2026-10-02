# 六机雷达接入检查

此入口已在当前 WSL 验证六个真实 PX4、六路 MAVROS 定位及六路雷达连接。它不启动 manager/agent，不解锁或起飞；空场景用于核对接线，不是六机搜索、在线建图或比赛验收。

依赖复用单机 quickstart 的已验证环境：Ubuntu-20.04、ROS Noetic、Gazebo11、PX4 编译产物、`stereo_20261002T140417Z` runtime、模型缓存与 X0。模型原件默认 `/root/vendor_eval/d43bac6/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf`。不会修改这些来源文件。

从 Windows 运行（输出目录必须不存在）：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'source /opt/ros/noetic/setup.bash; python3 /mnt/d/a/.robocup/robo_team/coordination/scripts/six_radar_connectivity.py --output /tmp/robo_team_six_radar_check_01'
```

`--px4`、`--runtime`、`--source-sdf` 可指定真实依赖位置；ROS/Gazebo 默认端口11375/11376，可通过 `--master-port`、`--gazebo-port` 更改。PX4端口从实际构建产物读取，禁止靠改 ROS 端口冒充隔离全部 FCU。脚本持有实例0..6的进程锁，核对 TCP/UDP 各自绑定端口，占用时退出，不接管其他进程。

模型生成保留全部传感器物理参数，复制 GPS/通信插件的运行时路径，逐机调整模型名、ROS话题/传感器帧及 MAVLink 仿真端口。Gazebo内部传输插件保留既有 world 命名空间，其话题已经含模型名，不能与 ROS robotNamespace 混改。六份 SDF 和源文件 SHA 保存在输出目录。MAVROS 定位、IMU 和 TF 配置逐机命名；`uav_N/map` 是逐机局部坐标系，尚未定义为共享世界坐标，后续协同必须显式应用出生点偏移。

运行在生成的空世界，飞机位于 x=-20,-12,-4,4,12,20、y=0、z=0.2。该分离位置只适用于这个无建筑开发用例，不可照搬到随机比赛地图。墙钟上限360s，完成/失败都清理本次自有进程组；不执行宽泛 pkill。结果仅在六机状态连接、未解锁、定位/雷达均新鲜且逐机帧一致时写 `SIX_RADAR_CONNECTED`。

证据：`validation/six_radar_connectivity_run06/` 保存结果、接线配置、执行源码 SHA 与完整压缩包（源码快照、六份 SDF、world、launch 和全部启动日志）。六机扫描均为512束、0.5–20m；定位帧分别为 `uav_N/map`，雷达帧为 `uav_N/laser_2d`，插件加载错误为0。结束后未发现遗留的 gzserver、PX4、roscore 或 MAVROS 进程。

测试：模型隔离/参数保留测试通过；实际 TCP监听与 UDP占用拒绝、无关TCP不阻塞UDP的3项测试通过。此前试跑发现端口检查混用协议和 TCP TIME_WAIT 误报，现已修正并以真实六机复跑验证。

下一阶段使用这套接线运行本仓库实际 manager/agent，先核对六机起飞、任务分配与授权事件，再补齐在线地图和几何预约。当前 `formal_competition_pass=false`。
