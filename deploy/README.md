# 选手算法容器（e77c3a2 基线）

此镜像只运行本队算法。裁判在 `robocup` 容器负责地图、六机模型、演员、官方通信和计分。**按官方 Docker 流程，裁判在第一架飞机起飞时启动 `score_cal_final.py` 并开始计分；`start_player.sh` 不启动裁判，也不自行开始比赛计时。**`/clock` 仅用于队伍内部按仿真时间计算图像年龄与控制时长。传感器模型须由裁判侧采用另交的 `赛事组无人机配置.zip`；本镜像不能安装或改变裁判世界。用户确认 `/clock` 可用，其余 Gazebo 环境真值话题不作为算法输入。

在有 Docker 和网络的 Linux x86-64 主机上，提交当前部署改动后执行：

```bash
./deploy/build_player.sh
docker save robocup-player:e77c3a2-<提交前12位> -o robocup-player.tar
```

单机运行时，由裁判先启动官方世界、演员及六路通信，然后在选手容器内执行：

```bash
cd /opt/robocup/algorithm
./start_player.sh
```

双机模式须先设置 `ROS_MASTER_URI` 为裁判机 ROS master 地址、`ROS_IP` 为选手机可达地址，再启动容器。停止本队进程用 `./stop_player.sh`。容器必须使用 host 网络连接 ROS；正式验收还需在目标机器执行 `docker save`、`docker load`、传感器实帧与六机起飞检查。

构建时 `prepare_context.py` 只导出 git 中的算法文件、两份权重和官方 ActorInfo 消息定义；不会带入日志、开发用演员控制、模拟裁判和 Gazebo 真值桥。`requirements.lock` 将 Ultralytics、CPU PyTorch、TorchVision、OpenCV、NumPy 等版本固定；Docker 构建时下载 wheel 并安装，`apt-packages.txt` 安装 ROS、cv_bridge、MAVROS 消息等系统依赖。镜像构建阶段执行离线的消息、图像转换与权重加载检查。**源码目录里只有依赖清单，不包含已经安装的库；只有实际完成 `docker build` 后库才在镜像中。**`start-team` 不启动官方世界、演员、裁判或第二套通信节点。

起飞前的 ROS 图审计允许裁判尚未启动，但仍检查禁用订阅、ActorInfo 类型和唯一发布者。裁判启动后可在选手容器里再执行 `python3 tools/audit_match_graph.py --clock-permission allowed --clock-permission-reference '用户确认 /clock 可用'` 核对裁判订阅。此检查只读取 ROS master 的节点注册信息，不读取计分内容。

本机目前没有 Docker/Podman，故这里只能验证构建上下文和脚本，尚不能声称镜像构建或赛场联调已通过。
