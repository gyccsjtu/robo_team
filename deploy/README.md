# 选手算法容器（e77c3a2 基线）

此镜像只运行本队算法。裁判在 `robocup` 容器负责地图、六机模型、演员、官方通信和计分。传感器模型须由裁判侧采用另交的 `赛事组无人机配置.zip`；本镜像不能安装或改变裁判世界。用户确认 `/clock` 可用，其余 Gazebo 环境真值话题不作为算法输入。

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

构建时 `prepare_context.py` 只导出 git 中的算法文件、两份权重和官方 ActorInfo 消息定义；不会带入日志、开发用演员控制、模拟裁判和 Gazebo 真值桥。镜像构建阶段执行离线的消息、图像转换与权重加载检查。`start-team` 不启动官方世界、演员、裁判或第二套通信节点。

本机目前没有 Docker/Podman，故这里只能验证构建上下文和脚本，尚不能声称镜像构建或赛场联调已通过。
