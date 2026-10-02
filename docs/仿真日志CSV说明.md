# 仿真诊断 CSV

启动仿真前可指定日志目录：

```bash
export ROBOCUP_LOG_DIR="$PWD/logs/ros_csv"
```

各节点会立即 flush，输出文件如下：

- `algorithm.csv`：拍卖周期最终分配、utility、算法内部 UAV/目标坐标和欧氏距离；捕获行包含 tracker 的 `last_err` 对应状态、`ERR_TOL`、确认进度、重置次数和事件。
- `control_uav_*.csv`：ORCA 返回速度和经过护栏后实际发布给 PX4 的速度，以及控制节点计算的 UAV/目标距离。
- `perception_*.csv`：YOLO 原始框和最终 track，包含框中心/宽高、世界坐标估计、置信度、range、身高、图像采集时间、图像年龄和推理耗时。
- `ground_truth.csv`：`monitor_coordination.py` 从 `/gazebo/model_states` 读取的 UAV 真值、差分速度，以及与 `/swarm/uav_status` 的误差。

关键字段：

- `distance_m` 是代码内部 UAV↔目标 `hypot`/三维距离，不是 Gazebo 画面测量值。
- `estimation_error_m` 是 tracker 的上报坐标↔目标真值误差；`capture_threshold_m` 在 tracker 捕获记录中是坐标误差阈值 `ERR_TOL`。拍卖/控制记录同时保留 `detect_radius_m`，不要将三者混用。
- `ros_time` 用于对齐 YOLO、拍卖、控制和真值；`wall_time` 只用于定位节点实际落盘时间。
- `image_stamp` 是相机消息的原始 ROS 时间戳；`image_age_s` 是处理时刻减采集时刻；`inference_s` 是 YOLO 推理耗时。图像坐标转换使用带时间历史回推的相机平移位置。

需要运行 `monitor_coordination.py` 才会生成 Gazebo 真值文件。日志目录可按实验轮次设置为不同路径，避免追加到旧文件。
