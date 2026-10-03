# 六机共享 GPU 环境

入口 `run_city_shared_gpu.sh` 使用现有两个独立环境：共享服务使用
`/root/robo_team_build/vision_cuda_20261003/bin/python`，六个感知客户端使用
`/root/robo_team_build/vision_env/bin/python`。共享服务使用 CUDA 0、FP32，图像无损 PNG；
客户端故障回退使用 CPU，避免服务故障重新创建六份 CUDA 上下文。
原始权重和传感器参数保持不变。共享GPU入口开发物理更新默认40Hz；
20Hz人工减速曾导致MAVROS墙钟心跳超时，实际调至40Hz后超时计数不再增长，
短窗口图像延迟约0.5–0.7秒，仍保留原时效闸门。CPU入口的默认20Hz不变。

先检查依赖（不启动仿真或服务）：

```bash
bash /mnt/d/a/.robocup/robo_team/run_city_shared_gpu.sh --check
```

在当前仿真结束并确认端口空闲后启动：

```bash
bash /mnt/d/a/.robocup/robo_team/run_city_shared_gpu.sh
```

每轮由原城市入口创建独立运行目录。默认端口19731供本轮共享服务使用；
不能与另一轮仿真并行占用同一组ROS/Gazebo/推理端口。
不修改 `.wslconfig`，不重启WSL。环境检查通过不代表六机共享推理性能、
目标消除或600秒比赛验收通过。

## 高度配置 v1.6

shared_gpu_run03_altitude_failure 在41.208仿真秒被官方裁判终止：uav_5真实高度6.018827m，同期MAVROS本地高度约4.747343m。城市开发入口将巡航从4.5m降到3.5m，并将本地高度强制下降阈值设为4.1/4.3/4.5m。这是针对实测估计偏差预留余量，不是对任意偏差的保证；真实高度只用于独立审计，未接入飞控。传感器参数与官方6m上限不变。下一次六机物理运行仍需验证高度与搜索效果。配置及源码哈希随每轮保存。
