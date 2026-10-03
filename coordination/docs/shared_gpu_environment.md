# 六机共享 GPU 环境

入口 `run_city_shared_gpu.sh` 使用现有两个独立环境：共享服务使用
`/root/robo_team_build/vision_cuda_20261003/bin/python`，六个感知客户端使用
`/root/robo_team_build/vision_env/bin/python`。共享服务使用 CUDA 0、FP32，图像无损 PNG；
客户端故障回退使用 CPU，避免服务故障重新创建六份 CUDA 上下文。
原始权重和传感器参数保持不变。开发物理更新默认20Hz。

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
