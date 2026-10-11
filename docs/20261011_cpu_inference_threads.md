# 六路 CPU 感知线程限制：2026-10-11

感知默认使用 `PR_TORCH_THREADS=2`。模型创建后注册 `on_predict_start` 回调，等 Ultralytics 完成 CPU 设备设置后，再检查并限制 PyTorch 的实际计算线程数；已经符合配置时不重复设置。首次 CPU 推理记录：

```text
[pr] CPU_THREADS uav=typhoon_h480_0 configured=2 actual=2 before=8
```

配置必须是正整数，空值、零、负数或非整数在感知模块初始化时明确报错。`PR_TORCH_THREADS=8` 可作为单变量对照。GPU 回调直接返回，不读取或改变 CPU 线程；不改变 interop、权重、传感器、检测频率或检测阈值。

## 实际设备设置会覆盖环境变量

本机为 AMD Ryzen 9 9955HX，16 物理核、32 逻辑核；系统 Python 3.8.10、PyTorch 2.4.1+cu121、Ultralytics 8.4.168。

安装版本的 `ultralytics/utils/__init__.py` 将 `NUM_THREADS` 定为 `min(8, max(1, os.cpu_count()-1))`。`utils/torch_utils.py` 的 `select_device` 在 CPU 分支调用 `torch.set_num_threads(NUM_THREADS)`，六个进程因此配置了合计 48 个计算线程。这是计算线程配置数量，不等同于任意时刻的活跃操作系统线程数。

独立探针设置 `OMP_NUM_THREADS=1`、`MKL_NUM_THREADS=1` 后，PyTorch 初始为 1，调用 `select_device('cpu')` 后实际变成 8。仅设置环境变量或在模型设备初始化前调用 `set_num_threads`，不能保证首次推理的线程限制生效。

回调探针证明首次 `on_predict_start` 看到 8 并设为 2，第二次保持 2。生产专项测试使用真实 v1 模型再次验证设备重设、回调执行及后续稳定性。

## 六进程固定样本对照

使用仓库 v1 权重和 `artifacts/run_audit_20261011_102820/uav2_static_blue.png`，图像为 752×480。六个独立进程同时推理，每配置预热两次后各测十次；两轮按相反顺序比较 1、2、4、默认 8 线程，每配置合计 120 次。计时包含完整 `model.predict` 调用。

基准在训练结束后运行，全部隐藏 CUDA，并使用 `SCHED_IDLE` 与 nice 19；不启动仿真。当时独立的 GPU 模型评估可以运行。每次推理记录实际 `torch.get_num_threads()`，均符合对应配置。

| 每进程线程 | 中位延迟 | P95 | 最大延迟 | 超过 800 ms |
|---|---:|---:|---:|---:|
| 1 | 45.79 ms | 56.37 ms | 61.69 ms | 0/120 |
| 2 | 34.66 ms | 39.61 ms | 46.34 ms | 0/120 |
| 4 | 35.57 ms | 41.41 ms | 51.64 ms | 0/120 |
| 默认 8 | 90.99 ms | 555.65 ms | 1262.12 ms | 4/120 |

2 线程的吞吐与尾延迟优于默认，同时给仿真、导航和 ROS 留出更多 CPU 余量。该基准使用一个固定样本，不能作为比赛验收，也不能证明旧轮全部推理超时均由线程造成。

原始逐帧证据位于：

- [首轮](../artifacts/cpu_threads_20261011/benchmark.json)
- [反序轮](../artifacts/cpu_threads_20261011/benchmark_reverse.json)
- [设备设置后的回调探针](../artifacts/cpu_threads_20261011/callback_probe.json)
- [合并统计、输入哈希、环境及可复现脚本](../artifacts/cpu_threads_20261011/summary.json)

## 验证

`tools/test_cpu_inference_threads.py` 的四项专项测试覆盖：真实 YOLO 首次 CPU 设备重设之后仍为 2、后续推理不重复设置、GPU 不改变线程、无效配置拒绝、显式配置和后续外部重设后的恢复。测试日志位于 `/tmp/robocup_cpu_threads/cpu_threads_tests.log`。

```bash
source /opt/ros/noetic/setup.bash
source coordination/devel/setup.bash
ROBOCUP_LOG_DIR=/tmp/robocup_cpu_threads_tests \
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' \
PYTHONPATH="$PWD:$PWD/coordination/src/robocup_swarm/scripts:$PWD/navigation:/home/gycc/catkin_ws/devel/lib/python3/dist-packages:${PYTHONPATH}" \
python3 -m unittest tools.test_cpu_inference_threads -v
```

正式六机运行仍需检查每路首次 `CPU_THREADS` 日志，并比较实际推理延迟、图像龄与过期丢弃率；保持权重、检测频率、传感器和其他阈值不变。
