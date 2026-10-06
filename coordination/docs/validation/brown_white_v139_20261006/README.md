# v1.39 离线证据

未启动仿真；最新实体证据仍在 ../fast_competition_v137_20261006/round_1_seed_8159。

复现：仓库根目录执行 `python3 coordination/docs/validation/brown_white_v139_20261006/replay_brown.py`。
脚本仅用既有事件接收时间排列观测，在既有579次官方上传时间调用纯核心，不是完整ROS回调或真实运动回放。旧核心最大重现偏差0.004877米，候选结果见brown_replay_result.json；真值仅用于离线评估。没有修改原始飞行归档。

tests.txt：全量608项通过，0失败、0跳过。offline_state.json：检查时三个服务端口关闭，选定仿真/观察器程序进程为空；不证明未来状态。

线上兼容与剩余问题见 ../../brown_white_recovery_v139.md。下一轮要求统一源码快照，两个新辅助模块不能漏带。源码哈希按LF字节计算，与Git blob比对。
