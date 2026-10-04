# v1.22启动段中断：修相机授权读时序

执行源5d2d117，原目录`/root/robocup_runs/codex_city_teammate_v122_20261004/flight`。
轨迹记录1931.028至1961.900秒，剩余全部六目标；主动SIGTERM仅发给本轮launcher PID585。
此段不能用作完整六机或目标消除验收，原始result保持CHECK_INTERRUPTED_OR_WALL_TIMEOUT。

在新相机授权消费者中发现时序缺陷：处理循环起始时间早于稍后收到的grant回调，
调用TaskGate.can_move时可能误锁STOP。已修查询瞬间取ROS时间，且只读CameraTask
遇旧查询只返回未知，不改变执行器授权。next_code_core_tests_414.log验证修正后的代码，
不属于这段运行源。下一轮需重新载入；不在运行快照上静默改文件或冒称热修成功。

保存原始证据、实际执行源SHA和明确中断原因；正式比赛false，0/6。
