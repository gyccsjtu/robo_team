# v1.32/a3deb4f实际单轮：309.324秒、3/6，主动停止修授权阻塞

用户直接要求改完再跑后启动，非新定时唤醒。种子8159，run_id4beb0d98-2a79-4b96-ad22-fff01ae3a050；原始WSL目录见archive_manifest.json。最终消除红4/5及白3，剩[0,1,2]，未达6/6。六机均实际移动，最高真实3.408048m；原生接触0、观察覆盖轨迹，140份执行源码SHA一致。

约308.100秒向核实命令的自有validator发SIGINT，最终采样309.324秒。原因是已消除目标被ROUTE_REFRESH重新授权，uav2持续零指令；不是完整600秒、服务自主崩溃或用户临时要求暂停。原始budget/result仍保存USER_INTERRUPTED/早期起栈摘要，真实停止原因另在agent_early_stop_decision.json；不能按原始result的fixture_only断言六机未飞。

精选433文件完整或gzip保存，gzserver全量只留WSL路径及SHA/尾部；manifest区分压缩SHA与原始SHA。window/保留预算、用户授权意图、观察器PID及进度。进度监视器的sim_s是从监视器启动计时的相对量，不能当作全轮时长；全轮使用city_trajectory首末原始时间。

关键证据：eliminated_target_regrant_evidence.json，algorithm/authority_events与control_uav_2，swarm_manager.log，以及postrun_audit。当前输出含白红精度、真实原图；蓝色独立观察开始较晚、0张图，不是全轮诊断。红色误差统计受真值瞬移插值/剩余集合处理限制，原始数据均保留。

后续计划与候选实施见[收尾问题及先行计划](../../../v132_postrun_plan_20261005.md)。本轮执行快照保持原样；后续核心修复不得冒称本轮已执行。
