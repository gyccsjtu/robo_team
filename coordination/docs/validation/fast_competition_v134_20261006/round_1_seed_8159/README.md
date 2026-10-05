# v1.34六机短轮：加油站接触收尾

- 执行a6f0307061d702e1c7c96c14b3b41fcc6d249a2f，seed8159，run_id `5f40b4cb-ae23-4054-8048-e8d44d1f5843`。
- 完整原始路径：`/root/robocup_runs/manual_v134_20261005T153320Z/round_1_seed_8159`。本地237份精选原始文件/清单约5.3MB；大型jsonl/log以gzip保留精确解压字节，gzserver仅尾部。每个原始文件的WSL位置、大小、SHA和压缩件SHA见archive_manifest.json。v135_candidate_replay.json与本README是另行生成的分析，不是原始执行输出。
- postrun_audit.json：轨迹46.240仿真秒、裁判0/6，六机水平位移3.194至33.847米，最高真实4.053047米；原生11913帧覆盖轨迹，2条建筑接触，人物/机间接触0。首接触1976.052，加油站gas_station_73与6号机rotor_1。129份swarm快照和12份辅助源码SHA匹配。
- 既有harness因CONFIRMED_STATIC_BODY_CONTACT收尾，未跑满600秒，不是服务自主崩溃。原始result/budget/early_stop不改写；起栈armed=false或原始CHECK_INTERRUPTED_OR_WALL_TIMEOUT不能替代实际飞行/停止原因。window/budget.json的历史authorization标签由既有harness写入；本次实际授权说明见window/planned_validation.json及v134_postrun_plan_20261006.md。
- 六机原图已处理证据574条，最大接收原图年龄0.660秒；6号机没有进入到点观察/补视点阶段。uav_1三次搜索结果分别NO_ACTUAL_PROGRESS、NO_ROUTE_COMMIT、NO_ROUTE_COMMIT，均mask0，没有伪装观察覆盖。
- 独立绿/蓝/棕精度及原图观察器启动晚于结束，无有效数据，空文件不能当漏检或精度通过；四个观察器和自有栈均退出，11375/11376/19731关闭。
- radar_recent_hit_clear_replay.json为旧版本有限数值射线地图构造回放；v135_candidate_replay.json为新候选离线对照。29个近期近障命中原被清除，新候选保留29/29；12个CSV差分样本中6个拒绝仅XY低速所给的停稳判断。四段局部净空构造查询旧/新都拒绝，未复现历史实际MOVING放行。null不猜inf，CSV控制时间不等于原位姿头时间，历史pose_s缺失，不能据此宣称实体接触解除。

修复顺序、接口和下一次仅一轮验证见[收尾计划](../../../v134_postrun_plan_20261006.md)及[v1.35候选](../../../fast_competition_fixes_v135.md)。原版并未完成比赛目标；候选也尚未实飞。
