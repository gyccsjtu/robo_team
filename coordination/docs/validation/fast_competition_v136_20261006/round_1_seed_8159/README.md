# v1.36完整600秒：3/6，建筑0、人物3条接触

执行协同代码e4da217b0927154281bf72c6c7de3490a040e123，启动HEAD7c2c688ccf67bc9948620ecb51eb46c86d2e2775，种子8159，run_id `e1cb10f0-31e3-467d-aa9e-9a179689d31c`。

- 原始配置600秒自然收尾，原始计时600.108秒，轨迹599.964秒。裁判3/6（绿及两个红），剩蓝、棕、白；原始result/budget和启动阶段状态未改写。
- 六机实际移动，最高真实3.363185米，采样最小三维间距2.835094米。150303帧原生接触覆盖轨迹；建筑/机间0，人物3条，具体碰撞体/时刻见完整city_contacts.log。不是六目标完成或全面避障成功。
- source_manifest.json是原始全仓预启动清单；executed_source_audit.json是派生核验，130份swarm与12份辅助执行源码SHA匹配。二者分开保存。
- 526份精选原件/无损压缩件的Git暂存字节SHA全部匹配，见git_index_sha_audit.json。候选五个代码/测试文件均与测试内容一致；swarm_agent.py工作副本含1826个CRLF，Git暂存转为LF，仅换行不同，其余四个逐字节一致，两种SHA分别保留。
- 全量现场 `/root/robocup_runs/heartbeat_v136_20261005T203253Z/round_1_seed_8159`。archive_manifest.json记录原始及派生文件、WSL完整路径/SHA和精选件SHA。大文本gzip无损；六份完整导航、534MB Gazebo及六份PX4原始飞控日志仅留WSL，不加入Git。PNG只选部分真实原图，帧上限48/颜色/飞机，不证明完整视觉覆盖。
- 五色只读观察器在首次目标检测前启动，晚于轨迹开始约9.5仿真秒，不能覆盖此前。白色原始YOLO检测11次但未建立观测，独立白色image/official均0；零上报不能证明准确率。蓝色原图343张、303张对齐误差小于1米，官方最长准确采样跨度14.7秒；该跨度不是正式裁判计时证明。完整统计及插值/瞬移/收图时间限制见independent_visual_postrun_audit.json。
- 路线只读订阅保存真实广播和短窗口。prior_snapshot_geometry_replay_inputs.json使用停控**之前**seq101广播与实测两个速度方向；这是几何输入，不是完整历史agent回调录制。navigation_*_latest.json和完整导航的WSL/SHA保留最终控制原因。
- 自有launcher637及8934/8935/8936/8938/8942/8945观察器均已退出，11375/11376/19731关闭，见process_cleanup_audit.json。未全局清场；下次恢复先核查实时状态。旧harness授权标签是历史字面值，实际定时授权记录在window/planned_validation.json。

先修计划：[v136_postrun_plan_20261006.md](../../../v136_postrun_plan_20261006.md)。尚未完成人物接触、蓝棕白持续确认和正式环境兼容。
