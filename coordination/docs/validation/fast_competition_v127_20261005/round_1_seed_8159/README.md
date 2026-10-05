# v1.27种子8159：建筑接触收尾，1/6

执行提交715d9ae，核心49720fe；run_id=67195354-b845-49df-abf8-a6ad82d020d6。2026-10-05 13:28北京时间起，约14:22结束。既有`validate_fast_city.py --single-seed 8159`按建筑接触结束自有进程，预算`STOPPED_AFTER_CONTACT`。原始result的起栈状态、`fixture_only`与`CHECK_INTERRUPTED_OR_WALL_TIMEOUT`保留；不得用它代替实际运动与停止原因。`postrun_audit.json`、`early_stop.json`（实际路径以清单为准）和budget共同核对。

| 实测项 | 结果 |
|---|---|
| 真实轨迹时段 | ROS1930.784–2443.264，共512.480秒，未跑满600秒 |
| 裁判消除 | 蓝色actor_1，1/6；剩绿、棕、白及两红，无终态6/6 |
| 接触 | 3条建筑、1条人物、0机间；不代表4次独立碰撞 |
| 原生观察覆盖 | 128496帧，1929.568–2443.548，最大间隔0.004秒，覆盖轨迹 |
| 最高真实高度 | 3.494127048m；配置局部2.2m不冒充真实高度 |
| 运动与执行 | 六机均实际移动；128份源码SHA与候选一致 |
| 白色定位 | 122张不同原图最大误差2.420m，442次实际上传最大1.999m；未消除 |

5号机（Gazebo typhoon_h480_4）ROS2442.048先接触actor_3，2443.172接触house_2_126旋翼，随后又记两条该房屋接触。2443.068已发XYZ停止，但仅早0.104秒，轨迹相邻样本速度约2.157m/s、MAVROS反馈约0.554m/s。时间先后不是完整因果证据，不能说碰墙已修。

2号机有一段真实白色退避：2421.572之后前6条命令仍有非零速度，2421.888开始固定XYZ；到管理器STOP前共196条XYZ保持、34条有偏航，之后没有白色新授权。新授权就绪分支和该停控段有现场证据，旧冷却后的完整恢复不能判通过。

## 归档与回放边界

全文源目录：`/root/robocup_runs/manual_v127_20261005_133000/round_1_seed_8159`；预算在其父目录。`archive_manifest.json`逐文件列出全文WSL路径、字节数、原始SHA、Git归档路径与归档SHA。大CSV/JSONL/普通日志无损gzip；超过5MB的Gazebo/导航日志仅保留16KB尾部，全文仍在WSL且SHA可核查。398张原图为只读诊断材料，人物/障碍真值仅用于审计，没有用于控制。

- `postrun_audit.json`：实际裁判、轨迹、接触覆盖、真实高度及执行SHA。
- `flight/postrun_problem_windows.json`及`contact_navigation_windows.json`：接触、导航短窗口和授权原图年龄。
- `contact_speed_diagnosis.json`：最后0.320秒估计位置差分约1.414m/s、即时MAVROS速度约0.554m/s，扫描姿态滚转/俯仰也在变化。速度来源和采样不同，窗口扫描每0.5秒存一次；不能当作完整雷达回放，尚未证明该用哪一个速度或建筑被漏扫。
- `flight/backoff_window_readonly.json`：完整实际退避命令统计与6条残留速度。
- `flight/pending_reacquisition_policy_replay.json`：收尾后v1.28候选，7段时间窗口中6段可仅偏航；TaskGate先前历史及冷却夹具不是完整实际历史。
- `flight/stop_slew_policy_replay.json`：旧方法7条水平命令精确重现，新方法首帧请求XYZ停止；守卫桩和固定高度用于隔离平滑，未重放实体动力学。

**回放文件不属于本轮执行源码或物理效果。** 本轮实际1/6且建筑接触，competition_goal_met及formal_competition_pass均false。候选说明见[上层v1.28实现记录](../../../fast_competition_fixes_v128.md)，下一步先按[收尾计划](../../../v127_postrun_plan_20261005.md)修复/诊断，当前窗口不再开仿真。
