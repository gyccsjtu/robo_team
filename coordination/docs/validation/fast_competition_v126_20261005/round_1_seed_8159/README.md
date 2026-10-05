# v1.26单轮8159：已结束，3/6，非完整600秒

执行dce0c64，run_id=e7fade08-f55e-4237-9e57-76f15ea3d19e。2026-10-05 08:18启动、08:58北京时间由Codex提前收尾，转入已证实的旧冷却与新授权冲突修复。六机实际运动；轨迹375.288仿真秒，裁判消除actor0/1/4，剩棕、白、另一红。[收尾审计](postrun_audit.json)给出原生接触覆盖、实际高度与位移；非地面接触0，但不能代表完整600秒或正式比赛通过。

[实际停止决定](agent_early_stop_decision.json)与[原始预算](budget.json)、[早停记录](early_stop.json)、[原始result](flight/result.json)并列。既有harness将信号停止称为USER_INTERRUPTED，raw result保留CHECK_INTERRUPTED_OR_WALL_TIMEOUT和起栈fixture状态；实际由Codex发SIGINT，非用户新请求、非自主组件崩溃、非完成比赛。这些原始字段未改写。子进程清场前状态在result中，null只代表该次轮询尚未退出。

旧冷却现场见[白色冲突](flight/white_cooldown_conflict_readonly.json)、[蓝白重授权审计](flight/cooldown_regrant_audit_readonly.json)。候选策略的[回放](flight/tracking_retry_policy_replay.json)只检验恢复条件，不生成物理轨迹或路线授权。回放候选源码与本轮实际执行源码不同，不能将v1.27效果归到本轮。

[白色独立精度](flight/independent_visual_accuracy_white.json)与[局部窗口](flight/white_partial_windows_readonly.json)是只读评价，人物真值未输入控制器。visual_frames保存实际发布观测对应的真实原图；white_sequence_readonly仅为追加漏检取证，但本轮未再触发白色，0张，不能充作漏检图。其原始只订阅脚本保留用于说明取证边界。remaining_targets_readonly.json第一条查询因ROS时钟尚未就绪标为sample_valid=false、时间为空，不能用于时间配准。

[archive_manifest.json](archive_manifest.json)列581份原文件的SHA、大小、完整WSL路径和归档方式。319份全文、无损gzip或尾部摘录约20.6MB；.gz解压后对应原SHA。完整gzserver、navigation、PX4 ULog等大文件留在/root/robocup_runs/goal_v126_20261005_082100/round_1_seed_8159，尾部不代替完整文件。128份执行源码快照SHA无差异，任务schema2、路线schema1、视觉schema2、共享metadata2。

后续按照[已冻结修复计划](../../../fast_competition_next_plan_20261005.md)，已实现候选见[v1.27说明](../../../fast_competition_fixes_v127.md)。本轮预算已用完，不在该根目录再次启动，不新增run系列脚本或原样重跑。
