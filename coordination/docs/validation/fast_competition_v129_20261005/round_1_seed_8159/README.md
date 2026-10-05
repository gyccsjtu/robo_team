# v1.29/8159实际收尾证据

执行cb33a45，run_id=29494783-75df-4543-b7ed-ca7b2f61f90a，2026-10-05北京时间16:37启动、16:51结束。轨迹1929.732至2055.212，125.480秒；裁判0/6，非完整600秒。六机均实际移动，最高真实3.677853m；原生31734帧覆盖轨迹，非地面接触0。不能据此说已完成避障或比赛。

Codex在发现同代次规划ticket连续取消后主动收尾。原始budget/result的USER_INTERRUPTED、CHECK_INTERRUPTED_OR_WALL_TIMEOUT和起栈状态保留；实际决策见agent_early_stop_decision.json，审计见postrun_audit.json。不是用户手动中断，也不是基础服务自行崩溃。

140份SHA一致：128份swarm_source_manifest对应的快照源码及12份execution_sources对应的运行辅助源码；不将两份清单重复说成128份合计。规划问题可执行反例见flight/planner_starvation_diagnosis.json，调度顺序为构造输入，不是完整物理回放。6号机绿色任务14次成功规划、首路线提交延迟9.180秒来自日志。

白色独立原图29张最大误差1.898m、93次上传最大2.457m。红色独立原图9张最大误差15.240m，全未小于1m；该轮没有任一红色消除。只读取证没有用于控制。

原始完整资料保留/root/robocup_runs/manual_v129_20261005T083707Z/round_1_seed_8159。archive_manifest.json逐项列原文件路径、长度、SHA及归档SHA；大日志无损gzip，gzserver仅留尾部且原始全文件SHA可查。真实原图保留flight/visual_frames，不把CSV重复发布当作不同原图。

自有runner/launcher、白红只读观察器和原图观察器均已退出；11375/11376/19731关闭。下一步先按../../../v129_postrun_plan_20261005.md修复规划提交饥饿并离线验证，再安排允许的下一物理轮；该归档中的源码仍是cb33a45，不是收尾后的新候选。
