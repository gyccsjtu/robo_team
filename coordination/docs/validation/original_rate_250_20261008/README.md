# 250Hz无头实体取证

实际执行e8a73f25a31db5b7cc6344aaf403c9e6884cbd9f；种子8159、run_id=9c9e1949-1701-4670-9366-e85dbbac2e2b。完整原始路径/root/robocup_runs/original_rate_250_20261008T021930Z/round_1_seed_8159。本轮97.492秒、裁判0/6，主动停止，非完整600秒。

postrun_summary.json为派生核验；result、budget、owned_interrupt、源码manifest、原始观察器JSONL与日志保留原字节。agent_early_stop_decision与旧runner的USER_INTERRUPTED并列，不改写原始停止口径。153份执行源码核验0差异，源码完整保留WSL路径/SHA；本次没有变更生产控制代码。

performance_samples.jsonl的30秒窗口实际实时因子0.422。freshness_receive_audit.jsonl为只读相机/雷达/运动接收记录；相机9/9年龄>1秒，实际六路frame_probe均无处理记录。空frames目录和0图不证明YOLO精度。final_process_audit确认自有进程和端口为空。

archive_manifest.json列96份精选文件的原始路径/SHA、Git归档路径/SHA；超过750KB使用gzip，解压字节对应原始SHA。大Gazebo/PX4/导航日志20份仅保留WSL路径/SHA，避免巨大仓库。详细问题和下一步见[报告](../../original_rate_validation_20261008.md)。
