# v1.30 / 8159：完整时限5/6

执行be2945f，run_id=0bcd8961-b09a-4d59-89f0-b7aec69a5706。配置600秒自然结束，实际轨迹1930.896–2530.748（599.852秒采样跨度），原始result状态CITY_SIX_AIRCRAFT_MOVING不是比赛完成判据。裁判实际删除actor_0/1/2/3/4，最后剩余[5]；最后一次消除Time usage=505.836秒，分数954.9。

六机均解锁移动，最高真实3.552336m。原生观察150284帧覆盖1929.968–2531.100，最大间隔0.004秒；建筑0、机间0、人物7条（同一时刻）。人物模型位置与接触记录矛盾尚未解决，不得把该轮称为全程无碰撞。128份swarm执行源码加12份辅助源码SHA一致。

原始完整目录：`/root/robocup_runs/scheduled_v130_20261005T093242Z/round_1_seed_8159`。archive_manifest.json保留每份原始文件路径、字节数和SHA；大CSV/JSONL/log用gzip，gzserver.log只归档末16KiB，完整422MB日志留在WSL。window/保存独立单轮预算、授权意图和只读观察器记录。postrun_audit.json是独立审计；原始result和budget没有改写。

规划交接检查和构造回调分别见flight/planning_handoff_checkpoint.json、flight/stale_handoff_method_checkpoint.json；前者在运行中采集，后者使用真实坐标/时间但模拟回调，不是完整实际调度或物理重放。camera_sequence_uav6是后来的搜索阶段24张原图，不是2034秒跟丢窗口，不能据此推断那次人物可见性。

白色68张原图最大误差1.999749m；红色观察器的“任一真值”统计没有额外剩余身份过滤，模型瞬移前后会被插值，43m原图/83m上传最大值不能直接解释为视觉几何失真。原始记录保留。所有真值只用于诊断。正式比赛false，项目目标未完成。

自有仿真和白/红/绿观察器已退出；原图观察器核对命令后单独SIGINT，三服务端口关闭。下一步见[收尾修复计划](../../../v130_postrun_plan_20261005.md)。
