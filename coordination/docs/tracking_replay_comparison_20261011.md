# 同画面离线图像关联对照

`tools/replay_tracking_comparison.py` 读取 `record_botsort_trial.py` 的真实 burst JPEG 与原始图像时间戳。同一 JPEG 只推理一次，共用检测、外观冲突处理及 NMS，再分别输入当前 `ImageMOT` 和固定提交 `997cfa006a41d60e58eff724c5762ca13a3d9cc3` 的 `PixelCueTracker`。

这是图像身份与像素提示的同输入对照。旧版完整世界坐标关联、桥接上报、任务分配、飞控及整场比赛结果未重放；不能据此归因旧新版整场胜率。没有人物身份标注，ID 数量与邻帧框连续性只是重建/连续性代理，不能证明真实 ID switch 或真人分类准确率。低分框仍不获取官方定位/上报权限。

JPEG 是有损编码，burst 默认每 0.2s 取一帧，且由目标上报触发选机。这些输入与完整在线原始相机流不同，不能由此计算整场召回率或声称在线追踪性能已验收。

## 执行

仿真录制由主控完成。本工具不连接 ROS、不发布消息，也不启动或停止仿真。真实 burst 尚未落盘时，只有工具验证结果，没有图像对照结果。

在 WSL 中使用已准备的专用 venv。用实际 observer 目录替换 `RECORDING`，输出必须是新的目录。无需 source ROS 环境：

```bash
RECORDING=/root/robo_team_build/botsort_sim_20261011/logs/latest/observer
REPLAY=/root/robo_team_build/botsort_sim_20261011/logs/tracking_replay_20261011_01
/root/robo_team_build/botsort_env_20261011/bin/python -B \
  /mnt/d/a/.robocup/robo_team_botsort_20261011/tools/replay_tracking_comparison.py \
  --recording "$RECORDING" --out "$REPLAY" \
  --weights /mnt/d/a/.robocup/robo_team_botsort_20261011/weights/best_yolo11n_bino_v1.pt \
  --legacy-repo /mnt/d/a/.robocup/robo_team_tracking_review_20261011 \
  --device cpu
```

如果该路径中的 `.pt` 是 Git LFS 指针，工具会报错；应传环境验证记录中已有的真实权重文件。`--device 0` 使用实际 CUDA 设备。CPU/GPU、负载与首轮模型初始化都会影响耗时，不能将不同运行环境的耗时当作算法收益。工具单列首次推理，后续推理统计不包含首次调用。

仅检查索引与固定旧代码是否可读，可以添加 `--inspect-only`。这不读取 JPEG、不加载 YOLO，也不产生追踪成绩。`--uav typhoon_h480_2` 选择单机；`--limit 100` 限制实际图像数量。默认排除 sparse 图像，`--include-sparse` 只供诊断，稀疏图像不能冒充连续 MOT 序列。

## 时间与公平比较边界

- 图像按原始 `sample_s` 排序，每个源 UAV 使用独立的新旧状态。索引还必须有 `image_saved=true`、原始 header、相对 JPEG 路径和正确 SHA-256；路径越界、图像损坏、尺寸不符及哈希错误都单独记缺证。
- 固定旧代码直接从 Git 对象读取；只执行其纯函数/类定义，原关联逻辑不重写。Windows 创建的 worktree 在 WSL 下使用映射后的 Git 目录，工具不修改 `.git`。
- 旧链的世界方位来自同机 MAVROS pose 原始 header 时刻，平移线性插值、旋转使用原代码 SLERP；最多允许 50ms 端点余量，默认相邻 pose 不超过 0.3s。没有可靠姿态时双方都跳过关联更新并记 `ABSTAIN_NO_IMAGE_TIME_POSE`，避免给新链多喂图像后再声称同输入收益；不使用接收时间或仅 yaw 猜旋转。每机固定世界平移原点会在旧关联器的相机运动重投影中抵消，因此可以使用该机局部 ENU；本工具不重建地面坐标。
- 当前 MOT 输入默认最低置信度 0.10，创建门限 red/green/blue/brown 为 0.40、white 为 0.30。旧 `PixelCueTracker` 的输入仍须满足原 `pixel_cue_box_ok`，最低置信度 0.45，形状与 ROI 门限保持原代码。`botsort_common_eligible` 与 `legacy_common_eligible` 只统计双方可比且有可靠姿态的原始帧；低分框续接另列在新 MOT 的全输入统计，不能把输入门限差异全归功于关联算法。
- 旧 update API 只返回已成熟提示；工具同时只读旧内部状态取得刚关联但未成熟的 ID。完全相同的歧义框无法确定内部归属时不猜 ID。

## 输出

`frames.jsonl` 保存每帧原始索引、JPEG 哈希、一次 YOLO 的原始检测、共享预处理后逐框的新旧 ID、双方提示、相机姿态和各阶段耗时。`summary.json` 保存权重/源代码/固定旧定义哈希、实际依赖版本、输入计数、错误与按 UAV 分开的统计。

工具先解析 `latest` 的实际目录，再一次读取固定索引字节，并记录该前缀的字节数和哈希；不会混入后来切换的另一轮试验，也不会把推理过程中新增的索引拿来冒充已回放输入。录制仍在进行时，结果为 `REPLAY_PARTIAL_RECORDING`，需在 observer 完成、`writer_drained=true` 后重放完整索引；`recorded_source_sha256` 保留录制时的源码来源。

关注各 UAV 下的 `continuity.botsort_common_eligible` 与 `continuity.legacy_common_eligible`：

- `unique_observed_ids`：实际出现的 ID 总数，多个真人进入画面也会使其增加。
- `adjacent_id_retention_ratio`：相邻有效帧中前一帧 ID 保留比例，默认连续性间隔为 0.5s；空检测帧不会补造预测框。
- `changed_id_on_unique_bbox_proxy`：同色、双方唯一且 IoU≥0.5 的相邻框上 ID 改变次数，只是局部框连续性代理。
- `observed_contiguous_id_span_s`：同一 ID 的连续真实观测跨度，不包含预测时长。

`counts.low_score_continued_old_id` 统计新 MOT 低于该颜色创建门限但续上已有 ID 的实际检测数。`botsort_ready_cues` / `legacy_ready_cues` 是成熟像素提示数，均不等于合格世界上报。`image_mot_stats.gmc_failures` 只计补偿异常；没有异常不证明补偿质量好。BoT-SORT 本次关闭 ReID，同色重叠、遮挡后身份恢复仍需要真实标注或人工核对。

没有有效 burst 图像返回 `ABSTAIN_NO_SAVED_BURST_IMAGES`；没有可比较的姿态/时间帧返回 `ABSTAIN_NO_COMPARABLE_FRAMES`；部分损坏返回 `PARTIAL_INVALID_IMAGES`；录制尚未结束返回 `REPLAY_PARTIAL_RECORDING`；完整工具回放为 `REPLAY_COMPLETE`。这些是离线工具执行状态，不是比赛 PASS。正常回放仅 `REPLAY_COMPLETE` 返回 0，缺证或部分回放返回 2；初始化错误返回 1。
