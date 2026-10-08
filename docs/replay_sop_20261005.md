# 全场复跑 SOP —— 2026-10-05 EMA 双轨 + DWA 关闭

> 目标：用一次 5 分钟完整比赛跑通，验证双轨制 EMA 是否解决"streak 永远断"、
> "远距误差超 1m 清零"、"actor 移动糊掉"三大问题。

## 1. 预热（preflight，不起任何节点）

```bash
cd /home/gycc/桌面/RoboCup_Team
./run_match.sh preflight
```

期望：所有 5 项体检通过（PX4 SITL、gazebo、`black_box.txt`、`obstacle.txt`、
`weights/best_yolo11n_bino_v1.pt`、ROBOCUP_METADATA_FILE）。

## 2. 起全场（5 分钟 hard cap）

```bash
# 默认参数（headless GUI=false，推荐首次跑，验证 base 配置）
./run_match.sh start
# 复盘看 actor 移动 / 双目画面（强制开 GUI，复盘调试用）
./run_match.sh start --gui
# 显式关 GUI（默认就是关，写出来作 reminder）
./run_match.sh start --no-gui
# 跑到 300s 自动 stop，或手 Ctrl-C
./run_match.sh stop
```

> CLI 优先级：`--gui` / `--no-gui` > 环境变量 `GAZEBO_GUI` > 默认 `false`。

想要更长 / 更短：

```bash
MATCH_HARD_CAP=600 ./run_match.sh start
```

### 2.1 Ablation 对照（不同时跑）

| 轮 | 远距门 BRIDGE_EMA_BYPASS_M | 移动门 BRIDGE_EMA_MOVE_BYPASS | 用途 |
|---|---|---|---|
| base | 10.0 | 0.5 | 默认，验证修复 |
| A 严 | 8.0 | 0.3 | 更早切透传 |
| B 松 | 12.0 | 0.7 | 更久 EMA 平滑 |

```bash
BRIDGE_EMA_BYPASS_M=8.0 BRIDGE_EMA_MOVE_BYPASS=0.3 ./run_match.sh start
```

## 3. 关键日志（每 10s 聚合写 stderr → 落到 LOGDIR）

LOGDIR 默认是脚本里 `$REPO_ROOT/.run_match/logs/<UTC戳>/`。复跑完一行命令找：

```bash
LOG=$(ls -td /home/gycc/桌面/RoboCup_Team/.run_match/logs/*/ | head -1)
echo "LOGDIR=$LOG"
ls "$LOG"
```

应看到 06_swarm_manager.log / 06_swarm_agent_*.log / 06_yolo_target_bridge.log /
score_cal.log 等（score_cal 是否单独日志看 §4.2）。

### 3.1 EMA 双轨切换计数（来自 yolo_target_bridge）

```bash
# 注意：grep 的 [EMA_DUAL] 是字符类，会误匹配；用 -F 按字面量，或转义
grep -F '[EMA_DUAL]' "$LOG/06_yolo_target_bridge.log" | tee /tmp/ema_dual.tsv
```

每行 10s 窗口，字段：`calls` `far%` `moving%` `ema_on%`。
- `ema_on%` 越低 ⇒ 越常透传（actor 移动频繁/远处多 ⇒ 期望高 far% / moving%）
- `far% + moving%` 近似 ≥ `100 - ema_on%`（互斥）

期望：base 配置下 `ema_on%` 落在 30%~70%，`far%` 10%~40%，`moving%` 20%~50%。
若 `ema_on% ≈ 100%` ⇒ 远距门 / 移动门没生效，回到单轨问题；
若 `ema_on% ≈ 0%` ⇒ actor 几乎总在动 / 永远远处，可能 track.range_m 没读对。

### 3.2 score_cal reset 频率 + streak 长度分布

```bash
grep -F '[SCORE_STATS]' "$LOG"/**/score_cal.log 2>/dev/null \
  | tee /tmp/score_stats.tsv
# 若 score_cal 没单独 log（合在某个 swarm 日志里），按关键字 grep：
grep -rhF '[SCORE_STATS]' "$LOG" | tee /tmp/score_stats.tsv
```

字段：`cb`（回调次数）`reset`（清零次数）`far_dist`（距离>1m）`discontinuous`
（间隔>1s）`streak_avg` `streak_max` `streak_buckets`。

判定标准（**修复成功的硬指标**）：
1. `streak_ge15 / streak_samples ≥ 50%`（半数以上 streak 撑过 15s 触发 find）
2. `streak_lt1 / streak_samples ≤ 10%`（几乎不再"瞬间断"）
3. `streak_avg ≥ 5s`（平均长度从原来 ~1s 升到 ≥ 5s）
4. `reset_by_distance ≤ reset_by_discontinuous`（若 EMA 真起作用，discontinuous 应为主因）

## 4. 复跑前后需要的诊断命令

### 4.1 真值 vs 上报坐标对照（可选）

```bash
# yolo_target_bridge 上报坐标的 1Hz 速率
rostopic hz /typhoon_h480_N/actor_red_info    # 单机
rostopic hz /typhoon_h480_0/actor_green_info  # 看 green 单球

# score_cal 是否仍在发 find（清零后第一个 find 是 streak 重建的标志）
rostopic echo /typhoon_h480_0/find_actor_0 -n1
```

### 4.2 score_cal 日志在哪

脚本里没有显式 `start_group` 起 score_cal.py 的行，**score_cal 由其他节点
内嵌或外部 `rosrun` 启动**。先用：

```bash
ps -ef | grep score_cal | grep -v grep
rosnode list | grep -i score
```

找到 pid/namespace 后，对应日志路径如下：
- 若脚本里有 `start_group score_cal "$LOGDIR/...log" ...` ⇒ 用该文件
- 否则用 `journalctl -u <its-launch-unit>` 或 `nohup.out`

## 5. 已知陷阱

1. **赛道坐标未读对**：`track.range_m` 是桥端的字段，若 track 模型没在 `_emit`
   时填好，`EMA_BYPASS_RANGE_M` 永不生效 → 永远走 EMA。判定：`far% = 0` 且
   `moving%` 异常低。
2. **`streak_last_start` 哨兵**：本版已统一为 `None`；若在某次回滚里出现
   `-1.0` 残留，混用会丢首轮统计（影响 1 次，不致命）。
3. **`score_cal` 进程崩溃**：基线是旧代码没 `_stats_dump` 钩子，会出
   `[SCORE_STATS]` 行但窗口未到 10s ⇒ 用 `force=True` 兜底；本版已加在
   `_finish()` 里。
4. **仿真 vs 真机**：`run_match.sh` 默认 Gazebo 仿真 + CPU ray 雷达（SKIP_RADAR=2）。
   真机场景下 ENU→map 坐标偏置会有差异，需检查 `world_offset_x/y` 是否对齐
   起飞世界坐标。

## 6. 复跑后填表

把以下数字贴回 `docs/ema_dual_track_dwa_close_20261005.md` 的"待办"段：

| 轮 | GUI | ema_on% | far% | moving% | streak_avg | streak_ge15% | reset_by_distance | 备注 |
|---|---|---|---|---|---|---|---|---|
| base | --no-gui | ? | ? | ? | ? | ? | ? | 默认（headless） |
| base-gui | --gui | ? | ? | ? | ? | ? | ? | 默认 + GUI（看 actor 移动） |
| A 严 | --no-gui | ? | ? | ? | ? | ? | ? | 8m / 0.3mps |
| B 松 | --no-gui | ? | ? | ? | ? | ? | ? | 12m / 0.7mps |
