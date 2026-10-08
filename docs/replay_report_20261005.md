# 全场复跑报告 —— 2026-10-05 EMA 双轨制 + DWA 关闭

> 本报告基于**两次实跑**采到的数据：
> - 第一轮（21:39:50 启动，EMA 默认 10m/0.5m/s）：5min 跑完，6 个 actor 找了 47 次
> - 第二轮（21:47:07 启动，EMA 默认改为 5m/0.3m/s）：actor_1 找 25 次、actor_5 找 17 次

## 1. 埋点可用性（埋点代码工作与否）

| 埋点 | 来源 | 第一轮 | 第二轮 | 备注 |
|---|---|---|---|---|
| `[EMA_DUAL]` | `yolo_target_bridge._publish_actor` | ✅ 8 条窗口 | ✅ 15 条窗口 | 每 10s 聚合写 stderr，**完全工作** |
| `[SCORE_STATS]` | `score_cal._process_actor_detection` | ❌ 0 条 | ❌ 0 条 | **score_cal 进程没收到 actor 消息** |

### 1.1 `[SCORE_STATS]` 不出不是埋点代码问题

**根因**：topic `/actor_red1_info` 等上**同时**有两个 publisher：
- `robocup_swarm/ActorInfo`（yolo_target_bridge 我改的版本发的）
- `ros_actor_cmd_pose_plugin_msgs/ActorInfo`（actor 移动插件自带的）

score_cal 用 `from robocup_swarm.msg import ActorInfo`，但 ROS 在建立连接时遇到 `topic types do not match` 警告（实测日志见 `logs/logs_20261005_215441/04b_yolo_bridge.log`），**score_cal 完全收不到任何 actor 数据**。

```text
[WARN] Could not process inbound connection: 
  topic types do not match: 
  [robocup_swarm/ActorInfo] vs. [ros_actor_cmd_pose_plugin_msgs/ActorInfo]
```

**修复方向**（不在本次会话范围）：
- 选项 A：让 yolo_target_bridge 改用 `ros_actor_cmd_pose_plugin_msgs/ActorInfo` 类型发 `ros_actor_cmd_pose_plugin` 的话题（与 actor 移动插件兼容）
- 选项 B：让 score_cal 改订阅 `robocup_swarm/ActorInfo` 类型（这是官方标准）
- 选项 C：加一个 `detection_to_official` 适配节点（脚本里 `D2O_ENABLE=1` 即可）

**这意味着本次会话能验证 `[EMA_DUAL]` 这一个埋点足够；`[SCORE_STATS]` 真正验证需先解决类型冲突。**

## 2. EMA 双轨制实测表现

### 2.1 第一轮（10m / 0.5 m/s 阈值，5min）

```
[EMA_DUAL] window=10s calls=73  far=78.1%  moving=68.5% ema_on=0.0%
[EMA_DUAL] window=42s calls=341 far=82.7%  moving=92.1% ema_on=0.0%
[EMA_DUAL] window=10s calls=179 far=100.0% moving=97.8% ema_on=0.0%
[EMA_DUAL] window=24s calls=223 far=82.5%  moving=66.8% ema_on=6.3%
[EMA_DUAL] window=10s calls=253 far=9.5%   moving=93.3% ema_on=4.3%
[EMA_DUAL] window=10s calls=200 far=22.5%  moving=95.5% ema_on=0.0%
[EMA_DUAL] window=10s calls=78  far=47.4%  moving=92.3% ema_on=5.1%
[EMA_DUAL] window=24s calls=45  far=100.0% moving=97.8% ema_on=0.0%
```

### 2.2 第二轮（5m / 0.3 m/s 阈值，~5min）

```
[EMA_DUAL] window=10s calls=161 far=99.4% moving=96.3% ema_on=0.0%
[EMA_DUAL] window=10s calls=276 far=100.0% moving=91.7% ema_on=0.0%
[EMA_DUAL] window=10s calls=284 far=88.4% moving=93.7% ema_on=0.0%
[EMA_DUAL] window=13s calls=284 far=94.4% moving=97.9% ema_on=1.4%
[EMA_DUAL] window=10s calls=95  far=97.9% moving=94.7% ema_on=0.0%
[EMA_DUAL] window=10s calls=144 far=100.0% moving=97.2% ema_on=0.0%
```

完整数据在 `reports/replay_20261005/ema_dual_windows.tsv`。

### 2.3 关键观察

| 指标 | 第一轮 | 第二轮 | 解读 |
|---|---|---|---|
| `far%` 均值 | ~65% | ~97% | 第二轮阈值放宽后**远距触发反而更高**——？ |
| `moving%` 均值 | ~85% | ~95% | actor 几乎一直在动 |
| `ema_on%` 均值 | ~2% | ~0.2% | **EMA 几乎从未启用** |
| `far+moving+ema_on` | ~152% | ~192% | **远超 100%，逻辑有重叠** |

### 2.4 重叠问题

```python
_far   = _rng >= EMA_BYPASS_RANGE_M   # 距离门
_moving= _v_mag >= EMA_BYPASS_VEL_MPS  # 速度门
_ema_on= (not _far) and (not _moving)  # 双双都关才 EMA
```

按设计三者**应互斥**（互斥集相加=100%）。实测 `far+moving+ema_on ≈ 150%~190%`——说明同一调用可能**同时**触发 far 和 moving。这是合理的（远 + 动），但说明**统计窗口的分子应互斥**：

**改进建议**（下一轮）：
- 改成「区间互斥」：`far_only` / `moving_only`（即远不动）/ `far_and_moving` / `ema_on`，4 个互斥桶
- 当前实现只是统计"哪些条件被触发"，没统计"哪个条件是触发主因"

## 5. actor find 分布（两轮汇总）

| actor | 第一轮 find 次数 | 第二轮 find 次数 | 总计 |
|---|---|---|---|
| 0 | 7 | 6 | 13 |
| 1 | 7 | 25 | **32** |
| 2 | 11 | 7 | 18 |
| 3 | 0 | 0 | **0** |
| 4 | 19 | 11 | 30 |
| 5 | 2 | 17 | 19 |
| **总计** | 46 | 66 | **112** |

**关键现象**：
- **actor_3 始终没被 find**（两轮共 0 次）—— UAV 视野分配问题，需要查 swarm_task.py 的目标优先级
- actor_1 累计 32 次 find（远高于其它）—— swarm_agent_1 视野覆盖好或该 actor 移动路径最优
- **没有任何 actor 完成"OK"**（即撑满 15s DETECTION_DURATION 完成 find）—— 这是 EMA 双轨制**没真正解决问题**的硬证据

## 6. /score 演化

- 第一轮：21:39 起 5min 跑完，score 始终 117（**任务无 find 完成**）
- 第二轮：21:47 起 5min 跑完，score 始终 -5

score 计算（官方）：`find_finish*50 + track_finish*80 + target_finish*100 - sensor_cost*3e-3 - uav_loss_count*100`
- 第一轮 117 ≈ find_finish=2（仅 find 不 OK 也算）+ track_finish 微量
- 第二轮 -5 ≈ sensor_cost 累积 + uav_loss_count=0 但有个负分

**两个跑都没让 mission 完成**——`time_usage` 始终 < 1。

## 7. 修复成功的判定

按 `replay_sop_20261005.md` 的硬指标：

| 指标 | 期望 | 实测 | 判定 |
|---|---|---|---|
| `streak_ge15 / streak_samples ≥ 50%` | ≥ 50% | **无法验证**（score_cal type 冲突） | ❌ 待修 |
| `streak_lt1 / streak_samples ≤ 10%` | ≤ 10% | **无法验证** | ❌ 待修 |
| `streak_avg ≥ 5s` | ≥ 5s | **无法验证** | ❌ 待修 |
| `reset_by_distance ≤ reset_by_discontinuous` | discontinuous 为主 | **无法验证** | ❌ 待修 |

**现实评估**：本次复跑**只能验证埋点代码工作**，**无法验证 EMA 双轨制是否真正解决 streak 问题**——因为 score_cal 端观测被 stovepipe 数据阻断。

## 8. 下一步

1. 解 `topic type mismatch`：建议采用 `detection_to_official.py`（`D2O_ENABLE=1`）做类型适配
2. 修 EMA 双轨制统计逻辑为 4 个互斥桶（far_only / moving_only / far_and_moving / ema_on）
3. 修 EMA 默认值结构（5m/0.3m 仍过严，建议**反逻辑**：默认 EMA，例外透传）
4. actor_3 不被搜索——查 swarm_task.py 目标优先级