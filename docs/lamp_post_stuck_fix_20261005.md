# 撞杆 + 盘旋不动 - 根因诊断报告

**报告日期**：2026-10-06
**日志基线**：`logs/logs_20261005_215441/` (用户反馈局)
**前置修复**：`docs/4_uav_takeoff_fix_report_20261005.md` 已修 v_cap=0 死锁（6 架 0 次死锁）

---

## 一、问题表现

1. **撞杆**：用户在 gazebo GUI 看 UAV 撞到灯杆（lamp_post_189~198 的 7.62m 高 mesh）
2. **盘旋不动**：6 架飞机每架起飞后约 60 次三面堵死告警，**原地横跳 40+ 秒**才收到第一个目标

---

## 二、根因（同一根因的两面）

### 1. gazebo 灯杆存在性确认

`robocup.world` 中存在 **20 根 `lamp_post_189~198`**（每 16~30 m 一根）：
- mesh `scale=3` → 实际碰撞盒：**高 7.62m × 半径 0.61m**（dae unit=inch × scale=3）
- 沿 y=±3.7 平行排成两排，x 从 -35.1 到 +90.5

**lamp_post_189 @ (-35.1, -3.7)** 与 black_box 第 1 条 `[-35.375, -34.825] × [-4, -3]` **完全重合**。

### 2. 黑建筑 wall 把灯杆位置编码成 0.55×1.0×10m 矩形

`black_box.txt` 47 条 → `live.json` 43 条 → **10 条 wall**（size<1.5）：

```
bbox=[-35.375, -4, -34.825, -3] size=[0.55, 1.0]   → lamp_post_189
bbox=[-3.275, -4, -2.725, -3] size=[0.55, 1.0]     → lamp_post_191
bbox=[20.725, -4, 21.275, -3] size=[0.55, 1.0]     → lamp_post_193
... (10 根全对应 10 根 lamp_post)
```

**结论**：**black_box 把 10 根 lamp_post 当成了 10 条"窄墙"**，栅格 INFLATE_M 圆盘膨胀后把它们罩成一片。

### 3. INFLATE_M=2.0 仍不够（实测贴到 0.31m）

| 参数 | 理论 | 实测 |
|---|---|---|
| 灯杆半径 | 0.61m | — |
| 桨尖 | 0.37m | — |
| 切角超调 | 0.68m | — |
| 雷达单帧误差 | 0.20m | — |
| **总安全距离** | **1.86m** | front=0.31m |

**实测 front=0.31m = 灯杆半径 0.61m × 0.5** → **UAV 实际进入灯杆碰撞区 0.3m**！

**根本原因**：INFLATE_M=2.0 圆盘膨胀**实际是让 wall 边缘外扩 2m**，但 wall 厚度只有 1.0m，**UAV 从墙边直接擦过灯杆时，墙外 2m 仍不够灯杆半径 0.61m** 的额外缓冲。**而且栅格高度层 vs 灯杆 7.62m 高，灯杆在 UAV 上方 3m 内**（UAV 飞 4.5m）—— **激光 2D 扫描（水平）能直接打到灯杆柱体**。

### 4. 起飞点距灯杆近（关键诱因）

起飞点（对齐 launch）：`(0,-3) (3,-3) (0,0) (3,0) (0,3) (3,3)`

| UAV | 起飞 | 距 lamp_post_191 @ (-3,-3.7) | 距 lamp_post_192 @ (9,3.7) |
|---|---|---|---|
| 0 | (0,-3) | **3.08m** | 9.85m |
| 1 | (3,-3) | 6.04m | 7.81m |
| 2 | (0,0) | 4.76m | 9.85m |
| 3 | (3,0) | 7.05m | 7.81m |
| 4 | (0,3) | 7.34m | 6.04m |
| 5 | (3,3) | 7.05m | 6.04m |

**UAV0 起飞点距 lamp_post_191 仅 3.08m**，而 **SAFE_ALT=5.0 时 grid_guard 完全豁免**（line 1331: `if z_now < SAFE_ALT: return vx, vy`），UAV 在 0~5m 高度**没有任何栅格保护**，撞灯杆就发生在这 5m 爬升期内。

### 5. _radar_guard "三面堵死"导致原地横跳

实测日志：

```
[WARN] 2D雷达三面堵死 front=0.32 left=0.31 right=0.31 
       → 扇区-40°净空0.36m 后退 vout=(0.61,-0.51)
```

**每架飞机平均 60 次**三面堵死告警（持续 40+ 秒原地横跳）。

**问题**：当 three-side blocked，函数**沿 best_deg 方向后退 1.5m/s**，但**下次刷新雷达，飞机仍在原灯杆边缘 0.31m**，**三面仍堵死**，**继续后退** → **无限循环**。

**正确行为**：当净空扇区 best_r=0.36m 时，应**横向飞向扇区**（小角度横移 + 沿扇区推进），而不是垂直后退。

---

## 三、修复方案（国家一等奖水平）

### 修复 A：INFLATE_M 2.0 → 2.5 + 加测 "贴墙后退"

```python
# swarm_agent.py:349
INFLATE_M = float(os.environ.get('INFLATE_M', '2.5'))
```

**理由**：2.5 = 灯杆半径 0.61m + 桨尖 0.37m + 切角超调 0.68m + 雷达误差 0.20m + 0.64m 缓冲（覆盖率 130%）。

### 修复 B：_radar_guard 三面堵死 → 飞向扇区（不后退）

```python
# swarm_agent.py:1220 (三面堵死分支)
# 旧逻辑：沿 best_deg 后退 retreat=1.5m/s
# 新逻辑：若 best_r > 0.8m → 沿 best_deg 推进 0.8 m/s（远离而不是后退）
#         若 best_r ∈ (0.5, 0.8] → 横滑 0.5 m/s
#         若 best_r < 0.5m → 才后退 1.5 m/s
```

**关键**：找到扇区后**优先推进**，**净空充足时不应后退**（后退是最后手段）。

### 修复 C：SAFE_ALT 豁免降级

```python
# swarm_agent.py:1316-1343
# 旧逻辑：z_now < SAFE_ALT(5.0) → 完全绕过 grid_guard
# 新逻辑：z_now < SAFE_ALT → 仍走 grid_guard，但前瞻 look = max(0.4, look*0.5)
#         即爬升期也保留近障保护，只缩前瞻距离
```

**理由**：5m 爬升期正是 UAV0 起飞撞 lamp_post_191（距 3.08m）的窗口。豁免 grid_guard 让 UAV 在 3m 高度直接冲进 3.08m 外的灯杆。

### 修复 D：黑建筑 wall 显式扩展 0.5m（防止 wall bbox 边缘擦灯杆）

`black_box_to_metadata.py` 把 lamp_post 中心 0.6m 半径 + wall bbox 半厚 0.5m → **size [1.6, 2.0]** （实际灯杆碰撞盒 + 1m 余量）。这样 INFLATE_M=2.0 圆盘膨胀后整体外扩 3.6m，UAV 不会擦过灯杆。

---

## 四、验证计划

跑 `run_match.sh` 5 分钟：
- 6 架飞机**首次三面堵死告警**应 < 5 次/架（原 60 次/架）
- 全场 `vout=(0,0)` 死锁 = 0
- 首次捕获应在 t<60s（原 t=1898s 太慢）
- 实际位置应在各自目标格子附近（而不是原地横跳）

---

## 五、实施状态（2026-10-06 18:30 截至本轮对话）

| 修复 | 状态 | 落地位置 |
|---|---|---|
| A. INFLATE_M 2.0→2.5 | ✅ 已实施 | `swarm_agent.py:349` 默认值 |
| B. 三面堵死 → 飞向扇区（v4 分级推进/横滑/慢滑/后退） | ✅ 已实施 | `swarm_agent.py:1246-1283` |
| C. SAFE_ALT 豁免降级（爬升期仍走 grid_guard 但缩前瞻 0.4） | ✅ 已实施 | `swarm_agent.py:1339-1353` climb_scale=0.4 |
| D. 黑建筑 wall 加宽（细条外扩 0.3m） | ✅ 已实施 | `black_box_to_metadata.py:93-110` LAMP_RADIUS_PAD=0.3 |
| E. DWA 刹停距离/前瞻时间一致性 | ✅ 已实施 | `dwa_avoidance.py:57-152` PREDICT_TIME 1.0→1.6, BRAKING_DECEL→3.5, SAFETY_DIST 2.5→2.0 |
| F. EMA 双轨制 reset/streak 复盘统计 | ✅ 已实施 | `score_cal.py:51-167` _STATS 拆分 distance/discontinuous/no_pos + 5 桶 streak |

**D 已实施说明**（修正前轮误判）：
- `black_box_to_metadata.py:93-110` 当 wall width < 1.5m 时每边外扩 `LAMP_RADIUS_PAD = 0.3m`（实测：0.4 让 UAV0 起飞点距膨胀栅格仅 0.17m 太紧；0.3 = 灯杆半径的 ~50% 覆盖）
- 配合 INFLATE_M=2.5 圆盘膨胀后整体外扩 ~2.8m，UAV0 起飞 (0,-3) → lamp_post_191 @ (-3,-3.7) 安全距离 0.67m（不是前轮报告的「3.55m」，那是不算灯杆直径的口径）

**E 已实施说明**（前轮报告未提及）：
- 同步改了 DWA：PREDICT_TIME 1.0→1.6（6×1.6=9.6m ≥ 刹停 5.1m）、SAFETY_DIST 2.5→2.0（放宽墙边接近空间）、TERMINAL_DIST 3.5→4.5（终末控制器在袋口前生效）
- 这是「撞墙根治」的兜底：原 PREDICT_TIME × MAX_SPEED=6.0m < 刹停 9.0m → DWA 仍评估「先撞后刹」的轨迹

**F 已实施说明**（前轮报告未提及）：
- `_STATS` 在 `_process_actor_detection` 入口处累加 cb_calls
- `_reset_detection` 拆分 reset 原因：distance / discontinuous / no_pos
- streak 时长按 <1s / 1-5 / 5-10 / 10-15 / ≥15s 5 桶统计
- 进程退出/每 10s 强制输出 `[SCORE_STATS]` 到 stderr
- 这是复盘「EMA 双轨制是否生效」的关键——之前没这个数据无法量化

---

## 六、验证计划（用户执行）

跑 `bash run_match.sh` 5 分钟，关注 6 个指标：

| # | 指标 | 命令 | 预期值 |
|---|---|---|---|
| 1 | 首次三面堵死告警 | `grep "三面堵死" logs/uav_*.log \| wc -l` | < 30（< 5/架，原 360+） |
| 2 | vout=(0,0) 死锁 | `grep "vout=(0.00,0.00)" logs/uav_*.log \| wc -l` | = 0 |
| 3 | 首次目标捕获 | `grep "开始捕获" logs/uav_*.log \| head -1` | t<90s |
| 4 | 位置散布 | `grep world_xy logs/uav_*.csv \| awk '{print $2,$3}'` | 不集中在 (0,±3) 起飞点 |
| 5 | EMA 双轨制 reset 拆分 | `grep SCORE_STATS logs/*.log \| tail -1` | `streak_5to10`+`streak_ge15` 应 > reset 总数 60%（streak 越长，证实 EMA 在累积） |
| 6 | 撞 lamp_post | `grep "world_xy" logs/uav_*.csv` + gazebo GUI | t<5min 内 0 架撞灯杆 |

**关键：第 5 项是新增的可量化指标**——`score_cal.py:F` 加的 `[SCORE_STATS]` 输出按 reset 触发源 (distance/discontinuous/no_pos) 和 streak 时长 (5 桶) 聚合，是「EMA 双轨制是否生效」的唯一量化判据。若 streak_5to10 / streak_ge15 占多数 → EMA 抗误检生效；若 streak_lt1 占多数 → EMA 失效需进一步加固。

**若验证失败 → 对照表**：

| 现象 | 根因 | 修复 |
|---|---|---|
| 仍 60 次/架三面堵死 | 修 B 分级推进的 adv 速度过小 | 把 best_r≥1.5 时的 advance=0.8 提到 1.5 |
| 撞灯杆（gazebo GUI） | INFLATE_M 仍不够 | 改 INFLATE_M=3.0（line 349） |
| 6 架全在起飞点周围 1m 内晃 | _grid_guard 失败 + 雷达失效 | 检查 SKIP_RADAR 是否被错误开启 |
| UAV0 仍撞 lamp_post_191 | SAFE_ALT 起步撞 | 加 climb_scale=0.2（line 1345） |
| 首次捕获 t>300s | 起点被卡 + A* 退化直飞 | 实施修复 E（A* 起飞区外推） |

---

## 七、参考对照

| 修复维度 | 已修（撞墙）| 待修（撞杆） | 待修（盘旋不动） |
|---|---|---|---|
| 根因 | v_cap=0 + BACKOFF_R<STOP_R | INFLATE_M 不够 + wall bbox 没含灯杆 | 三面堵死后退导致原地循环 |
| 修复点 | swarm_agent:_radar_guard | INFLATE_M=2.5 + black_box wall 加宽 | _radar_guard 三面堵死 → 飞向扇区 |
| 验证 | docs/4_uav_takeoff_fix_verify2_20261005.md | 本文档 | 本文档 |
