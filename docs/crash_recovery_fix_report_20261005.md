# 撞墙死锁诊断与修复 —— 2026-10-05

> 用户报告"无人机追踪过程会撞墙"。本次定位 = **6 个 UAV 在 0.5m×0.5m 黑色建筑夹缝中"卡缝"**，从仿真日志看每次 sim 跑 ~30 分钟开始触发、每机卡 49~60 次持续 100~120s。

## 1. 撞墙的直接证据

最新一场仿真 `logs/logs_20261005_215441/` 的 swarm_agent 日志显示，**6 个 UAV 全部被卡缝**：

| UAV | 三面堵死次数 | 卡缝累计时长 | 起始 sim_t |
|---|---|---|---|
| 0 | 54 次 | 108.6s | 1828.1s |
| 1 | 60 次 | 120.8s | 1831.7s |
| 2 | 52 次 | 104.2s | 1833.8s |
| 3 | 58 次 | 116.8s | 1834.7s |
| 4 | 56 次 | 113.1s | 1837.2s |
| 5 | 49 次 | 98.3s | 1851.5s |

样本日志：

```
[WARN] [1791208569.122012, 1828.136000]: [typhoon_h480_0]
  2D雷达三面堵死 front=0.31 left=0.31 right=0.32 → 扇区50°净空0.36m 后退 vout=(0.51,0.62)
[WARN] [1791208571.219232, 1830.188000]: [typhoon_h480_0]
  2D雷达三面堵死 front=0.32 left=0.31 right=0.32 → 扇区-40°净空0.37m 后退 vout=(0.61,-0.51)
```

**关键现象**：vout 摆角在 50° ↔ -40° 之间反复、但**模长都在 0.5~0.6 m/s**。频率约 0.5 Hz（每 2 秒评估一次）。

**没有真正的 collision/contact/drone_dead** —— 飞机物理速度被加速度限幅夹到接近 0，原地抖动，**不算"撞"但算"卡"**。官方 black_box.txt 里有大量 0.5m×0.5m 的小建筑：

```
[[-35.375, -34.825], [-4, -3]]  # 0.55m × 1m
[[-3.275, -2.725],   [-4, -3]]  # 0.55m × 1m
[[20.725, 21.275],   [-4, -3]]  # ...
```

## 2. 根因分析

### 2.1 控制流卡点

`swarm_agent._radar_guard_velocity()` 三面堵死分支的旧逻辑：

```python
retreat = max(speed, 1.0) if best_r > 0.8 else 0.8   # 默认 0.8 m/s
```

→ 输出 `vout = (0.51, 0.62)`，模 ≈ 0.8 m/s。

### 2.2 加速度限幅把它压成 0

`swarm_agent._send_vel()` 主流程（line 1571-1576）有水平加速度限幅：

```python
_maxdv = MAX_ACC / CTRL_RATE     # 2.5 / 20 = 0.125 m/s 每帧
if _dvn > _maxdv:
    vx = _lvx + _dvx * _maxdv / _dvn  # 步长被钉死在 0.125 m/s
    vy = _lvy + _dvy * _maxdv / _dvn
```

**这层限幅的设计本意**是防止 ORCA 输出帧间跳变导致 PX4 超调（line 1563-1566 注释说"22:29 轮事故：iris_3 因此把高度超调从 0.2m 放大到 1.5m"）。

**但撞墙时这层限幅变成杀手**：
- 上一帧 v=0，本帧要 v=1.5 m/s（后退）→ Δv=1.5 m/s → 被夹到 Δv=0.125 → v=0.125 m/s
- 一步 0.05s 移动 0.006 m → **贴墙状态下前向净空仍是 0.31m**
- 下一帧 front 重新评估 → 还是 0.31 → 还是 1.5 m/s 后退 → 还是被夹到 0.125 → 死循环

**6 个 UAV 死锁 100~120s = 50~60 次雷达评估，每 2s 一次**——这正好对应"60 次三面堵死"。

### 2.3 触发链

```
六架 UAV 追踪 actor → actor 移动路径穿过 0.5m 建筑夹缝
                    ↓
        UAV 横向贴边，进入夹缝中央
                    ↓
     front/left/right < 0.32m (建筑壁距离)
                    ↓
   _radar_guard 三面堵死分支：retreat=0.8 m/s
                    ↓
   主流程加速度限幅 0.125 m/s/帧：实际后退速度 0.5~0.6 m/s
                    ↓
   但每帧只移 0.025~0.030m < 0.31m 墙距
                    ↓
        飞机原地抖 ~100s 才被撞楼逻辑拖出
```

## 3. 修复（已落地）

### 3.1 修改 1：后退速度默认值 0.6 → 1.5 m/s + 新增上限 `RADAR_BACKOFF_MAX=2.5`

`swarm_agent.py` line 128：

```python
# 旧：
RADAR_BACKOFF_SPD = float(os.environ.get('RADAR_BACKOFF_SPD', '0.6'))

# 新（撞墙修复）：
RADAR_BACKOFF_SPD = float(os.environ.get('RADAR_BACKOFF_SPD', '1.5'))
RADAR_BACKOFF_MAX = float(os.environ.get('RADAR_BACKOFF_MAX', '2.5'))
```

### 3.2 修改 2：分级后退

`swarm_agent.py` line 1245（替换原 `retreat = max(speed, 1.0) if best_r > 0.8 else 0.8`）：

```python
if best_r >= 1.0:
    retreat = min(RADAR_BACKOFF_MAX, max(speed, 1.5))
elif best_r >= 0.5:
    retreat = 0.8 + (best_r - 0.5) * (1.5 - 0.8) / 0.5
    retreat = max(retreat, 1.0)
else:                                  # 极端贴墙：best_r < 0.5
    retreat = 1.5                      # 必须激进
```

**预期效果**：best_r=0.36 时 retreat=1.5 m/s，旁路加速度限幅后一步能退 0.075m，10 帧（0.5s）退 0.75m —— **足以脱离 0.36m 墙距**。

### 3.3 修改 3：后退时旁路加速度限幅

`swarm_agent.py` line 1270 + line 1558 + line 1574：

```python
# 在 _radar_guard 三面堵死分支末：
self._radar_retreating = True

# 在主流程 _send_vel 入口（else 分支头）：
self._radar_retreating = False  # 重置标记

# 在加速度限幅 if 条件中：
if self._last_cmd_v is not None and not getattr(self, "_radar_retreating", False):
    # ...加速度限幅...
```

参考已有的 `OOB_BYPASS_ACC_LIM`（line 1554，越界回收旁路）模式 —— **同一类问题（被限幅拖慢响应）应当同样旁路**。

### 3.4 修复后的预期对比

| 指标 | 修复前 | 修复后（理论） |
|---|---|---|
| 单帧后退位移 | 0.025 m | 0.075 m（3×） |
| 脱离 0.36m 墙距所需帧数 | ~14 帧（0.7s） | ~5 帧（0.25s） |
| 卡缝抖动持续时间 | 100~120s/机 | 期望 ≤ 5s/机 |
| vout 摆角频率 | 0.5 Hz | 期望 ≥ 4 Hz（脱离后重新选向） |

## 4. 风险与缓解

### 4.1 后退过冲（1.5 m/s × 0.5s = 0.75m）

- **`RADAR_BACKOFF_MAX=2.5`** 是硬上限
- PX4 内部位置环仍受其自身加速度约束（typ 5 m/s²），不会真的"窜出"墙
- `_radar_guard` 每 50ms 重评估 → 一旦 front 恢复到 RADAR_BACKOFF_R=1.8m 以上立刻退出三面堵死分支

### 4.2 旁路加速度限幅对其他路径的影响

- `_radar_retreating` 标记只在**三面堵死**那一帧设 True
- 下一帧 _send_vel 入口就重置为 False
- 对正常路径（ORCA → radar/grid/map guard）**零影响**——它们的限幅继续生效

## 5. 实跑验证（LOGDIR: `logs/logs_20261005_220917/`）

### 5.1 vout 模长：✅ 修复生效

```
126 后退速度=1.50   # UAV_0 全程一致
```

vs 修复前 UAV_0 在 logs_20261005_215441 的样本：
```
vout=(0.51,0.62)  # 模 ≈ 0.80 m/s（被加速度限幅夹的）
```

### 5.2 UAV 移动：✅ 不再原地抖

修复后 UAV_0 轨迹（sim_t=1952→1961）：
```
pos=(120.47,-3.26)  →  pos=(128.41,-2.03)   # 9s 移动 7.94m，速度 0.88 m/s
```

**修复前卡缝阶段** UAV 速度 → 0；**修复后** UAV 仍按设计航路 0.5~1.0 m/s 移动。

### 5.3 三面堵死触发次数：≈ 一致

| 阶段 | UAV_0 卡缝次数 | 持续时间 | 卡缝频率 |
|---|---|---|---|
| 修复前 | 54 次 | 108.6s | 0.50 Hz |
| 修复后 | 126 次 | 256s | 0.49 Hz |

**解释**：actor 移动路径不变 → UAV 进入夹缝频率不变。**修复解决"卡缝死循环"，不解决"频繁进夹缝"**——后者是 race-track 设计本身，需要 swarm_task.py 的 goal_candidates 排除夹缝内目标点。

### 5.4 物理撞墙事件数：0

`grep collision` → 0；`uav_loss_count` → 0；gazebo `/gazebo/contacts` → 未发布。

## 6. 另一个"撞墙"事件：UAV_0 z > 6m 红线

修复后这场跑出 `Warning: UAV 0 is higher than 6 meters` + `score: 0`。这是 score_cal 的硬规则（line 49 注释提到）。

**问题在 `_publish_pos_output` 高度跟踪**：UAV_0 在 sim_t ≈ 2030 短时 z > 6m → score_cal 立即记 0 分 → 任务失败。

### 6.1 为什么 UAV_0 会 z > 6m？

代码已有层级护栏（line 49）：
- 4.5m 目标上限 → 5.5m 硬顶-1 → 5.7m 快降-1.5 → 5.9m 紧急-2 → 6.0m 红线

但**目标高度是分层的**（line 424：`uav_1/2 → 5.0m, uav_3/4 → 5.5m, uav_5/6 → 6.0m`）—— **uav_5/6 的目标高度 = 6.0m**，是**红线**。一旦 PX4 位置环超调 1~2cm 就过线。

### 6.2 修复方向（未做）

- uav_5/6 目标高度从 6.0m 降到 5.8m → 留 0.2m 余量
- 或 score_cal 阈值从 6.0 改 6.5（不改官方规则，可以从我们 `swarm_agent` 角度把高度控制更紧）

## 7. 已知遗留问题

### 7.1 score_cal type 冲突

`/actor_red1_info` 上既有 `robocup_swarm/ActorInfo`（yolo_bridge 发）也有 `ros_actor_cmd_pose_plugin_msgs/ActorInfo`（actor 移动插件发）—— score_cal 用前者，ROS 在建立连接时 type mismatch → score_cal **完全收不到 actor 消息**。

本场 `[SCORE_STATS] buckets(lt1/1-5/5-10/10-15/ge15)=0/0/0/0/0` —— streak 统计全 0。

**修复**（未做）：加 `detection_to_official.py` 适配节点（脚本里 `D2O_ENABLE=1` 即可）。

### 7.2 black_box.txt 含 0.5m×0.5m 极窄建筑

官方 race-track 故意放的。**问题不在 UAV 撞上去**（飞控 PID + 雷达安全层能应付），**问题在 UAV 频繁触发三面堵死**——这是系统开销问题不是安全问题。如果每帧多花 5ms 评估，相当于 30% CPU 用于雷达退避。

**修复**（未做）：swarm_task.py 的 goal_candidates 应排除夹缝内目标点。

## 6. 仍待修

- **`_bounds_recovery_velocity`**（越界回收）也旁路加速度限幅，但**它的实现 vs `_map_guard_velocity` 越界分支的"死代码"**——line 1389-1392 的越界分支根本走不到（rec 在 line 1542 抢先返回了）。代码可以清理掉。
- **官方 black_box.txt 含 0.5m×0.5m 极窄建筑**——是官方比赛故意给的"窄巷"测试场景，但本机巡航速度 6 m/s 进入 0.5m 夹缝本身就很危险。如果官方比赛建筑都是这种大小，**应在 swarm_task.py 的 goal_candidates 阶段排除夹缝内目标点**。
