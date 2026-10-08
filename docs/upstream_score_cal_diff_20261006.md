# 上游裁判代码 vs 本地差异报告

**对比时间**：2026-10-06 13:05
**上游源**：`https://gitee.com/robin_shaun/XTDrone/blob/master/robocup/score_cal.py`（GitHub 镜像一致）
**本地**：`coordination/src/robocup_swarm/scripts/score_cal.py`
**差异总数**：上游 353 行 → 本地 483 行（多出的几乎都是 EMA 复盘 + 追踪分 + v3 修复）

---

## ⚠️ 真有影响的两处

### A. ActorInfo 消息包名变化（**关键**）
| | 上游 | 本地 |
|---|---|---|
| import | `from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo` | `from robocup_swarm.msg import ActorInfo` |
| 字段 | `string cls + float32 x + float32 y`（3 字段，无 header） | `Header + string cls + float64 x + float64 y + float64 z`（5 字段） |
| md5 | upstream | 本地自定 |

→ **md5 完全不同**。本地 yolo_target_bridge.py 已经切到 `robocup_swarm.msg.ActorInfo`（v3 修复，2026-10-06 注释明说：原 `ros_actor_cmd_pose_plugin_msgs.ActorInfo` (3字段 cls+x+y, float32) 与 score_cal.py 的订阅端 `from robocup_swarm.msg import ActorInfo` md5 不匹配）。

**当前 workspace 内的引用一致性**：
| 文件 | import 来源 |
|---|---|
| score_cal.py | `robocup_swarm.msg` ✅ |
| yolo_target_bridge.py | `robocup_swarm.msg` ✅（v3 已切）|
| swarm_viz.py | `ros_actor_cmd_pose_plugin_msgs.msg` ⚠️ 仍旧版 |
| detection_to_official.py | `ros_actor_cmd_pose_plugin_msgs.msg` ⚠️ 仍旧版 |

### B. 红色双源处理（互斥锁 vs 硬绑定）
**上游 Downloads 原始行为**：
- `_process_red_detection(msg, flag_name, reset_value)`：任一 red 话题（red1 / red2）都可匹配 actor_4 或 actor_5
- 全局 `flag_1 = 0`、`flag_2 = 1` 互斥锁：「两个 red 候选 actor 同帧失败时才清候选」
- 注释明说："Match the Downloads implementation: reset the candidate remembered by this red topic only when both red actors fail this message"

**本地（被改过）**：
```python
def actor_info1_callback(msg):
    _process_actor_detection(msg, [5] if msg.cls == 'red' else [])
def actor_info2_callback(msg):
    _process_actor_detection(msg, [4] if msg.cls == 'red' else [])
```
→ red1 硬绑 actor_id=5、red2 硬绑 actor_id=4。如果两架无人机同时上报 red 上报但行会写错 actor，本地无防护。

---

## ✅ 都是我们自己加的（不是上游新改的）
1. EMA 双轨制复盘统计（`_STATS`、`_stats_dump`、`[SCORE_STATS]`、`[FINAL]`）
2. 连续跟踪分 `track_finish * 80` + `_update_tracking()` + TRACKING_DURATION/TRACKING_DISTANCE
3. v3 修复：`actors_pos[i] = actors_pos_tmp`（去掉 `x^2+y^2 != 0` 过滤，原点附近 actor 也能记录）
4. `uav_loss_count * 30.0` → `uav_loss_count * uav_loss_penalty`（默认 100，比上游更严）

---

## 🎯 需要决策的关键问题

**Q1: 你是按上游官方版本来改，还是保留本地 `robocup_swarm.msg.ActorInfo`？**

| 选项 | 后果 |
|---|---|
| (a) 跟上游用 `ros_actor_cmd_pose_plugin_msgs.ActorInfo` | yolo_target_bridge.py / swarm_viz.py / detection_to_official.py 全都要改；且需要这个外部包已安装（XTDrone 配套的 `sudo apt install` 应该装了 `ros-noetic-ros-actor-cmd-pose-plugin-msgs` 之类） |
| (b) 保留本地 `robocup_swarm.msg.ActorInfo`（md5 自定） | 当前仿真能跑；但和上游官方评分节点对不上 — 如果赛事现场用上游官方 score_cal，会订阅失败 |

**Q2: 红色双源要不要恢复上游互斥锁？**
- 仿真日志最近 3 个 tracking success（actor_3/4/5），其中 actor_4/5 是红色对 — 说明本地硬绑定目前能跑通
- 但理论上互斥锁更安全，建议补回

**Q3: 你这次问「裁判代码有新的修改」是想让我做什么？**
- 把本地对齐到上游？还是另起话题（比如上游 commit e9e4ef8 改的是 ObstacleAvoid.py，可能你也有想问的）？