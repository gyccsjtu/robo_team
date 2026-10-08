# 「起飞后不动却四处走 + 一个 actor 都没消除」根因分析

**仿真时间**：2026-10-06 12:22 → 日志目录 logs_20261006_122252
**结论先说**：`/score_cal` 收不到任何 yolo_bridge 的检测 → 永远不更新 `/left_actors` → `find actor_X` 永不触发 → `actor_X is OK` 永不触发 → 裁判不消除 actor。

---

## 根因：ActorInfo 消息类型冲突

仿真日志（04b_yolo_bridge.log 第 2 行）有 6 条 `WARN: Could not process inbound connection`：
```
[WARN] topic types do not match: [robocup_swarm/ActorInfo] vs. [ros_actor_cmd_pose_plugin_msgs/ActorInfo]
  callerid: /score_cal
  md5sum: b19c42c4e8a205a44c00afb2c6f8754e
  message_definition: <5 字段 Header+cls+x+y+z, float64>
  topic: /actor_green_info    (其他 5 个同)
```

### 哪 6 个节点涉及 `/actor_*_info`

| 节点 | import | publisher type 字段 | 仿真运行时实际效果 |
|---|---|---|---|
| `detection_to_official.py` | `from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo`（3字段 cls+x+y float32） | `ros_actor_cmd_pose_plugin_msgs/ActorInfo` (md5 d57df6c0908e158da5273b4646f729dd) | **先启动并注册所有 6 个 `/actor_*_info` topic**，把 topic type 定为上游 3 字段版 |
| `swarm_viz.py` | `from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo` | (订阅 `/actor_*_info`，不是 publish) | 收什么也是 3 字段 |
| `yolo_target_bridge.py` | `from robocup_swarm.msg import ActorInfo`（v3 修复 2026-10-06 已改回 5 字段 Header+cls+x+y+z float64） | `robocup_swarm/ActorInfo` (md5 b19c42c4e8a205a44c00afb2c6f8754e) | **被 ROS master 拒因为同一 topic 已被 detection_to_official 占 3 字段** → 它注册的 publisher 失效 |
| `/score_cal` (仿真跑的) | `from robocup_swarm.msg import ActorInfo`（XTDrone 本地副本，已修） | 订阅 `robocup_swarm/ActorInfo` | 收到的 topic type 是 3字段 (d57df6c0…) → md5 不匹配 → 静默丢消息 → `actor_info_callback` 永远不触发 |

### 链路
```
yolo_bridge (5字段, 想 publish)
  → ROS master 拒因为 /actor_*_info 已被 detection_to_official.py 注册为 3 字段
    → yolo_bridge publisher 不工作
      → /score_cal (订阅 5 字段) 收到 topic 的 type 是 3 字段
        → md5 b19c42c4 (5字段) vs d57df6c0 (3字段) 不匹配
          → ROS 拒绝连接 → actor_info_callback 0 调用
            → /left_actors 永远是 [0..5]
              → find actor_X 永远 0 次
                → score_cal 不触发 actor 删除
                  → 一个 actor 都没消除
```

### UAV「四处走」但「不动」的解释
- UAV 起飞正常 (协调层 `/swarm/target_states` 走的是 yolo_bridge 自己**单独**的 publisher type `robocup_swarm.msg.TargetState`，没和 `/actor_*_info` 共享 topic，所以这部分数据流通畅)
- swarm_manager 拿到的目标列表驱动 A* 路径 → UAV 飞向目标
- 但因为官方 `/score_cal` 不收检测，swarm_manager 永远拿不到"目标被官方确认"信号 → swarm_manager 不给 UAV 触发「攻击/靠近」行为
- UAV 反复执行「飞过去 → 找不到确认 → 退回搜索 → 换目标」循环，看起来四处走但不消 actor

---

## 修复方向

**两种路线二选一**：

### 路线 A：把 detection_to_official.py 也切到 robocup_swarm.msg
- 改 `detection_to_official.py` 第 64 行 + swarm_viz.py 第 166 行都改成 `from robocup_swarm.msg import ActorInfo`
- 所有 publisher 统一用 5 字段版，与 score_cal.py 的订阅端对齐
- 优点：保留本地 robocup_swarm.msg 的语义（带 z 和 Header）
- 缺点：如果以后比赛现场跑上游官方 score_cal（用 ros_actor），会订阅失败

### 路线 B：把 yolo_bridge 也切回 ros_actor_cmd_pose_plugin_msgs
- 改 yolo_bridge 的 ActorInfo 来源、publish 实例字段填充从 5 字段 → 3 字段（去掉 Header, x/y 改 float32, 不发 z）
- 让所有节点都用上游版，兼容官方比赛
- 优点：贴近上游官方标准
- 缺点：本地 score_cal.py（带 EMA/追踪分）也得切回上游版，丢掉 v3 + EMA 复盘

---

## 我建议先做哪一步

**先确认一下当前仿真 ROS graph 上 `/actor_*_info` 到底有谁还在 publish** —— 因为 detection_to_official.py 通常是被前面 map 节点启的（如果你仿真没启它，那唯一的 publisher 就是 yolo_bridge，可能问题更简单）。下一步操作建议：
1. `rosnode list` + `rostopic info /actor_green_info` 看 publisher 列表
2. 检查 detection_to_official.py 是否真的在跑（`ps -ef | grep detection_to_official`）
3. 根据 (2) 选路线 A 或 B

---

## 副产物：上游 score_cal.py 最新版（之前问题）

之前你问的「裁判代码有新的修改」—— 上游 master 最新版 robocup/score_cal.py 把 import 改回 `from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo`，但 **XTDrone 本地副本（仿真用的）还是 robocup_swarm.msg 版本**。本仓库 `coordination/src/robocup_swarm/scripts/score_cal.py` 也是 robocup_swarm.msg 版本。**当前仿真里没有「上游改回 ros_actor」这个新机制**。md5 不匹配的 root cause 是 `detection_to_official.py` 在 publish 端没切，yolo_bridge 切了，反而撞上了。