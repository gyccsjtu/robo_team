# 交接文档：撞墙根因深挖 + 6 个 ACTOR 全部消除

> 交接日期：2026-10-09 | 承接提示：本文件是**给下一任 AI 的任务交接记录**，所有行号以当前工作区代码为准，先读代码再动手。
>
> ⚠ 2026-10-09 追加：本文 §5「修复方案（未实施）」**已按 v27 全部落地**（详见 §9），下一任直接跑仿真验证 §9.2，不再需要自行实现。

---

## 0. 任务目标（用户原话锚点）

> "继续深挖问题 撞墙为什么 必须实现6个ACTOR 全部消除"

两条主任务：
1. **找出"撞墙"（近障困局/原地打转）的根本原因** —— 已基本定位（见 §3）。
2. **必须让 6 个 ACTOR 全部消除** —— 消除链路必须跑通（目前历史所有轮次**真消除数 = 0**，见 §4 成绩表）。

---

## 1. 运行环境与启动方式（重要）

- 项目根：`/home/gycc/桌面/RoboCup_Team`
- 仿真启动（必须在用户 ROS 主机执行，非本工作区）：
  ```bash
  cd /home/gycc/桌面/RoboCup_Team
  bash run_match.sh      # 自动生成 logs/logs_<STAMP>/，5min hard_cap 自动关停
  ```
- 官方裁判：`~/XTDrone/robocup/score_cal.py`（比赛用这份，coordination/src 下有同步副本，两者 md5 已对齐）
  - 判据速查：`actor_id_dict={'green':[0],'blue':[1],'brown':[2],'white':[3],'red':[4,5]}`
  - 话题绑定：`/actor_green_info→actor_0(绿) blue→1 brown→2 white→3`；`/actor_red2_info→actor_4`、`/actor_red1_info→actor_5`（**red 是反的**）
  - 消除判据：**误差 <1m 且 相邻播报间隔 ≤1s，连续 15s** 任一违规即 `_reset_detection` 清零重来
  - **真消除唯一标志**：score_cal 打印 `actor_N is OK`（_delete_actor 成功，从 left_actors 移除）
  - 新计分（2026-10-09 [MERGED]，track×80 已删除）：`score = find_finish×50 + target_finish×100 − sensor_cost×3e-3 − uav_loss×30`
  - ⚠ `find_finish` = "进入确认期的 actor 数"，不是消除数！消除数看 `is OK` 条数。
- 环境变量注意：`run_match.sh` 的 `export` 是**最终生效层**——历史两次教训：代码改默认值但 env 里还有旧值覆盖（`BACKUP_AFTER`、`BRIDGE_EXTRAP_DELAY` 均中招）。**改代码必须同步查/改 run_match.sh env**。

---

## 2. 关键文件与代码行号速查（已核实）

| 文件 | 位置 | 内容 |
|---|---|---|
| `coordination/src/robocup_swarm/scripts/swarm_agent.py` | 2097-2100 | **双守卫调用链：雷达→栅格→地图依次覆盖（撞墙结构性根因）** |
| ↑ | 1535-1700+ | `_radar_guard_velocity`：EMA 滤波 + 自回波(<0.40m)过滤 + 硬刹停 |
| ↑ | 1774-1870+ | `_grid_guard_velocity`：锥形前瞻 ±15°/±30°，全堵→刹停 |
| ↑ | ~3203-3225 | `_enter_close_mode`：到圈即贴脸统一入口（幂等，锁 30s） |
| ↑ | 3227-3260 | `_check_start_orbit`：自动发现目标→到圈即贴脸 |
| ↑ | 2705-2719 | 追踪任务(task_type=1)到圈分支→`_enter_close_mode` |
| ↑ | 1240-1276 | `_confirmed_cb`：manager `/swarm/confirmed` 兜底入口 |
| ↑ | 3139-3184+ | `_update_orbit`/FOV 检查（|bearing-yaw|≤50°，yaw 已修） |
| ↑ | 2640-2746 | 追踪任务主流程：stale 拦截、`_approach_cap`、A* 重规划 |
| `coordination/src/robocup_swarm/scripts/yolo_target_bridge.py` | 57-75 | `OFFICIAL_CLS_OF_TAG`（红= `cls:'red'`）、`RED_DUAL`、`RED_TAGS`、`RED_FOCUS_HYST_M` |
| ↑ | 672-706 | `_publish_actor`：**B3 后 red1 只发 red1 流、red2 只发 red2 流** |
| ↑ | 562-609 | `_red_eligible` / `_pick_red_focus`（焦点选择，只影响 left 判定） |
| ↑ | 611-631 | `_note_red_left` / `_left_cb`（官方删红球号→作废焦点） |
| ↑ | 708-745 | `_emit`→`_publish_actor`（唯一播报出口，含闸门 GATE_DBG） |
| ↑ | 747-785 | `_keepalive`：外推保持器（限幅 `EXTRAP_MAX_D`，保 ≤1s 间隔） |
| ↑ | 98-105 | 播报闸门：进入 ≤`ACTOR_PUB_MAX_RANGE_M=9.5`，滞回退出 >`15.0` |
| `coordination/src/robocup_swarm/scripts/swarm_manager.py` | ~1481 | 确认超时告警一次性处理等（详见历史 memory） |

**红球重要更正（以最新代码为准！）**：
- 桥文件顶部注释 L34-70 写的是旧方案"锁定一个红球，把同一坐标同时发 red1+red2 两条流"——**该方案已被 B3（2026-10-09）废弃，注释过时未更新**。
- 当前实际（L696-703）：`RED_DUAL=1` 时两条红轨**各发各的流**（red1 只喂 `/actor_red1_info`，red2 只喂 `/actor_red2_info`），避免交叉坐标清除对方累计状态。
- `_pick_red_focus` 现在仅用于 `/left_actors` 少号时判断"哪个球被消除"。
- 因此 **"同一 msg 被算作两个 actor（串人）"问题在 B3 后应已消失**——接手时先跑 `python3 yolo_target_bridge.py --selftest` 确认，再看新版日志中 RESET_DBG 是否还有串人证据。

---

## 3. 已确认根因链（撞墙 + 无法消除）

### R1【P0 结构性】双守卫互相打架 → 原地振荡（"撞墙"主因）
- 链：`_send_vel` 中 雷达守卫 → 栅格守卫 → 地图守卫 **无条件依次覆盖**（2097-2100）。
- 现象（v26 日志，logs_20261009_215116）：agent_3 在 90~126s，雷达守卫把速度转向右侧（避开前障），栅格守卫随即判定"前方被占→转向 -170°"，两守卫每 50ms 互相抵消 → 原地打转；agent_0 在 (-37,-19) 困 80s。
- 统计：agent_0/1 近障次数 243/231，agent_2/4 仅 10/8 —— 撞墙集中在前排/低空机。
- **修复方向（未实施）**：① 守卫优先级固定：雷达近障（front<RADAR_WARN_R 强介入段）时禁止栅格再覆盖，或栅格只输出"要不要借一步"而非"硬转向 -180°"；② 360° 死锁检测：若 N 秒内守卫交替反向 N 次 → 强制随机脱困方向并短时锁 1-2s。

### R2【P0】EKF 慢漂移钳制 → world_xy 失真 → 假到达
- `world_xy property` 含动态速度钳制（阈值 = 1.2 + 指令速度峰值×1.3）与重锚定逻辑；慢漂移时 world 与真实物理脱节。
- 现象：agent_0 在**距 t4 真身 48m** 处打印"到达盘旋圈切贴脸 t4"——`dist < ORBIT_RADIUS(=6→实际到圈分支 ORBIT_RADIUS)` 判定用了被污染的 world_xy。
- 统计：agent_0/1 EKF 钳制各 53/31 次 → 假到达直接触发贴脸模式（见 R3），飞机朝错误点飞。
- **修复方向（未实施）**：到圈判定前校验 world 可信（近期有无钳制/重锚定/熔断事件），有则禁止"到圈即贴脸"；或到圈时用"目标坐标 - 本机整机雷达/LOS"交叉验证。

### R3【P1】"到圈即贴脸"把确认链路加速，但也放大了假到达
- v26 已生效：`_enter_close_mode` 三处入口（追踪到圈/自动发现/manager confirmed 兜底）→ 官方 15s 从贴脸首帧起算，省去原 15s 串行等待。方向正确。
- 但 confirmed 回调 `_t_seen` 兜底分支（L1257-1262）：本机 targets 缓存有该 tid 且 8s 内有位置更新即视为"本机目标"→ 触发假贴脸（可能贴的是他人目标/旧位置）。
- `_fly_orbit` 贴脸半径 R=1.5m/ω=0.02rad/s，稳态误差 <0.5m 天然满足官方判据——**贴脸本身没问题，问题在"飞错了地方还贴脸"（R2）和"贴脸期间断播"（R6）**。

### R4【P1】manager 侧资源周转
- 时间墙 25s 强制重开老格 + 90s 未抵达回收 + t5 确认后 90s 未消除→释放+黑名单 60s —— 机制已在 v18 落盘生效，但 v26 仍有 12 次 90s 未抵达回收与 4 次时间墙重开，说明**接近段被 R1/R2 拖慢**。

### R5【P1】红球（见 §2 更正）
- B3 后双流独立，"串人"待新版日志复验。焦点滞回 `RED_FOCUS_HYST_M=3.0`；`_note_red_left` 依赖 `/left_actors` 频率（历史上 v22 只收到 1 次，接通性待查）。

### R6【P1】消除链最后一公里：断播 / 误差超标
- 官方硬判据误差<1m 且间隔≤1s 连续 15s。v26 体感：t5 确认 7%→47%→87%→lost reset×4 永不消除；GATE_DBG 39 次未归零。
- 已修项：`DROP_TIME=4.0`（撑间歇丢失）、`EXTRAP_MAX_D` 限幅、`_keepalive` 外推保 ≤1s、闸门滞回。
- 剩余瓶颈（v23 记忆）：dist=1.03m 系统性误差三轮不变（疑 `TARGET_Z`/体心模型/相机外参固定偏差，需专项标定）；检测断流 2.07s（coast 18 帧撑不住）。

---

## 4. 历轮成绩表（基准，防止承接方误判"版本回退"）

| 轮次 | score | find_finish | 真消除(is OK) | 备注 |
|---|---|---|---|---|
| v21 | 464.9 | 3 | 0 | 旧计分含 track×80 |
| v22b | 174.9 | 2 | 0 | |
| v23b | 204.9 | — | 0 | 误检治理生效 |
| v23c | 254.9 | 2 | 0 | streak ge15=7 条突破 |
| v24 | 254.9 | 2 | 0 | 旧计分 |
| v26 | **94.9** | 2 | **0** | 新计分(删track)，6 机 0 崩溃、红线/uav_loss 全 0 |
| 历史最佳 | v16.1=354.9 | v18b=3 | 0 | 高分为运气局（actor 近） |

**结论：真消除历史恒 0——所有轮次都是 find 分，没有一次 target_finish。** 6 actor 全消除 = 至少把"误差<1m 连续 15s"在 6 条目标流上同时跑通。

---

## 5. 修复方案（三层，未实施，供承接方执行）

### 第 1 层：消除撞墙（R1 + R2）
1. `_send_vel` 守卫链加**优先级互斥**：雷达强介入（front ≤ 分级介入阈值）时栅格守卫跳过；栅格仅在地图/激光寂静时生效。
2. 360° 振荡检测：记录连续守卫转向，M 次内 N 次反向 → 强制"沿当前自由扇区直行 2s"，且 2s 内两守卫降权为"速度限幅"而非"方向改写"。
3. R2：`world_xy` 钳制/重锚定命中后，`_world_trust` 标记 3s 内为 False；`_check_start_orbit` 与到圈分支在 `_world_trust=False` 时**禁止进入贴脸/盘旋**，改为悬停等世界坐标自愈。

### 第 2 层：消除链路可信化（R3/R5/R6）
4. confirmed 回调的 `_t_seen≤8s` 兜底加**距离门槛**（本机与目标 <DETECT_RADIUS 且 LOS 可见才算本机目标）。
5. 播报只允许"贴脸圈内"（参考 R=1.5m 贴脸 + 力保误差<1m），中远距检测只登记不播报——待评估与现有闸门 9.5m 的关系。
6. 系统性 1.03m 误差：查 `TARGET_Z=1.25` 体心 vs 地面交点模型、yolo 相机外参；目标把贴脸稳态误差压到 <0.6m。
7. 断播 2s 根因：跟踪 track 死亡原因（coast 预算? 目标出视野?），提高贴脸期视场覆盖。

### 第 3 层：全图覆盖与周转
8. 派格/搜索 spread 权重复核（W_NOVELTY 已 0.8）；v26 搜索格 max_x=32.5 未达 x>60，覆盖不足导致东区 actor 找不到。

### 验证指标（v27 起）
- 撞墙：agent 近障次数前/后排方差收敛；无"守卫交替反向"持续 >10s；vout 前向均值 >0.4 m/s
- 消除：`actor_N is OK` ≥6；`is OK`/每流 `区段` 逐步增加；RESET_DBG far_dist 项清零
- 不回归：6m 红线 0、uav_loss 0、START_OCCUPIED 0、崩溃 0

---

## 6. 验证 SOP 与已知坑

1. 改代码后：`python3 -m py_compile <file>` + `read_lints` + 相关 `--selftest`（bridge 有 11 项自测）。
2. **必须同步检查 run_match.sh env**（§1 双例教训）。
3. **不能给 `bash -c "..."` 字符串内续行链插注释**——注释会吃整条环境变量前缀链（v23 首跑全瞎教训）。改完看启动横幅验证参数生效。
4. 磁盘：`~/.ros/log/*` 历史会话会占满 → score_cal 启动即崩（Errno 28）→ 全线被 kill。跑仿真前检查磁盘。
5. `replace_in_file` 对个别文件会静默失败（报告成功但 mtime/内容不变）——编辑后必须 grep+mtime 验证；失效时用临时 python 补丁脚本落盘（用完删除）。
6. 单轮方差大：同配置多次跑再下结论。
7. 离线单测（mock rospy 等）历史做法：临时脚本放 /tmp，测完删除。

---

## 7. 交接时已知的红色警示

- **真消除恒 0 是最高优先级问题**——所有修复最终都要落到"6 条流同时 15s 达标"。
- 别信旧注释（红球 B3 已改实现）；别信旧计分口径（v26 起 track 分已删）。
- 提升 find_finish 容易，**消灭"确认半路 lost reset"才是消除的关键**（t5 87%→lost 例子）。
- EKF 雪崩/慢漂移是 PX4 SITL 多机仿真层顽疾，软件只能兜底（熔断复飞粒度已有），不要把预算耗在根治 EKF 上。

---

## 8. 下一任接手第一步（30 分钟内完成）

1. 读 §2 列出的 6 个关键代码段 + 本节档；
2. `git log --oneline -12` 确认当前 HEAD 状态；
3. 跑 bridge 自测：`cd coordination/src/robocup_swarm/scripts && python3 yolo_target_bridge.py --selftest`；
4. grep 最近一轮日志确认：双守卫交替日志、假到达日志、RESET_DBG 串人是否仍存在；
5. 从 §5 第 1 层开始落地修复。