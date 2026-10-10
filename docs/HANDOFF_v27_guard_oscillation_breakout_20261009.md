# v27 修复报告：守卫振荡脱困 + world 假到达防护 + 断播清零治理

> 2026-10-09 · 基于 v26 仿真（logs_20261009_215116, score=94.9, find_finish=2, 真消除=0）的四大遗留缺陷修复。
> 状态：R1-R6 已落盘，py_compile + lint 0 错误，待 run_match.sh 仿真验证。

## 0. v26 验证口径修正（重要）

新计分（2026-10-09 [MERGED] 删 track×80）：
`score = find_finish×50 + target_finish×100 − sensor_cost×3e-3 − uav_loss×30`

- v26 = 94.9 = find2×50 − 5.1（sensor_cost≈1700）。**此前"v26 得分暴跌 160"是跨计分口径错误**：旧口径下 v24=254.9 含 track2×80，同口径下 v24/v26 表现持平。
- 真消除唯一标志 = score_cal 打印 "actor_N is OK"（_delete_actor 成功）。v26 两轮 find_finish 均=2，is OK 均=0，**真消除始终 0**。
- 历次真消除为 0 的根因链已逐层收窄：v26 关键实证 = red1/red2（东区双红球）140s 已被锁定追踪，但 range 长期 10~28m 进不了 9.5m 闸门；且 PUB_DBG max_gap 分布 = [0.8, 1.54, 13.73, 45.4]s，两档大断播（>1s 即被官方 _reset_detection 清零重来）。

## 1. 三大根因（v26 实测）

| 根因 | v26 实证 | 对应修复 |
|---|---|---|
| 守卫交替振荡死锁 | agent_3 困 90s、agent_0 困 80s，雷达/栅格守卫每帧交替反向互相抵消 → 追踪机永远收缩不了贴脸圈（range 卡 10~28m） | **R1** 振荡脱困 |
| EKF 钳制/重锚定污染 world | world 被钳后与物理位置脱节，agent 判定"到圈"实为假到达（距真身 48m） | **R2** world 可信度闸门 |
| 贴脸期 YOLO 短暂丢 track → 保持器硬跳 | t5 确认 87%→lost reset×4；211s 处 max_gap=13.73s 断播 | **R6** 断播清零 |

## 2. 六项修复清单

### R1：守卫交替振荡脱困（swarm_agent.py）
- `_radar_strict` 标记：雷达近障线性段/三面堵死/后退分支置 True；`_send_vel` 每帧重置。
- `_grid_guard_velocity` 开头互斥分支：`_radar_strict=True` 时只按 `_radar_strict_d` 计算 `v_cap = sqrt(2·MAX_ACC·max(0, d_min−RADAR_SAFE_GAP))` 夹速度，return 不改向。
- `_oscillation_breakout`：3s 窗口内 ≥4 次 >120° 方向翻转 → 强制沿锁定航向直行 2s（1.2m/s）；窗口超长且翻转不足则重置（防误触发）；雷达强介入时让位。
- 新参数：`OSC_WATCH_S=3.0 / OSC_SWITCH_N=4 / OSC_BREAKOUT_S=2.0 / OSC_BREAKOUT_SPD=1.2`。

### R2：world 可信度闸门（swarm_agent.py）
- 慢漂移钳制、病态重锚定、EKF 原点跳变三处均设 `_world_trust_until = now + WORLD_TRUST_HOLD_S(3.0)`。
- 新方法 `_world_trusted()`；三处防护点：
  1. `_check_start_orbit` 开头（不可信不开始新盘旋）
  2. 追踪到圈分支 `dist < ORBIT_RADIUS and not trusted` → 悬停等自愈，不切贴脸
  3. `_fly_orbit` 贴脸段开头（不可信 `_send_vel(0,0)` 返回）

### R3：confirmed 回调距离/LOS 门槛（swarm_agent.py confirmed_cb）
- `_t_seen ≤ 8s` 兜底分支内加 `_d < DETECT_RADIUS*1.2 and los.visible(...)`，防止把中远距目标纳入本机目标池。

### R6：bridge 断播清零（yolo_target_bridge.py _keepalive）
- `age > DROP_TIME`（4.0s）才算彻底死透不硬撑；
- `max_t = max(ACTOR_KEEPALIVE_MAX_T, DROP_TIME − 1.0)` = 3.0s：观测断流但 track 未死透时，补发外推撑到接近 DROP_TIME；
- 仍受 `EXTRAP_MAX_D=0.5` 限幅保护（外推超 0.5m 截断），宁多撑不断流。

## 3. env 一致性核验（防"代码改白改"重演）

| 参数 | 代码默认 | run_match.sh env | 一致 |
|---|---|---|---|
| DROP_TIME | 8.0（`BRIDGE_DROP_TIME`） | 4.0 | ✓ |
| EXTRAP_DELAY | 0.45（`BRIDGE_EXTRAP_DELAY`） | 0.45 | ✓ |
| EXTRAP_MAX_D | 0.5（`BRIDGE_EXTRAP_MAX_D`） | 0.5 | ✓ |
| EARLY_PHASE_SEC | 30.0 | 30.0 | ✓ |
| OSC_/WORLD_TRUST_* | 代码默认 | 无 env 覆盖 | ✓ |

## 4. v27 仿真验证指标

1. **range 贴脸**：red1/red2 贴脸期 range 应 <9.5m（v26 卡 10~28m），GATE_DBG 不再刷屏拦截
2. **断播治理**：PUB_DBG `max_gap ≤1.0s`（v26 有 1.54/13.73/45.4s 三档超标）；max_gap>2s 档 0 次
3. **贴脸圈收敛**：出现「到达盘旋圈 → 直接切贴脸」日志后不再伴随「world 不可信悬停」反复
4. **振荡脱困**：「振荡死锁脱困」日志出现后不再原地打转，agent 位移恢复（v26 agent_3 90s 困局）
5. **消除链路**：至少 1 个 actor 打印 "actor_N is OK"（真消除，v26=0）→ score > 94.9
6. **不回归**：6m 红线 0、START_OCCUPIED 0、uav_loss 0、crash 0

## 5. 已知残余（v28 候选）

- **东区覆盖**：v26 搜索格最东只到 col12 (x=32.5)，东区 actor (x=63~72) 完全靠探测发现。任务分配要够到东区仍靠飞行提速 + 覆盖率 reopen；若 v27 贴脸成功，可回看东区覆盖缺口是否仍挡消除
- **EKF 雪崩**：PX4 SITL 多机 GPS/气压仿真根因未根治（软件兜底：熔断→复飞闭环已生效）
- **系统性 1.03m 误差**：三轮同值，疑 TARGET_Z/体心/外参固定偏差，需专项标定
