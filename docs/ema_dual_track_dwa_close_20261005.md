# 复盘：EMA 双轨制 + 关闭 DWA 启动 — 2026-10-05

## 触发问题（10-05 复盘）
1. **EMA 单轨平滑把「actor 在动」糊成大滞后**：`yolo_target_bridge._publish_actor` 上 `XPOS_EMA_ALPHA=0.20` 把所有上报坐标无脑低通。actor 真值 1.3m/s + 2s 检测间隔时，EMA 滞后 ≈2.5m，远超官方 err_threshold=1m → score_cal `_reset_detection` 把 15s 计时清零 → streak 永远攒不满 → 25s 倒计时白等。
2. **DWA 6 个影子 601 次 set_mode 失败后退场**：`run_match.sh` 第 898-937 行启动 6 个 `dwa_avoidance.py` 影子，依次 set_mode 失败 601 次后所有 6 个进程挂掉；避障层全程缺席。而 `swarm_agent.py` 已经自带 `_grid_guard`（OOB_RECOVER_SPEED=5.0）、`_map_guard`（MAP_GUARD_SOFT=6.0 / MAP_GUARD_MARGIN=2.0）、`_radar_guard`（RADAR_WARN_R=5.5）三层边界守卫，覆盖规则 §2.5(7) 的安全要求。

## 修复
### Q1 — EMA 双轨制（`yolo_target_bridge.py`）
- 新增常量：
  - `EMA_BYPASS_RANGE_M = 10.0`（env `BRIDGE_EMA_BYPASS_M`）：`track.range_m ≥ 10m` 时关掉平滑。
  - `EMA_BYPASS_VEL_MPS  = 0.5` （env `BRIDGE_EMA_MOVE_BYPASS`）：track 速度 ≥ 0.5 m/s 时关掉平滑。
- 改 `_publish_actor`：
  - `_ema_on = (not _far) and (not _moving)` 才走 EMA；其余透传。
  - 透传分支同步刷新 `_xpos_ema[tag]=(x,y)`，避免回到近距静止态时第一帧误差突跳。

> 关键点：score_cal 的 `distance_sq >= 1²` 清零是「连续 15s 误差<1m」硬约束；EMA 在 actor 跑动时是负优化。**远距 + 移动 → 透传原始；近距 + 静止 → EMA 抗噪**。

### Q2 — 删 DWA 启动块（`run_match.sh`）
- 删 890-938 整段：`if [ "${ENABLE_AVOID:-1}" = "1" ]` 包起的 6 影子启动 + 30s 节点探测循环。
- 替换为 7 行说明 + 兼容分支：
  - `ENABLE_AVOID=1` → 提示"已弃用"（不启动任何节点，避免历史脚本外部设 1 又重启 6 个失败 dwa）。
  - `ENABLE_AVOID=0`/未设 → ok 行，明确告知"全队仅靠 swarm_agent 雷达安全层"。
- `swarm_agent.py` 启动逻辑（948-965）原样保留，不动。

## 验证
- `python3 -m py_compile coordination/src/robocup_swarm/scripts/yolo_target_bridge.py` → `PY_COMPILE_OK`
- `bash -n /home/gycc/桌面/RoboCup_Team/run_match.sh` → `BASH_N_OK`
- `python3 -c "import ast; ast.parse(...)"` → `AST_OK`
- selftest 需要 ROS 运行时环境，无法离线跑；逻辑改动局限在 `_publish_actor` 内部函数，未触及其他调用点。

## 影响面
- `yolo_target_bridge.py`：
  - 新增常量 2 个（皆带 env 覆盖，default 不变）。
  - `_publish_actor` 多读 `tr.range_m`/`tr.vx`/`tr.vy`（track 已有字段，不增对外接口）。
- `run_match.sh`：
  - 删 6 影子启动块 + 1 段配置注释 → -45 行 / +7 行；
  - 保留 `ENABLE_AVOID` 环境变量作为"文档遗留"，避免外部 launcher / 调试脚本因 unset 而行为变化。

## 待办
- 全场复跑 5 分钟（`run_match.sh`），统计 `_publish_actor` 双轨切换次数、`_reset_detection` 触发频率、`actor_*_info` 1s 间隔未命中的样本数（应明显下降）。
- 配合 `BRIDGE_EMA_BYPASS_M`/`BRIDGE_EMA_MOVE_BYPASS` 跑 2 次 ablation：
  - A：远距门 8m / 移动门 0.3 m/s（更严）；
  - B：远距门 12m / 移动门 0.7 m/s（更松）；
  比 find/track 时间集中度，挑最优。