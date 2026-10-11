# 2026-10-11 视觉跟随制动与高度窗口

基线为已结束的 `logs/logs_20261011_111144`（`da2cd1c`，官方 6/6、538.332 s）。本变更没有启动仿真；离线测试不证明飞控真实倾角或消除用时改善。

## 证据与范围

蓝目标 u1 于 ROS 2892.100 进入观察圈。2892.052→2892.300 的最终水平指令从 `(-1.546, 1.435)` 变为 `(0.026, 0.724)`，0.248 s 平均变化率 6.956 m/s²。旧 `horizontal_slew_limit(close_mode=True)` 在减速时允许 8 m/s²，且允许同时转向。默认位置输出以 `local_xy + velocity × 1 s` 生成设定点，该片段对应 1.485 m 设定点变化。

相机姿态于 2892.200→2892.996 从 +18.26° 摆至 −14.05°；2892.592–2893.596 四帧脚部触底，`head_height` 控制观测继续但官方上报受保护。恢复脚点后的官方报告归档误差约 0.274 m，不把这次测距模式切换称为官方 2 m 错报。

## 控制边界

- 只在当前追踪任务有本机几何有效图像（图像和定位均不超过 0.5 s、非像素续看）、目标坐标不超过 1 s、距离不超过 12 m 时启用。
- 在友机避让前，限制相对目标的径向接近速度至 `sqrt(2 × 1.2 × max(distance − radius − 0.25, 0))`。保留目标平移与切向分量，不提高原指令速度预算。路径接近与观察圈跟随共用此入口。
- 新鲜本机视觉允许在观察半径外 0.5 m 接入近距控制，避免提前制动停在旧 8 m 模式入口之外。默认观察半径仍为 8 m。
- 普通、未受雷达/栅格/地图改写且无近友机的视觉控制，在制动距离足够时采用 1.2 m/s²，包括减速和转向；无本机视觉、过期或像素续看保留原有逻辑。
- 软制动接入前，用上次最终水平命令相对目标速度的向内分量 `v_close` 检查 `v_close² / (2 × min(MAX_ACC, 1.2)) + v_close / CTRL_RATE <= max(distance − radius − 0.25, 0)`；后一项保留一拍推进余量。不足时继续既有普通 2.5 m/s²、近圈减速 8 m/s²能力，不根据本拍已缩小或已转向的期望速度误判可行性。该条件每拍检查；向外运动或与目标一起平移不会仅因总速度大而被禁用软限幅。
- 友机避让仍在径向偏好之后，雷达、栅格、地图、越界回收保持原优先级。所有零速/硬停继续优先，绝不通过平滑恢复旧速度。当前没有可靠的停令来源分类，因此普通跟随请求零速也仍会立即停；这是明确保留的限制。

## 高度与距离

`tracking_view.view_height_window()` 用水平距离 `r` 计算：

```
lower = max(2.6 m, r × 0.33 / sqrt(1 − 0.33²))
upper = min(height ceiling,
            r × tan(atan2(0.48 × image_height, fy) − 10°))
```

752×480、fy=376 时，8 m 的窗口为 2.797–3.151 m。保留原先持续 0.6 s 才调整、最多 0.15 m/s 的变化率以及倾斜超过 8° 时冻结高度控制。窗口边界不使用会遗留越界余量的 2 cm 图像死区。

6.3 m 时上界 2.481 m，低于 2.6 m 底线：冻结高度，返回 `needs_space/range_floor`，通过现有安全观察点和避障恢复距离。过远导致下界超过高度上限时也冻结高度，但不请求继续远离，保留原接近流程。高度窗口是稳态目标；当前高度在窗口外时仍按上述速率收敛，不伪装成瞬时满足几何条件。

窗口使用本机 `local_z` 与水平距离的针孔近似，未消除相机外参、姿态动态和定位误差；实际相机余量及 PX4 响应仍须由新轮图像、姿态与控制诊断验证。

## 验证与诊断

`tools/test_view_motion_envelope.py` 覆盖历史蓝目标转向输入到实际位置输出、各级硬停不复活速度、无本机画面回退、友机/越界优先、移动目标相对制动、视觉入圈、4 m/8 m 针孔高度回放及不可行高度窗口。运动回放是确定性运动学和命令约束测试，不模拟 PX4 动力学。

高速迟发现回归通过真实 `_control → _approach_cap → _apply_friend_avoidance → _send_vel → 位置输出`：静止目标距 12 m，`_escape_spent` 允许 6 m/s，首次得到新鲜图像时上次命令已有 6 m/s。修复前首拍仅降至 5.94 m/s，新增断言先失败；需软制动距离 15.3 m，而可用余量仅 3.75 m。修复后首拍降至 5.875 m/s，90 拍运动学回放最小距离 8.09125 m、末速 0。另覆盖本拍转为切向仍须考虑上拍向内速度，以及目标平移、向外高速仍可软限幅。213 项相关回归通过。

既有 `TRACK_VIEW` 节流日志新增 `needs_space/range_floor`。既有控制 CSV 新增 `visual_slew`（该拍实际使用视觉 slew）、`horizontal_stop`、`view_sample_s`、`view_needs_space`、`view_range_floor`，以及 `view_brake_room/need`（米）、`view_brake_fallback`（制动余量不足，拒绝软限幅）。不增加高频日志流；公共 CSV logger 检测到表头变化会保留旧文件为 `.legacy.<timestamp>`。

下一轮最小验收：

- 按目标列出控制 CSV 中非零速追踪行的 `visual_slew=True` 行数、总行数及占比，说明本轮软限幅实际生效范围。CSV 按原 0.2 s 节流，统计值是日志采样占比。
- 按官方 ActorInfo 实际发布时间找出移动蓝/白每段大于 1 s 的断报，在同一图像时间轴核对框底 `y2/height`、四元数推导的相机倾斜、`geometry_ok`、测距模式及 `projection_reject/association_reject` 的 `sin_down/assoc_residual_m/assoc_gate_m/observation_gap_s`，区分画面裁切、几何拒绝、关联拒绝或没有新图像；不把缺少诊断行解释为几何通过。
- 对已有接近命令约 6 m/s 才获得新鲜视觉的片段，确认 `view_brake_need > view_brake_room`、`view_brake_fallback=True` 且 `visual_slew=False`，检查实际下发命令继续使用原刹车能力。若本轮没有该片段，标记未实飞覆盖，保留上述离线回放证据，不称已验证。

所有测试使用 `ROBOCUP_LOG_DIR=/tmp/robocup_decoy_tests PYTHONDONTWRITEBYTECODE=1`。建议回归命令：

```bash
source coordination/devel/setup.bash
ROBOCUP_LOG_DIR=/tmp/robocup_decoy_tests PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/gycc/robocup_build/coordination_ws/devel/lib/python3/dist-packages:$PYTHONPATH \
python3 -m unittest tools.test_view_motion_envelope tools.test_tracking_view \
  tools.test_tracking_plan tools.test_match_blockers tools.test_obstacle_height_safety -q
```
