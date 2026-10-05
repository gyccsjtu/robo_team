# v1.30：追踪规划交接

**后续实体结果（2026-10-05）：be2945f在8159按600秒配置自然完成，裁判5/6。** 本轮没有SUPERSEDED_TICKET，规划交接已产生实际进展；尚未6/6。执行、接触和高度证据见[v1.30收尾计划](v130_postrun_plan_20261005.md)。下方“尚未物理验证”为该轮启动前历史状态；当前新候选见[v1.31/v1.32](fast_competition_fixes_v131_v132.md)。

2026-10-05。执行v1.29/cb33a45后，按[v1.29收尾计划](v129_postrun_plan_20261005.md)修复同代次目标坐标更新持续取消在途规划的问题。仍采用c77063e行为配置；没有重新打开回退掉的视觉豁免、冷却重试或计数重置。

## 已实现

目标任务在pending/inflight计算期间先完成当前请求；同代次route_pending等待授权的1秒内也不因新坐标取消。完成或失败后，下一次控制更新会读取最新相机坐标，不修改原图时间或创造新观测。搜索任务仍按原策略保留最新目标。

追踪节流的上次目标/时刻只在请求实际入队时推进，合并掉的更新不再冒充新规划；原3m/2s门限不变。OBSERVED_FRONTIER的终点与人物坐标不同也不取消同代次待授权offer。

规划线程按authority→plan顺序锁住inflight到route_pending的交接；计算本身仍在锁外。代次变化、STOP、过期仍拒绝结果；路线offer仍不是移动许可，manager实际授权、最终雷达和占用语义不变。增加PLAN_OFFERED及PLAN_RESULT_DISCARDED日志，区分SUPERSEDED_TICKET、GENERATION_CHANGED和AUTHORITY_STOPPED_OR_EXPIRED，供下一轮查证，不改变协议。

任务schema2、路线schema1、视觉schema2、共享metadata2、阻塞反馈schema1均保持兼容。city_control_config新增tracking_planner_handoff_revision=v1.30_same_generation_finishes_before_camera_replacement，已有behavior_rollback_revision=v1.29继续表示行为回退配置。六机使用同一新快照。

## 离线验证与限制

coordination/tests：482项通过，8.790秒。新增10项覆盖相机更新时在途结果交接、实际agent提交回调、无授权不能执行、新代次可排队、STOP/过期拒绝、搜索更新、授权等待上限、失败释放、交接锁连续性、前沿终点差异及实际入队节流。提交回调测试使用接受型路线闸门替身；manager协议检查由已有核心测试覆盖，不宣称真实ROS授权。

[录制坐标调度回放](v130_planner_handoff_replay.json)：真实绿色坐标35.95/-6.61、35.63/-6.67，计算中插入后一坐标；旧cb33a45 ticket8→9、首结果offer0，新候选ticket保持8、offer1。调度顺序和规划返回人为构造，不重放物理或真实完整回调，源码SHA为用于回放的UTF-8/LF文本。

尚未证明六机运动中是否恢复追踪、连续15秒定位或裁判消除。白色定位误差、红色严重误检/误差和长期卡点继续保留。此次只改一个已证实阻塞，未放宽雷达/速度/高度/裁判/WSL配置。

补充只读原图诊断：[红色颜色样本](v130_red_color_diagnostic.json)中18张历史误差小于1m正例均满足现有red颜色判据，当前7张误差大于5m的帧只有5张会被该判据拒绝，2张仍会漏过。当前CLASS_COLOR只核验绿白；仅把red加进去不能声称已修。白色[三张原图人体框](v130_white_person_box_diagnostic.json)在最大误差帧的衣服框底238.14与匹配人体框底241.199相差3.059像素，人体证据不等于脚点测距正确；缺原始相机姿态，未计算替代世界坐标。未改变这两项控制行为，下一轮仍仅验证规划交接。

下一次允许的物理轮复用validate_fast_city.py --single-seed 8159，以新预算/统一快照验证：同代次更新是否不再连续SUPERSEDED_TICKET，任务→offer→commit延迟，实际移动和裁判进展。最多600秒，6/6或接触/基础服务故障收尾；出现已证实阻塞、等待无益可明确理由提前收尾。每轮后先归档、列问题和计划，再修改，不原样重跑。
