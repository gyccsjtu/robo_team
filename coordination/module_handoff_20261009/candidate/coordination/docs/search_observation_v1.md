# 搜索观察接口 v1（冻结，2026-10-05）

兼容增补v1.2（2026-10-06，v1.35候选）：city的agent使用新鲜velocity_local与自身已接受ENU位姿差分的较大三维速度判断到视点停稳及结果上报；差分无效时不得凭速度话题低读数开始观察或认定阻塞停稳。反馈/回执/已处理原图字段仍schema1不变，manager继续按既有原图投影及水平MotionCache复验，不声称独立复核每帧竖直速度。其他任务、STOP及占用语义不变；六机使用统一快照。

实现修订v1.34；任务授权schema2、路线schema1、视觉schema2和共享metadata2不变。新增话题均为JSON/String、schema_version=1。city六机同快照启用SWARM_SEARCH_OBSERVATION=1；旧接入可不订阅新话题，但不可混用旧“到点圆形覆盖”与新搜索完成口径。

冻结补充v1.1：/swarm/search_feedback_ack字段schema_version、run_id、uav_id、generation、seq、view_idx、accepted（true）。这是搜索结果收讫回执，不是运动授权。非终结视点等待有效回执才进入下一点；重发相同窗口只回执，不重复累计。最终结果以既有STOP流程换任务。实现必须同时升级manager与六个agent；新raw帧/反馈schema1字段不变。

## 已处理原图 /swarm/processed_camera_frame

字段严格为schema_version、run_id、uav_id、generation、cell（两个整数）、seq（逐机递增整数）、image_s（原图仿真时间）、sample_s（推理完成仿真时间）、inference_complete（必须true）、size（宽高）、intrinsics（fx,fy,cx,cy）、camera_xyz、camera_rotation（link到world的行优先3×3）。使用实际标定、同原图对齐的既有相机位姿；不改传感器。无检测框也发送，推理失败、重复图、没有合法搜索授权不发送。

捕获搜索授权代次在推理前冻结；原图不得早于本代次授权。接收时原图年龄≤1秒、完成采样年龄≤0.5秒，无未来时间。检查run/机号/代次/格、单调序号与不同原图时间、有限几何及旋转矩阵。缓存保留原始收到时刻：观察结果可引用本窗口内曾新鲜接收的帧，但不能凭重复引用刷新图像年龄。最后引用帧仍须≤1秒。

## 搜索反馈 /swarm/search_feedback

字段严格为schema_version、run_id、uav_id、generation、cell、seq、sample_s、position_xy、view_idx（0..2）、view_xy、phase、outcome、finished（布尔）、window_start_s、blocked_since_s、frames。

阶段GO_TO_VIEW→OBSERVE→NEXT_VIEW；没有可执行补点或失败时REVIEW_PENDING。NEXT_VIEW且finished=true表示本次有限观察结束、请求下一任务，不表示比赛完成。outcome为OBSERVED、NO_FRESH_FRAMES、VIEW_OCCLUDED、NO_CONNECTED_VIEW、NO_ROUTE_COMMIT、NO_ACTUAL_PROGRESS、EXECUTION_GATE_BLOCKED。后三种必须持续缺乏实体进展至少6秒。正常观察在2.5秒后完成，3秒仍不足则退出；每格最多三点，补点距原代表点≤8米，只选在线已观测、机体膨胀后连通的候选，并重新取得实际路线授权。

原代表点若已得到至少五个此前未观察/已到龄的实际采样点（可在邻格），则本次视点已提供有效新视野，可以请求下一任务；原格未观察部分仍REVIEW。否则才补点，避免水平相机看不到自身脚下而迫使每个7米格都飞满三点。五点收益是本地策略，不是官方阈值。候选连接段同时复验最新扫描净空，缓存路径不是新扫描通行证明。反馈/回执六秒未交付时用现有navigation_feedback schema1请求STOP，单独记录SEARCH_FEEDBACK_TIMEOUT，不以超时释放占用。

frames最多32项，每项为seq、image_s、visible；visible最多64项，每项为cell、mask（1..31）。mask对应格内五个固定采样点（中心及四个内缩角点）。每个有效点必须被至少三张不同的已推理原图实际投影覆盖，且本机当前原始雷达对连接视线提供新鲜自由证据；未知或遮挡不贡献mask。manager独立复验原图缓存、代次/窗口、投影及实测停稳位置，再累计点的原图时间。二维雷达只支持平面遮挡检查，五点记录不是完整三维可见性证明，也不能证明无人。

反馈采用新鲜motion_state（≤0.5秒）复验位置（≤0.75米）及停稳速度（≤0.25米/秒；行进阻塞≤0.15）。同代次同视点只接受一次完成反馈；旧代次、重复帧、错误身份、未来/过期、未推理帧均不累计。OBSERVED可以只得到部分点；未观察部分保留待复查，不因到点或派单标记覆盖。

## 观察与占用分离

兼容修订v1.3（2026-10-08）：wire schema1不变。GO_TO_VIEW新增有界往返退出：20秒新鲜自身位置窗口（样本间隔不得超过1秒），包围盒对角线不超过4米且到当前视点最佳距离改善不足0.5米时，请求本机XYZ停止。实际连续6秒速度≤0.15米/秒且漂移≤0.5米后才发送现有NO_ACTUAL_PROGRESS；缺少/过期运动证据不完成停稳。新代次/新视点不继承窗口。正常慢速接近和大范围绕路不满足该条件。仅作为搜索失败反馈，不代表未到达处已观察，不改任务/通道占用退役。六机统一升级agent与search_observation模块；旧manager可接收原字段，但需要保持本节失败语义。

观察注册表以原图仿真时间记录五个采样点。五点均在最近30秒实际观察后，格可记COVERED（仅采样观察完成语义）；到龄重新REVIEW，持续回访。派单时间、路线穿过和相机原始话题收到时间均不刷新观察时间。allocator的计划可见承诺另存，不能写入实际last_seen。

ASSIGNED及owner由任务占用维护。完成/失败反馈只请求STOP并撤销未执行意向；实测连续1秒三维停稳后才退休旧任务，新任务用于离开。原锁及路线占用仍到真实退出才释放；观察注册表不得释放它们。只有实际TASK_RELEASED/未执行TASK_INTENT_CANCELLED后才能将格展示为COVERED、REVIEW或FREE。追踪打断时保存未完成视点；换机至少保留未观察格和实际观察点记录，候选须用接替机自己的在线地图重建。

首版保留现有搜索/追踪速度与高度、STOP/唯一发布权/裁判语义，不增加真值控制。此冻结是接口定义；实现和实体效果分别记在修订与验证记录中，不能以本文件证明已完成比赛。
