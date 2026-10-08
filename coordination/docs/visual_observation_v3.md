# 视觉观测v3（2026-10-07冻结）

## 官方出口兼容修订3.2 / 候选v1.42（2026-10-08）

视觉线上字段及schema_version不变。开启BRIDGE_SOURCE_REPORTING=1且BRIDGE_APPROACH_REPORTING=1时，内部融合轨迹继续供发现、接近和导航；官方坐标使用独立的同机来源轨迹。该来源必须自身激活，并取得原12米、三张不同原图、跨度0.6秒的许可。官方轨迹最新原图时间必须等于该来源许可的最新原图时间，两者年龄均不超过1秒；另一个来源的新原图不打断它，也不得借它的许可上传另一来源坐标。选中来源仍有效时保持，失效后才选择另一个已经合格的来源；不跨来源平均坐标，不修改原图时间。

city默认启用（CITY_SOURCE_REPORTING=1），通用默认关闭；CITY_SOURCE_REPORTING=0恢复旧全局融合出口。六机快照必须包含official_report_sources.py与新版bridge、city入口；manager/agent线上接口无需变化。trace增加可选report_source_uid及report_fusion_scope；旧分析器须允许这些附加诊断字段。该候选已有离线检查，尚无实体效果证明。

本次用户批准“内部发现→接近→稳定观察→官方上报”方案。所有v2原字段、运行/机号/序号、原图时间、观测标识及有限坐标规则保持；新增必填camera_xyz（三个有限数，world_enu，与sample_s同图对齐的相机位置）及person_frame_verified（严格bool，同一实际原图的人体与衣服颜色证明）。schema_version=3。不以当前服务位姿替代旧图位置，不以预测或coast生成新观测。

新版VisualEvidence严格接受完整v2或完整v3；v2用于内部发现/调度，开启BRIDGE_APPROACH_REPORTING时不能开启官方上报。旧v2模块会拒绝v3，manager/六agent/bridge/感知必须统一新快照，不混用。confirmed_visual原样转发v3，仍不生成路线或任务授权。共享metadata仍2，已有同图证明字段保持；人体证明来自共享服务或本地同图验证，客户端不加载新的CUDA模型。

ReportReadiness使用桥实际接受的合格观测：同一机、同一颜色至少三张严格递增原图，连续相邻间隔≤1秒，原图跨度≥0.6秒、每图水平距离≤12米，才能取得官方上报许可。原图距离由xyz与camera_xyz计算。白色三图均需person_frame_verified=true。内部轨迹激活仍独立要求满足；已接受但尚未激活的合格图可以计入三图，未接受图不能计入，许可也不能绕过内部存活状态。

开始后仍需当前新鲜有效原图；有效原图距离≤22米可维持。任何本来源不合格图、间隔>1秒、时钟回退均清除此来源许可；失去所有新鲜许可立即停官方上报。融合轨迹最新原图不能借更旧来源许可。再次开启重新需要三图及跨度，不能复用旧图补裁判15秒。官方已消除颜色清许可；red1/red2仍为内部几何槽、官方cls=red，任一剩余真值匹配口径保持。

只约束官方ActorInfo出口，内部TargetState/发现/confirmed_visual继续更新，避免没有官方上报就不能派机的循环。位置补偿、目标锁、STOP、路线及占用退役保持，不用人物真值计算接近距离。

通用BRIDGE_APPROACH_REPORTING默认0；city默认1，CITY_APPROACH_REPORTING=0恢复原出口策略。此开关不降级实际视觉v3。city保留10米追踪观察距离、搜索3/追踪2.6米每秒、局部2.2米及原WSL/传感器配置。开发相机位姿来源仍未完成正式兼容，不能宣称正式PASS。
