# 视觉观测v3（2026-10-07冻结）

## 兼容修订3.4 / v1.45白色内部候选v6（2026-10-08冻结）

正式观测仍为v3，原候选v4/v5保持接收。新增schema_version=6只允许white、evidence_kind=navigation_candidate；完整字段为v5全部字段，额外candidate_reason=white_early_proof、person_hits和track_hits（严格int且≥2），person_frame_verified必须true，motion_identity_verified必须false。其余运行、序号、原图、相机位置、有限数和1秒新鲜性规则不变。两次人体支持必须来自FreshPerson接受的不同原图，前后间隔≤1秒；只在当前实际命中、既有1.3–2.4米身高/打分、量程≤22米合格时生成。只对旧判决hits/static/self的早期输出缺口使用，不绕过score/height等失败。

桥检查水平距离0<d≤22米，仅用独立candidate_core做一致性检查，接受后即转发原v6到confirmed_visual，不再等待该候选桥三图激活。v5仍要求22<d≤45米并按原方式激活。v6不进入正式融合、ReportReadiness、OfficialReportSources或TargetDetection，不改成v3、不生成官方许可；manager/agent接收它只用于原有30秒有界接近/保持视野，未取得后续合格v3时仍按既有失败/STOP/停稳及占用退役处理。任务和路线授权不变。

旧VisualEvidence会拒绝v6，所以perception、white_discovery.py、bridge、visual_observation.py、manager及六agent必须统一快照。共享服务和客户端仍使用原有同图证明协议，没有增加CUDA模型。PR_WHITE_EARLY_DISCOVERY通用默认0，city通过CITY_WHITE_EARLY_DISCOVERY默认1；置0只关闭新的白色早期通道。

绿色正式观测仍为v3：city启用BRIDGE_GREEN_ORIGINAL时加入raw颜色输出；独立官方同来源core使用最近1秒原始点的置信度平方加权时间回归，跨度至少0.2秒，速度长度封顶3m/s。重复/倒序原图拒绝，长断口重新开始，不做跨相机差分，不修改原图时间。使用原有限时/有限距离外推，取消仅此绿色分支的低速减半；其他颜色旧路径保持。CITY_GREEN_ORIGINAL默认1，但只有官方来源隔离及接近上报两开关均启用才生效；通用BRIDGE_GREEN_ORIGINAL默认0。六机必须统一green_source_motion.py、bridge、perception和city配置。不把这一估计用于裁判真值或修改15秒判据。

## 官方出口兼容修订3.2 / 候选v1.42（2026-10-08）

内部接入兼容修订3.3 / v1.44：本机新鲜confirmed原图可用于本机追踪引导，即使已有稍新的其他来源位置；只拟合各相机自己的速度，不跨来源做差分。全局目标位置仍单调，不被较旧图覆盖。manager对这种延迟图只登记该来源观测，不改最新目标点或派发任务。线上字段及schema保持，agent/manager/own_visual_guidance.py必须统一快照。本机缓存只能引导，不能生成观测、上报许可或路线授权；过期退回既有来源处理，已消除清除缓存。

视觉线上字段及schema_version不变。开启BRIDGE_SOURCE_REPORTING=1且BRIDGE_APPROACH_REPORTING=1时，内部融合轨迹继续供发现、接近和导航；官方坐标使用独立的同机来源轨迹。该来源必须自身激活，并取得原12米、三张不同原图、跨度0.6秒的许可。官方轨迹最新原图时间必须等于该来源许可的最新原图时间，两者年龄均不超过1秒；另一个来源的新原图不打断它，也不得借它的许可上传另一来源坐标。选中来源仍有效时保持，失效后才选择另一个已经合格的来源；不跨来源平均坐标，不修改原图时间。

city默认启用（CITY_SOURCE_REPORTING=1），通用默认关闭；CITY_SOURCE_REPORTING=0恢复旧全局融合出口。六机快照必须包含official_report_sources.py与新版bridge、city入口；manager/agent线上接口无需变化。trace增加可选report_source_uid及report_fusion_scope；旧分析器须允许这些附加诊断字段。该候选已有离线检查，尚无实体效果证明。

本次用户批准“内部发现→接近→稳定观察→官方上报”方案。所有v2原字段、运行/机号/序号、原图时间、观测标识及有限坐标规则保持；新增必填camera_xyz（三个有限数，world_enu，与sample_s同图对齐的相机位置）及person_frame_verified（严格bool，同一实际原图的人体与衣服颜色证明）。schema_version=3。不以当前服务位姿替代旧图位置，不以预测或coast生成新观测。

新版VisualEvidence严格接受完整v2或完整v3；v2用于内部发现/调度，开启BRIDGE_APPROACH_REPORTING时不能开启官方上报。旧v2模块会拒绝v3，manager/六agent/bridge/感知必须统一新快照，不混用。confirmed_visual原样转发v3，仍不生成路线或任务授权。共享metadata仍2，已有同图证明字段保持；人体证明来自共享服务或本地同图验证，客户端不加载新的CUDA模型。

ReportReadiness使用桥实际接受的合格观测：同一机、同一颜色至少三张严格递增原图，连续相邻间隔≤1秒，原图跨度≥0.6秒、每图水平距离≤12米，才能取得官方上报许可。原图距离由xyz与camera_xyz计算。白色三图均需person_frame_verified=true。内部轨迹激活仍独立要求满足；已接受但尚未激活的合格图可以计入三图，未接受图不能计入，许可也不能绕过内部存活状态。

开始后仍需当前新鲜有效原图；有效原图距离≤22米可维持。任何本来源不合格图、间隔>1秒、时钟回退均清除此来源许可；失去所有新鲜许可立即停官方上报。融合轨迹最新原图不能借更旧来源许可。再次开启重新需要三图及跨度，不能复用旧图补裁判15秒。官方已消除颜色清许可；red1/red2仍为内部几何槽、官方cls=red，任一剩余真值匹配口径保持。

只约束官方ActorInfo出口，内部TargetState/发现/confirmed_visual继续更新，避免没有官方上报就不能派机的循环。位置补偿、目标锁、STOP、路线及占用退役保持，不用人物真值计算接近距离。

通用BRIDGE_APPROACH_REPORTING默认0；city默认1，CITY_APPROACH_REPORTING=0恢复原出口策略。此开关不降级实际视觉v3。city保留10米追踪观察距离、搜索3/追踪2.6米每秒、局部2.2米及原WSL/传感器配置。开发相机位姿来源仍未完成正式兼容，不能宣称正式PASS。
