# Swarm 任务授权接口 v2（2026-10-03 冻结）

兼容增补v2.7（2026-10-08冻结，远距候选观察机排序）：任务线上schema2、目标唯一锁及STOP/占用退役保持。manager实际接受视觉v6/v7后保留该来源的原图时间；仅其时间仍等于VisualEvidence记录的该机该色最新原图、年龄≤1秒且飞机与目标距离≤45米时，该来源与原20米内新鲜观察机同享排序优先级。有效连接、导航资格和任务拒绝条件仍检查；无新鲜证据时保留原排序和空闲机偏好，不制造原图、不续期、不免除交接。后续非候选图清除此来源候选排序标记。这是接近任务选择，不是官方上报许可，也不改旧owner接管条件。manager、tracker_selection.py、visual_observation.py及六agent使用统一快照。

v2.6同时取消已过原图导航窗口的同目标pending意向，调用既有withdraw及TASK_INTENT_CANCELLED/STOP流程；不移除旧搜索格或路线锁，不取消其他目标及搜索pending，不把意向取消当作实测停止。该修订需manager和六agent统一快照。

兼容增补v2.6（2026-10-07，交接观察候选）：线上schema2及STOP/ACK/停稳/占用退役字段保持不变。STOP时仍请求XYZ停止，不生成路线或移动授权；允许通过既有偏航通道保持本机已经接受实际原图的观察方位。线索仅来自confirmed_visual2中本机原图，首次接收年龄不超过1秒，之后原图年龄最多8秒；重复图、查询、别机观测不能续期。有效位姿、已完成起飞且非降落/返航必需；已消除/无限冷却目标不观察。新授权/FAILED/冷却不由观察线索取消。原图仍按原视觉门上报，8秒线索不能充当15秒确认。通用导航关闭时保持旧STOP朝向行为；city统一六agent启用。离线保持方位不证明实体重获成功。

兼容增补v2.5（2026-10-06，v1.35候选）：ACK字段与schema2不变；统一新agent的speed_mps取新鲜velocity_local与已接受自身ENU位姿差分的较大三维模长。新鲜差分至少覆盖0.2秒、窗口0.3秒，变换改变/时间回退/过期/定位隔离时不得确认停稳。相反速度不能平均成零；竖直漂移也不得确认STOPPED。原连续1秒、0.15m/s阈值与核心独立采样检查、实际退出及占用规则保持。缺任一必需来源不发新ACK，不能据TTL释放占用。协议字段兼容，但放行语义须manager及六agent统一快照；不直接解除定位隔离。

兼容增补v2.4（2026-10-05，v1.34）：manager在有效搜索结果后写schema2事件SEARCH_VIEW_RESULT；details.feedback为冻结search_observation_v1.md中的schema1结果，details.observed_mask为原搜索格当前实际五点观察掩码。该事件不是TASK_RELEASED、不是人物发现/消除。已处理相机帧和搜索结果/回执使用独立schema1话题；任务消息、STOP/停稳退休/实际退出规则保持schema2。旧事件消费者可忽略新诊断事件，升级接入须六机与manager同快照。

兼容增补v2.3（2026-10-05冻结，v1.33候选）：纯核心新增 `eliminate_target(tid,now)`，在同一run中永久封禁官方已消除的目标；取消仅该目标的pending，对该目标活跃代次发STOP（STOP_REQUESTED.reason=TARGET_ELIMINATED），保留其他任务和合法pending搜索。后续offer被TASK_INTENT_BLOCKED拒绝（details.key、reason=TARGET_ELIMINATED），refresh/到期不会重派。两红内部槽只有官方两个红色都不存在时才封禁；不能把一个红色被删除当作内部身份匹配证据。原始任务/路线锁保持停稳退休及实际退出流程。

新增schema2事件 `TASK_INTENT_CANCELLED`，details含key与reason（INTENT_REPLACED/TASK_WITHDRAWN/RUN_CLOSED/TARGET_ELIMINATED）；仅表示pending未执行意向被取消，不表示旧任务占用释放。相同key更新不产生取消事件。manager只对同run、同owner的STATE_ASSIGNED搜索格，且当前无该格活跃/退休锁、无任何飞机pending该格时取消格预留并置FREE，绝不据此置COVERED。已停稳但尚未退出的旧搜索格继续占用，原持有者可接新搜索以退出；原格不会对其他机放行。未知格预留、活跃搜索或pending搜索仍视为忙碌。闭合/撤销也使用此事件收回未执行意向。

线上字段与task_type仍为schema2；route1/visual2/sharedmetadata2/阻塞反馈1不变。升级manager与纯核心及search_occupancy同一快照，agent无需JSON迁移。旧manager若忽略取消事件，仍可能保留幽灵搜索预留，不能只升级核心。此兼容变更有离线回放覆盖，实体搜索恢复尚待下一轮证明。

兼容增补v2.2（2026-10-04冻结）：新增纯核心withdraw(uid,now)，撤销该机尚未执行的意向并对活跃任务发STOP，STOP_REQUESTED事件reason=TASK_WITHDRAWN。撤销不释放活跃/退休任务锁或几何通道，不将原任务自动加入重试队列；旧机仍须新鲜连续1秒停稳ACK及退出证据。用于相机接替：原持有者1.5秒没有自身原图，而另一名合格飞机有1秒内真实观测，候选持续至少0.75秒原图时间后才请求撤销；新机仍等待原目标锁按原规则实际释放，不能据候选/TTL直接GRANT。初次派遣时新鲜观察者优先级先于空闲/搜索池。无JSON字段或task_type变化，新增行为需升级manager/核心；旧agent无需数据迁移。这是待实飞的接替行为，不是接替完成证明。

该观察者选择范围使用既有20m检测距离，移除额外5m派遣裕量：实飞接替候选从14.50m走到16.12m，旧15m门会在0.75秒去抖完成前使候选消失。范围仅用于选择当前已经产生有效实际图像的飞机；超20m的机仍可按既有无观察者接近流程规划。没有放宽雷达/机间距/路线门，也没有改变相机参数。

兼容增补v2.1：目标唯一性涵盖等待停稳的交接意向。其他飞机已持有目标锁，或已登记同一目标的pending意向时，新申请在改变申请机任务前拒绝，输出TASK_INTENT_BLOCKED（schema2事件，details含key和owner）。没有新增消息字段；manager须同时检查持锁和pending意向，不能把尚未授权的主机当作目标无主并增派第二机。被拒申请不进入自动等待队列；目标真正空闲后必须重新提出请求，避免迟到候选无新意图即自动执行。撤销/替换意向不等于释放旧任务物理占用。旧接入方若依赖被拒目标自动接替，须迁移为显式重试。

新增 `/swarm/authorized_assignment` 与 `/swarm/authority_ack`，均为 String(JSON)。旧 SearchAssignment 不变，作为 manager 内部意图与诊断输出；agent 的执行入口迁移到新话题，不接受旧话题直接控制。manager 与全部 agent 启动前必须注入同一非空 ROBOCUP_RUN_ID；重启整轮必须换 ID。集中式 manager 为唯一写者，本接口不声称支持通信分区下多领导者。

授权字段：schema_version=2、run_id、uav_id、seq（每机严格递增）、generation（每机任务切换递增）、action=GRANT/STOP、expires_s、task={cell_ix,cell_iy,target_x,target_y,task_type,target_id}。数值必须有限。task_type 按现执行语义：0 搜索、1 目标观测、2 RTL、3 降落。run 不符、过期授权、旧 seq、旧代次拒绝；STOP/本地过期后的同代次 GRANT 不能重新启动。

搜索格与目标均有唯一任务持有者；备份请求不能获得同一目标的第二份执行授权。相同任务的坐标刷新和续约不重置盘旋；切换任务先 STOP，实测三维速度 <=0.15m/s 连续1s、位姿及速度采样新鲜后发 STOPPED。ACK 包含 schema_version/run_id/uav_id/seq/generation/sample_s、xyz（三维 ENU）、speed_mps、stopped_s、status=STATE/STOPPED。ACK序号连续增长，旧状态不能改变锁。

核心另从严格递增的实际 sample_s 和速度样本累计停稳区间，样本间隔超过0.5s重置；不能凭单条 STOPPED 的 stopped_s 声称完成。新 agent 只接受首代次1；已有代次必须先进入STOP才能接受新代次，禁止同轮单独重启agent冒充原执行者。

STOPPED 允许原机切换到另一任务，但旧任务锁进入 RETIRED，仍保留原持有者。只有同轮该机当前代次的新鲜位置报告证明已离旧任务点 >4.5m，才释放旧任务锁。失联、租约到期、发出STOP均不释放。目标锁被保留时其他机等待，不能凭广播TTL抢占。4.5m 为本地开发退出阈值，不是官方安全距离。

边界：这是任务唯一锁与水平任务执行门，未完成几何通道预约、三维起降/退出路径证明。退出点坐标来自最后一次任务指派，不代表移动目标当前位置或完整障碍地图。原拍卖覆盖与追踪字典只是意图；是否实际执行以新授权事件为准。RTL/降落指派仍需停止交接，但其垂直轨迹没有本接口安全证明。未具备完整证据不能宣称正式比赛PASS。

事件流使用相同schema_version和run_id，事件含TASK_GRANTED、STOP_REQUESTED、STOPPED_ACKED、TASK_RETIRED、TASK_RELEASED、TASK_BLOCKED、ACK_REJECTED。ROS薄层负责线程串行化和实测采样；纯核心只检查结构、时效和状态机，ACK身份认证仍需传输层。

2026-10-03 兼容补充：纯核心增加 `close(now)`，永久封闭当前运行、取消待发任务并为所有活动代次发STOP，后续offer不能重新开放。线上的字段与schema_version=2不变，旧执行器可直接消费STOP；STOPPED及位置退出证据仍按原规则处理占用，关闭不会释放锁。manager只在曾收到非空官方清单后又收到有效空清单时调用，忙碌机队也立即执行。完成后的动作是保持位置；水平雷达不能证明降落通道安全。

实际manager的CooperativeTracker启用official_only：本地观测持续15秒不自动消除目标，也不推断25秒时目标已瞬移。缺少位置误差真值时CSV的error_ok留空，不视为误差达标。该模式不改变离线样例的默认行为，正式结果仍须由官方反馈确认。
