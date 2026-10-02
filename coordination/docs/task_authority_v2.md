# Swarm 任务授权接口 v2（2026-10-03 冻结）

新增 `/swarm/authorized_assignment` 与 `/swarm/authority_ack`，均为 String(JSON)。旧 SearchAssignment 不变，作为 manager 内部意图与诊断输出；agent 的执行入口迁移到新话题，不接受旧话题直接控制。manager 与全部 agent 启动前必须注入同一非空 ROBOCUP_RUN_ID；重启整轮必须换 ID。集中式 manager 为唯一写者，本接口不声称支持通信分区下多领导者。

授权字段：schema_version=2、run_id、uav_id、seq（每机严格递增）、generation（每机任务切换递增）、action=GRANT/STOP、expires_s、task={cell_ix,cell_iy,target_x,target_y,task_type,target_id}。数值必须有限。task_type 按现执行语义：0 搜索、1 目标观测、2 RTL、3 降落。run 不符、过期授权、旧 seq、旧代次拒绝；STOP/本地过期后的同代次 GRANT 不能重新启动。

搜索格与目标均有唯一任务持有者；备份请求不能获得同一目标的第二份执行授权。相同任务的坐标刷新和续约不重置盘旋；切换任务先 STOP，实测三维速度 <=0.15m/s 连续1s、位姿及速度采样新鲜后发 STOPPED。ACK 包含 schema_version/run_id/uav_id/seq/generation/sample_s、xyz（三维 ENU）、speed_mps、stopped_s、status=STATE/STOPPED。ACK序号连续增长，旧状态不能改变锁。

核心另从严格递增的实际 sample_s 和速度样本累计停稳区间，样本间隔超过0.5s重置；不能凭单条 STOPPED 的 stopped_s 声称完成。新 agent 只接受首代次1；已有代次必须先进入STOP才能接受新代次，禁止同轮单独重启agent冒充原执行者。

STOPPED 允许原机切换到另一任务，但旧任务锁进入 RETIRED，仍保留原持有者。只有同轮该机当前代次的新鲜位置报告证明已离旧任务点 >4.5m，才释放旧任务锁。失联、租约到期、发出STOP均不释放。目标锁被保留时其他机等待，不能凭广播TTL抢占。4.5m 为本地开发退出阈值，不是官方安全距离。

边界：这是任务唯一锁与水平任务执行门，未完成几何通道预约、三维起降/退出路径证明。退出点坐标来自最后一次任务指派，不代表移动目标当前位置或完整障碍地图。原拍卖覆盖与追踪字典只是意图；是否实际执行以新授权事件为准。RTL/降落指派仍需停止交接，但其垂直轨迹没有本接口安全证明。未具备完整证据不能宣称正式比赛PASS。

事件流使用相同schema_version和run_id，事件含TASK_GRANTED、STOP_REQUESTED、STOPPED_ACKED、TASK_RETIRED、TASK_RELEASED、TASK_BLOCKED、ACK_REJECTED。ROS薄层负责线程串行化和实测采样；纯核心只检查结构、时效和状态机，ACK身份认证仍需传输层。
