# v1.47：修正远距候选的实际选机缺陷

## 现场证据更正

WorkBuddy实体轮/root/robocup_runs/wb_v146_20261008T142000Z/round_1_seed_8159在核查时仍运行。执行690477b旧快照，本候选没有热改或重启该轮，也未终止其进程。

“蓝色候选未转发、没有任务”不成立：两条schema7原图1957.212/1959.112，桥收到时刻1957.852/1959.656，accepted=true、alive=false。执行桥源码允许early_blue直接转发，alive=false不是阻塞。manager在1957.856日志记录“发现新目标t1，由uav_4首次检测”，并派uav_6追踪；同刻实际schema2 TASK_GRANTED给uav_6、generation2、key=[target,t1]。agent_uav_6也在1957.856收到任务。

实际卡点：uav_6在1958.300、1960.984、1963.832、1966.768四次ONLINE_PLAN START_CLEARANCE_UNKNOWN，没有成功规划offer；1967.148因原图超过既有8秒导航窗失败，manager1967.468请求STOP。1968.548收到停稳ACK并退役目标任务，1968.600释放旧占用。该停控/退役链不应删掉来伪造接近成功。

## 原因与修复

原tracker_rank仅给20米内新鲜观察机优先级；但v1.46允许45米内候选接近。uav_4已有真实相机视野却在约37.5米外，失去排序优先级；刚空闲uav_6约42.5米外，因空闲池偏好被选中。这是接近语义与选机语义冲突，不是桥不支持多目标，也不能据此推断相机渲染是唯一瓶颈。

manager记录自身实际接收的v6/v7候选来源及原图时间。仅该时间等于VisualEvidence里该机该色最新原图、年龄≤1秒且实际分配距离≤45米时，给原观察机与旧近距观察机同等优先级。随后正常授权发布流程会停止它的旧搜索任务、等待停稳交接并申请路线；不直接释放搜索/路线占用。不修改原20米范围对普通观测的排序、官方12米三图许可、目标唯一锁、旧owner接管、激光门或速度参数。

非候选新图清除同来源标记；候选过期、时间不匹配、飞机不可连接/导航不合格/任务拒绝时不获得新增优先级。v6白色同样受益，但没有白色实体效果证明。接口兼容v2.7，线上task schema2和视觉schema7不变。统一manager/tracker_selection/bridge及六机新快照；配置记录tracker_selection_revision=v1.47_fresh_far_candidate_observer_ranking。

桥增加独立navigation_candidate_dispatch诊断，实际publish后才记录forwarded=true。它仅证明发布调用，不证明manager接收或任务授权，后者仍查看manager原始日志与TASK_GRANTED。

## 已验证与待验证

coordination/tests全量849项通过，0失败/错误。新增真实manager接收、排序与dispatch方法检查：相同几何/池构造下，新鲜远距候选选择原观察机；过期、原图改变、导航或任务拒绝时保留原空闲机行为。普通远距观察排序不变。输入将实体反例几何简化到共线37.5/42.5米，非完整历史ROS/权限状态回放，不能声称已产生新的实体TASK_GRANTED。

现场摘录、原两条候选、目标授权事件、agent6规划失败与1957.756轨迹样本存于validation/live_chain_break_v146_20261008。manifest哈希是仍在追加的源文件在捕获时的完整前缀SHA，不是最终轮哈希；原文件没有改写。

候选尚未实体执行，下一新快照需验证：原观察机旧搜索STOP交接→目标TASK_GRANTED→实际路线commit→距离缩短→近距合格v3上报。即使选机修正，也不能证明起点未知、停走、坐标误差及白色覆盖全部解除；6/6仍未完成。当前WorkBuddy轮继续运行，不能把旧快照后续结果当本候选效果。
