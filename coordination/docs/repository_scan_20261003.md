# robo_team 首次扫描与实施计划

目标：用户指定的 https://github.com/gyccsjtu/robo_team 。基线提交 `86eb484ab9b569321ab81636df7a07d2970cf71d`；工作分支 `codex/radar-coordination-20261003`。本次只读扫描代码并记录计划，未修改算法、启动本仓库仿真或推送。比赛规则优先，历史说明不作为安全证明。

## 实际执行链

`run_match_yolo_avoid_swarm.sh` → `run_match.sh` → `swarm_manager.py` + 六个 `swarm_agent.py`，消费感知桥发布的目标状态。相关文件均位于 `coordination/src/robocup_swarm/scripts/`。

- `swarm_task.py`：覆盖格、分配效用、租约、视线与目标追踪，纯 Python。
- `swarm_manager.py`：集中式任务分配、追踪机/备份机调度、续租与官方剩余目标反馈。
- `swarm_agent.py`：每机执行、A*、盘旋观测、友机避碰、雷达安全层、MAVROS 速度发布。
- `cooperative_tracker.py`：观测融合与连续确认的纯逻辑模块。
- `robocup_navigation/coordination/core.py` 等：仓库另含旧 schema1 预约核心，但上述比赛启动链未调用它，不能把它的单测等同于比赛 swarm 链验收。

## 已读源码确认的问题

| 问题 | 源码证据 | 影响与边界 |
|---|---|---|
| 租约到期直接清 owner 并重新分配 | `swarm_task.py:598`、`swarm_manager.py:1009`、`:1132` | 旧机失联不证明已停止；搜索格租约也不等同于通道预约，现有链没有完整停止交接证明 |
| 任务指派缺运行 ID、授权代次与停止确认 | `msg/SearchAssignment.msg`、`swarm_agent.py:553` | 按 uav_id 过滤后直接接受，无法可靠隔离旧运行/旧授权 |
| 盘旋认领仅为 uid:tid 字符串且按新鲜期失效 | `swarm_agent.py:1598`、`:1637`、`:1650` | 最近一条广播覆盖缓存，没有唯一锁代次与安全交接机制 |
| 雷达无帧/过期时原速放行 | `swarm_agent.py:844` | 默认 RADAR_GUARD=1 并不证明感知有效 |
| 雷达主要检查机头前方，而非当前速度方向 | `swarm_agent.py:864` | 侧飞/后退时无法据此证明运动方向净空；近障侧移也未验证整条侧移段 |
| SearchAssignment 的 task_type 注释与执行不符 | `msg/SearchAssignment.msg`、`swarm_agent.py:579` | 消息写 2=接力，实际执行 2=RTL，3=降落，须冻结一致接口 |
| 可选规划失败直飞 | `swarm_agent.py:1250` | 默认 PLAN_FALLBACK=0，但显式开启后失败被转换为直飞成功，不能作为已证安全路线 |
| 启动入口和文档不能直接迁机照抄 | 根目录一键脚本、`coordination/README.md` | 硬编码他人 /home 路径、宽泛 pkill；README 仍说不含多机协同，与当前代码不符 |

源文件里多处真值相关旧注释与实现已经不同：agent 默认用 MAVROS 和注入起飞点，只有显式 SWARM_CALIB=truth 才做一次真值标定；目标状态的实际来源要沿启动脚本和桥核对，不能凭 `_truth_cb` 函数名判违规。二维雷达也不证明完整三维净空。

## 此前误选仓库的改动

本地 `../radar_product/` 基于旧 robocup_ws 提交 `ef3feb009e645d7b2f70fc33ef428a140f6b373b`。改动尚未提交/推送：修复旧 schema2 退出授权对 terminal_safe 的绕过，新增 schema3 单机滚动雷达授权、纯 Python 前缀净空检查和 ROS 单机实验接入。它没有改本仓库的 swarm_task/manager/agent，也没有完成六机搜索。

旧原型 run03 的独立仿真采样记录了终点水平误差 0.4655 m、最高高度 5.3337 m，但碰撞证据不足、正式比赛结论为 false。这只能作为旧原型实验，不是本仓库验收。

## 实施顺序

1. 在本分支为实际 swarm 链建立纯逻辑基线与失败复现，核对官方规则、现有消息和启动参数。
2. 先冻结任务/目标锁接口，明确运行标识、代次、续租、停止确认、退出占用与失败接替，写明旧消息迁移方式。
3. 在 swarm_task 等本仓库纯 Python 核心实现锁与失败接替；ROS manager/agent 只作薄接入。失联或 TTL 到期保留尚未证实退出的占用。
4. 修正运动方向雷达保护及缺帧动作，保持单一 setpoint 发布者；用本仓库六机命名空间、端口、坐标变换做独立审计。
5. 复用已验证 runtime，在隔离环境运行本仓库代码，按真实轨迹、距离、停止确认和完整证据验收；缺证为 ABSTAIN。

不整包移入旧原型；只有与本仓库接口一致且经复验的纯逻辑或工具才复用。以上步骤是计划，尚未实施。
