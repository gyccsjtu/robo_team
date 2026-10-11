# 首轮按分区长轴推进

## 本轮证据与范围

`logs/logs_20261011_122632` 使用原逐列排序，正式结果是 6/6、465.096 s、2109.732 分。
东侧三个分区初始都选距起飞点最近的西侧入口 `x=53.5`，之后按短轴逐列蛇形推进。
实测 UAV 3 首次到 `x=50` 是 ROS 2499.424，首次到 `x=80` 是 2712.672；
UAV 5 对应为 2531.476 和 2686.128。期间的追踪、避障、补看也消耗时间，不能把全部延迟归给排序。

此次只改变 `swarm_task.TaskAllocator` 的分区路线排序：宽分区沿 x 逐行蛇形，
高分区保留沿 y 逐列蛇形，正方形保留原规则。所有格仍在原分区内，原有最近入口、
完成后才推进游标、南北外排延后、障碍过滤、接管和补看规则继续执行。
不使用演员坐标、固定建筑坐标或仿真真值；不改变速度或感知门槛。

## 单变量开关与启动识别

默认 `SEARCH_ROUTE_AXIS=long_axis`；导出 `SEARCH_ROUTE_AXIS=column` 后启动 manager
可精确恢复原排序。`run_match.sh` 没有覆盖此变量，会通过启动环境继承。
非法值直接报错。每次构造分配器输出：

```text
[SEARCH_ROUTE] version=long_axis_v1 axis=long_axis sector_sweep=0:x,1:x,2:x,3:x,4:x,5:x
```

`axis=column` 对应各分区 `y` 扫描。启动日志可确认实际模式，无需改变启动脚本。

## 可重复几何对照

`tools/test_search_frontier.py` 的 `replay(axis, zone)` 使用本轮每机首拍卖时自身位置、
合法边界 `[-55,155] × [-65,65]`、7 m 栅格和实际 `allocate/complete_waypoint`。
各分区分别回放，每个可用格只完成一次，其他分区不参与抢占。表中距离包括从起点进入分区；
时间只是距离除以 3 m/s，不含扫描、加减速、转弯、追踪或避障。

| 东侧分区 | 到 x≥100 原/新格数 | 到 x≥100 原/新距离 m | 到 x≥140 原/新格数 | 到 x≥140 原/新距离 m | 全区原/新距离 m |
|---|---:|---:|---:|---:|---:|
| 1 | 32 / 9 | 272.8 / 111.8 | 62 / 15 | 482.8 / 153.8 | 730.7 / 699.8 |
| 3 | 40 / 9 | 323.4 / 106.4 | 76 / 15 | 575.4 / 148.4 | 766.7 / 718.0 |
| 5 | 43 / 8 | 348.2 / 103.2 | 79 / 14 | 600.2 / 145.2 | 782.2 / 782.2 |

到 x≥140 的纯直线飞行时间分别从 160.9/191.8/200.1 s 降为 51.3/49.5/48.4 s。
六区总长从 4512.6 m 变为 4325.4 m；北西分区 4 从 801.5 m 增为 810.8 m（约 1.2%）。
全区格数依次 90、90、90、90、105、105，全部保留且无重复。

## 验证与限制

新增测试检查东侧首达、六区完整唯一遍历、排除不可用格后仍遍历余下格、高分区保留纵向路线、
以及没有因航点完成而虚记相机覆盖。既有搜索测试继续检查中断、边界格、接管、补看和相邻航点。

运行测试需 source ROS 与主库 coordination/devel，同时把隔离工作树的脚本和导航包放到
`PYTHONPATH` 前部，设置 `ROBOCUP_LOG_DIR=/tmp/robocup_search_frontier_tests`
和 `PYTHONDONTWRITEBYTECODE=1`。不启动节点或仿真。

```bash
source /opt/ros/noetic/setup.bash
source /home/gycc/桌面/RoboCup_Team/coordination/devel/setup.bash
export ROBOCUP_LOG_DIR=/tmp/robocup_search_frontier_tests PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/tmp/robocup_search_frontier/coordination/src/robocup_swarm/scripts:/tmp/robocup_search_frontier/coordination/src/robocup_navigation/src:/home/gycc/robocup_build/coordination_ws/devel/lib/python3/dist-packages:$PYTHONPATH
cd /tmp/robocup_search_frontier
/usr/bin/python3 -m unittest tools.test_search_frontier tools.test_search_plan tools.test_match_blockers tools.test_unavailable_tracking
```

上述 157 项定向测试通过；未在该工作树运行正式比赛。

这不是 180 s 完赛证据。新行顺序可能更早遇到建筑；完成一段后回到早先未完成的入口附近仍可能
出现较长接续航段，现有规划器负责可达性。只有正式对照轮的首次真实图像覆盖、有效发现、
障碍停顿和裁判结果能验证收益。当前正在验收的运行从未加载此改动。
