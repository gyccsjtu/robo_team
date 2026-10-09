# 替换模块交接包

本分支以主干 `d4976a2cb4f4dc8212e094839d31f019a0a762d8` 为修改基线，把本地评估过的模块、直接依赖、调用参考、接口说明和原有测试放在本目录，方便修改者在同一份代码里完成迁入。主干的实际运行文件保持基线内容；本包尚未接入默认 `run_match.sh`。

候选源来自同一仓库已推送分支 `codex/radar-coordination-20261003` 的固定提交 [`f4050badc03373329c03923903a640b7859aee16`](https://github.com/gyccsjtu/robo_team/tree/f4050badc03373329c03923903a640b7859aee16)，通过 `git archive` 原样导出，不依赖提供者电脑上的绝对路径、未提交文件或未推送提交。此前本地评估提交的相关源码、接口和测试与该远程版本一致。

## 给修改者的入口

1. 阅读 [模块对应与迁移接线](MODULES.md)，确认首批范围、依赖和接口。
2. 候选代码位于 [candidate/coordination/src/robocup_swarm/scripts](candidate/coordination/src/robocup_swarm/scripts/)、[candidate/perception](candidate/perception/) 和 [candidate/coordination/src/robocup_navigation](candidate/coordination/src/robocup_navigation/)。它们是迁移来源，包含旧 agent、manager、bridge 等调用参考；避免把这些大文件整体覆盖到主干。
3. 从本分支新建修改分支，逐项迁入仓库根目录下的当前运行文件。第一批建议雷达最终保护、最终执行输出、相机几何与跟踪时间；随后接任务授权和原图证据，再接地图、机队和路线，最后有界退出。
4. 每次迁移先重跑相应原有测试，再在主干默认运行链路补故障注入与 ROS/PX4/Gazebo 验证。迁移后的测试应验证真实主干调用，而不是继续只测候选快照。

```bash
git clone --branch codex/module-handoff-20261009 --single-branch https://github.com/gyccsjtu/robo_team.git
cd robo_team
git switch -c your-module-integration
python -m venv .venv
# 激活环境后安装离线测试依赖
python -m pip install -r coordination/module_handoff_20261009/requirements-offline.txt
python -X utf8 -B coordination/module_handoff_20261009/verify_handoff.py --output-dir /tmp/robo_team_handoff_check
```

Windows 可把最后一个输出目录换为可写的 Windows 路径。测试使用 Python 标准库 `unittest`，部分几何测试依赖 NumPy；无需安装 ROS、Gazebo、PX4、YOLO 或模型权重。当前记录的验证环境是 Python 3.12.14、NumPy 2.3.5；建议 Python 3.11 或更新版本。

## 内容与出处

- [source_manifest.json](source_manifest.json)：每份上游文件的来源路径、字节数、SHA-256 和 Git blob ID；校验整个候选快照。
- [verify_handoff.py](verify_handoff.py)：校验快照后按三组独立进程重跑候选测试，输出可读日志和结构化结果。缺依赖、测试失败、错误、跳过或校验失败均不算完整通过。
- [validation](validation/)：本交接分支的实际重跑记录，222 条离线测试通过；未执行飞行仿真。
- [候选接口说明](candidate/coordination/docs/)：雷达速度、停车输出、任务授权 v2、视觉证据、地图/规划、机队运动、路线预约、搜索观察等原有文档。

`candidate` 下保留原目录结构，便于原有测试读取实际 agent、manager、perception 源文件。其 `CATKIN_IGNORE` 与 `COLCON_IGNORE` 标记防止构建工具把参考快照误当运行包；本包不会通过启动配置启用候选实现。整个上游文件清单以 manifest 为准。

接口修改须更新版本、变更记录与兼容/迁移说明。所有授权参与者共享新的 `ROBOCUP_RUN_ID`；旧 assignment 的诊断消息不应继续驱动飞行。TTL、失联和发出 STOP 不等于旧飞机已经停止或区域已清空，锁和占用释放仍需实测证据。

二维雷达、平面预约、开发场景启动净空和离线测试不能证明正式随机场景的三维安全。保留当前限高、爬升锁和恢复超时的最终执行语义，补实际安装/坐标/采样时间及制动标定。原始官方裁判参考在快照中保持原样，仅用于消息兼容测试；不应把开发裁判副本安装成官方替代。红人报告遵循用户提供的红色与坐标口径，其来源说明见 [red_matching_organizer_20261003.md](candidate/coordination/docs/red_matching_organizer_20261003.md)。

本交接包是可复用源码与迁移依据，尚未完成主干接入、六机飞行或正式比赛验收。
