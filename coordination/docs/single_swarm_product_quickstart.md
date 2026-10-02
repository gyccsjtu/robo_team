# 本仓库单机雷达协同原型：构建、运行与验收

2026-10-03。控制代码来自 gyccsjtu/robo_team 的开发分支；不是先前 radar_product 的控制算法。运行真实 Gazebo/PX4/MAVROS 和本仓库 SwarmAgent、SwarmManager/任务授权 v2。两段任务为开发验证用例，尚未执行完整六机搜索与目标感知。

## 当前机器已核准依赖

| 项目 | 路径/要求 |
|---|---|
| 发行版 | WSL Ubuntu-20.04，ROS Noetic、Gazebo11、MAVROS、GeographicLib已安装 |
| 仓库 | Windows D:/a/.robocup/robo_team；WSL /mnt/d/a/.robocup/robo_team |
| 本仓库独立构建输出 | /root/robo_team_build/codex_authority_v2 |
| 已验证 runtime | /root/robocup_runtime/stereo_20261002T140417Z，gazebo14库、gps2库、actor_lib2库；GPS兼容层须启用 |
| PX4 | /root/third_party_official/PX4-Autopilot，build/px4_sitl_default/bin/px4 已编译 |
| XTDrone | /root/third_party_official/XTDrone_official |
| 模型和实验世界 | /root/vendor_eval/d43bac6/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf；该目录 robocup_real/_genvm/m1/robocup.world |
| 场景模型缓存 | /root/robocup_resources/vm_gazebo_cache_20261002/models |
| 旧环境只读引用 | /root/robocup/robocup_ws/devel/setup.bash，供既有Gazebo插件环境；控制Python及自建消息来自本仓库独立构建 |
| 隔离端口 | ROS11345、Gazebo11346；单机PX4/MAVROS仍使用4560/14560/14580/14540等既有端口，不能与其他单机栈并跑 |

不是换机器安装指南。换机器需先准备表中已有资源；启动脚本允许EVAL/RUNTIME/XT/PX4/WORLD环境变量覆盖相应路径。runtime的build_corrected.sh不是已核准的可移植重建入口，不应照旧交接文档直接重建。

## 可直接执行：Windows PowerShell

首次构建本仓库两包（不修改旧工作区）：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'TEAM_BUILD_DIR=/root/robo_team_build/codex_authority_v2 bash /mnt/d/a/.robocup/robo_team/coordination/scripts/build_isolated_swarm.sh'
```

启动、执行、独立采样、保存结果、清理本轮进程：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'source /root/robo_team_build/codex_authority_v2/devel/setup.bash; python3 /mnt/d/a/.robocup/robo_team/coordination/scripts/run_single_swarm_product.py --output /tmp/robo_team_single_run_01 --timeout 180'
```

输出目录必须尚不存在，第二轮换新名字。默认从(0,-3)起飞到估计高度4.5m，执行(8,-3)、(8,-10)两个搜索任务。逻辑ID uav_1；模型 typhoon_h480_0；MAVROS/scan实际命名空间通过参数显式映射。仅agent发布飞控速度；不启动radar_avoid独立控制器。

启动环境、模型适配和GPS兼容条件由脚本设置，保留生成SDF与启动日志。开发模型激光归一到512束、0.5–20m；不能以此宣称整模/兼容插件与正式环境等价。本轮仍读取已存开发metadata做A*；随机场景等价性未验证，不能视为在线自主建图已经完成。

## 看哪些输出

- audit_report.json：独立最终位置/速度、最高高度、两次到达真实偏差、样本覆盖、STOPPED_ACK数量、有限原型验收结果。
- mission/result.json、telemetry.jsonl：控制估计与失败原因。日志“完成”不作为成功依据。
- truth.jsonl：独立Gazebo轨迹；只在审计进程订阅，不回灌控制。
- algorithm/authority_events.jsonl：授权、STOP、停稳、旧锁保留与退出释放事件。
- source_snapshot/、source_manifest.json：本轮实际执行源码冻结副本和哈希；工作区后续编辑不改变正在执行的源码。
- stack/ 下带运行ID的目录：生成模型、进程PID、PX4/MAVROS/Gazebo日志。

原型验收要求两段控制到达、实测最终误差<=0.6m、最终速度<=0.15m/s、采样最高高度<6m，且有真实停稳交接事件。任一不足则原型标志false，进程退出非零；启动/节点异常也写FAILED。碰撞完整证据缺失仍ABSTAIN，formal_competition_pass固定false。

## 本次真实飞行证据

工作区 tmp/evidence_archive/robo_team_single_flight_20261003_run02：原型到达验证为true；最终水平误差0.29036m，末速0.05742m/s，最高4.85506m，805个独立样本，最大间隔0.056s；两次到达时真实水平误差分别0.07690m、0.21791m，STOPPED_ACK一次。run01保留为失败复现：栅格中心和任务点不同导致末段停在约0.35m偏差，终止后修复；run01期间源码有修改，不作为源码一致性验收。

加入进程锁和端口占用检查后，run03再次通过：最终水平误差0.27484m、末速0.06122m/s、最高4.82664m，796个独立样本，最大间隔0.052s，STOPPED_ACK一次。完整轨迹、事件、结果、哈希和实际执行源码压缩包已保存到仓库 `coordination/docs/validation/single_swarm_run03/`，供另一台机器直接核对。全套207条测试中204通过，3条为初始提交已存在的replan_route测试错误；端口占用拒绝已用真实UDP占用测试。

后续仍须完成在线建图接入、六机端口/进程锁隔离、空间通道预约、完整目标感知与比赛裁判联调、碰撞证据采集。此文档交付的是可执行单机协同原型，不是六机成品或正式比赛PASS。
