# BoT SORT 替换与六机开发仿真结果

2026-10-11。人物检测后的图像关联已替换为固定版本 BoT-SORT，接入世界坐标身份绑定和追踪恢复。六轮实际 Gazebo/PX4 开发试跑中，修复了重复飞控指令导致的起飞停滞；最后六机均通过原有 2.5m 起飞门限。Gazebo 随后发生 SIGSEGV，未取得有效追人片段，**追踪效果提升仍为 ABSTAIN，不能判定通过**。

开发分支为 `codex/botsort-tracking-20261011`，基于用户指定旧快照 `997cfa006a41d60e58eff724c5762ca13a3d9cc3`。没有切到后续主干方案。

## 已完成的替换

- `perception/image_mot.py` 接入 [Ultralytics BoT-SORT 源码](https://github.com/ultralytics/ultralytics/tree/69c95d1add13a232382e53b3aa5094ec0c7738db/ultralytics/trackers)，固定提交 `69c95d1add13a232382e53b3aa5094ec0c7738db`。源码版本 8.4.175；检测环境仍为 Ultralytics 8.3.0，原检测权重未更换。源码、AGPL-3.0 许可及兼容修改说明保存在 `perception/vendor/ultralytics_trackers/`。
- 每种颜色独立关联，共用每帧一次相机运动补偿；低分真实检测可以续接已有身份。真实图像时间控制失效和重建，重复帧不增加观测次数。预测框不充当新观测。ReID 关闭，没有另下载身份模型；同色重叠及遮挡后身份恢复仍需要有效视频验证。
- `perception_real.py` 将图像身份接到世界坐标轨迹，避免后续贪心近邻再次交换身份。保留定位、置信度及上报门限；低分续接不取得裁判上报权限。
- `yolo_target_bridge.py` 为非红色候选身份恢复增加实际连续观测、时间、距离和运动残差条件；候选恢复不写入合格裁判证据。红色不跨身份恢复。`swarm_agent.py` 修复旧目标影响远程转场、重复/延迟样本累积逃逸或静止判断的问题。
- `run_match.sh` 的协同入口只让对应 `swarm_agent` 向 MAVROS 发控制设定点。旧 XTDrone 通信桥会在没有命令时持续发送默认原始设定点，与 agent 的位置设定点争用同一个飞控；非协同入口仍保留该桥。图审计现在跨三种 MAVROS 输入话题检查唯一控制者。人物环境控制器仍读取 Gazebo 自己的里程计。
- 修复开发环境的六机 PX4/MAVROS 端口不匹配、固定 gimbal 端口、actor 插件路径及 Windows/WSL 工作树源码索引。原始官方文件只读；端口修正、消息构建和场景试验使用独立开发副本。

本轮未更改公开 ROS 消息 schema、任务授权或通道安全协议；图像身份和恢复状态是内部扩展。`PR_IMAGE_TRACKER=legacy` 可选择原图像关联器进行诊断，但这不构成旧版整场运行的完整复现。

## 实际试跑

全部为本机 ROS1 Noetic、Gazebo 11.15.1、PX4 的开发仿真。编号对应原始日志目录，便于复查失败记录。

| 轮次与目录尾名 | 环境 | 实际结果 |
|---|---|---|
| 1 `183127` | 原开发世界 | 六机模拟器链路建立，但 MAVROS 0/6 connected；停止后修复私有端口配置 |
| 2 `183554` | 端口修复，CUDA | 六机 MAVROS/相机就绪；三个感知进程 device busy，停止后改 CPU |
| 3 `183813` | CPU，原开发世界 | 六路感知和启动检查通过；Gazebo 在 sim1790.892 SIGSEGV，无有效追踪数据 |
| 4 `184519` | 独立移除人物碰撞插件试验 | 78 张稀疏相机图像；飞机未爬升，报告为空；sim1801.596 再次崩溃 |
| 5 `184850` | 独立关闭阴影，保留人物碰撞插件 | 六机 TAKEOFF_STALLED，稀疏图像174张；sim1832.840 崩溃 |
| 6 `185723` | 与第5轮同类场景，修复重复飞控输入 | 六机原起飞门限通过并开始水平机动；sim1803.216 崩溃，未取得有效追人序列 |

关闭阴影和移除人物碰撞插件均没有解决已观测崩溃，均不用于正式比赛验收。保留官方人物碰撞插件的最后一轮中，MAVROS 本地 z 最大值按飞机0..5依次为 **4.256、2.860、3.455、3.678、2.590、2.516m**。这是本地位姿记录，不是独立测量的真实高度或持续安全证明。

最后一轮 observer 正常排空写入：9358条事件、60张稀疏图像、0张有效触发 burst 图像；六路共572个真实已处理帧。墙钟处理输出约 **1.54～1.55 FPS/路**，原图时间戳约5帧/ROS秒；仿真运行缓慢，两者不能混称。456个目标报告和583个发现观测消息均为空，没有 target_states、tracking_view 或 ActorInfo 结果。13个带框 CSV 样本均为 green 框的几何拒绝，原因 `sin_down_below_0.30`；这些框不足以证明检测到了真人。仅这些样本的推理平均0.282s、p95 0.354s，不能当作全帧性能或 BoT-SORT 独立耗时。

最后一轮图注册审计通过，六机未再出现 TAKEOFF_STALLED；这只证明相应启动和起飞事实。追踪连续性、定位精度、旧新算法效果、碰撞和连续净空均缺少足够证据，保持 ABSTAIN。裁判日志未消除人物，任务未完成。

内核故障指令位于 `libOgreMain.so.1.9.0` 的 `Ogre::EdgeListBuilder::buildTrianglesEdges`。没有完整 core/backtrace，尚不能证明启动崩溃的具体模型或插件根因。记录见 `botsort_gazebo_crash_20261011.json`。

## 检查与证据

固定库真实依赖下，图像关联、身份恢复、实际观测时间及既有坐标/裁判边界检查 **190项通过**。飞控唯一来源、现有图审计和团队入口另 **21项通过**。结果见 `botsort_checks_20261011.json`；这些检查不替代仿真追踪质量验收。

本目录 `validation/botsort_20261011/` 保留最后一轮分析、事实摘要、observer metadata 和启动时源码哈希。运行时 manifest 的 Git HEAD 仍为旧基线，dirty 文件的实际源码另逐一记录哈希，不能只凭 HEAD 忽略本轮代码修改。

完整原始日志、CSV、图像及事件归档在本机 `D:/a/.robocup/tracking_validation_20261011/simulation_attempts_20261011.zip`，共498个文件，13,252,789字节，SHA256 `48cd1a9078737947ad9544cf36ca7af88971f3b8920d4796ccacb05cc864c6c0`；旁边 `archive_manifest.json` 给出逐文件哈希。该归档还保留一次进入仿真前的源码索引失败记录。全部本轮飞行/感知/observer进程已停止。

## 本机复跑与交接

代码、接入和失败证据可交给接手者；当前不是追踪效果验收完成。先排除 Gazebo 崩溃，再录制真实人物进入视野后的连续 burst，用同一图像和原检测权重运行 `tools/replay_tracking_comparison.py`。离线对照只比较图像关联；完整追飞还需独立实际飞行与定位证据。

已准备的本机环境执行以下 PowerShell 命令。默认 CPU；六路 CUDA 并发失败尚未解决。启动和 observer 需要两个终端，不能同时运行另一套占用相同端口的仿真。

```powershell
$trial = '/mnt/d/a/.robocup/robo_team_botsort_20261011/coordination/scripts/container/run_botsort_trial.sh'
# 最后一轮的开发场景；依然会遇到已知 Gazebo 崩溃。
wsl.exe -d Ubuntu-20.04 -u root -- env BOTSORT_DISABLE_SHADOWS=1 bash $trial start
wsl.exe -d Ubuntu-20.04 -u root -- env BOTSORT_DISABLE_SHADOWS=1 bash $trial observer
wsl.exe -d Ubuntu-20.04 -u root -- env BOTSORT_DISABLE_SHADOWS=1 bash $trial stop
```

环境首次准备、私有资产和依赖见 `botsort_sim_environment_20261011.md`；同画面对照及证据边界见 `tracking_replay_comparison_20261011.md`。接手时保留 vendor 的固定提交和许可，使用实际图像时间及原始源码 manifest；不得将候选身份恢复、预测框或重复消息写成连续裁判观测。
