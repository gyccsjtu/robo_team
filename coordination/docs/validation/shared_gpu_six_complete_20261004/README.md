# 六机真实运动仿真：六目标全部消除

运行ID `fd36c87a-0e37-4a02-bbf0-549cf9df9cda`，执行提交
`1004277e1d3c20ba4bbdd8d3f892005f2444cc0b`，视觉v2.9。
WSL原目录 `/root/robocup_runs/codex_city_teammate_v122b_20261004/flight`。
共享GPU、原WSL内存、局部巡航2.2m、实际物理运动，未输入目标真值控制。

- 开发裁判依次成功删除 actor_3、5、4、0、2、1；最后打印 Mission finished。
  最后一条 Time usage为405.456秒，最终分数2169.432。
- 六机均解锁并实际运动，最大水平位移分别70.35、48.68、68.77、106.89、47.56、67.10m。
- 采样最高真实高度3.568033m，最小采样机间距2.962136m；不是连续轨迹极值证明。
- 原生机体接触观察101808帧，1929.584至2336.812，最大间隔0.004秒，覆盖完整轨迹。
  406条地面起落架接触记录，**非地面机体接触0条**，没有墙体或人物接触记录。
- 自身相机位姿为开发Gazebo link pose，正式规则许可尚未确认；
  仅此开发城市已验证，实体机及其他随机地图未验收，formal_competition_pass保持false。

## 为什么原result.json仍写5/6与故障

最后一个目标在裁判回调内删除后，原裁判立即signal_shutdown，来不及再发布
`/left_actors=[]`。启动器仅凭旧话题[1]把裁判退出0误判组件故障，随即收尾。
**原始result.json没有改写。** `mission_final_audit.json`记录独立核实后的6/6及理由；
`judge.log`、执行裁判源码SHA、全部六个实际成功删除分支与原生物理证据均保存。
它们不同于无条件打印“航点完成/全部到达”的单机日志。
最后源码的打印位于_delete_actor成功之后，Mission finished仅在target_finish==actor_num时执行。

仓库后来修为同run/同源码SHA的结构化裁判终态，缺证、剩余目标、异常退出或0分不能判成功。
`next_code_terminal_tests_431.log`是收尾修复及候选优化的测试，**不属于本轮执行快照**。
新蓝色运动窗与绿色停住豁免未在这轮执行，均改为CITY_EXPERIMENTAL_V123=1才启用；
默认保留此成功轮的视觉v2.9行为，不再为未启用候选重复跑仿真。

## 队友复现与证据

从仓库根目录按city_swarm_quickstart.md启动；保持共享GPU与云台覆盖库路径，
默认不要设置CITY_EXPERIMENTAL_V123。完整运行入口为run_city_swarm.sh。
目录包括原始事件、轨迹、命令、算法CSV、逐机日志、原图、裁判副本、
启动脚本/感知/核心源码快照及SHA。原始独立精度包含蓝色静态NPC错误上报，
不能只截最后成功片段冒充全程零误检。蓝色观察器一次服务失败已明确记档。
283MiB Gazebo重复诊断日志保留WSL并记录外部SHA，Git内保存尾段。
archive_sha256.json覆盖归档时的314个文件，README为后写说明。
