# v1.25实际六机轮：3/6，约500秒提前中断

执行提交5a03aa3，运行标识718f3abb-317d-48e2-8a29-acc1f4aa2bea，种子8159，上限600仿真秒。预算ENDED，实际499.968秒，完整现实运行约52分50秒。f687e2c每次追踪计数修复不在该执行快照中。

裁判删除actor_0/1/4：绿、蓝、一个红；仍剩actor_2/3/5：棕、白、另一个红。红色上传red及坐标，与任意剩余红色真值匹配，不要求red1/red2固定身份。3/6不是比赛完成。

六机均解锁且实际运动，位移64.36–134.64m。原生接触观察125266帧，最大采样间隔0.00400000000036秒，覆盖实际轨迹；建筑/人物/机间接触0。最高真实高度4.069254m，采样最小三维间距2.617995m。128份执行源码SHA无不一致。

提前结束错误CITY_MODEL_STATE_FAILURE，GetModelState连接被对端重置。缺少清场前gzserver退出码，无法确认原因。本轮首次事后内核检查未见OOM/segfault，后续空闲WSL的启动记录不能当作退出原因。judge.log尾部ROSInterruptException发生于收尾，不能认定它触发了Gazebo故障。

独立白色观察器不是全轮覆盖：62条对齐上报中约64.5%误差严格小于1m，平均0.851m、最大1.779m。17对同原图坐标中原始/过滤后平均误差0.677/0.715m，不足以直接删除滤波。ROS2365–2373.5窗口19张白色原图均获人体/衣服证明，轨迹31不变，但漏帧中断连续证明；详见white_postrun_diagnosis.json。独立人物真值没有输入控制器。

完整运行在WSL：/root/robocup_runs/goal_v125_20261005_065119/round_1_seed_8159。archive_manifest.json记录所有归档时原文件SHA、长度和WSL路径；manifest only文件未提交内容。*.gz是无损原始日志/世界文件，解压后SHA与清单一致。gzserver_tail.log仅保留最后16KB，完整约410MB日志和导航长日志、PX4 ULog仍在WSL，不能把尾部当完整日志。

任务schema2、路线schema1不变，共享证明metadata2。own camera Gazebo link pose的正式比赛许可和雷达完整三维证明仍未核实；formal_competition_pass=false。后续先读[下一轮计划](../../../fast_competition_next_plan_20261005.md)，完成离线修复后才开下一次有明确目的的仿真。
