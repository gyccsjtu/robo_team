# v1.35完整600秒：3/6与无建筑接触

- 执行f2f7371415c31f6bb8807eae62b6ce3fb6dc5723，8159，run_id `d175ed5f-6fbb-431b-bf76-89173844f4e7`。
- source_manifest.json是原始全仓预启动清单；130份执行快照与12份辅助源码的派生核验单独保存在executed_source_audit.json。初次归档曾误用同名分析覆盖此原始副本，随后恢复原始精确字节并分开命名；WSL原件始终未修改。
- 完整现场 `/root/robocup_runs/heartbeat_v135_20261005T164449Z/round_1_seed_8159`。精选约22MB；archive_manifest.json逐文件记录WSL路径、原始SHA及gzip/选取件SHA。六个完整navigation大文件留WSL，最新短窗口保留；gzserver只尾部，PNG为部分原始帧。新生成分析文件独立命名，不冒充原始输出。
- 配置600秒自然结束，原始计时600.088秒，实际轨迹跨度599.888秒（1930.904–2530.792）。裁判actor2棕153.860秒、actor5红414.356秒、actor1蓝453.460秒，合计3/6；剩actor0绿、actor3白、actor4另一红。原始result/budget/judge日志未改写。
- 原生150305帧接触观察覆盖轨迹，非地面机体接触0；六机水平位移47.909–121.647米，最高真实3.445664米，采样最小三维间距2.960369米。130份swarm及12份辅助源码SHA全部匹配。不是正式环境PASS，也不是六目标完成。
- 独立观察仅订阅/读取服务，无控制输入。白/红晚于其他颜色启动；绿色最后原图2177.816、白色2170.280；红色后段三张再发现坐标有大离群值，没有恢复持续上报。逐色数据与跳变过滤限制见independent_visual_postrun_audit.json。棕86/86可对齐原图误差小于1米；不能用该局部结果证明其他颜色或持续15秒全部达标。
- 早段蓝色固定装饰人物误报与后段真实蓝目标消除分别存在。绿色原图最大4.970米，官方上传大误差还包含瞬移后的短暂延用；红色原图最大139.919米须单独查身份/几何，不能都归为短暂延用。独立误差是分析，不替代正式judge.log。
- control_throughput_analysis.json保留六机CSV命令速度统计（中位数0.125–0.479m/s，峰值>2.3m/s）。旧导航日志无完整已授路线，不能完整回放拦截。v136_constructed_route_replay.json是新旧实际回调的构造调度，非历史完整回调或实体回放。
- 自有launcher和观察器9072–9075、66741、66742已退出，11375/11376/19731关闭，见process_cleanup_audit.json。旧harness授权标签仍是历史字面值，本轮实际唤醒授权在window/planned_validation.json。

先修计划[v135_postrun_plan_20261006.md](../../../v135_postrun_plan_20261006.md)，候选与边界[fast_competition_fixes_v136.md](../../../fast_competition_fixes_v136.md)。
