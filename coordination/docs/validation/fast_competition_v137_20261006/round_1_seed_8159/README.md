# v1.37真实六机600秒：4/6，接触0；v1.38候选未实体执行

- 执行/start HEAD：`f5b20efe41d44769591b011d85f5581bf6fe9202`，seed8159，run_id `8ce6d37b-f755-4220-8445-381bcee33040`。
- 本轮配置600秒自然结束，原始计时600.116秒；轨迹1930.680–2530.632、跨度599.952秒。没有主动提前结束、建筑接触收尾或服务自行崩溃。
- 实际裁判 **4/6**：actor0绿117.556秒、actor1蓝157.640秒、actor4一红212.348秒、actor5另一红425.744秒。剩actor2棕与actor3白。原judge/result/budget保留；不能把起栈`connectivity_armed=false`当未起飞，也不将4/6标作比赛完成。
- 六机水平最大位移分别55.237/48.104/131.058/59.403/59.951/91.192米；最高真实3.289213米，抽样最小三维机间距2.810964米。150356帧原生接触从1929.448到2530.868覆盖轨迹，最大帧间隙0.004秒；建筑/人物/机间接触均0。virtual sonar与地面记录另列，不当作机体撞楼。
- 130份swarm、12份辅助源码SHA全部匹配。原`source_manifest.json`与派生`executed_source_audit.json`分开，未覆盖原清单；共享GPU六客户端ready、实际inference_device=shared。

`validation_audit.json`列实体结果；原结果及裁判在`flight/`，执行源码在`flight/swarm_sources/`及`flight/execution_sources/`。`archive_manifest.json`列逐文件SHA、原WSL绝对路径、复制/压缩策略；无损gzip条目同时核对解压SHA。完整大导航、Gazebo、PX4日志保留：

`/root/robocup_runs/heartbeat_v137_20261006T020228Z/round_1_seed_8159`

精选证据约45.5MiB，含15条完整短窗口NAV、实际广播日志压缩件、51张白色原始检测PNG、正观测捕获日志及部分PNG。未选PNG不是不存在，原文件路径/SHA在清单中。没有修改图片像素，少量观察器截图是只读取证，不进入控制。

`archive_git_index_sha_audit.json`已核对365份原始/派生文件的Git暂存字节SHA及3个gzip解压SHA，均匹配；该新生成核验、清单自身和README不计入这365份源字节比较。`candidate_source_manifest_v138.json`另列9个候选源码/测试文件：agent工作副本继承的CRLF与Git规范化LF具有两种SHA，已明确分别记录，其余内容一致；原已执行快照仍按`-text`原字节保存。

## 独立视觉核验不是裁判替身

`independent_visual_postrun_audit.json`使用真值前后样本插值（间隔不超过0.2秒、不跨位置跳变、不外推）。删除界限使用实际`left_actors`观察器首次收到移除的时刻，见`actual_judge_clock_evidence.json`；它仍是回调接收上界，不是裁判删除时间。已更正“ready时钟+Time usage等于准确删除时刻”的假设，并重新计算派生指标；原日志与裁判时间没有修改。

| 颜色 | 原图观测/可对齐/误差<1米 | 实际上传/可对齐/误差<1米 | 独立准确上传最长采样跨度 |
|---|---|---|---|
| 绿 | 67/67/67 | 202/201/201 | 14.2秒 |
| 蓝 | 108/98/73 | 220/218/185 | 14.9秒 |
| 棕 | 264/263/261 | 579/577/526 | 12.0秒 |
| 白 | 17/17/15 | 47/47/47 | 4.6秒 |
| 红 | 130/130/124 | 348/348/333 | 15.0秒 |

以上采样跨度、原图跨24秒等均不是裁判计时。绿、蓝及两红已由实际judge确认消除；不能用观察器少于15秒否认实际裁判。蓝/红大离群误检、删除后的旧真值，以及棕瞬移后的约72.6米误差分别保留，不能归为普通过滤误差。白色丢观测后的原图未保存，不能断言其根因为遮挡或YOLO。

## 实际修复范围与候选边界

`runtime_retained_route_geometry_audit.json`用实际NAV与先于NAV的广播重构6个场景，v1.37保留刹停空间允许实际非零命令，旧当前折线判定拒绝。grant seq为重构，非完整agent回调。`short_orbit_recorded_inputs.json`保留三段真实短路线，原0.2弧度盘旋约1.6米使位置环请求偏慢。

`orbit_geometry_recorded_input_audit_v138.json`是下一候选的有限几何查询；9米直线grant为构造，真实先前保留广播只供实测刹停复核，目标速度未记录而未加入回退。它不证明实际地图可规划/管理器可授权/实体可追逃。`v138_final_unittest.log`为597项离线检查，v1.38未在本轮执行。

raw coordinate、source velocity、linear fit等hypothesis文件是离线拒绝的方案，不在生产代码中。原型按观察器接收日程近似重构，非完整相机/bridge真实回调；个别假设只覆盖到约2198秒，不能称完整600秒回放。

## 收尾与下一步

自有launcher612已结束。8个只读取证进程均已退出；五个准确度观察器忽略shutdown且查错result目录，复核完整cmdline后先SIGINT/SIGTERM，仍存活才逐个SIGKILL。详见`process_cleanup_audit.json`，未全局清场，11375/11376/19731关闭；下次须重新核对实时状态。

按[收尾计划](../../../v137_postrun_plan_20261006.md)先离线修复，见[v1.38候选和接入](../../../fast_competition_fixes_v138.md)。这一实际窗口的一轮预算已用完，下次实际定时唤醒再验证。正式相机、地图边界/出生、三维盲区及棕白15秒持续定位未完成，不宣称正式PASS。
