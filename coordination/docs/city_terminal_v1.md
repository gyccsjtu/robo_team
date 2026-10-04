# 城市裁判终态文件 schema1

仅为隔离开发裁判包装器与运行脚本之间的本地结果证据，不改变官方源码或判据。
原规则的最后一次消除可能在话题回调中先调用 signal_shutdown，来不及发布
`/left_actors=[]`；正常裁判退出不能直接等同故障，也不能只凭退出0判成功。

包装器在原裁判程序返回后原子写入 ROBOCUP_JUDGE_TERMINAL 指定文件：
schema_version=1、run_id、judge_sha256、mission_finished、left_actors、
target_finish、score。成功必须是同run、同执行源码SHA、正常退出0、
mission_finished=true、left_actors=[]、target_finish=6、有限正分数。
缺文件、错误运行、错误源码、超时/限高失败、剩余目标或异常退出均不能证明成功。
ROS旧话题值保留为诊断，不覆盖原始事件。

这解决结果收尾竞态，不能证明无碰撞或正式比赛通过。真实运动与接触证据仍需独立检查。
