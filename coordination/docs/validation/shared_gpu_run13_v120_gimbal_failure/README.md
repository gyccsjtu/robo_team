# v1.20真实六机：发现云台连线故障后主动结束

本轮包含观察者优先/停稳接替和每原图一次上报，仍是原有云台插件。
307.932仿真秒主动结束，最后六目标全部剩余：0/6；原始error保留为CHECK_INTERRUPTED_OR_WALL_TIMEOUT。
result中的SIX_RADAR_CONNECTED仅是启动连通状态，不是飞行或比赛PASS。

实际机体转向没有带动相机：云台插件固定sysid1和目的UDP13030，实际本机系统ID2..7、目的端口13031..13036。
body_camera_heading.json是自身link实测，camera heading近0而机体已转0.49/1.71rad。
独立开发覆盖v1.21随后才构建/使用，不能把新库视为本轮已执行；接替请求数量与高度/接触汇总见partial_trial_audit.json。

只停止本轮启动器与自身帧观察进程，无全局清场。原始轨迹、裁判、事件、执行源码和SHA保留。
*.log需要显式纳入Git，.gitattributes禁用换行转换；本轮未跑满600秒、完整六目标未完成，formal_competition_pass=false。
