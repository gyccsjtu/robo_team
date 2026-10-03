# 六机云台连线接口 v2（2026-10-04冻结）

这是启动连线接口的schema_version=2；任务schema2/v2.2与视觉schema2/v2.8字段保持。
v1.20实测机头已转，但CGO3相机仍朝世界正东。原插件系统ID固定1、所有实例向UDP13030发包，
而实际飞机实例1..6、系统ID2..7、PX4云台端口13031..13036；旧连线漏掉了这条链。

v2每个uavs条目增加px4_gimbal_port与gimbal_local_port，分别从真实px4-rc.mavlink
的udp_onboard_gimbal_port_local/remote基值加px4_instance派生，不能按模型后缀推断。
既有mavlink_system_id仍是rcS核准的instance+1；端口、身份、接收解析状态逐机独立。

生成的gimbal_controller插件使用独立开发覆盖库，SDF包含mavlink_system_id、px4_udp_port、
gimbal_udp_port。覆盖库只改变连线和接收解析隔离，不改变传感器、云台PID或机械几何。
原插件已有follow模式：收到本机AUTOPILOT_STATE_FOR_GIMBAL_DEVICE后，水平相机随机头转；
其实际响应须另行六机取证，构建通过不算飞行验证。

迁移：prepare_radar_fleet与radar_fleet_models/启动器一起升级。模型生成器仍接受旧v1用于历史
夹具；带云台覆盖的新城市启动要求v2，缺字段/覆盖库应失败，不静默沿用世界正东相机。
独立构建入口build_namespaced_gimbal_overlay.py，依赖完整runtime内的compile_commands/Ninja
及已核准官方源；只写全新输出目录，记录原源/覆盖源/库SHA与编译链接命令。
原官方文件保持只读。正式环境等价性与完整六目标仍未完成，不宣称官方发布新插件。
