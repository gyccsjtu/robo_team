# 真实相机与演员识别开发观察

入口将仓库actor_min.world中的actor_0复制到独立开发world，保留walker/walk_0.dae、动画和原插件，仅修改开发场景的初始位置至(-15,3)，避免与前方友机遮挡。六机仍为8m间隔；本用例禁止解锁和飞行，不改变传感器参数。最新版仅将uav_1设为2.2m高度的静态相机安装架，用于验证地面交点几何；不是飞行或停机安全证明。原始camera_info与图像保存后由仓库现有YOLO权重识别，并执行实际perception_real节点。仅像素命中而没有实际观测消息不能判本用例通过。

视觉环境使用/root/robo_team_build/vision_env，ROS消息仍来自隔离catkin。Linux下载TLS失败后，由Windows下载CPU Torch官方wheel及固定版本依赖，离线安装到单独venv；原/root/robocup和官方依赖源码均未改。依赖及wheel/模型SHA需随结果记录。

run01..12相机有图像、演员插件有位姿、骨骼消息和渲染Visual也存在，但没有人物像素。auto_start和SDF pose假设均未解决问题。run13将LIBGL_ALWAYS_SOFTWARE=1切换至llvmpipe后，原权重识别green置信度0.892；run14移除额外script后仍识别0.900。此前后端为Mesa21.2.6 D3D12 RTX4060，人物骨骼网格不可见。当前隔离入口默认软件渲染，传感器参数不变；正式环境须另做实际图像检查。render_observer.cc是可选只读诊断插件，不给感知/控制输入真值。

run15的实体支撑台遮住低处目标；run16将整机设static导致无MAVROS定位，按失败保留。当前使用固定到world的base_link关节，机体仍为动态模型，六机定位与雷达检查保留。相机位姿独立20Hz采样，200个样本按真实图像时间对齐。图像每帧只关联一次，coast不更新观测证据时间；原AR_MAX=2.2会拒绝实际人物AR约2.25..2.5，改为3.8（宽松4.5），仍保留身高、置信度、运动和附着物判据。

run20原始相机green置信度0.908；实际perception产生5条独立图像观测，实际桥产生3条source=1/uav_1/t0检测与官方green坐标消息。证据见validation/camera_bridge_run20；这不是六机视觉飞行、15s消除或坐标精度验收，physical_coordinate_accuracy_verified仍false。该轮退出时另出现boost线程析构错误，不能称全过程正常退出，后续增加ROS订阅关闭流程待复验。自身相机Gazebo位姿仍是开发接入，正式允许的自定位/云台姿态来源尚须确认与替换。

run21通过实际相机→perception→桥→manager登记，无合成观测。manager使用v2.1确认话题验证本轮原图证据；该用例不启动飞行agent，execution_authority_granted=false。已正常退出，新增signal_shutdown后未复现run20的析构错误。完整结果及执行源码在validation/camera_manager_run21。后续跨机旧采样拒绝补丁由适配器单测验证，不能冒充已经由run21执行。

可复现：`DISPLAY=:0 bash run_radar_swarm.sh --camera-actor-probe --output /tmp/new_camera_run`。必须先完成隔离catkin构建及固定版本vision_env安装。软件渲染为当前WSL默认，显式LIBGL_ALWAYS_SOFTWARE=0可作故障对照，不应当作识别通过环境。
