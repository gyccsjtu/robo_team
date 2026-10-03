# v1.21未起飞：ROS初始化内存分配失败

启动连线已接新云台库。3号agent在rospy.init_node/rospkg扫描过程中出现OSError errno12 Cannot allocate memory，启动器检测组件退出后自动收尾。没有解锁、0.08仿真秒、无六目标结果。未看到内核OOM杀进程证据，不把它说成确证内核OOM。后续仅压低OpenBLAS/NumExpr客户端线程池；WSL内存配置不变，收益待实测。

此目录是起栈故障证据，不能证明云台运动已修好。完整执行快照仍在WSL原运行目录及source manifest。
