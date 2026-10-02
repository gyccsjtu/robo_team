# 雷达避障模块说明（radar/）

## 是什么
二维激光雷达避障：无人机沿航点巡逻时，用 /scan 雷达数据实时避障（雷达闭环 + A*），避免撞建筑/杆子。

## 启用条件（缺一不可）
1. 雷达模型两处一致（md5 相同）：
   ~/PX4_Firmware/Tools/sitl_gazebo/models/typhoon_h480_lidar/
   ~/XTDrone/sitl_config/models/typhoon_h480_lidar/
2. 场景用带雷达 launch：RoboCup_Team/launch/robocup_lidar.launch
3. 航点文件 6 份：radar/waypoints/typhoon_h480_N.txt（每行 "x y" 世界坐标）
4. 启动命令：SKIP_RADAR=0 ./run_match.sh start

## 航点分区（当前方案）
地图范围 x∈[-55,135]、y∈[-65,65]，按 x 分 6 列，每机一列、4 个巡逻点：
机0: (-39,-50) (-24,-50) (-24,50) (-39,50)
机1: (-7,-50) (8,-50) (8,50) (-7,50)
机2: (24,-50) (40,-50) (40,50) (24,50)
机3: (56,-50) (71,-50) (71,50) (56,50)
机4: (87,-50) (103,-50) (103,50) (87,50)
机5: (119,-50) (135,-50) (135,50) (119,50)

## 验证
- 启动后逐机确认 /scan 有帧：rostopic hz /typhoon_h480_N/scan
- 状态：./run_match.sh status（雷达进程数）
- 日志：/tmp/robocup_match/latest/05_radar_*.log
