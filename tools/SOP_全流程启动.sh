#!/usr/bin/env bash
# ============================================================================
# RoboCup 全流程启动 SOP（不直接执行，仅作速查 + 可粘贴片段）
# ----------------------------------------------------------------------------
# 用法：
#   把它当 cheatsheet 读，按场景把对应代码段复制到终端
# ============================================================================

# ============================ A. 一次性环境准备 ============================
# 每次开新终端都要 source 三件套（catkin + 协同编译 + env_robocup）
# ——————————————————————————————————————————————————————————————
# (可选)放进 ~/.bashrc 自动化，但跑仿真前手动 source 一次更稳
# source /opt/ros/noetic/setup.bash
# source $HOME/catkin_ws/devel/setup.bash
# source /home/gycc/桌面/RoboCup_Team/coordination/devel/setup.bash
# source /home/gycc/桌面/RoboCup_Team/perception/scripts/env_robocup.sh

# ============================ B. 场景 1：仿真自验（协同层 + 真值兜底）==========
# 适用：仿真下要看到"监控去追 actor"
# 注意：SIM_TARGET_NODE=1 会用 target_sim_node 喂真值，仅自验用
# ----------------------------------------------------------------------------
cd /home/gycc/桌面/RoboCup_Team
SIM_TARGET_NODE=1 ./run_match.sh start
# 验证
#   rostopic hz /swarm/target_states   # ≥10Hz 且 Publishers=target_sim_node
#   rostopic hz /swarm/assignment      # ≥5Hz 且 task_type=1(track)
#   rostopic hz /typhoon_h480_0/mavros/setpoint_velocity/cmd_vel_unstamped  # ≥10Hz

# ============================ C. 场景 2：真机/正式比赛（生产配置）============
# 适用：真机飞行 + 官方 YOLO 真机权重；严禁开 SIM_TARGET_NODE
# ----------------------------------------------------------------------------
cd /home/gycc/桌面/RoboCup_Team
./run_match.sh start
# 或显式写全开关（保留审计默认值）
#   YOLO_BRIDGE=1 SIM_TARGET_NODE=0 D2O_ENABLE=0 ./run_match.sh start

# ============================ D. 场景 3：不重启旁路挂 target_sim_node ==========
# 适用：run_match.sh 已在跑、想快速看"开了 SIM_TARGET_NODE 后 agent 会不会动"
# 关键：必须带 UAV_IDS="typhoon_h480_0..5"（前缀必须有，否则 swarm_manager 找不到）
# ----------------------------------------------------------------------------
cd /home/gycc/桌面/RoboCup_Team
source coordination/devel/setup.bash
cd coordination/src/robocup_swarm/scripts
UAV_IDS="typhoon_h480_0,typhoon_h480_1,typhoon_h480_2,typhoon_h480_3,typhoon_h480_4,typhoon_h480_5" \
  python3 -u target_sim_node.py 2>&1 | tee /tmp/target_sim.log

# ============================ E. 场景 4：服务器无头仿真（云端/CI）=============
# 适用：云端 GPU 服务器无显示器，用 noVNC 看画面
# ----------------------------------------------------------------------------
ssh -p 25316 root@region-42.seetacloud.com
setsid nohup bash /root/run_swarm_6uav.sh > /root/_demo_sim.log 2>&1 &
watch -n2 'grep -c "Successfully spawned entity" /root/_demo_sim.log'  # 计数到 6 即可
bash /root/start_gzclient.sh   # 另起一个窗口挂 gzclient
# 本地：ssh -p 25316 -L 6080:localhost:6080 -N root@region-42.seetacloud.com
# 浏览器：http://localhost:6080/vnc.html

# ============================ F. 调试命令清单 ============================
# ----------------------------------------------------------------------------
# 1) 端到端 topic 频率体检（10s 采样）
source /home/gycc/桌面/RoboCup_Team/coordination/devel/setup.bash
for t in /gazebo/model_states /coordination/target_report /swarm/target_states /swarm/assignment /swarm/uav_status; do
  r=$(timeout 4 rostopic hz $t 2>&1 | grep -E "^average rate" | tail -n 1 | awk '{print $3}')
  echo "  $t  rate=$r"
done
for i in 0 1 2 3 4 5; do
  r=$(timeout 3 rostopic hz /typhoon_h480_${i}/mavros/setpoint_velocity/cmd_vel_unstamped 2>&1 | grep -E "^average rate" | tail -n 1 | awk '{print $3}')
  echo "  cmd_vel_${i}  rate=$r"
done

# 2) 看谁在发某个 topic（找双发布者撞车）
rostopic info /swarm/target_states -v | sed -n '/Publishers:/,/Subscribers:/p' | head -n 25

# 3) 关键组件健康
rosnode list | sort
rosnode list | grep -E "swarm_manager|swarm_agent|yolo_target_bridge|target_sim|perception_real|radar_avoid|score_cal|control_actor|px4|sitl_" | head

# 4) 体检后处理：跑脚本自带 status 子命令（推荐）
cd /home/gycc/桌面/RoboCup_Team && ./run_match.sh status

# ============================ G. 收尾 ============================
# ----------------------------------------------------------------------------
# 优雅停止（清空所有 run_match.sh 起的子进程）
cd /home/gycc/桌面/RoboCup_Team && ./run_match.sh stop

# 硬杀（脚本 stop 卡住时用 —— 仅应急）
pkill -9 -f "swarm_manager|swarm_agent|yolo_target_bridge|target_sim|perception_real|radar_avoid|score_cal|control_actor"
pkill -9 -f "px4|gazebo|mavros_node|multirotor_communication"