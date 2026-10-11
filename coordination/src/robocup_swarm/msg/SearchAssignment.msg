# 集中式管理器指派给某架无人机的搜索任务（自建消息）。
# target_x/y 为任务格中心的地图系坐标，agent 可据此规划/飞行。
Header header

string  uav_id        # 指派给谁
int32   cell_ix       # 任务格索引 x
int32   cell_iy       # 任务格索引 y
float64 target_x      # 任务格中心 x（地图系）
float64 target_y      # 任务格中心 y（地图系）
uint8   task_type     # 0=搜索 1=人物跟踪 2=RTL 3=降落 255=取消
string  target_id     # task_type=1/2 时的目标 ID（task_type=0 时为空）

# Search protocol v2. One reservation covers the entire short segment.
# task_type=255 with search_task_id cancels ONLY this task/revision.
# Empty search_task_id is the feature-disabled legacy single-cell protocol.
string search_task_id
uint32 search_revision
int32[] search_cell_ix
int32[] search_cell_iy
geometry_msgs/Point[] search_waypoints
time search_valid_until
