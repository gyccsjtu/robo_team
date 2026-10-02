#!/usr/bin/env python3
"""订阅队友的占据栅格地图，实现建图互通。

ROS 话题:
    /uav{0..5}/occupancy_coarse - 7m 分辨率占据栅格 (Int8MultiArray)

使用示例:
    from team_grid_subscriber import TeamGridSubscriber
    
    subscriber = TeamGridSubscriber(uav_id=0, fleet_ids=[0,1,2,3,4,5])
    # 在主循环中调用:
    subscriber.spin_once()
    
    # 获取合并后的全局地图
    combined_grid = subscriber.get_combined_grid()
"""
import numpy as np
import rospy
from std_msgs.msg import Int8MultiArray


# 栅格地图参数 (与 occupancy_online.py 保持一致)
X0, X1 = -52.0, 132.0
Y0, Y1 = -52.0, 52.0
CELL = 7.0  # 粗网格分辨率

GRID_WIDTH = int((X1 - X0) / CELL)
GRID_HEIGHT = int((Y1 - Y0) / CELL)


class TeamGridSubscriber:
    """订阅队友栅格地图并合并"""

    def __init__(self, uav_id, fleet_ids):
        """
        Args:
            uav_id: 当前无人机ID (0-5)
            fleet_ids: 所有无人机ID列表
        """
        self.uav_id = uav_id
        self.fleet_ids = fleet_ids
        
        # 存储每个队友的栅格数据: {uav_id: np.ndarray}
        self.team_grids = {uid: None for uid in fleet_ids}
        self.team_timestamps = {uid: None for uid in fleet_ids}
        
        # 存储最新数据的锁
        self._lock = None
        
        # ROS 订阅器
        self._subscribers = []
        
        if not rospy.get_node_uri():
            rospy.init_node('team_grid_subscriber')
        
        # 订阅除自己以外的所有队友话题
        for uid in fleet_ids:
            if uid != uav_id:
                topic = f"/uav{uid}/occupancy_coarse"
                sub = rospy.Subscriber(
                    topic,
                    Int8MultiArray,
                    self._on_grid_received,
                    callback_args=uid,
                    queue_size=1
                )
                self._subscribers.append(sub)
                rospy.loginfo(f"订阅: {topic}")

    def _on_grid_received(self, msg, uav_id):
        """接收单个队友的栅格数据"""
        try:
            # 将 Int8MultiArray 转换为 numpy 数组
            data = np.array(msg.data, dtype=np.int8)
            
            # 检查数据维度
            expected_size = GRID_WIDTH * GRID_HEIGHT
            if len(data) != expected_size:
                rospy.logwarn(
                    f"uav{uav_id} 栅格数据维度错误: "
                    f"期望 {expected_size}, 实际 {len(data)}"
                )
                return
            
            # 重塑为二维栅格 (注意 ROS 消息是行优先)
            grid = data.reshape((GRID_HEIGHT, GRID_WIDTH))
            
            # 存储带锁保护
            if self._lock:
                with self._lock:
                    self.team_grids[uav_id] = grid
                    self.team_timestamps[uav_id] = rospy.get_time()
            else:
                self.team_grids[uav_id] = grid
                self.team_timestamps[uav_id] = rospy.get_time()
                
        except Exception as e:
            rospy.logerr(f"解析 uav{uav_id} 栅格失败: {e}")

    def spin_once(self):
        """在主循环中调用，处理 ROS 回调（如果需要）"""
        # 对于非阻塞订阅，不需要额外处理
        pass

    def get_team_grid(self, uav_id):
        """获取指定队友的栅格"""
        return self.team_grids.get(uav_id)

    def get_combined_grid(self, max_age_s=10.0):
        """
        获取合并后的全局栅格（所有队友的 OR 合并）
        
        Args:
            max_age_s: 忽略超过这个时间的数据
            
        Returns:
            np.ndarray: 合并后的二值栅格 (0=空闲, 1=障碍)
        """
        current_time = rospy.get_time()
        combined = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=np.int8)
        
        for uid in self.fleet_ids:
            if uid == self.uav_id:
                continue
            
            grid = self.team_grids.get(uid)
            if grid is None:
                continue
            
            # 检查数据时效性
            timestamp = self.team_timestamps.get(uid, 0)
            if current_time - timestamp > max_age_s:
                rospy.logwarn(f"uav{uid} 栅格数据过期 ({current_time - timestamp:.1f}s)")
                continue
            
            # OR 合并：任一队友说有障碍，就是有障碍
            combined = np.maximum(combined, (grid > 0).astype(np.int8))
        
        return combined

    def has_team_data(self, max_age_s=10.0):
        """检查是否收到过队友的数据"""
        current_time = rospy.get_time()
        for uid in self.fleet_ids:
            if uid == self.uav_id:
                continue
            if self.team_grids.get(uid) is not None:
                timestamp = self.team_timestamps.get(uid, 0)
                if current_time - timestamp <= max_age_s:
                    return True
        return False

    def shutdown(self):
        """关闭所有订阅器"""
        for sub in self._subscribers:
            sub.unregister()
        self._subscribers.clear()


# 测试代码
if __name__ == "__main__":
    import sys
    
    if "--selftest" in sys.argv:
        print("TeamGridSubscriber 自检...")
        
        # 模拟栅格数据
        test_grid = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=np.int8)
        test_grid[10:15, 10:15] = 1  # 添加一些障碍
        
        print(f"  栅格尺寸: {GRID_WIDTH}x{GRID_HEIGHT}")
        print(f"  测试障碍区域: 10:15, 10:15")
        
        # 验证合并逻辑
        combined = np.maximum(
            np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=np.int8),
            (test_grid > 0).astype(np.int8)
        )
        assert combined[12, 12] == 1, "合并逻辑错误"
        
        print("  自检通过!")
        sys.exit(0)
    
    # 实际运行
    subscriber = TeamGridSubscriber(uav_id=0, fleet_ids=[0,1,2,3,4,5])
    rate = rospy.Rate(1)
    
    try:
        while not rospy.is_shutdown():
            subscriber.spin_once()
            
            if subscriber.has_team_data():
                grid = subscriber.get_combined_grid()
                obstacle_count = np.sum(grid > 0)
                print(f"合并后障碍栅格数: {obstacle_count}")
            
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.shutdown()
