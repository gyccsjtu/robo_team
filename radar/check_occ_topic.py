#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线占据图话题体检 —— 验证协同层能收到我方发的 /<uav>/online_map/grid。

为什么要单独写一个：`rostopic hz` / `rostopic echo` 在 `use_sim_time=true`
下**抓不到数据**（本项目踩过这个坑：报了 "no new messages" 但其实话题在发），
所以必须用 Python 订阅来验，否则会误判成"发布坏了"。

用法：
    python3 check_occ_topic.py [话题名] [监听秒数]
    python3 check_occ_topic.py /typhoon_h480_0/online_map/grid 30

退出码：0 = 话题正常且编码合规；1 = 收不到 或 编码违规。
"""
import sys
import time

import rospy
from nav_msgs.msg import OccupancyGrid

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/typhoon_h480_0/online_map/grid'
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

_cnt = [0]
_last = [None]


def cb(m):
    _cnt[0] += 1
    _last[0] = m


def main():
    rospy.init_node('occ_topic_check', anonymous=True)
    rospy.Subscriber(TOPIC, OccupancyGrid, cb, queue_size=1)
    print('[chk] 订阅 %s，监听 %.0f s' % (TOPIC, DUR), flush=True)

    t0 = time.time()
    t_hb = t0
    while not rospy.is_shutdown() and time.time() - t0 < DUR:
        time.sleep(0.2)
        if time.time() - t_hb >= 5.0:          # 心跳：证明"活着"且"在收"
            t_hb = time.time()
            print('[chk] 心跳：累计收到 %d 条' % _cnt[0], flush=True)

    print('[chk] ---- 汇总 ----', flush=True)
    print('[chk] 收到消息 %d 条 / %.0f s' % (_cnt[0], DUR), flush=True)
    m = _last[0]
    if m is None:
        print('[chk] ✗ 一条都没收到 —— 查三件事：节点在跑吗 / 话题名对吗 / '
              '<group ns> 是否等于模型名', flush=True)
        return 1

    vals = {}
    for v in m.data:
        vals[v] = vals.get(v, 0) + 1
    occ = vals.get(100, 0)
    unk = vals.get(-1, 0)
    other = dict((k, v) for k, v in vals.items() if k not in (100, -1))
    print('[chk] 尺寸 %d x %d  分辨率 %.2f m  原点 (%.1f, %.1f)'
          % (m.info.width, m.info.height, m.info.resolution,
             m.info.origin.position.x, m.info.origin.position.y), flush=True)
    print('[chk] 格值分布：障碍(100)=%d  未知(-1)=%d  其它=%s'
          % (occ, unk, other if other else '无'), flush=True)
    if other:
        print('[chk] ✗ 出现 0/FREE 或其它取值 —— 违反「只增不减、绝不发 FREE」',
              flush=True)
        return 1
    print('[chk] ✓ 只含 100(障碍) 与 -1(未知)，符合单调保守约定', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
