#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""swarm_map.py —— 六机在线图互联（方案 B：各机自发布，各机自合并）

协议（为什么这样设计）
====================
每架飞机已经用 radar_avoid --online-map 发布自己扫到的图：

    /typhoon_h480_N/online_map/grid   (OccupancyGrid, latch=True)
    格值语义：100 = 障碍（有回波证据）   -1 = 未知   绝不发 0(FREE)

互联 = 每架机多订阅其余 5 架的同名话题，把对方的 100 格并入自己的
OnlineOccupancy.hits。三条铁律：

  1. **-1（未知）绝不覆盖任何东西。**  "他没看见"不是"那里是空的"。
  2. **只增不减。**  与 occupancy_online 同一单调保守结构 —— 互联在数学上
     不可能让任何一架飞机更冒险（多一架飞机说"有墙"就多绕一点）。
  3. **几何不对齐就整帧丢弃。**  分辨率/尺寸/原点任一不一致 ⇒ 拒绝合并并
     告警，绝不按错位索引写栅格（错位一格 = 凭空造/毁 1m 障碍）。

线程安全：OccupancyGrid 回调跑在 rospy 的线程里，对 occ.hits 的写入是
单字节置 1（GIL 下原子），读方（A* 重规划）最多看到"更新一点点的图"，
不存在撕裂读。合并频率 = 对方发布频率（1Hz 量级），CPU 可忽略。

合规锚点：规则 §2.5(6)「无人机之间通过发布和订阅 ROS 话题进行通信」；
位姿/图均为世界系（真机 = 卫导坐标，规则 §2.5(11) 保证有信号）。

自检（不需要 ROS）::

    python3 swarm_map.py --selftest
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    from astar_plan import X0, Y0, X1, Y1, CELL
except Exception:
    X0, X1 = -52.0, 132.0
    Y0, Y1 = -52.0, 52.0
    CELL = 1.0

# OccupancyGrid 里的"障碍"哨兵值（rosbag/发布端一致）
OCC_VAL = 100


# ---------------------------------------------------------------- 纯函数
def merge_peer_into(hits, data, cell, nx, ny, ox, oy,
                    peer_cell, peer_nx, peer_ny, peer_ox, peer_oy,
                    verbose=False):
    """把一帧 peer 的 OccupancyGrid 数据并入本机 hits 栅格。返回合并格数。

    参数：本机栅格 (cell,nx,ny,ox,oy) 与 peer 帧 (peer_*)。
    规则：几何完全一致才逐格合并；peer 值 == OCC_VAL 且本机该格为空 ⇒ 置 1。
    本机已有的障碍不动（只增不减）；peer 的 -1 / 0 一律忽略。
    """
    if (peer_cell != cell or peer_nx != nx or peer_ny != ny or
            abs(peer_ox - ox) > 1e-6 or abs(peer_oy - oy) > 1e-6):
        if verbose:
            print("[SWARM] 几何不一致，整帧丢弃: cell %s/%s nx %s/%s ny %s/%s "
                  "ox %s/%s oy %s/%s"
                  % (peer_cell, cell, peer_nx, nx, peer_ny, ny,
                     peer_ox, ox, peer_oy, oy))
        return -1
    n = nx * ny
    if len(data) != n:
        return -1
    merged = 0
    for k in range(n):
        if data[k] == OCC_VAL and not hits[k]:
            hits[k] = 1
            merged += 1
    return merged


# ---------------------------------------------------------------- ROS 封装
class SwarmMapMerge(object):
    """订阅若干 peer 的 /<peer>/online_map/grid，实时并入本机 OnlineOccupancy。

    用法（radar_avoid --online-map 分支内）::

        swarm = SwarmMapMerge(occ, args.uav, ["typhoon_h480_1", ...])
        ... 退出前 swarm.stop()
    """

    def __init__(self, occ, uav, peers):
        import rospy
        from nav_msgs.msg import OccupancyGrid as _OG
        self.occ = occ
        self.uav = uav
        self.peers = list(peers)
        # 每个 peer 的统计：帧数 / 累计并入格数 / 几何是否校验通过
        self.stats = {p: {"msgs": 0, "merged": 0, "geom_ok": True,
                          "last_merged": 0}
                      for p in self.peers}
        self._subs = []
        for p in self.peers:
            if p == uav:
                rospy.logwarn('[swarm] --peers 含自己(%s)，跳过', p)
                continue
            topic = "/%s/online_map/grid" % p
            self._subs.append(rospy.Subscriber(
                topic, _OG, self._cb, callback_args=p, queue_size=1))
            rospy.loginfo('[swarm] 订阅队友在线图: %s （100=障碍，-1 永不覆盖）',
                          topic)

    def _cb(self, msg, peer):
        st = self.stats[peer]
        st["msgs"] += 1
        merged = merge_peer_into(
            self.occ.hits, msg.data,
            self.occ.cell, self.occ.nx, self.occ.ny,
            self.occ.x0, self.occ.y0,
            msg.info.resolution, msg.info.width, msg.info.height,
            msg.info.origin.position.x, msg.info.origin.position.y)
        if merged < 0:
            if st["geom_ok"]:
                import rospy
                rospy.logwarn('[swarm] %s 的图几何与本机不一致 ⇒ 整帧丢弃'
                              '（拒绝错位合并）', peer)
                st["geom_ok"] = False
            return
        if merged:
            st["merged"] += merged
            st["last_merged"] = merged
            import rospy
            rospy.loginfo('[swarm] 从 %s 并入 %d 个障碍格（累计 %d）'
                          '—— 他们的墙现在也是我的墙', peer, merged,
                          st["merged"])

    def summary(self):
        """一行汇总（供主循环周期打印）。"""
        parts = ["%s:%d格/%d帧" % (p, self.stats[p]["merged"],
                                   self.stats[p]["msgs"])
                 for p in self.peers if p != self.uav]
        return " ".join(parts) if parts else "(无 peer)"

    def stop(self):
        for s in self._subs:
            try:
                s.unregister()
            except Exception:
                pass


# ---------------------------------------------------------------- 自检
def selftest():
    """不需要 ROS：纯函数级验证合并语义与三铁律。"""
    print("=" * 72)
    print("swarm_map 自检（不需要 ROS）")
    print("=" * 72)
    ok = True
    cell, nx, ny = 1.0, 6, 4
    ox, oy = 0.0, 0.0

    def fresh():
        return bytearray(nx * ny)

    # --- 用例 1：peer 的 100 能并入 ---
    hits = fresh()
    peer = [(-1)] * (nx * ny)
    peer[1 * nx + 2] = 100          # (2,1) 处一堵墙
    m = merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                        cell, nx, ny, ox, oy)
    print("用例1  peer 障碍并入: merged=%d  %s"
          % (m, "✓" if m == 1 and hits[1 * nx + 2] == 1 else "✗"))
    if m != 1:
        ok = False

    # --- 用例 2：-1 绝不覆盖（未知不是空） ---
    hits = fresh()
    hits[1 * nx + 2] = 1            # 本机已有障碍
    peer = [(-1)] * (nx * ny)       # 对方全未知
    m = merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                        cell, nx, ny, ox, oy)
    print("用例2  全未知帧并入后本机障碍保留: merged=%d  %s"
          % (m, "✓" if m == 0 and hits[1 * nx + 2] == 1 else "✗"))
    if m != 0 or hits[1 * nx + 2] != 1:
        ok = False

    # --- 用例 3：只增不减 —— peer 的 0(FREE) 不能擦掉本机障碍 ---
    hits = fresh()
    hits[1 * nx + 2] = 1
    peer = [0] * (nx * ny)          # 对方宣称全空（正常情况不会发生）
    m = merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                        cell, nx, ny, ox, oy)
    print("用例3  peer FREE 不能擦本机障碍: merged=%d 保留=%d  %s"
          % (m, hits[1 * nx + 2],
             "✓" if m == 0 and hits[1 * nx + 2] == 1 else "✗"))
    if hits[1 * nx + 2] != 1:
        ok = False

    # --- 用例 4：几何不对齐 ⇒ 整帧丢弃（返回 -1） ---
    hits = fresh()
    peer = [100] * (nx * ny)
    m = merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                        0.5, nx, ny, ox, oy)          # 分辨率不同
    m2 = merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                         cell, nx + 1, ny, ox, oy)    # 尺寸不同
    m3 = merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                         cell, nx, ny, 1.0, oy)       # 原点不同
    good = (m == -1 and m2 == -1 and m3 == -1 and sum(hits) == 0)
    print("用例4  三种几何不一致全部拒绝: %s" % ("✓" if good else "✗"))
    if not good:
        ok = False

    # --- 用例 5：单调性 —— 反复合并，障碍数只增不减 ---
    hits = fresh()
    peer = [(-1)] * (nx * ny)
    peer[2] = 100
    prev = sum(hits)
    mono = True
    for _ in range(50):
        merge_peer_into(hits, peer, cell, nx, ny, ox, oy,
                        cell, nx, ny, ox, oy)
        cur = sum(hits)
        if cur < prev:
            mono = False
            break
        prev = cur
    print("用例5  50 次反复合并单调不减: n=%d  %s"
          % (prev, "✓" if mono else "✗"))
    if not mono:
        ok = False

    # --- 用例 6：与真实 OnlineOccupancy 联动（同目录时） ---
    print()
    try:
        from occupancy_online import OnlineOccupancy
        occ = OnlineOccupancy()
        peer = [(-1)] * (occ.nx * occ.ny)
        # 世界系 (5,0) 与 (5,1) 两格障碍（对齐 astar_plan 栅格）
        for (wx, wy) in [(5.0, 0.0), (5.0, 1.0)]:
            i, j = occ.to_idx(wx, wy)
            peer[j * occ.nx + i] = 100
        m = merge_peer_into(occ.hits, peer, occ.cell, occ.nx, occ.ny,
                            occ.x0, occ.y0, occ.cell, occ.nx, occ.ny,
                            occ.x0, occ.y0)
        print("用例6  与 OnlineOccupancy 联动: merged=%d hits=%d  %s"
              % (m, occ.n_hits(), "✓" if m == 2 and occ.n_hits() == 2
                 else "✗"))
        if m != 2:
            ok = False
    except Exception as e:
        print("用例6  跳过（occupancy_online 不可用: %s）" % e)

    print()
    print("=" * 72)
    print("自检结果: %s" % ("全部通过 PASS" if ok else "存在失败 FAIL"))
    print("=" * 72)
    return 0 if ok else 1


def main():
    import argparse
    ap = argparse.ArgumentParser(description="六机在线图互联（方案 B）")
    ap.add_argument("--selftest", action="store_true",
                    help="离线自检（不需要 ROS）")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    print(__doc__)
    print("本模块作为库使用：from swarm_map import SwarmMapMerge")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
