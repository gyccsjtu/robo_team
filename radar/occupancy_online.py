#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""occupancy_online.py —— 在线占据栅格（A 方案 / 自主建图），纯保守增量

设计原则（这就是「零风险」的形式化）
====================================
本模块只往地图里加障碍，永远不删障碍（单调保守）。上层这样用：

    final_obstacles = B（black_box.txt 栅格） ∪ A（本模块）

任一侧说"有障碍" ⇒ 最终就有障碍。由此推出三条硬性质：

  · A 漏检     ⇒ B 兜底
  · B 漏检     ⇒ A 兜底
  · A 不可能把"本来要绕的"变成"可以直穿的"

新增功能（2026-10）：
  · ROS 发布：1m 分辨率占据栅格 + 7m 降采样粗网格
  · 与队友协同层互通（GRID_SIZE_M=7）

ROS 接口：
  /uavX/occupancy_fine      - 1m 分辨率 (Int8MultiArray)
  /uavX/occupancy_coarse    - 7m 降采样 (Int8MultiArray)

自检
====
  python3 occupancy_online.py --selftest
不需要 ROS，本机即可跑。

启动ROS节点:
  python3 occupancy_online.py --ros --uav 1
"""

import math
import os
import sys

# ROS 导入（可选）
try:
    import rospy
    from std_msgs.msg import Int8MultiArray
    from sensor_msgs.msg import LaserScan
    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False


_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# 生产路径：复用 radar_avoid 里经过实测的坐标变换（有 __main__ 守卫，可安全导入）。
def _load_ra_scan():
    import sys as _s
    _m = _s.modules.get('__main__')
    _f = getattr(_m, 'scan_to_obstacles', None) if _m is not None else None
    if callable(_f):
        return _f
    try:
        from radar_avoid import scan_to_obstacles as _g
        return _g
    except Exception:
        return None


_ra_scan_to_obstacles = _load_ra_scan()
_HAVE_RA = _ra_scan_to_obstacles is not None

# 栅格参数与 astar_plan 对齐（对齐后索引可直接互相比较）
try:
    from astar_plan import X0, Y0, X1, Y1, CELL
except Exception:
    X0, X1 = -52.0, 132.0
    Y0, Y1 = -52.0, 52.0
    CELL = 1.0

RANGE_MIN = 0.5
RANGE_MAX = 20.0


def _fallback_scan_to_obstacles(msg, yaw, pos_enu, mount, stride=1):
    """radar_avoid 不可用时的等价兜底"""
    cy, sy = math.cos(yaw), math.sin(yaw)
    mx, my = mount[0], mount[1]
    ux, uy = pos_enu[0], pos_enu[1]
    out = []
    ang = msg.angle_min
    inc = msg.angle_increment
    n = len(msg.ranges)
    for i in range(0, n, stride):
        r = msg.ranges[i]
        a = ang + i * inc
        if r != r or r == float('inf') or r == float('-inf'):
            continue
        if RANGE_MIN < r < RANGE_MAX:
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            bx = lx + mx
            by = ly + my
            ex = ux + bx * cy - by * sy
            ey = uy + bx * sy + by * cy
            out.append((ex, ey, r))
    return out


class OnlineOccupancy(object):
    """在线占据栅格：吃 LaserScan + 位姿，维护增量障碍图。"""

    def __init__(self, cell=CELL, x0=X0, y0=Y0, x1=X1, y1=Y1,
                 seen_stride=2, seen_step=1.0):
        self.cell = cell
        self.x0, self.y0 = x0, y0
        self.nx = int((x1 - x0) / cell) + 1
        self.ny = int((y1 - y0) / cell) + 1
        self.hits = bytearray(self.nx * self.ny)
        self.seen = bytearray(self.nx * self.ny)
        self.frames = 0
        self.n_returns_total = 0
        self.seen_stride = max(1, int(seen_stride))
        self.seen_step = max(cell * 0.5, float(seen_step))
        self.track_seen = True

    def to_idx(self, x, y):
        i = int(round((x - self.x0) / self.cell))
        j = int(round((y - self.y0) / self.cell))
        return max(0, min(self.nx - 1, i)), max(0, min(self.ny - 1, j))

    def to_xy(self, i, j):
        return self.x0 + i * self.cell, self.y0 + j * self.cell

    def inside(self, x, y):
        return (self.x0 <= x <= self.x0 + (self.nx - 1) * self.cell and
                self.y0 <= y <= self.y0 + (self.ny - 1) * self.cell)

    def feed(self, scan_msg, yaw, pos_enu, mount, attitude=None, stride=1,
             mark_seen=None):
        """吃一帧雷达，更新 hits / seen"""
        if scan_msg is None:
            return 0
        stride = max(1, int(stride))
        if _HAVE_RA:
            obs = _ra_scan_to_obstacles(scan_msg, yaw, pos_enu, mount,
                                        stride=stride, attitude=attitude)
        else:
            obs = _fallback_scan_to_obstacles(scan_msg, yaw, pos_enu, mount,
                                              stride=stride)

        for (ex, ey, _r) in obs:
            if not self.inside(ex, ey):
                continue
            i, j = self.to_idx(ex, ey)
            self.hits[j * self.nx + i] = 1

        do_seen = self.track_seen if mark_seen is None else bool(mark_seen)
        if do_seen:
            self._mark_seen(scan_msg, yaw, pos_enu, mount)

        self.frames += 1
        self.n_returns_total += len(obs)
        return len(obs)

    def _mark_seen(self, msg, yaw, pos_enu, mount):
        """把每束射线扫过的格标为 seen"""
        cy, sy = math.cos(yaw), math.sin(yaw)
        mx, my = mount[0], mount[1]
        ux, uy = pos_enu[0], pos_enu[1]
        ang = msg.angle_min
        inc = msg.angle_increment
        n = len(msg.ranges)
        step = self.seen_step
        for i in range(0, n, self.seen_stride):
            r = msg.ranges[i]
            a = ang + i * inc
            if r != r or r == float('inf') or r == float('-inf') or r <= RANGE_MIN:
                r_use = RANGE_MAX
                stop = RANGE_MAX - step
            else:
                r_use = min(r, RANGE_MAX)
                stop = max(0.0, r_use - step)
            lx = r_use * math.cos(a)
            ly = r_use * math.sin(a)
            bx = lx + mx
            by = ly + my
            ex = ux + bx * cy - by * sy
            ey = uy + bx * sy + by * cy
            d = stop
            if d <= 0.0:
                continue
            k = int(d / step)
            for t in range(1, k + 1):
                s = t * step
                px = ux + (ex - ux) * (s / r_use if r_use > 0 else 0.0)
                py = uy + (ey - uy) * (s / r_use if r_use > 0 else 0.0)
                if not self.inside(px, py):
                    break
                ii, jj = self.to_idx(px, py)
                self.seen[jj * self.nx + ii] = 1

    def extra_cells(self, blocked, cell=None, x0=None, y0=None, nx=None, ny=None):
        """A 有回波、但 B 的 blocked 栅格里是自由"""
        cell = self.cell if cell is None else cell
        x0 = self.x0 if x0 is None else x0
        y0 = self.y0 if y0 is None else y0
        nxb = self.nx if nx is None else nx
        nyb = self.ny if ny is None else ny
        out = []
        for j in range(self.ny):
            base = j * self.nx
            y = self.y0 + j * self.cell
            for i in range(self.nx):
                if not self.hits[base + i]:
                    continue
                x = self.x0 + i * self.cell
                ib = int(round((x - x0) / cell))
                jb = int(round((y - y0) / cell))
                if 0 <= ib < nxb and 0 <= jb < nyb:
                    if blocked[jb * nxb + ib] == 0:
                        out.append((x, y))
                else:
                    out.append((x, y))
        return out

    def coverage(self):
        """seen 占整图的比例"""
        n = float(self.nx * self.ny)
        return sum(self.seen) / n if n > 0 else 0.0

    def n_hits(self):
        return sum(self.hits)

    def report(self, blocked=None, warn_extra=20):
        """打印状态 + 审计结论"""
        extra = []
        if blocked is not None:
            extra = self.extra_cells(blocked)
        print("[ONLINE-OCC] 帧=%d 回波累计=%d 命中格=%d 扫描格=%d (覆盖率 %.1f%%)"
              % (self.frames, self.n_returns_total, self.n_hits(),
                 sum(self.seen), self.coverage() * 100.0))
        if blocked is not None:
            if not extra:
                print("[ONLINE-OCC] 审计: B 覆盖外命中格 = 0  ✓")
            elif len(extra) <= warn_extra:
                print("[ONLINE-OCC] 审计: B 覆盖外命中格 = %d"
                      % len(extra))
            else:
                print("[ONLINE-OCC] ⚠ 审计: B 覆盖外命中格 = %d > %d"
                      % (len(extra), warn_extra))
        return extra

    def snapshot(self):
        return {"frames": self.frames,
                "n_returns": self.n_returns_total,
                "n_hits": self.n_hits(),
                "seen": sum(self.seen),
                "coverage": self.coverage(),
                "raised": _HAVE_RA}


# ------------------------------------------------------------------ 降采样器 (1m -> 7m)
class OccupancyDownsampler(object):
    """1m -> 7m 降采样器（保守策略）"""

    def __init__(self, x0=X0, y0=Y0, fine_cell=CELL):
        self.x0 = x0
        self.y0 = y0
        self.fine_cell = fine_cell

    def downsample(self, fine_hits, fine_nx, fine_ny, coarse_cell=7.0):
        """降采样

        Args:
            fine_hits: bytearray，1m栅格障碍标记
            fine_nx, fine_ny: 细栅格尺寸
            coarse_cell: 目标分辨率，默认 7.0m

        Returns:
            coarse_hits: bytearray
        """
        x1 = self.x0 + (fine_nx - 1) * self.fine_cell
        y1 = self.y0 + (fine_ny - 1) * self.fine_cell

        nx = int(math.ceil((x1 - self.x0) / coarse_cell)) + 1
        ny = int(math.ceil((y1 - self.y0) / coarse_cell)) + 1

        coarse_hits = bytearray(nx * ny)
        samples_per_side = int(coarse_cell / self.fine_cell)  # 7

        for ix in range(nx):
            for iy in range(ny):
                fx0 = ix * samples_per_side
                fy0 = iy * samples_per_side
                fx1 = min(fx0 + samples_per_side, fine_nx)
                fy1 = min(fy0 + samples_per_side, fine_ny)

                blocked = 0
                for fxi in range(fx0, fx1):
                    for fyi in range(fy0, fy1):
                        if fxi < fine_nx and fyi < fine_ny:
                            idx = fyi * fine_nx + fxi
                            if idx < len(fine_hits) and fine_hits[idx]:
                                blocked = 1
                                break
                    if blocked:
                        break

                coarse_hits[iy * nx + ix] = blocked

        return coarse_hits, nx, ny


# ------------------------------------------------------------------ ROS 集成
class OccupancyPublisher(object):
    """ROS 发布器：发布 1m 细栅格 + 7m 粗栅格"""

    def __init__(self, uav_id=1, namespace=""):
        if not _HAVE_ROS:
            raise RuntimeError("ROS not available")

        self.uav_id = uav_id
        self.ns = namespace if namespace else f"/uav{uav_id}"

        # 1m 细栅格话题
        self.pub_fine = rospy.Publisher(
            f"{self.ns}/occupancy_fine",
            Int8MultiArray, queue_size=1)

        # 7m 粗栅格话题
        self.pub_coarse = rospy.Publisher(
            f"{self.ns}/occupancy_coarse",
            Int8MultiArray, queue_size=1)

        self.downsampler = OccupancyDownsampler()

    def publish(self, occ):
        """发布占据栅格"""
        # 发布 1m 细栅格
        msg_fine = Int8MultiArray()
        msg_fine.data = list(occ.hits)
        self.pub_fine.publish(msg_fine)

        # 降采样到 7m 并发布
        coarse_hits, nx, ny = self.downsampler.downsample(
            occ.hits, occ.nx, occ.ny, coarse_cell=7.0)

        msg_coarse = Int8MultiArray()
        msg_coarse.data = list(coarse_hits)
        self.pub_coarse.publish(msg_coarse)


# ------------------------------------------------------------------ 自检
class _FakeScan(object):
    def __init__(self, ranges, angle_min, angle_increment):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in (-b - sq, -b + sq):
        if t > 0.0:
            return t
    return None


def _synth_scan(n_beams, pole_xy, pole_r, rmax=RANGE_MAX):
    inc = 2 * math.pi / (n_beams - 1)
    a0 = -math.pi
    rs = []
    for k in range(n_beams):
        a = a0 + k * inc
        t = _ray_circle(0.0, 0.0, math.cos(a), math.sin(a),
                        pole_xy[0], pole_xy[1], pole_r)
        rs.append(t if (t is not None and t < rmax) else float('inf'))
    return _FakeScan(rs, a0, inc)


def selftest():
    print("=" * 72)
    print("occupancy_online 自检")
    print("=" * 72)
    print("栅格: cell=%.1f  x[%.0f,%.0f]  y[%.0f,%.0f]"
          % (CELL, X0, X1, Y0, Y1))
    ok = True

    # 用例1
    print("\n用例1: 飞机在 (0,0) 朝向 +x，灯杆中心 (5,0)")
    occ = OnlineOccupancy()
    scan = _synth_scan(512, (5.0, 0.0), 0.0875)
    n_ret = occ.feed(scan, yaw=0.0, pos_enu=(0.0, 0.0), mount=(0.0, 0.0))
    print("  有效回波束数 = %d" % n_ret)
    i, j = occ.to_idx(5.0, 0.0)
    if occ.hits[j * occ.nx + i]:
        print("  ✓ 灯杆所在格已被标记为障碍")
    else:
        print("  ✗ 灯杆所在格未被标记")
        ok = False

    # 用例2: 降采样
    print("\n用例2: 降采样 1m -> 7m")
    ds = OccupancyDownsampler()
    # 全空
    fine = bytearray(185 * 105)
    coarse, nx, ny = ds.downsample(fine, 185, 105)
    print(f"  全空输入 -> 粗栅格障碍数: {sum(coarse)}")
    assert sum(coarse) == 0, "全空应该输出全空"
    # 中心一个障碍
    fine[52 * 185 + 40] = 1
    coarse, nx, ny = ds.downsample(fine, 185, 105)
    print(f"  单障碍输入 -> 粗栅格障碍数: {sum(coarse)}")
    assert sum(coarse) > 0, "单障碍应该被检测到"
    print("  ✓ 降采样功能正常")

    print("\n" + "=" * 72)
    print("自检结果: %s" % ("通过" if ok else "失败"))
    print("=" * 72)
    return 0 if ok else 1


def main():
    import argparse
    ap = argparse.ArgumentParser(description="在线占据栅格")
    ap.add_argument("--selftest", action="store_true", help="离线自检")
    ap.add_argument("--ros", action="store_true", help="启动 ROS 节点")
    ap.add_argument("--uav", type=int, default=1, help="UAV ID")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.ros:
        if not _HAVE_ROS:
            print("错误：ROS 不可用")
            return 1
        return ros_main(args)

    print(__doc__)
    return 0


def ros_main(args):
    """ROS 节点主函数"""
    rospy.init_node(f"occupancy_online_uav{args.uav}", anonymous=True)

    uav_id = args.uav
    ns = f"/uav{uav_id}"

    occ = OnlineOccupancy()
    publisher = OccupancyPublisher(uav_id=uav_id)

    scan_topic = f"{ns}/scan"

    latest_pose = None
    latest_yaw = None
    latest_mount = (0.0, 0.0)

    def pose_callback(msg):
        nonlocal latest_pose, latest_yaw
        latest_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        import math
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        latest_yaw = yaw

    def scan_callback(scan_msg):
        nonlocal occ
        if latest_pose is None or latest_yaw is None:
            return
        occ.feed(scan_msg, latest_yaw, latest_pose, latest_mount)
        publisher.publish(occ)
        occ.report()

    rospy.Subscriber(f"{ns}/local_pose", pose_callback, pose_callback)
    rospy.Subscriber(scan_topic, LaserScan, scan_callback)

    rospy.loginfo(f"OccupancyOnline 启动: UAV{uav_id}")
    rospy.loginfo(f"发布: {ns}/occupancy_fine (1m), {ns}/occupancy_coarse (7m)")

    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
