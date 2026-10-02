#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达避障「交接自检」—— 在 VM 上跑，确认雷达传感器可用。

用法（VM 上，栈已起）：
    source ~/robocup_real/env_robocup.sh
    python3 vm_radar_check.py [--uav typhoon_h480_0] [--secs 10]

检查项（对应「队友整合后会不会出问题」的 5 个静默失败模式）：
  1. scan 话题是否存在、频率是否 ~250 Hz
  2. 有效束数（应接近 512；为 0 ⇒ 雷达没装 / 话题名错）
  3. **近距假回波**（<3.5 m 的束数应 ≈ 0；非 0 ⇒ sonar 自检锥体回来了）
  4. 激光参数是否与 XTDrone 自带一致（512 束 / ±180° / 0.5~20 m）
  5. mavros 话题是否在同一命名空间下（ns 写错会静默失效）
  6. 是否有**调试/诊断节点**残留（规则 2.5(11) 裁判监控所有订阅 ⇒ 违规）
"""
import argparse
import math
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uav', default='typhoon_h480_0')
    ap.add_argument('--ns', default=None, help='mavros ns（默认 /<uav>）')
    ap.add_argument('--secs', type=float, default=10.0)
    a = ap.parse_args()
    uav = a.uav
    ns = (a.ns or ('/%s' % uav)).rstrip('/')
    scan_topic = '/%s/scan' % uav

    import rospy
    from sensor_msgs.msg import LaserScan
    from mavros_msgs.msg import State

    rospy.init_node('vm_radar_check', anonymous=True, disable_signals=True)

    frames = []
    meta = {}

    def on_scan(m):
        frames.append(list(m.ranges))
        if not meta:
            meta['n'] = len(m.ranges)
            meta['amin'] = m.angle_min
            meta['amax'] = m.angle_max
            meta['rmin'] = m.range_min
            meta['rmax'] = m.range_max
            meta['frame'] = m.header.frame_id

    rospy.Subscriber(scan_topic, LaserScan, on_scan, queue_size=1)

    print('=' * 66)
    print('雷达避障交接自检   uav=%s   scan=%s' % (uav, scan_topic))
    print('=' * 66)

    # ---- 1) scan 话题 ----
    t0 = time.time()
    while not frames and time.time() - t0 < a.secs:
        rospy.sleep(0.05)
    if not frames:
        print('[1] scan 话题      : ❌ 收不到任何帧')
        print('    ⇒ 三种可能：① launch 的 sdf 不是 typhoon_h480_lidar（雷达没装）')
        print('               ② --scan-topic 写错（模型名不等于 ns 名）')
        print('               ③ 起世界时该模型没 spawn')
        print('结论：RADAR_CHECK_FAIL')
        return 1
    t_first = time.time()
    n0 = len(frames)
    rospy.sleep(a.secs)
    hz = (len(frames) - n0) / max(1e-6, (time.time() - t_first))
    print('[1] scan 话题      : ✅ 存在，频率 %.1f Hz（期望 ~250）' % hz)

    # ---- 4) 激光参数 ----
    ok_param = (meta.get('n') == 512
                and abs(meta.get('amin', 0) + math.pi) < 0.01
                and abs(meta.get('amax', 0) - math.pi) < 0.01
                and abs(meta.get('rmin', 0) - 0.5) < 0.01
                and abs(meta.get('rmax', 0) - 20.0) < 0.01)
    print('[4] 激光参数      : %s  samples=%d  angle=[%.2f,%.2f]  range=[%.2f,%.2f]  frame=%r'
          % ('✅ 与 XTDrone 自带一致' if ok_param else '⚠ 与自带不一致（规则要求必须一致）',
             meta.get('n', -1), meta.get('amin', 0), meta.get('amax', 0),
             meta.get('rmin', 0), meta.get('rmax', 0), meta.get('frame', '')))

    # ---- 2)/3) 有效束与近距假回波 ----
    tot_valid = 0
    near_frames = 0
    near_hist = {}
    minr = 1e9
    per = []
    for r in frames[-40:]:
        fin = [x for x in r if x == x and math.isfinite(x) and x > 0]
        per.append(len(fin))
        tot_valid += len(fin)
        if fin:
            minr = min(minr, min(fin))
        near = [x for x in fin if x < 3.5]
        if near:
            near_frames += 1
            near_hist[len(near)] = near_hist.get(len(near), 0) + 1
    avg_valid = (sum(per) / float(len(per))) if per else 0.0
    print('[2] 有效束数      : 平均 %.1f / 512   （悬停在 (-6,4) 空旷点应接近 0）' % avg_valid)
    print('[3] 近距假回波    : %s  <3.5m 回波出现在 %d/%d 帧，帧内最多 %d 束'
          % ('✅ 无自检假回波' if near_frames == 0 else '❌ 出现自检假回波',
             near_frames, len(frames[-40:]), max(near_hist) if near_hist else 0))
    print('    最近回波距    : %.2f m' % (minr if minr < 1e8 else -1))

    # ---- 5) mavros 命名空间 ----
    st = {'got': False}

    def on_state(m):
        st['got'] = True
        st['connected'] = m.connected

    rospy.Subscriber(ns + '/mavros/state' if not ns.endswith('/mavros') else ns + '/state',
                     State, on_state, queue_size=1)
    t0 = time.time()
    while not st['got'] and time.time() - t0 < 6:
        rospy.sleep(0.1)
    print('[5] mavros ns     : %s  %s'
          % (ns, '✅ 订阅到 %s/mavros/state（connected=%s）'
             % (ns, st.get('connected')) if st['got'] else
             '❌ 订阅不到 ⇒ ns 写错（会静默失效：4 个订阅全落空且不报错）'))

    # ---- 6) 调试节点残留 ----
    #   注意：**排除本检查脚本自身**（它当然在跑），否则每次必报假阳性。
    import subprocess
    self_node = rospy.get_name()
    out = subprocess.run(['rosnode', 'list'], capture_output=True, text=True,
                         timeout=15).stdout
    bad = [l for l in out.splitlines()
           if l != self_node
           and any(k in l for k in ('diag_scan', 'watch_lidar_rel',
                                    'probe_', 'dump_scan'))]
    print('[6] 调试节点残留  : %s' % ('✅ 无' if not bad else '❌ %s' % bad))
    print()
    fails = []
    if not ok_param:
        fails.append('激光参数不一致')
    if near_frames:
        fails.append('近距自检假回波')
    if not st['got']:
        fails.append('mavros ns 订阅不到')
    if bad:
        fails.append('调试节点残留')
    if fails:
        print('结论：RADAR_CHECK_WARN  %s' % ' / '.join(fails))
        return 1
    print('结论：RADAR_CHECK_PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
