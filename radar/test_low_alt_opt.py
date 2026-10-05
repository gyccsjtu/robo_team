#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低空保守化 + 高度看门狗 离线单测（2026-10-04 十五次修正）。

不需要 ROS。覆盖：
  A. clearance_bonus 纯函数语义（零回归边界 + 线性插值 + 单调）
  B. subgoal_from_scan(scan_z=...) 端到端接线：
     - 兼容：scan_z=None / >=SCAN_Z_REF 与历史行为逐字节一致
     - 生效：同一帧在扫描面 2.45m（队友撞房高度）比 5.5m 更保守
  C. lateral_guard_shift 的 clearance 覆盖参数
  D. 启动期高度守卫**已移除**：--alt 2.0/2.45/2.8 一律不被拦截
     （比赛不可重开 ⇒ 拦启动是死代码；低空后果只在运行期承接）
  E. 比赛防弃赛静态断言：无 ap.error / 无"固定次数放弃" / 心跳可自助重启
  F. 起飞等待链的时钟语义（十六次修正）：重试/沉降走**墙钟**、物理采样
     窗口仍走**仿真钟**；悬停以"自离地起累计"为基准（不白等一倍）
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from test_radar_offline import install_stubs   # noqa: E402


class Scan(object):
    """自定义每束距离的扫描帧（FakeScan 只能等距）。"""

    def __init__(self, ranges, ang0=-math.pi, ang1=math.pi):
        self.angle_min = ang0
        self.angle_max = ang1
        self.angle_increment = (ang1 - ang0) / len(ranges)
        self.ranges = ranges


def make_wall_scan(half_deg=15.0, dist=2.5, n=360):
    """正前方 ±half_deg 一堵 2.5m 墙，其余全 inf。yaw=0 时墙在航向正前。"""
    inc = 360.0 / n                      # 每束 1°
    ranges = [float('inf')] * n
    for deg in range(-int(half_deg), int(half_deg) + 1):
        idx = int(round((deg + 180.0) / inc))
        if 0 <= idx < n:
            ranges[idx] = dist
    return Scan(ranges)


def main():
    install_stubs()
    import radar_avoid as R

    fails = []
    checks = 0

    def ok(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            fails.append(msg)
        print(('  PASS  ' if cond else '  FAIL  ') + msg)

    print('=== A. clearance_bonus 纯函数 ===')
    ok(R.clearance_bonus(None) == 0.0, 'None ⇒ 0（兼容缺省）')
    ok(R.clearance_bonus(5.5) == 0.0, '5.5m（历史验证层）⇒ 0')
    ok(R.clearance_bonus(R.SCAN_Z_REF) == 0.0, '边界 z=REF ⇒ 0（连续）')
    ok(abs(R.clearance_bonus(R.SCAN_Z_FLOOR) - R.ALT_BONUS_MAX) < 1e-12,
       '边界 z=FLOOR ⇒ 满额 %.2f' % R.ALT_BONUS_MAX)
    b245 = R.clearance_bonus(2.45)        # 队友撞房高度
    ok(abs(b245 - 0.4 * (4.0 - 2.45) / 2.0) < 1e-12,
       '2.45m ⇒ %.3f（线性插值）' % b245)
    ok(0.0 < b245 < R.ALT_BONUS_MAX, '2.45m 补偿应介于 0 与满额之间')
    zs = [2.0, 2.45, 2.53, 3.0, 3.9, 4.0, 4.5, 5.58]
    bs = [R.clearance_bonus(z) for z in zs]
    ok(all(bs[i] >= bs[i + 1] - 1e-12 for i in range(len(bs) - 1)),
       '补偿随扫描面升高单调不增：%s' % ['%.2f' % b for b in bs])
    ok(bs[-1] == 0.0 and bs[0] == R.ALT_BONUS_MAX,
       '两端饱和：低端满额、高端为 0')

    print('=== B. subgoal_from_scan(scan_z) 端到端 ===')
    msg = make_wall_scan()
    cur = (0.0, 0.0)
    goal = (10.0, 0.0)
    mount = R.MOUNT_DEFAULT
    r_none = R.subgoal_from_scan(msg, 0.0, cur, goal, mount)
    r_ref = R.subgoal_from_scan(msg, 0.0, cur, goal, mount,
                                scan_z=R.SCAN_Z_REF)
    r_hi = R.subgoal_from_scan(msg, 0.0, cur, goal, mount, scan_z=5.5)
    ok(r_none == r_ref == r_hi,
       '兼容：scan_z=None / REF / 5.5 三者输出逐字节一致 %s'
       % ((r_none[0], r_none[1]),))
    r_lo = R.subgoal_from_scan(msg, 0.0, cur, goal, mount, scan_z=2.45)
    ok(r_lo != r_hi, '2.45m 结果应与 5.5m 不同（补偿已接线）')
    ok(abs(r_lo[1]) > abs(r_hi[1]) + 1e-9,
       '2.45m 子目标应离墙更远：|sub_y| %.3f > %.3f'
       % (abs(r_lo[1]), abs(r_hi[1])))
    ok(abs(r_lo[1] - r_hi[1]) <= R.SAFE_GAP * 2.0 + 1e-9,
       '单帧修正幅度受 amt_cap(=SAFE_GAP*2=%.2f) 约束：Δ=%.3f'
       % (R.SAFE_GAP * 2.0, abs(r_lo[1] - r_hi[1])))
    ok(r_lo[3] == r_hi[3],
       'nearest 通道深度不受补偿影响（%.2f）' % (r_hi[3],))

    print('=== C. lateral_guard_shift clearance 覆盖 ===')
    lat_x, lat_y = 0.0, 1.0               # 行进 +x 的左法向
    base = R.lateral_guard_shift(3.0, 0.0, lat_x, lat_y,
                                 0.9, None, 1.0, 0.7)
    boosted = R.lateral_guard_shift(3.0, 0.0, lat_x, lat_y,
                                    0.9, None, 1.0, 0.7,
                                    clearance=R.CLEARANCE + b245)
    ok(base[0] == 3.0 and boosted[0] == 3.0, '单侧守门不动纵向分量')
    ok(boosted[1] < base[1],
       '低空 clearance 更大 ⇒ 侧向推开更多：%.3f < %.3f'
       % (boosted[1], base[1]))
    ok(abs(base[1] - (0.0 - min(0.7, (R.CLEARANCE + R.SAFE_GAP - 0.9)))) < 1e-9,
       '默认 clearance 语义与历史一致：sub_y=%.3f' % base[1])

    print('=== D. 启动期高度守卫已移除（比赛不可重开 ⇒ 不拦截）===')
    # 原设计 --alt<2.4 拒绝启动、2.4~2.6 告警 —— 已删。此处用假 RadarPilot
    # 打桩，让 main 一路走到 wait_ready 再返回，证明参数**没有被拦**：
    # 若守卫还在，ap.error 会在构造之前抛 SystemExit(2)。
    argv = sys.argv
    real_pilot = R.RadarPilot

    class _FakePilot(object):
        def __init__(self, uav, **kw):
            self.ns = kw.get('ns')
            self.scan_topic = kw.get('scan_topic')

        def wait_ready(self, timeout):       # 走到这里 = 参数已被接受
            return False                     # ⇒ main 走正常 return 1

    R.RadarPilot = _FakePilot
    try:
        for alt in ('2.0', '2.45', '2.8'):
            sys.argv = ['radar_avoid.py', '--alt', alt, '--dry-run']
            try:
                rc = R.main()
                ok(rc == 1,
                   '--alt %s 未被拦截（rc=%s，已走到 wait_ready）' % (alt, rc))
            except SystemExit as e:
                ok(False, '--alt %s 被启动守卫拦下了（SystemExit %s）—— '
                          '比赛不可重开，不应拦' % (alt, e.code))
    finally:
        R.RadarPilot = real_pilot
        sys.argv = argv

    print('=== E. 比赛防弃赛：无启动期拒绝 / 无静默失效 ===')
    with open(os.path.join(HERE, 'radar_avoid.py'), encoding='utf-8') as f:
        src = f.read()
    ok('ap.error' not in src,
       '源码无 ap.error（不存在"参数层拒绝启动"）')
    ok('TAKEOFF_RETRIES' not in src,
       '起飞重试已由"固定 3 次就放弃"改为**有界预算**重试')
    ok(src.count('def restart_heartbeat') == 1
       and src.count('self.restart_heartbeat()') >= 3,
       '心跳线程具备自助重启（定义 1 处 + 调用 %d 处）'
       % src.count('self.restart_heartbeat()'))
    ok('def ready_missing' in src,
       '就绪缺口可诊断（ready_missing）—— 超时日志能点名缺哪一项')
    ok(src.count('拒绝起飞') <= 6,
       '无"拒绝起飞"代码闸门（仅注释保留历史说明，实测 %d 处）'
       % src.count('拒绝起飞'))

    print('=== F. 起飞等待链的时钟语义（十六次修正） ===')
    import time as _wall
    _p = R.RadarPilot.__new__(R.RadarPilot)    # 跳过 __init__（要 MAVROS）
    _t0 = _wall.time()
    _p._wall_sleep(0.30)
    _d = _wall.time() - _t0
    ok(0.20 <= _d <= 0.60,
       '_wall_sleep(0.30) 实际墙钟 %.2fs（不受 /use_sim_time 影响）' % _d)
    _t0 = _wall.time()
    _p._wall_sleep(0.0)
    _dd = _wall.time() - _t0
    ok(_dd < 0.10, '_wall_sleep(0.0) 立即返回（%.3fs）' % _dd)
    with open(os.path.join(HERE, 'radar_avoid.py'), encoding='utf-8') as f:
        _src = f.read()
    ok('_ref = t_lift if t_lift is not None else t_hold' in _src,
       '悬停基准 = 自**离地**起累计（t_lift），不再"到达高度后另数一遍"')
    ok(_src.count('LIFTOFF_DZ') >= 1,
       '离地时刻以真值 z 高于地面 0.3m 代理（对齐 LandDetector 打点语义）')
    ok(_src.count('self._wall_sleep(') >= 8,
       '重试间隔/参数沉降已改墙钟（%d 处）' % _src.count('self._wall_sleep('))
    ok('default=32.0' in _src and "'--takeoff-hold'" in _src,
       '--takeoff-hold 默认 32 真实秒（等效旧"到达高度后 30s"）')
    ok(_src.count('rospy.sleep(0.25)') == 1
       and _src.count('rospy.sleep(1.0 / CTRL_HZ)') >= 2,
       'EKF/wait_stable 的物理采样窗口**仍用仿真钟**（不得改墙钟）')

    print()
    print(' Ran %d checks, %d failed' % (checks, len(fails)))
    if fails:
        print(' FAILURES:')
        for f in fails:
            print('  - ' + f)
        return 1
    print(' ALL OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
