#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TargetFollower（动态目标跟随状态机）离线单测（不需要 ROS）。

复用 test_radar_offline.install_stubs 给 radar_avoid 打桩后导入，
只测状态机纯逻辑：SEARCH / APPROACH / TRACK / LOST 转移、
saved_tail 恢复、移动阈值、末尾回目标点。
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from test_radar_offline import install_stubs   # noqa: E402


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

    print('=== A. 初始与默认参数 ===')
    f = R.TargetFollower()
    ok(f.state == R.TF_ST_SEARCH, '初始 state=SEARCH，实测 %s' % f.state)
    ok(f.update((0.0, 0.0), 100.0) is None,
       '无目标流时 update 应返回 None（不干预航线）')
    ok(f.enter == 12.0, '--target-enter 默认 12.0，实测 %.1f' % f.enter)
    ok(f.resume == 10.0, '--target-resume 默认 10.0，实测 %.1f' % f.resume)
    fg = R.TargetFollower(goal=(30.0, -20.0))
    ok(fg.goal == (30.0, -20.0), 'goal 应原样存储，实测 %s' % (fg.goal,))

    print('=== B. on_target 上报 ===')
    ok(f.on_target(50.0, 0.0, 100.0) is True,
       '首次上报恒为 moved=True（主循环必须立刻注入航点）')
    ok(f.target == (50.0, 0.0, 100.0),
       'target 应记录 (x, y, t)，实测 %s' % (f.target,))
    ok(f.on_target(50.5, 0.0, 101.0) is False,
       '移动 0.5m(<1m) 不应触发重写')
    ok(f.on_target(51.6, 0.0, 102.0) is True,
       '移动 1.6m(>1m) 应触发重写')
    ok(abs(f.dist((48.6, 0.0)) - 3.0) < 1e-9,
       'dist 应算到最新目标 (51.6,0)，实测 %.3f' % f.dist((48.6, 0.0)))

    print('=== C. SEARCH -> APPROACH（远目标） ===')
    f2 = R.TargetFollower(enter=12.0, resume=10.0, goal=(30.0, -20.0))
    f2.on_target(50.0, 0.0, 100.0)
    a = f2.update((0.0, 0.0), 100.0)
    ok(a is not None and a[0] == 'follow',
       "远目标应返回 ('follow', x, y)，实测 %s" % (a,))
    ok(a[1] == 50.0 and a[2] == 0.0, 'follow 坐标应是目标点，实测 %s' % (a,))
    ok(f2.state == R.TF_ST_APPROACH,
       '距离 50m > enter 应为 APPROACH，实测 %s' % f2.state)
    a2 = f2.update((1.0, 0.0), 100.5)
    ok(a2 is not None and a2[0] == 'follow',
       '目标流新鲜期间应持续 follow（每帧都追）')

    print('=== D. TRACK 进出 ===')
    f3 = R.TargetFollower(enter=12.0, resume=10.0)
    f3.on_target(5.0, 0.0, 100.0)
    f3.update((0.0, 0.0), 100.0)
    ok(f3.state == R.TF_ST_TRACK,
       '距离 5m ≤ enter 应直接 TRACK，实测 %s' % f3.state)
    # 目标走远（actor 移动）
    f3.on_target(30.0, 0.0, 101.0)
    f3.update((0.0, 0.0), 101.0)
    ok(f3.state == R.TF_ST_APPROACH,
       'TRACK 中目标超出 enter 应退回 APPROACH，实测 %s' % f3.state)
    # 再逼近回 10m 内
    f3.on_target(9.0, 0.0, 102.0)
    f3.update((0.0, 0.0), 102.0)
    ok(f3.state == R.TF_ST_TRACK,
       'APPROACH 中距离回到 enter 内应转 TRACK，实测 %s' % f3.state)
    f4 = R.TargetFollower(enter=12.0)
    f4.on_target(12.0, 0.0, 100.0)
    f4.update((0.0, 0.0), 100.0)
    ok(f4.state == R.TF_ST_TRACK,
       '边界：距离恰好 == enter 应算 TRACK（<=），实测 %s' % f4.state)

    print('=== E. 目标流中断 -> LOST（原地等） ===')
    f5 = R.TargetFollower(enter=12.0, resume=10.0, fresh=2.0,
                          goal=(30.0, -20.0))
    f5.on_target(10.0, 5.0, 100.0)
    f5.update((0.0, 0.0), 100.0)
    ok(f5.state == R.TF_ST_TRACK, '前置：TRACK，实测 %s' % f5.state)
    a = f5.update((0.0, 0.0), 103.0)      # age=3 > fresh=2，<= resume
    ok(f5.state == R.TF_ST_LOST,
       '中断 3s(>fresh) 应转 LOST，实测 %s' % f5.state)
    ok(a is not None and a[0] == 'follow' and a[1] == 10.0 and a[2] == 5.0,
       'LOST 应守最后已知目标点 (10,5)，实测 %s' % (a,))
    a = f5.update((0.0, 0.0), 109.0)      # age=9，仍 <= resume
    ok(f5.state == R.TF_ST_LOST and a[0] == 'follow',
       'LOST 等待期(≤resume)内应持续守点，实测 %s' % f5.state)

    print('=== F. LOST 恢复 / 超时恢复航线 ===')
    f5.on_target(11.0, 5.0, 110.0)        # 目标流回来了
    a = f5.update((0.0, 0.0), 110.0)
    ok(f5.state in (R.TF_ST_APPROACH, R.TF_ST_TRACK) and a[0] == 'follow',
       'LOST 中目标流恢复应继续追，实测 state=%s' % f5.state)
    ok(f5.saved_tail is None,
       'LOST 恢复不应动 saved_tail（由主循环写入/消费），实测 %s'
       % (f5.saved_tail,))
    # 再次中断并超过 resume ⇒ restore
    f6 = R.TargetFollower(enter=12.0, resume=10.0, fresh=2.0,
                          goal=(30.0, -20.0))
    f6.on_target(10.0, 5.0, 100.0)
    f6.update((0.0, 0.0), 100.0)
    f6.saved_tail = [(1.0, 2.0), (3.0, 4.0)]
    a = f6.update((0.0, 0.0), 111.0)      # age=11 > resume=10
    ok(a is not None and a[0] == 'restore',
       "超时(age>resume)应返回 ('restore', tail, goal)，实测 %s" % (a[0],))
    ok(list(a[1]) == [(1.0, 2.0), (3.0, 4.0)],
       'restore 应原样返回 saved_tail，实测 %s' % (a[1],))
    ok(f6.state == R.TF_ST_SEARCH,
       'restore 后应回 SEARCH，实测 %s' % f6.state)
    ok(f6.update((0.0, 0.0), 112.0) is None,
       'restore 后 target 应清空（update 返回 None，不干预航线）')
    ok(f6.saved_tail is None,
       'restore 后 saved_tail 应清空（下次接管重新存）')

    print('=== G. 航线末尾回任务目标点 ===')
    f7 = R.TargetFollower(enter=12.0, resume=10.0, fresh=2.0,
                          goal=(30.0, -20.0))
    f7.on_target(10.0, 0.0, 100.0)
    f7.update((0.0, 0.0), 100.0)
    a = f7.update((0.0, 0.0), 111.0)      # saved_tail 为 None（接管点在末尾）
    ok(a[0] == 'restore' and list(a[1]) == [(30.0, -20.0)],
       'saved_tail 为空应回任务目标点，实测 %s' % (a[1],))
    f8 = R.TargetFollower(enter=12.0, resume=10.0, fresh=2.0)
    f8.on_target(10.0, 0.0, 100.0)
    f8.update((0.0, 0.0), 100.0)
    a = f8.update((0.0, 0.0), 111.0)      # 无 tail 且无 goal
    ok(a[0] == 'restore' and list(a[1]) == [],
       '无 tail 且无 goal 应返回空航线（主循环直接收尾驻留），实测 %s'
       % (a[1],))

    print()
    print('总计 %d 项，失败 %d 项' % (checks, len(fails)))
    for f_ in fails:
        print('  !! ' + f_)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
