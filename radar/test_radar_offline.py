#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""radar_avoid 的离线单元自检（不需要 ROS）。

为什么单独写：
  radar_avoid.py 顶部 import rospy，本机（Windows）没有 ROS。
  所以这里用桩替代 rospy / 消息类型，只测纯几何与势场函数。

验证项：
  A. 极坐标 -> ENU 变换：4 个标准朝向（0/90/180/270 度）下的落点是否与手算一致
  B. 安装偏移是否正确叠加
  C. 排斥方向：障碍在左 -> 往右让；在右 -> 往左让；在身后 -> 不干预
  D. 限幅
  E. 距离筛选：inf / 超量程 / 低于 range_min 是否被正确剔除
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------- 给 radar_avoid 打桩，让它能在无 ROS 环境 import ----------
def install_stubs():
    rospy = types.ModuleType('rospy')
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    rospy.logerr = lambda *a, **k: None
    rospy.init_node = lambda *a, **k: None
    rospy.sleep = lambda *a, **k: None
    rospy.Time = type('T', (), {'now': staticmethod(lambda: 0)})
    rospy.Rate = type('R', (), {'__init__': lambda s, hz: None,
                                'sleep': lambda s: None})
    rospy.Subscriber = type('S', (), {'__init__': lambda s, *a, **k: None})
    rospy.Publisher = type('P', (), {'__init__': lambda s, *a, **k: None,
                                     'publish': lambda s, m: None})
    rospy.ServiceProxy = type('SP', (), {'__init__': lambda s, *a, **k: None})
    rospy.ROSInterruptException = Exception
    rospy.is_shutdown = lambda: False
    sys.modules['rospy'] = rospy

    gm = types.ModuleType('geometry_msgs')
    gmm = types.ModuleType('geometry_msgs.msg')

    class _P(object):
        def __init__(self, **k):
            self.x = self.y = self.z = 0.0

        class _H(object):
            frame_id = ''
            stamp = 0
        header = _H()

        class _Q(object):
            x = y = z = 0.0
            w = 1.0
        orientation = _Q()

        class _Po(object):
            x = y = z = 0.0
        pose = _Po()

    gmm.Point = _P
    gmm.PoseStamped = _P
    gm.msg = gmm
    sys.modules['geometry_msgs'] = gm
    sys.modules['geometry_msgs.msg'] = gmm

    sm = types.ModuleType('sensor_msgs')
    smm = types.ModuleType('sensor_msgs.msg')

    class Scan(object):
        pass

    smm.LaserScan = Scan
    sm.msg = smm
    sys.modules['sensor_msgs'] = sm
    sys.modules['sensor_msgs.msg'] = smm

    mv = types.ModuleType('mavros_msgs')
    mvm = types.ModuleType('mavros_msgs.msg')

    class _Simple(object):
        def __init__(self, **k):
            for kk, vv in k.items():
                setattr(self, kk, vv)

    mvm.ParamValue = _Simple
    mvm.State = _Simple
    mv.msg = mvm
    sys.modules['mavros_msgs'] = mv
    sys.modules['mavros_msgs.msg'] = mvm

    for name in ('mavros_msgs.srv',):
        m = types.ModuleType(name)
        m.CommandBool = _Simple
        m.ParamSet = _Simple
        m.SetMode = _Simple
        sys.modules[name] = m

    st = types.ModuleType('std_msgs')
    stm = types.ModuleType('std_msgs.msg')

    class Bool(object):
        def __init__(self, data=False):
            self.data = data

    class FMA(object):
        def __init__(self, data=None):
            self.data = data or []

    stm.Bool = Bool
    stm.Float32MultiArray = FMA
    st.msg = stm
    sys.modules['std_msgs'] = st
    sys.modules['std_msgs.msg'] = stm


class FakeScan(object):
    """构造一帧可预测的雷达数据：每条射线距离恒定。"""

    def __init__(self, n, dist, ang0=-math.pi, ang1=math.pi):
        self.angle_min = ang0
        self.angle_max = ang1
        self.angle_increment = (ang1 - ang0) / n
        self.ranges = [dist] * n


def main():
    install_stubs()
    sys.path.insert(0, HERE)
    import radar_avoid as R

    fails = []
    checks = 0

    def ok(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            fails.append(msg)
        print(('  PASS  ' if cond else '  FAIL  ') + msg)

    print('=== A. 极坐标 -> ENU（yaw=0，无人机在原点，障碍在 8m 处） ===')
    # 构造单射线：一条正前方
    n = 4
    sc = FakeScan(n, 8.0)
    # 让 4 条射线分别是 0, 90, 180, 270 度
    sc.angle_min = 0.0
    sc.angle_increment = math.pi / 2.0
    obs = R.scan_to_obstacles(sc, 0.0, (0.0, 0.0), (0.0, 0.0))
    xs = sorted(round(o[0], 3) for o in obs)
    ys = sorted(round(o[1], 3) for o in obs)
    # 期望：(8,0) (0,8) (-8,0) (0,-8)
    ok(xs == [-8.0, 0.0, 0.0, 8.0], 'x 分量 = %s' % xs)
    ok(ys == [-8.0, 0.0, 0.0, 8.0], 'y 分量 = %s' % ys)

    print('=== B. yaw=90° 时同一条射线应转到 +y 方向 ===')
    sc2 = FakeScan(1, 8.0, 0.0, 0.0)
    sc2.angle_increment = 0.0
    obs2 = R.scan_to_obstacles(sc2, math.pi / 2.0, (0.0, 0.0), (0.0, 0.0))
    # 机头朝 +y，雷达 0 度 = 机头方向 => ENU 里是 (0, 8)
    ok(abs(obs2[0][0]) < 1e-6 and abs(obs2[0][1] - 8.0) < 1e-6,
       'yaw=90 度时 0 度射线 -> (%.3f, %.3f)' % (obs2[0][0], obs2[0][1]))

    print('=== C. 安装偏移叠加 ===')
    sc3 = FakeScan(1, 8.0, 0.0, 0.0)
    sc3.angle_increment = 0.0
    # 雷达装在机头前方 2m，yaw=0 -> ENU x 应为 8+2=10
    obs3 = R.scan_to_obstacles(sc3, 0.0, (0.0, 0.0), (2.0, 0.0))
    ok(abs(obs3[0][0] - 10.0) < 1e-6, '安装偏移 x: 期望 10.0 实际 %.3f' % obs3[0][0])

    print('=== D. 距离筛选（inf / 超量程 / 太近）应剔除 ===')
    sc4 = FakeScan(3, 10.0, 0.0, 0.0)
    sc4.angle_increment = 0.0
    sc4.ranges = [float('inf'), 25.0, 0.2]   # inf / 超 20m / 低于 0.5m
    obs4 = R.scan_to_obstacles(sc4, 0.0, (0.0, 0.0), (0.0, 0.0))
    ok(len(obs4) == 0, '三个非法读数全被剔除，剩余 %d 个' % len(obs4))

    print('=== E. 排斥方向：障碍在左 -> 往右让（shift 为负） ===')
    # 无人机原点，航向 +x（heading=0）。障碍在左前方（y>0）
    # 距离必须在影响半径内
    obsL = [(6.0, 2.0, 6.32)]
    sh, nr, bl = R.repulse_vector(obsL, (0.0, 0.0), 0.0)
    ok(sh < 0, '障碍在左(y=+2) -> shift=%.3f 应为负(往右让)' % sh)
    # nearest 是函数内用 dx,dy 重算的欧氏距离 = sqrt(36+4) = 6.324555，
    # 不是传入的 dist 字段（6.32 只是四舍五入值）⇒ 不能做浮点等值比较
    ok(nr is not None and abs(nr - math.hypot(6.0, 2.0)) < 1e-6,
       'nearest=%.4f 应为 %.4f (= hypot(6,2))' % (nr if nr else -1, math.hypot(6.0, 2.0)))
    # 净空 = 6.3246 - 0.80 = 5.5246 > BLOCK_GAP(2.0) ⇒ 不算 blocked
    ok(not bl, '净空 5.52 > BLOCK_GAP(2.0) -> blocked 应为 False，实测 %s' % bl)

    print('=== F. 排斥方向：障碍在右 -> 往左让（shift 为正） ===')
    obsR = [(6.0, -2.0, 6.32)]
    sh2, nr2, bl2 = R.repulse_vector(obsR, (0.0, 0.0), 0.0)
    ok(sh2 > 0, '障碍在右(y=-2) -> shift=%.3f 应为正(往左让)' % sh2)

    print('=== F2. 贴脸障碍应判 blocked ===')
    sh2b, nr2b, bl2b = R.repulse_vector([(2.0, 0.0, 2.0)], (0.0, 0.0), 0.0)
    ok(bl2b, '正前方 2.0m（净空 1.2m < BLOCK_GAP 2.0）-> blocked 应为 True')

    print('=== F3. 正前方障碍（lateral≈0）应产生显著排斥而非零 ===')
    sh2c, _, _ = R.repulse_vector([(4.0, 0.0, 4.0)], (0.0, 0.0), 0.0)
    ok(abs(sh2c) > 0.01, '正前方 4m 障碍 -> |shift|=%.3f 应大于 0.01' % abs(sh2c))

    print('=== G. 障碍在身后 -> 不干预 ===')
    obsB = [(-6.0, 0.0, 6.0)]
    sh3, nr3, bl3 = R.repulse_vector(obsB, (0.0, 0.0), 0.0)
    ok(abs(sh3) < 1e-9 and nr3 is None,
       '身后障碍 -> shift=%.3f nearest=%s（应为 0 / None）' % (sh3, nr3))

    print('=== H. 超出影响半径 -> 不干预 ===')
    obsF = [(12.0, 0.0, 12.0)]   # influence=9.0
    sh4, nr4, bl4 = R.repulse_vector(obsF, (0.0, 0.0), 0.0)
    ok(abs(sh4) < 1e-9, '12m 远（影响半径 9m）-> shift=%.3f 应为 0' % sh4)

    print('=== I. 限幅 ===')
    ok(R.clamp_shift(9.9) == R.MAX_SHIFT, 'clamp(9.9) = %s' % R.clamp_shift(9.9))
    ok(R.clamp_shift(-9.9) == -R.MAX_SHIFT, 'clamp(-9.9) = %s' % R.clamp_shift(-9.9))
    ok(R.clamp_shift(0.5) == 0.5, 'clamp(0.5) = %s' % R.clamp_shift(0.5))

    print('=== J. 对称性：左侧障碍与右侧镜像 -> shift 应反号等值 ===')
    a, _, _ = R.repulse_vector([(5.0, 1.5, 5.22)], (0.0, 0.0), 0.0)
    b, _, _ = R.repulse_vector([(5.0, -1.5, 5.22)], (0.0, 0.0), 0.0)
    ok(abs(a + b) < 1e-9, '镜像对称: a=%.4f b=%.4f 和应≈0' % (a, b))

    # ---- 以下 3 项为 09-27 由 diag_repulse_field.py 暴露的缺陷做的回归护栏 ----
    print('=== K. 回归：排斥量必须"够硬"（旧版在 6m 处只有 0.09，等于没避障）===')
    sh6, _, _ = R.repulse_vector([(6.0, 0.0, 6.0)], (0.0, 0.0), 0.0)
    ok(abs(sh6) > 0.30,
       '正前方 6m 障碍 |shift|=%.3f 应 > 0.30（旧缺陷值 0.092）' % abs(sh6))
    sh4b, _, _ = R.repulse_vector([(4.0, 0.0, 4.0)], (0.0, 0.0), 0.0)
    ok(abs(sh4b) > 0.80,
       '正前方 4m 障碍 |shift|=%.3f 应 > 0.80（旧缺陷值 0.250）' % abs(sh4b))

    print('=== L. 回归：横向对齐度必须让"已偏开"的障碍衰减（旧版完全无此能力）===')
    # 障碍中心距同为 6m，横向偏置从 0.3 增到 4.0 => 修正量必须单调递减
    seq = []
    for lat in (0.3, 0.8, 1.5, 2.5, 4.0):
        ox = (6.0 ** 2 - lat ** 2) ** 0.5
        s, _, _ = R.repulse_vector([(ox, lat, 6.0)], (0.0, 0.0), 0.0)
        seq.append(abs(s))
    mono = all(seq[i] > seq[i + 1] for i in range(len(seq) - 1))
    ok(mono, '横向偏置 0.3→4.0m 的 |shift| 单调递减: %s'
       % ['%.3f' % v for v in seq])
    # 旧版这三个值完全相同（0.092 / 0.092）=> 横向误差无法收敛
    ok(abs(seq[0] - seq[-1]) > 0.10,
       '正对(|lat|=0.3) 与 已偏开(|lat|=4.0) 的差异 %.3f 应 > 0.10'
       % abs(seq[0] - seq[-1]))

    print('=== M. 回归：正对障碍(lateral==0)的方向必须确定且不震荡 ===')
    s_pos, _, _ = R.repulse_vector([(6.0, 0.0, 6.0)], (0.0, 0.0), 0.0)
    s_neg, _, _ = R.repulse_vector([(6.0, -0.0, 6.0)], (0.0, 0.0), 0.0)
    ok(s_pos == s_neg and s_pos != 0.0,
       'lateral=+0.0 与 -0.0 结果应一致且非零: %.6f / %.6f' % (s_pos, s_neg))

    # ---- N. 绕行定侧视场必须收窄（09-27 场景 4 抓出的致命缺陷）----
    print('=== N. 回归：窄通道两侧墙角不得触发定侧（SIDE_FOV_HALF）===')
    # 窄通道：飞机 (0,0)、heading=0°；两侧建筑近端角点 (10, ±3)
    # ⇒ 相对飞机方向 = ±16.7°。旧判据（±90° 半平面）会把它们当
    #   "前方障碍"并按最近者定侧 ⇒ 选侧反了 ⇒ 钻进建筑（净空 -7.18m）。
    obs_wall = [(10.0, 3.0, math.hypot(10.0, 3.0)),
                (10.0, -3.0, math.hypot(10.0, -3.0))]
    sc = R.SideCommit()
    side_wall = sc.update(obs_wall, (0.0, 0.0), 0.0)
    ok(side_wall is None,
       '通道两侧墙角(±16.7°) 不应定侧，实测 side=%s（应 None）' % side_wall)

    # 对照：真正挡在正前方的障碍（lateral≈0）必须定侧
    sc2 = R.SideCommit()
    side_front = sc2.update([(8.0, 0.0, 8.0)], (0.0, 0.0), 0.0)
    ok(side_front is not None,
       '正前方 8m 障碍必须定侧，实测 side=%s（应非 None）' % side_front)

    # 视场边界：+8° 在内、+20° 在外（SIDE_FOV_HALF=12°，09-27 九次修正）
    # 🔴 阈值从 30° 收到 12°：窄通道墙角点在 ±16.7°，30° 视场会误收
    #    ⇒ 压力测试 A/E 全线 FAIL（净空 -9.09m）。12° 排除墙角、
    #      仍覆盖正对着的墙。
    sc3 = R.SideCommit()
    s8 = sc3.update([(8.0, 8.0 * math.tan(math.radians(8.0)), 0.0)],
                    (0.0, 0.0), 0.0)
    ok(s8 is not None, '+8° 障碍应在视场内并定侧，实测 %s' % s8)
    sc4 = R.SideCommit()
    s20a = sc4.update([(8.0, 8.0 * math.tan(math.radians(20.0)), 0.0)],
                      (0.0, 0.0), 0.0)
    ok(s20a is None,
       '+20° 障碍应在视场外、不定侧，实测 %s（应 None）' % s20a)

    # 介入距离：9m 外不应定侧
    sc5 = R.SideCommit()
    s_far = sc5.update([(15.0, 0.0, 15.0)], (0.0, 0.0), 0.0)
    ok(s_far is None,
       '15m 外障碍(> AVOID_ENGAGE_RANGE %.1f) 不应定侧，实测 %s'
       % (R.AVOID_ENGAGE_RANGE, s_far))

    print('=== N2. 回归：side 的符号 = 绕行方向（与障碍位置相反）===')
    # 🔴 09-27 十次修正：side 是"往哪边绕"，不是"障碍在哪边"。
    #   障碍在机体系左侧(lateral>0) ⇒ 必须往**右**绕 ⇒ side 应为 -1。
    #   旧代码照抄 lateral 符号（side=+1）⇒ 直奔障碍
    #   （压力测试 E 实测净空 -8.44m、碰撞 351 帧）。
    scn = R.SideCommit()
    s_left = scn.update([(8.0, 1.0, 8.0)], (0.0, 0.0), 0.0)
    ok(s_left == -1,
       '障碍在左(lateral=+1) ⇒ 应往右绕 side=-1，实测 %s' % s_left)
    scn2 = R.SideCommit()
    s_right = scn2.update([(8.0, -1.0, 8.0)], (0.0, 0.0), 0.0)
    ok(s_right == 1,
       '障碍在右(lateral=-1) ⇒ 应往左绕 side=+1，实测 %s' % s_right)

    print('=== O. 回归：release() 只累积计数、不立即清侧（防绕行中丢失）===')
    sc6 = R.SideCommit(release_frames=10)
    sc6.update([(8.0, 0.0, 8.0)], (0.0, 0.0), 0.0)
    ok(sc6.side is not None, '前置条件：已定侧 %s' % sc6.side)
    # 连续 release 9 帧（< release_frames）=> 侧必须保持
    held_ok = True
    for _ in range(9):
        if sc6.release() is None:
            held_ok = False
            break
    ok(held_ok, 'release 9 帧(<10) 侧应保持，实测 %s' % sc6.side)
    # 第 10 帧 => 解锁
    sc6.release()
    ok(sc6.side is None, 'release 第 10 帧应解锁，实测 %s' % sc6.side)

    print('=== P. 回归：定侧后换边需满足 COMMIT_MIN_FRAMES 帧 ===')
    sc7 = R.SideCommit()
    # 障碍在右(lateral=-1) ⇒ 定"往左绕" = side=+1
    sc7.update([(8.0, -1.0, 8.0)], (0.0, 0.0), 0.0)
    s_first = sc7.side
    ok(s_first == 1, '右侧障碍应定 side=+1（往左绕），实测 %s' % s_first)
    # 立刻把证据改到另一侧（lateral>0 且仍在视场内）=> 不应立刻换边
    # ⚠ 障碍必须放在 SIDE_FOV_HALF(12°) 视场内，否则 has_front=False
    #   会走 release 分支、把 side 解锁掉，测的就不是"换边"了。
    obs_other = [(8.0, 1.0, 8.0)]           # 方向 atan2(1,8) = +7.1°，在视场内
    sc7.update(obs_other, (0.0, 0.0), 0.0)
    ok(sc7.side == s_first,
       '保持帧数不足时不应换边，实测 %s（应 %s）' % (sc7.side, s_first))
    # 持续喂另一侧证据，累计到 COMMIT_MIN_FRAMES 后应换边
    # 障碍在左侧(lateral>0) ⇒ 应换成"往右绕" = side=-1
    for _ in range(R.COMMIT_MIN_FRAMES + 2):
        sc7.update(obs_other, (0.0, 0.0), 0.0)
    ok(sc7.side == -1,
       '累计 %d 帧后应换到 side=-1（往右绕），实测 %s'
       % (R.COMMIT_MIN_FRAMES, sc7.side))

    print()
    print('总计 %d 项，失败 %d 项' % (checks, len(fails)))
    for f in fails:
        print('  !! ' + f)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
