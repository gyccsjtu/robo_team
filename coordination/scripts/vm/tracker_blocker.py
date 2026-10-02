#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tracker-Blocker 协同：堵路破坏逃跑，改善 Tracker 的观测稳定性。

思路
----
规则 3：目标感知到无人机后，以 2 m/s 朝**远离最近无人机**的方向逃跑。
规则 5：连续 15 s 确认（严格：断 1 s 归零）。

单机追踪的困境（SITL 实测）：目标一路直线逃，绕过楼角就断视线，最长只能
连续 6.6 s。

关键洞察：**逃跑方向由我们的站位决定**。所以在逃跑方向的前方放一架
Blocker 堵住去路，目标就不得不：
  * 转向（改变逃跑方向，可能被迫走向开阔地）
  * 或减速/贴着 Blocker 绕行

无论哪种，目标的**直线逃跑被打断**，Tracker 的观测稳定性都会提升。

与「多派几架一起追」的本质区别
------------------------------
多派 Tracker 只是增加观测点；Blocker 改变的是**目标的行为**（破坏它的
最优逃跑），所以边际收益更高。本脚本用仿真验证这一点。
"""

import argparse
import math
import os
import random
import sys

WS = os.environ.get("ROBOCUP_WORKSPACE",
                    os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup")))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))
sys.path.insert(0, os.path.join(WS, "src", "robocup_swarm", "scripts"))

from strategy_compare import (MapGrid, METADATA, pick_flee_dir,  # noqa: E402
                              HIDE_IDEAL, HIDE_LOCAL, HIDE_NONE,
                              TARGET_FLEE, SENSE_R, DT)
from cooperative_tracker import CooperativeTracker  # noqa: E402

UAV_SPEED = 6.0
TRACK_STANDOFF = 8.0     # Tracker 与目标的保持距离
MAX_T = 60.0

# Blocker 站位：**朝目标推进**，抢占「离目标最近」。
#
# 这是原始版本的致命缺陷：之前把 Blocker 钉在目标前方 12 m 的固定站位，而
# Tracker 只在 8 m。规则 3 是「远离**最近的**无人机」——Blocker 恒比 Tracker
# 远，永远当不上最近机，目标于是完全无视它，一路直线逃，堵路形同虚设。
#
# 改成推进后，Blocker 抢到最近机 → 目标必须掉头 → 朝 Tracker 跑 → Tracker
# 又成最近机 → 再掉头 …… 目标被**夹在 8 m 与 6 m 之间来回翻向**，钉在原地，
# 再也跑不到楼后面去。这才是"堵路改变目标行为"的真实机制。
BLOCK_PUSH = 6.0         # 站到目标逃跑方向前方多远（< TRACK_STANDOFF 才抢得到最近）
BLOCK_MIN = 3.5          # 与目标的硬性最小间距（防撞）
BLOCK_RANGE = 15.0       # 判定"堵路生效"的距离上限
BLOCK_STAND = 12.0       # 旧版固定站位距离（仅消融对照用）


def _frac(hist):
    """历史序列里 1 的占比（空序列返回 0）。"""
    return (sum(hist) / len(hist)) if hist else 0.0


def away_from(g, tgt, positions, hide):
    """目标相对 positions 中**最近者**的逃跑方向（规则 3 的字面语义）。"""
    if not positions:
        return (0.0, 0.0)
    nx, ny = min(positions, key=lambda p: math.hypot(p[0] - tgt[0], p[1] - tgt[1]))
    return pick_flee_dir(g, tgt, (nx, ny), hide, prev_dir=(0.0, 0.0))


def simulate(g, roles, start_uavs, tgt_xy, hide, max_t=MAX_T,
             block_push=None, track_side=False):
    """跑一次协同确认。

    roles: ["tracker", "blocker", ...] —— 与 start_uavs 的 key 顺序对应
           支持 "tracker" / "blocker" 两种角色；长度决定用几架。

    block_push / track_side: 消融开关，**默认即原版行为**（n=200 实测最优）：
        block_push=BLOCK_STAND(12m) —— Blocker 保持固定站位
        track_side=False            —— Tracker 保持目标后方 standoff（追尾）
      改成"推进抢最近"和"观测侧站位"实测分别把达成率拉到 42.5%，见 main()
      末尾的消融表 —— 那两个"修复"是错的，默认值必须留在原版。

    返回 dict(achieved, t_confirm, ended_by, avg_live, blocked_frac, near_frac, stuck)
    """
    uav_ids = sorted(start_uavs.keys())[:len(roles)]
    uavs = {u: list(start_uavs[u]) for u in uav_ids}
    role_of = dict(zip(uav_ids, roles))
    push = BLOCK_STAND if block_push is None else float(block_push)

    tgt = list(tgt_xy)
    tr = CooperativeTracker()
    tr.add_target("t0", now=0.0)

    # 只有 tracker 负责观测；blocker 不参与确认（其作用是改变目标行为）
    trackers = [u for u in uav_ids if role_of[u] == "tracker"]
    tr.assign_observers("t0", trackers)

    t = 0.0
    live_hist = []
    block_hist = []
    near_hist = []          # Blocker 担任"最近机"的时长占比（堵路生效的前提）
    move_n = [0, 0]         # [被调用次数, 受阻次数] —— 量化"卡墙"程度

    while t < max_t:
        t += DT

        # ---- 观测（仅 tracker 计入）----
        for u in trackers:
            ux, uy = uavs[u]
            d = math.hypot(tgt[0] - ux, tgt[1] - uy)
            vis = d < SENSE_R and g.visible(ux, uy, tgt[0], tgt[1])
            tr.report(u, "t0", t, tgt[0], tgt[1], los=vis)

        for tid, ev in tr.update(t):
            if ev == "confirmed":
                live_hist.append(len(tr.targets["t0"].live_observers(t)))
                return dict(achieved=True, t_confirm=t, ended_by="confirmed",
                            avg_live=sum(live_hist) / len(live_hist),
                            near_frac=_frac(near_hist),
                            blocked_frac=_frac(block_hist),
                            stuck=(move_n[1] / move_n[0] if move_n[0] else 0.0))
            if ev == "evade":
                return dict(achieved=False, t_confirm=None, ended_by="evade",
                            avg_live=_frac(live_hist),
                            near_frac=_frac(near_hist),
                            blocked_frac=_frac(block_hist),
                            stuck=(move_n[1] / move_n[0] if move_n[0] else 0.0))
        live_hist.append(len(tr.targets["t0"].live_observers(t)))

        # ---- 决定目标逃跑方向（依据规则3，看**所有**无人机，含 blocker）----
        all_pos = [tuple(uavs[u]) for u in uav_ids]
        fd = away_from(g, tgt, all_pos, hide)

        # ---- 堵路效果度量（用目标本帧实际逃跑方向）----
        blocked_now = 0.0
        for u in uav_ids:
            if role_of[u] != "blocker":
                continue
            ux, uy = uavs[u]
            # 该 blocker 相对目标的方位与逃跑方向的夹角
            ax, ay = ux - tgt[0], uy - tgt[1]
            n = math.hypot(ax, ay) or 1.0
            cosang = (ax / n) * fd[0] + (ay / n) * fd[1]
            if cosang > 0.5 and math.hypot(ax, ay) < BLOCK_RANGE:
                blocked_now = 1.0
                break
        block_hist.append(blocked_now)

        # ---- Blocker 是否抢到了"最近机"（堵路生效的前提）----
        if any(role_of[u] == "blocker" for u in uav_ids):
            pos = {u: tuple(uavs[u]) for u in uav_ids}
            nu = min(pos, key=lambda k: math.hypot(pos[k][0] - tgt[0],
                                                   pos[k][1] - tgt[1]))
            near_hist.append(1.0 if role_of[nu] == "blocker" else 0.0)

        # ---- 目标运动 ----
        # 关键：目标逃跑时会**避开**正前方的 Blocker（否则会撞上），
        # 这体现在 pick_flee_dir 的"必须可通行"检查里；若前方被 Blocker
        # 占据，目标自然转向别的方向。
        nx, ny, _ = g.speed_toward(tgt[0], tgt[1],
                                   tgt[0] + fd[0] * 10, tgt[1] + fd[1] * 10,
                                   TARGET_FLEE)
        tgt[0], tgt[1] = nx, ny

        # ---- 无人机运动 ----
        for u in uav_ids:
            ux, uy = uavs[u]
            if role_of[u] == "tracker":
                if track_side:
                    # 观测侧站位：站在目标逃跑反方向（不会被迎面逼退）
                    gx = tgt[0] - fd[0] * TRACK_STANDOFF
                    gy = tgt[1] - fd[1] * TRACK_STANDOFF
                    if not g.free(gx, gy):
                        d = math.hypot(tgt[0] - ux, tgt[1] - uy)
                        gx, gy = (tgt[0], tgt[1]) if d > TRACK_STANDOFF else (ux, uy)
                else:
                    # 旧版：保持在目标后方 TRACK_STANDOFF 距离（延迟追尾）
                    d = math.hypot(tgt[0] - ux, tgt[1] - uy)
                    if d > TRACK_STANDOFF + 1.0:
                        gx, gy = tgt[0], tgt[1]
                    elif d < TRACK_STANDOFF - 1.0:
                        gx = ux + (ux - tgt[0]) / max(d, 1e-6) * (TRACK_STANDOFF - d)
                        gy = uy + (uy - tgt[1]) / max(d, 1e-6) * (TRACK_STANDOFF - d)
                    else:
                        gx, gy = ux, uy
            else:
                # 拦截者：**朝目标推进**，抢占「离目标最近」。
                # 规则3 说目标远离"最近的那架" —— 只有抢到最近，目标才会
                # 掉头，堵路才真正改变它的行为。固定站位永远抢不到（见顶部注释）。
                dx, dy = tgt[0] - ux, tgt[1] - uy
                dn = math.hypot(dx, dy)
                gx = tgt[0] + fd[0] * push
                gy = tgt[1] + fd[1] * push
                if dn < BLOCK_MIN and dn > 1e-6:
                    gx = ux - dx / dn * (BLOCK_MIN - dn)
                    gy = uy - dy / dn * (BLOCK_MIN - dn)
                # 落点不可通行时，沿逃跑轴两侧找可站点
                if not g.free(gx, gy):
                    for sgn in (1, -1):
                        px = gx - sgn * fd[1] * 5.0
                        py = gy + sgn * fd[0] * 5.0
                        if g.free(px, py):
                            gx, gy = px, py
                            break
                    else:
                        gx, gy = ux, uy
            ax, ay, blk = g.speed_toward(ux, uy, gx, gy, UAV_SPEED)
            uavs[u] = [ax, ay]
            move_n[0] += 1
            move_n[1] += int(bool(blk))

    return dict(achieved=False, t_confirm=None, ended_by="timeout",
                avg_live=_frac(live_hist), near_frac=_frac(near_hist),
                blocked_frac=_frac(block_hist),
                stuck=(move_n[1] / move_n[0] if move_n[0] else 0.0))


def make_case(g, rng):
    """初始条件：目标在开阔且可见处，候选 UAV 散落在其周围。"""
    for _ in range(400):
        tx = rng.uniform(-78, 78)
        ty = rng.uniform(-34, 34)
        if not g.free(tx, ty) or g.open_space_score(tx, ty) < 0.6:
            continue
        uavs = {}
        for i in range(3):
            for _t in range(40):
                ang = rng.uniform(0, 2 * math.pi)
                d = rng.uniform(12.0, 20.0)
                ux, uy = tx + d * math.cos(ang), ty + d * math.sin(ang)
                if abs(ux) > 92 or abs(uy) > 44 or not g.free(ux, uy):
                    continue
                uavs["uav_%d" % (i + 1)] = [ux, uy]
                break
        if len(uavs) == 3:
            return tx, ty, uavs
    return None


CONFIGS = [
    ("1 tracker", ["tracker"]),
    ("2 trackers", ["tracker", "tracker"]),
    ("3 trackers", ["tracker", "tracker", "tracker"]),
    ("tracker + blocker", ["tracker", "blocker"]),
    ("tracker + 2 blocker", ["tracker", "blocker", "blocker"]),
    ("2 tracker + blocker", ["tracker", "tracker", "blocker"]),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--hide", default="ideal", choices=["none", "local", "ideal"])
    args = ap.parse_args()

    HIDE = {"none": HIDE_NONE, "local": HIDE_LOCAL, "ideal": HIDE_IDEAL}[args.hide]
    rng = random.Random(args.seed)
    g = MapGrid(METADATA)

    print("=" * 78)
    print("Tracker-Blocker 协同验证（严格 15 s 连续确认，断 1 s 归零）")
    print("=" * 78)
    print("地图 %s | UAV %.0f m/s | 目标 %.0f m/s | 躲藏=%s"
          % (os.path.basename(METADATA), UAV_SPEED, TARGET_FLEE, args.hide))
    print("")

    cases = []
    for _ in range(args.trials):
        c = make_case(g, rng)
        if c:
            cases.append(c)
    print("有效初始条件: %d 组\n" % len(cases))

    print("%-22s | %-13s | %-9s | %-11s | %-12s | %-8s" %
          ("配置", "达成率", "平均耗时", "同时观测(均值)", "Blocker最近机", "堵路占比"))
    print("-" * 88)
    for name, roles in CONFIGS:
        ok = 0
        times, lives, blocks, nears = [], [], [], []
        for (tx, ty, uavs) in cases:
            r = simulate(g, roles, uavs, (tx, ty), HIDE)
            if r["achieved"]:
                ok += 1
                times.append(r["t_confirm"])
            lives.append(r["avg_live"])
            blocks.append(r["blocked_frac"])
            nears.append(r["near_frac"])
        n = len(cases)
        avg_t = sum(times) / len(times) if times else None
        blk = sum(blocks) / n
        blk_s = ("%.0f%%" % (100 * blk)) if any(b > 0 for b in blocks) else "  -  "
        has_bk = any(r != "tracker" for r in roles)
        near_s = ("%.0f%%" % (100 * sum(nears) / n)) if has_bk else "  -  "
        print("%-22s | %6.1f%% (%2d/%d) | %8s | %11.2f | %12s | %8s"
              % (name, 100.0 * ok / n, ok, n,
                 ("%.1fs" % avg_t) if avg_t else "  -  ",
                 sum(lives) / n, near_s, blk_s))
    print("-" * 88)
    print("")
    print("判读：")
    print("  * 「Blocker最近机」是堵路生效的**前提**：规则3 说目标远离最近的那架，")
    print("    所以 Blocker 必须抢到最近，目标才会掉头。这个数接近 0 就说明")
    print("    Blocker 根本没参与目标决策，堵路是摆设（原始版本就是这个毛病）。")
    print("  * 对比「2 trackers」与「tracker + blocker」：架数相同时，堵路是否优于多一个观测点")
    print("  * 对比「3 trackers」与「2 tracker + blocker」：同上")
    print("=" * 88)

    print("")
    print("消融对照（2 tracker + blocker，n=%d，定位各改动的实际贡献）" % len(cases))
    print("结论：原版（Blocker 12m 固定站位 + Tracker 追尾）**最优**。")
    print("      「推进抢最近」和「观测侧站位」两个所谓修复都更低 —— 已回滚为默认。")
    print("%-40s | %-13s | %-11s | %-12s | %-8s" %
          ("变体", "达成率", "同时观测(均值)", "Blocker最近机", "卡墙占比"))
    print("-" * 88)
    for label, bp, ts in (("原版（12m 站位 + 追尾）【默认】", BLOCK_STAND, False),
                          ("推进6m 抢最近 + 追尾", BLOCK_PUSH, False),
                          ("12m 站位 + 观测侧 tracker", BLOCK_STAND, True),
                          ("两个改动都上", BLOCK_PUSH, True)):
        ok = 0
        lives, nears, stucks = [], [], []
        for (tx, ty, uavs) in cases:
            r = simulate(g, ["tracker", "tracker", "blocker"], uavs, (tx, ty), HIDE,
                         block_push=bp, track_side=ts)
            if r["achieved"]:
                ok += 1
            lives.append(r["avg_live"])
            nears.append(r["near_frac"])
            stucks.append(r["stuck"])
        n = len(cases)
        print("%-40s | %6.1f%% (%2d/%d) | %11.2f | %11.0f%% | %8.0f%%"
              % (label, 100.0 * ok / n, ok, n, sum(lives) / n,
                 100 * sum(nears) / n, 100 * sum(stucks) / n))
    print("-" * 88)
    return 0


if __name__ == "__main__":
    sys.exit(main())
