#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
red 双流专项核查器 —— 验证「/actor_red1_info 与 /actor_red2_info 各自锁定一个红衣人」。

m3_verify.py 把 red1/red2 合并成 cls="red" 统计，看不出两条流是不是在乱指。
而 M5 的判分恰恰取决于**每条流是否稳定指向固定的人**：官方 actor_infoN_callback 里
    if red_cnt == 2 and flag_N != 0:  count_flag[flag_N] = False
一帧报偏（离 actor_4、actor_5 都 >1 m）就清除该流的连续计时。两条流若在帧间互换，
15 s 永远累不到。

本脚本按消息逐条判定归属（离哪个 actor 最近且在 OWN_TH 内），给出：
  1) 每条流的归属直方图        -> 理想：red1 全指一个、red2 全指另一个
  2) 归属切换次数              -> 理想 0（换一次就可能清零一次）
  3) 每条流的最大发布间隔      -> 要 <1s（裁判门限）
  4) 每个 actor 的最长连续保持  -> 逐字复刻官方口径，>=15s 才算 M5 能得分
  5) 双流同时有效覆盖时长       -> 两个红衣人"同时在跟"的净时长

用法:  DC_DUR=120 python3 dual_check.py
"""
import json
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

ERR_TH = 1.0
INTERVAL_TH = 1.0
HOLD_TH = 15.0
OWN_TH = float(os.environ.get("DC_OWN_TH", "3.0"))   # 归属判定门限（两人相距 14m，3m 足够）
OUT = os.environ.get("DC_OUT", "/tmp/dual_check.json")
DUR = float(os.environ.get("DC_DUR", "120"))
# DC_DUMP=1 时把"无归属"消息的原始坐标一并落盘，用于区分三种成因：
#   · 聚成一坨固定点  -> 静态红色误检抢了槽位（要回到感知侧修）
#   · 稳定滞后于真人  -> 管道延迟（看图→推理→反投影 的固有滞后）
#   · 围绕真人乱抖且误差 3~8m -> 部分遮挡导致 bbox 中心偏移 / 深度估计跳变
DUMP = os.environ.get("DC_DUMP", "0") == "1"

truth = {4: None, 5: None}
flow = {"red1": [], "red2": []}          # [(t, owner, err)]  owner ∈ {4,5,None}
raw = {"red1": [], "red2": []}           # DUMP=1 时: [(t, x, y, owner, d, 真值A, 真值B)]


def on_truth(i):
    """真值走 /actor_N/pose。

    ⚠ 这里必须用 /actor_N/pose，**不能用 get_model_state**：
    Actor 的位姿由 ActorPluginRos::OnUpdate 每帧从它**自己维护的 currentPose_**
    积分后 SetWorldPose 写回，它根本不读 Gazebo 的模型状态；而 set_model_state
    写进去的值下一帧就被它覆盖。于是 get_model_state 读到的是"外部刚写进去的
    瞬时值"，与渲染位置高频打架。`/actor_N/pose` 发布的才是 currentPose_，
    也就是真正被渲染的那个位置 —— 这一点是反投影画面验证过的。
    """
    def cb(msg):
        truth[i] = (msg.pose.position.x, msg.pose.position.y)
    return cb


def mk(flowname):
    def cb(m):
        now = rospy.get_time()
        best, bd = None, 1e9
        for i in (4, 5):
            if truth[i] is None:
                continue
            d = ((m.x - truth[i][0]) ** 2 + (m.y - truth[i][1]) ** 2) ** 0.5
            if d < bd:
                best, bd = i, d
        owner = best if bd <= OWN_TH else None
        flow[flowname].append((now, owner, bd))
        if DUMP:
            raw[flowname].append((now, m.x, m.y, owner, bd,
                                 truth[4], truth[5]))
    return cb


def pctl(sorted_vals, q):
    """分位数（输入必须已升序）"""
    if not sorted_vals:
        return float("nan")
    i = min(len(sorted_vals) - 1, max(0, int(round(q * (len(sorted_vals) - 1)))))
    return sorted_vals[i]


def longest_hold(msgs):
    """逐字复刻官方：误差<1m 且相邻间隔<1s 才连续；一帧不满足就清零重计。
    返回 (最长连续秒数, 断链次数)"""
    best, cur_start, prev_t, brk = 0.0, None, None, 0
    for t, err in msgs:
        if err >= ERR_TH or (prev_t is not None and t - prev_t >= INTERVAL_TH):
            if cur_start is not None:
                brk += 1
            cur_start = None
        else:
            if cur_start is None:
                cur_start = t
            else:
                best = max(best, t - cur_start)
        prev_t = t
    return best, brk


def diagnose(bad):
    """无归属消息的成因初判（纯函数，便于离线单测）。

    bad: [(t, x, y, owner, d, truthA, truthB)]，已筛出 owner is None 的那些。

    三类成因的判别特征：
      · 静态误检抢槽：所有点聚成一坨固定点 -> 中位离中心小、跨度小
      · 管道延迟    ：报点稳定滞后于真人，离真人距离 ≈ 速度 × 延迟
      · 部分遮挡    ：bbox 中心被遮挡物拉偏 / 深度跳变 -> 围绕真人散开，误差 3~8m

    返回 dict；无数据时 n_bad=0。
    """
    if not bad:
        return {"n_bad": 0}
    xs = sorted(r[1] for r in bad)
    ys = sorted(r[2] for r in bad)
    cx, cy = xs[len(xs) // 2], ys[len(ys) // 2]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    dsp = sorted(((r[1] - cx) ** 2 + (r[2] - cy) ** 2) ** 0.5 for r in bad)
    spread = dsp[len(dsp) // 2]
    tips = []
    for r in bad[:400]:
        cand = [((r[1] - t[0]) ** 2 + (r[2] - t[1]) ** 2) ** 0.5
                for t in (r[5], r[6]) if t is not None]
        if cand:
            tips.append(min(cand))
    tips.sort()
    med_tip = tips[len(tips) // 2] if tips else float("nan")
    max_tip = tips[-1] if tips else float("nan")
    if spread < 1.5 and span < 4.0:
        kind = "疑似静态误检抢槽（聚成一点）"
    else:
        kind = "疑似遮挡/深度误差（围绕真人散开）"
    return {"n_bad": len(bad), "center": (cx, cy), "span": span,
            "median_from_center": spread, "median_to_truth": med_tip,
            "max_to_truth": max_tip, "kind": kind}


def main():
    rospy.init_node("dual_check", anonymous=True)
    for i in (4, 5):
        rospy.Subscriber("/actor_%d/pose" % i, PoseStamped, on_truth(i), queue_size=1)
    rospy.Subscriber("/actor_red1_info", ActorInfo, mk("red1"), queue_size=50)
    rospy.Subscriber("/actor_red2_info", ActorInfo, mk("red2"), queue_size=50)

    print("[dc] red 双流核查启动，时长 %.0f 仿真秒（归属门限 %.1fm）" % (DUR, OWN_TH),
          flush=True)
    # ⚠ 必须先等 ROS 时间就绪再取 t0。踩过的坑：init_node 之后立刻
    # rospy.get_time() 会返回 0，而循环里第一条就到了真实仿真时间，
    # 于是 el 是个天文数字、第一次迭代就 break —— 整轮"秒退"，
    # 汇总表全是 0，看起来像"感知一条都没发"。
    rospy.sleep(2.0)
    t = time.time()
    while (truth[4] is None or truth[5] is None) and time.time() - t < 20.0:
        rospy.sleep(0.3)
    print("[dc] 真值就绪: actor_4=%s actor_5=%s"
          % ("无" if truth[4] is None else "(%.1f,%.1f)" % truth[4],
             "无" if truth[5] is None else "(%.1f,%.1f)" % truth[5]), flush=True)
    t0 = rospy.get_time()
    last = -1e9
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        el = rospy.get_time() - t0
        if el >= DUR:
            break
        if el - last >= 10.0:
            last = el
            print("[dc] t=%4.0fs  red1=%-5d red2=%-5d  真值 A4=%s A5=%s"
                  % (el, len(flow["red1"]), len(flow["red2"]),
                     "无" if truth[4] is None else "(%.1f,%.1f)" % truth[4],
                     "无" if truth[5] is None else "(%.1f,%.1f)" % truth[5]), flush=True)
        rate.sleep()

    res = {}
    print("=" * 78)
    print("%-6s %7s %22s %8s %10s %10s" %
          ("流", "条数", "归属分布(指向哪个actor)", "切换次", "最大间隔", "无归属率"))
    print("-" * 78)
    for name in ("red1", "red2"):
        msgs = flow[name]
        cnt = {}
        swaps = 0
        last_own = "INIT"
        for t, own, d in msgs:
            cnt[own] = cnt.get(own, 0) + 1
            if last_own != "INIT" and own != last_own:
                swaps += 1
            last_own = own
        ivs = [msgs[i][0] - msgs[i - 1][0] for i in range(1, len(msgs))]
        mx_iv = max(ivs) if ivs else None
        none_r = (cnt.get(None, 0) / len(msgs)) if msgs else 1.0
        dist = " ".join("%s:%d" % ("无" if k is None else "actor_%d" % k, v)
                        for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]))
        print("%-6s %7d %22s %8d %10s %9.1f%%" %
              (name, len(msgs), dist, swaps,
               "%.2fs" % mx_iv if mx_iv is not None else "无", none_r * 100))
        res[name] = {"n": len(msgs), "owner_hist": {str(k): v for k, v in cnt.items()},
                     "swaps": swaps, "max_interval": mx_iv, "no_owner_rate": none_r}

    print("-" * 78)
    print("%-8s %10s %10s %10s %8s" % ("actor", "有效上报", "最长连续", "断链次数", "达标"))
    for i in (4, 5):
        msgs = []
        for name in ("red1", "red2"):
            for t, own, d in flow[name]:
                if own == i:
                    msgs.append((t, d))
        msgs.sort()
        hold, brk = longest_hold(msgs)
        ok = hold >= HOLD_TH
        print("actor_%d %10d %9.1fs %10d %8s" % (i, len(msgs), hold, brk,
                                                 "PASS" if ok else "FAIL"))
        res["actor_%d" % i] = {"n": len(msgs), "hold": round(hold, 2),
                               "ok15": ok, "breaks": brk}

    # 双流同时有效覆盖：0.5s 分桶，两条流各自都有"有效归属"消息
    buckets = {}
    for name in ("red1", "red2"):
        for t, own, d in flow[name]:
            if own is not None and d < ERR_TH:
                buckets.setdefault(int(t / 0.5), set()).add(name)
    both = sum(1 for v in buckets.values() if len(v) == 2)
    res["dual_cover_s"] = both * 0.5
    print("-" * 78)
    print("双流同时有效覆盖: %.1f s（两个红衣人同时在跟的净时长）" % (both * 0.5))

    # 误差分布：汇总表只给"最长连续"这一个结论，看不出误差是"几乎全帧 <1m"
    # 还是"在 1m 上下反复横跳" —— 而这两者的改进方向完全不同，所以这里必须补上。
    # "仅有效归属" 才代表真实能力；"全部" 里混了断流期间的无效帧。
    print("-" * 78)
    print("%-10s %-8s %8s %8s %8s %8s %10s" %
          ("流", "口径", "p50", "p90", "p99", "max", "误差<1m"))
    for name in ("red1", "red2"):
        rows = flow[name]
        for label, sel in (("全部", rows),
                           ("仅有效归属", [r for r in rows if r[1] is not None])):
            if not sel:
                continue
            a = sorted(r[2] for r in sel)
            okr = 100.0 * sum(1 for e in a if e < ERR_TH) / len(a)
            print("%-10s %-8s %7.2fm %7.2fm %7.2fm %7.2fm %9.1f%%" %
                  (name if label == "全部" else "", label,
                   pctl(a, .50), pctl(a, .90), pctl(a, .99), a[-1], okr))
            if label == "全部":
                res.setdefault(name, {})["err_p50"] = round(pctl(a, .50), 3)
                res[name]["err_p99"] = round(pctl(a, .99), 3)
                res[name]["err_lt1_rate"] = round(okr, 2)

    if DUMP:
        # 无归属成因初判：把这批消息的坐标做一次粗聚类
        print("-" * 78)
        res["diag"] = {}
        for name in ("red1", "red2"):
            bad = [r for r in raw[name] if r[3] is None]
            dg = diagnose(bad)
            res["diag"][name] = dg
            if dg["n_bad"] == 0:
                print("%s: 无归属于 0 条（槽位全程锁住真人）" % name)
                continue
            print("%s: 无归属 %d/%d 条 | 聚类中心=(%.1f,%.1f) 跨度=%.1fm "
                  "中位离中心=%.1fm | 到最近真人距离 中位=%.1fm 最大=%.1fm"
                  % (name, dg["n_bad"], len(raw[name]),
                     dg["center"][0], dg["center"][1], dg["span"],
                     dg["median_from_center"], dg["median_to_truth"],
                     dg["max_to_truth"]))
            print("   -> %s" % dg["kind"])
        with open(OUT.replace(".json", "_raw.json"), "w") as f:
            json.dump(raw, f, ensure_ascii=False)

    print("=" * 78)

    with open(OUT, "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print("结果已写入 %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
