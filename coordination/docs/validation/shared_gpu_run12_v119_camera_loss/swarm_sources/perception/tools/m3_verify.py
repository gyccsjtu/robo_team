#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M3 验收器（v2，双口径）—— 官方判分口径 + 感知质量口径

为什么要有两套口径
------------------
官方 score_cal.py 的 red 判定有两套状态：**每个 actor 一套**
`count_flag / find_time / topic_arrive_time`，**外加每条 red 流一个** `flag_1/flag_2`
（记"这条流当前认领的是哪个红衣人"）。它回答的是"裁判会不会判过"，
**完全不产出精度**：一条 red 消息在官方代码里会同时与 actor_4、actor_5 比。
而"感知到底报得多准"要靠下面那套**就近归属**才能回答。

旧版（v1）把一条 red 消息按 `actor_id_dict['red'] = [4,5]` **同时**累加到
actor_4 和 actor_5 的误差里 —— 于是 red 的"误差中位"里混进了两个红衣人自身的
间距（~14 m），**数据不可读**。v2 拆成两套：

  官方口径  逐字复刻 `actor_info_callback` / `actor_info1_callback` /
            `actor_info2_callback`（含共享的 topic_arrive_time、flag_N 哨兵、
            `red_cnt == 2` 清零）→ best_hold / ok15 / 清零次数
  感知口径  每条消息按坐标**就近归属**到 actor
            → 命中率 / 误差中位 / 上报间隔 / 双距超差计数

官方 red 判定的三个要点（读源码得出，容易记错）
----------------------------------------------
1. **不要求"一条流固定指某个人"**。每个 actor 各自有 `count_flag`，
   两条 red 流**共同供养**两个 actor 的计时；`flag_N` 只决定"这条流报错时去清谁"。
2. **致命的是一条流"改认领"**（跳槽）：被抛弃的那个 actor 再没人刷新
   `topic_arrive_time`，它的计时就饿死 —— 这正是实测中"换轨"事件对应的现象。
3. `topic_arrive_time[]` 被 red1/red2 **共享**（同一个 6 元素数组、两条流都写），
   所以 red 的"间隔"语义是"关于该 actor 槽位的任意一条 red 消息的相邻间隔"。

判定门限与官方一致：误差 < 1 m 且 相邻消息间隔 < 1 s 且 连续 15 s。
任何一帧不满足 -> count_flag 清零，重新计时。

与官方裁判的两点刻意差异（为了让它在我们的修复环境里能真正跑）
------------------------------------------------------------
  1) **真值源**：官方用 `get_model_state('actor_N')`，但 Gazebo 11 对 Actor
     恒返回 (0,0,1.019) —— 裁判拿到的真值永远是原点，判定必然失败。
     这里改用插件发布的 `/actor_N/pose`，是 Actor 的唯一可靠真值。
  2) **不删模型、不计分**：M3 只验感知链路，不触发 `del_model`。
     因此判定成功后**继续跟踪**（官方此刻会 remove 该 actor 并停止统计），
     于是 best_hold 可能长于官方会记录的 15 s —— 这是"连续化扩展"，
     `ok15` 的结论与官方完全一致。

输出：每个 actor 的 官方口径连续时长/是否 15 s + 感知口径命中率/误差/间隔，
落盘 JSON 供自动化读取。
"""
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

ERR_TH = 1.0        # 官方 err_threshold
INTERVAL_TH = 1.0   # 官方 topic_arrive_interval
HOLD_TH = 15.0      # 官方连续保持

actor_id_dict = {'green': [0], 'blue': [1], 'brown': [2], 'white': [3], 'red': [4, 5]}
CLASS_OF = ['green', 'blue', 'brown', 'white', 'red', 'red']
TOPICS = {
    "green": "/actor_green_info", "blue": "/actor_blue_info",
    "brown": "/actor_brown_info", "white": "/actor_white_info",
    "red1": "/actor_red1_info", "red2": "/actor_red2_info",
}
# 官方两个 red 回调的哨兵值：red1 用 0、red2 用 1。
# 都是刻意的 —— red 的合法 id 只有 4/5，0 和 1 都不可能被认领，所以安全。
SENTINEL = {"red1": 0, "red2": 1}

OUT = os.environ.get("M3_OUT", "/tmp/m3_result.json")
DUR = float(os.environ.get("M3_DUR", "60"))       # 验收时长（仿真秒）


class Judge:
    """逐字复刻官方判定状态机，并并行维护一套"感知质量"统计。

    纯 Python（不依赖 rospy），便于离线单测 —— 见 test_judge.py。
    """

    def __init__(self, n=6, hold_th=HOLD_TH, err_th=ERR_TH, interval_th=INTERVAL_TH):
        self.n = n
        self.hold_th = hold_th
        self.err_th = err_th
        self.interval_th = interval_th

        # ---------- 真值 ----------
        self.truth = [None] * n            # (x, y) —— None 表示还没收到

        # ---------- 官方口径 ----------
        self.count_flag = [False] * n
        self.find_time = [0.0] * n
        self.arrive_t = [0.0] * n          # 官方的 topic_arrive_time（red1/red2 共享！）
        self.flag = dict(SENTINEL)         # 每条 red 流"认领"的 actor
        self.best_hold = [0.0] * n
        self.ok15 = [False] * n
        self.ok15_t = [None] * n
        self.n_reset = [0] * n             # count_flag 被清零次数
        self.n_break_if = [0] * n          # 其中"因不合格帧直接清"的次数（单流类）
        self.n_claim = {"red1": 0, "red2": 0}    # flag_N 改认领次数（跳槽）
        self.claim_seq = {"red1": [], "red2": []}  # 认领历史，用于看跳槽时间点

        # ---------- 感知口径（就近归属）----------
        self.q_msg = [0] * n               # 归属到该 actor 的消息数
        self.q_err = [[] for _ in range(n)]
        self.q_iv = [[] for _ in range(n)]
        self.q_last = [None] * n
        self.q_ok = [0] * n                # 误差<1m 且 间隔<1s 的条数
        self.far = {"red1": 0, "red2": 0}  # 离所有同类 actor 都 >= err_th 的条数

    # ------------------------------------------------------------------
    def set_truth(self, i, x, y):
        self.truth[i] = (x, y)

    def on_msg(self, cls_key, flow, x, y, now):
        """一条上报消息。

        cls_key : 'green' | 'blue' | 'brown' | 'white' | 'red'
        flow    : None（单流类）| 'red1' | 'red2'（red 的两条流）
        """
        ids = actor_id_dict[cls_key]
        self._official(ids, x, y, now, flow)
        self._quality(ids, x, y, now, flow)

    # ------------------------------------------------------------------
    def _official(self, ids, x, y, now, flow):
        """复刻官方回调体（单流走 else 分支的直接清零，red 走 red_cnt 机制）。"""
        red_cnt = 0
        for i in ids:
            t = self.truth[i]
            if t is None:
                # 官方此刻会 AttributeError；我们跳过（启动初期）。不计入 red_cnt，
                # 免得两条流都没真值时被误判成"离两人都远"。
                continue
            # 官方：无条件更新，且在判断之前（这决定了 interval 的语义）
            iv = now - self.arrive_t[i]
            self.arrive_t[i] = now
            d2 = (x - t[0]) ** 2 + (y - t[1]) ** 2
            if d2 < self.err_th ** 2 and iv < self.interval_th:
                if not self.count_flag[i]:
                    self.count_flag[i] = True
                    self.find_time[i] = now
                    if flow is not None:
                        self.flag[flow] = i
                        self.n_claim[flow] += 1
                        self.claim_seq[flow].append((round(now, 2), i))
                elif now - self.find_time[i] >= self.hold_th:
                    # 官方在此 del_model + left_actors.remove(i)，此后不再统计。
                    # 我们继续跟踪（更利于诊断），结论等价 —— 见文件头说明 2)。
                    if not self.ok15[i]:
                        self.ok15[i] = True
                        self.ok15_t[i] = now
                self.best_hold[i] = max(self.best_hold[i], now - self.find_time[i])
            else:
                if flow is None:
                    if self.count_flag[i]:
                        self.n_reset[i] += 1
                        self.n_break_if[i] += 1
                    self.count_flag[i] = False
                else:
                    # red：离"本条流涉及的每个 id"都 >= 1m 才累到 2
                    red_cnt += 1
                    sent = SENTINEL[flow]
                    if red_cnt == 2 and not self.flag[flow] == sent:
                        j = self.flag[flow]
                        if self.count_flag[j]:
                            self.n_reset[j] += 1
                        self.count_flag[j] = False
                        self.flag[flow] = sent

    # ------------------------------------------------------------------
    def _quality(self, ids, x, y, now, flow):
        """就近归属：这条消息算到离得最近的那个 actor 头上。"""
        best, bd = None, None
        for i in ids:
            t = self.truth[i]
            if t is None:
                continue
            d = math.hypot(x - t[0], y - t[1])
            if bd is None or d < bd:
                best, bd = i, d
        if best is None:
            return
        if bd >= self.err_th and flow is not None:
            self.far[flow] += 1
        i = best
        iv = None if self.q_last[i] is None else now - self.q_last[i]
        self.q_last[i] = now
        self.q_msg[i] += 1
        self.q_err[i].append(round(bd, 3))
        if iv is not None:
            self.q_iv[i].append(round(iv, 3))
        if bd < self.err_th and (iv is None or iv < self.interval_th):
            self.q_ok[i] += 1

    # ------------------------------------------------------------------
    def summary(self):
        res = {}
        lines = []
        for i in range(self.n):
            cls = CLASS_OF[i]
            n = self.q_msg[i]
            row = {"cls": cls, "truth_seen": self.truth[i] is not None,
                   "official": {"best_hold": round(self.best_hold[i], 2),
                                "ok15": self.ok15[i], "resets": self.n_reset[i]}}
            if n:
                es = sorted(self.q_err[i])
                ivs = sorted(self.q_iv[i])
                row["msg"] = n
                row["hit_rate"] = round(self.q_ok[i] * 100.0 / n, 1)
                row["err_med"] = round(es[len(es) // 2], 3)
                row["err_mean"] = round(sum(es) / n, 3)
                row["interval_med"] = round(ivs[len(ivs) // 2], 3) if ivs else None
                row["interval_max"] = round(ivs[-1], 3) if ivs else None
                row["err_p90"] = round(es[int(len(es) * 0.9)], 3)
            res["actor_%d" % i] = row
            lines.append(row)
        res["_red_flows"] = {
            "far": dict(self.far),
            "claims": dict(self.n_claim),
            "claim_seq": {k: v[:40] for k, v in self.claim_seq.items()},
        }
        return res, lines


# ----------------------------------------------------------------------
def summarize(J):
    res, lines = J.summary()
    print("\n" + "=" * 96)
    print("M3 验收汇总（v2 双口径）")
    print("  官方口径 = 误差<1m 且 间隔<1s 且 连续15s（逐字复刻 score_cal.py）")
    print("  感知口径 = 就近归属后的 命中率/误差/间隔（回答「报得多准」）")
    print("=" * 96)
    print("%-8s %6s | %8s %6s %6s | %7s %7s %8s %8s %8s"
          % ("actor", "类别", "官方连续s", "15s", "清零", "消息数", "命中率",
             "误差中位", "误差P90", "间隔最大"))
    print("-" * 96)
    for i, r in enumerate(lines):
        o = r["official"]
        if "msg" not in r:
            print("%-8s %6s | %8.1f %6s %6d | %7s %7s %8s %8s %8s"
                  % ("actor_%d" % i, r["cls"], o["best_hold"],
                     "PASS" if o["ok15"] else "-", o["resets"],
                     "-", "-", "-", "-", "-"))
            continue
        print("%-8s %6s | %8.1f %6s %6d | %7d %6.1f%% %8.2f %8.2f %8.2f"
              % ("actor_%d" % i, r["cls"], o["best_hold"],
                 "PASS" if o["ok15"] else "FAIL", o["resets"],
                 r["msg"], r["hit_rate"], r["err_med"], r["err_p90"],
                 r["interval_max"] if r["interval_max"] is not None else -1))
    print("-" * 96)
    rf = res["_red_flows"]
    print("red 双流：双距超差(far) %s | 认领(跳槽)次数 %s"
          % (rf["far"], rf["claims"]))
    for k in ("red1", "red2"):
        if rf["claim_seq"][k]:
            print("  %s 认领历史(t, actor): %s"
                  % (k, " ".join("%.1f->a%d" % (t, a) for t, a in rf["claim_seq"][k][:12])))
    passed = [r["cls"] + str(i) for i, r in enumerate(lines) if r["official"]["ok15"]]
    print("官方口径达成连续 15 s 的目标: %s" % (", ".join(passed) if passed else "无"))
    with open(OUT, "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print("结果已写入 %s" % OUT)
    print("=" * 96)
    return res


def main():
    rospy.init_node("m3_verify", anonymous=True)
    J = Judge()
    for i in range(6):
        rospy.Subscriber("/actor_%d/pose" % i, PoseStamped,
                         lambda m, i=i: J.set_truth(i, m.pose.position.x,
                                                    m.pose.position.y),
                         queue_size=1)
    for key in ("green", "blue", "brown", "white"):
        rospy.Subscriber(TOPICS[key], ActorInfo,
                         lambda m, k=key: J.on_msg(k, None, m.x, m.y, rospy.get_time()),
                         queue_size=1)
    for key in ("red1", "red2"):
        rospy.Subscriber(TOPICS[key], ActorInfo,
                         lambda m, k=key: J.on_msg("red", k, m.x, m.y, rospy.get_time()),
                         queue_size=1)

    print("[m3] 验收器 v2 启动，时长 %.0f 仿真秒" % DUR, flush=True)
    time.sleep(2)                      # 等 rospy.get_time() 有真实基准（否则 t0=0）
    rate = rospy.Rate(2)
    t_start = rospy.get_time()
    last_print = 0
    while not rospy.is_shutdown():
        el = rospy.get_time() - t_start
        if el >= DUR:
            break
        if int(el) - last_print >= 10:
            last_print = int(el)
            act = [i for i in range(6) if J.q_msg[i] > 0]
            print("[m3] t=%3.0fs  有上报的 actor=%s  官方连续=%s  red认领=%s"
                  % (el, act, [round(J.best_hold[i], 1) for i in act], J.flag),
                  flush=True)
        rate.sleep()
    summarize(J)


if __name__ == "__main__":
    main()
