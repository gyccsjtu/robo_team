#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""red 双流原始数据（dual_check 的 DC_DUMP 落盘）离线分析。

dual_check.py 只给"最长连续 15s 是否达标"这一个结论。它有两处看不见：
  1) **误差分布**。裁判门限是 1 m，可 15 s 连续只要有一帧 >1 m 就清零。
     "最长连续 40.8s" 背后到底是"几乎全帧 <1m"还是"误差在 1m 上下反复横跳"，
     汇总表看不出来 —— 而这两者的改进方向完全不同。
  2) **误差随时间的走向**。掉到 1m 以上的那些帧是均匀分布（噪声），
     还是集中在某几段（遮挡/摆动端点/延迟）？

本脚本只读 /tmp/dual_check_raw.json，纯离线，不再动仿真。

用法:  python3 m5_dump_analyze.py [/tmp/dual_check_raw.json]
"""
import json
import os
import sys

ERR_TH = 1.0


def pct(v, q):
    """简单分位数（v 已升序）"""
    if not v:
        return float("nan")
    i = min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))
    return v[i]


def seg(bad, gap=1.0):
    """把"超差帧"按时间聚成段：返回 [(起始t, 结束t, 帧数, 峰值误差)]"""
    out = []
    cur_s = cur_e = None
    cur_n = 0
    cur_max = 0.0
    for t, e in bad:
        if cur_s is None:
            cur_s = cur_e = t
            cur_n, cur_max = 1, e
        elif t - cur_e <= gap:
            cur_e, cur_n = t, cur_n + 1
            cur_max = max(cur_max, e)
        else:
            out.append((cur_s, cur_e, cur_n, cur_max))
            cur_s = cur_e = t
            cur_n, cur_max = 1, e
    if cur_s is not None:
        out.append((cur_s, cur_e, cur_n, cur_max))
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/dual_check_raw.json"
    if not os.path.exists(path):
        print("找不到 %s（dual_check 要用 DC_DUMP=1 跑过才有）" % path)
        return 1
    with open(path) as f:
        raw = json.load(f)

    print("=" * 82)
    print("red 双流误差分布（门限 %.1f m —— 超一帧，该流的 15s 连续计时就清零）" % ERR_TH)
    print("=" * 82)

    for name in ("red1", "red2"):
        rows = raw.get(name) or []
        if not rows:
            print("%s: 无数据" % name)
            continue
        rows.sort(key=lambda r: r[0])
        own = [r[4] for r in rows if r[3] is not None]     # 归到某个 actor 的
        allc = [r[4] for r in rows]
        t_end = rows[-1][0] - rows[0][0]
        rate = len(rows) / t_end if t_end > 0 else 0.0

        print()
        print("%s: %d 条 / %.1f s  平均 %.2f 条/s  有效归属 %d 条(%.1f%%)"
              % (name, len(rows), t_end, rate, len(own),
                 100.0 * len(own) / len(rows)))
        for label, arr in (("全部", allc), ("仅有效归属", own)):
            if not arr:
                continue
            a = sorted(arr)
            p50, p90, p99 = pct(a, .50), pct(a, .90), pct(a, .99)
            okr = 100.0 * sum(1 for e in a if e < ERR_TH) / len(a)
            print("   %-10s p50=%.2fm p90=%.2fm p99=%.2fm max=%.2fm | "
                  "误差<%.0fm 占比=%.1f%%"
                  % (label, p50, p90, p99, a[-1], ERR_TH, okr))

        # 逐秒的"是否超差"，用来判断误差是均匀噪声还是集中在几段
        bad = [(r[0] - rows[0][0], r[4]) for r in rows
               if r[3] is not None and r[4] >= ERR_TH]
        segs = seg(bad)
        noc = [r for r in rows if r[3] is None]
        print("   超差帧 %d 条，聚成 %d 段" % (len(bad), len(segs)))
        for (s0, s1, n, mx) in sorted(segs, key=lambda x: -x[2])[:6]:
            print("      t=%6.1f~%6.1fs  %3d 帧  峰值误差 %.2fm" % (s0, s1, n, mx))
        if noc:
            nseg = seg([(r[0] - rows[0][0], r[4]) for r in noc])
            print("   无归属帧 %d 条，聚成 %d 段" % (len(noc), len(nseg)))
            for (s0, s1, n, mx) in sorted(nseg, key=lambda x: -x[2])[:4]:
                print("      t=%6.1f~%6.1fs  %3d 帧  离最近真人 %.1fm" % (s0, s1, n, mx))

    print()
    print("=" * 82)
    print("看结论的口径：")
    print("  · '仅有效归属' 的 p99 才是真正的能力上限；p50 才是稳态精度")
    print("  · 超差帧若聚成少数几段 -> 是遮挡/摆动端点造成的事件，不是整段噪声")
    print("  · 无归属帧若同样聚成几段 -> 与超差帧同源，属同一事件")
    print("=" * 82)
    return 0


if __name__ == "__main__":
    sys.exit(main())
