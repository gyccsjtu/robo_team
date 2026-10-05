#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采样瞬移台状态文件，统计「整轮里无人机有多少时间真的在观测点上」。

为什么需要它
------------
`m3_ann_run.sh` 原先只在开头"等 15 s"就开跑验收；而瞬移台日志里的
`实距=13.00m` 是 `|目标−观测点|` 的**纯算术恒等式**，恒等于 keep，
跟无人机在哪毫无关系。于是"无人机在不在观测点"从来不是已知项。

实测代价（brown 轮）：无人机前 55 s（占整轮 41%）趴在起飞点、
离目标 75 m，而日志全程打印"实距=13.00m 视线=通"。**整轮数据作废。**

用法: ht_watch.py [dur_s=120] [out_json=/tmp/ht_watch.json] [status=/tmp/ht_status.json]
输出: {"samples","bad","bad_frac","off_max","longest_bad_s","first_good_s"}
"""
import json
import os
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/ht_watch.json"
STATUS = sys.argv[3] if len(sys.argv) > 3 else "/tmp/ht_status.json"
TOL = float(os.environ.get("HT_OFF", "2.0"))


def summarize(samples, tol=TOL):
    """samples: [(t, off)]，t 为相对秒。返回统计字典。

    纯函数，可离线单测。
    """
    res = {"samples": len(samples), "bad": 0, "bad_frac": 0.0,
           "off_max": 0.0, "longest_bad_s": 0.0, "first_good_s": None}
    if not samples:
        return res
    res["off_max"] = max(o for _, o in samples)
    cur = 0.0
    for t, o in samples:
        if o > tol:
            res["bad"] += 1
            cur += 1.0
            res["longest_bad_s"] = max(res["longest_bad_s"], cur)
        else:
            cur = 0.0
            if res["first_good_s"] is None:
                res["first_good_s"] = t
    res["bad_frac"] = round(1.0 * res["bad"] / len(samples), 3)
    return res


def main():
    samples = []
    last = {}
    t0 = time.time()
    while time.time() - t0 < DUR:
        try:
            with open(STATUS) as f:
                d = json.load(f)
            last = d
            samples.append((round(time.time() - t0, 1), float(d.get("off", 9e9))))
        except Exception:
            pass
        time.sleep(1.0)
    r = summarize(samples)
    # 瞬移台**自己**每秒也在量，它的自计比外部 1 Hz 采样可靠
    # （实测有一次 23.5m 的短时脱位被外部采样恰好错过）
    for k in ("meas_n", "bad_cnt", "off_max", "long_bad_s",
              "aim_deg", "aim_max", "aim_bad_cnt"):
        if k in last:
            r["self_" + k] = last[k]
    try:
        with open(OUT, "w") as f:
            json.dump(r, f, ensure_ascii=False)
    except Exception:
        pass
    if r["samples"]:
        print("[watch] 外部采样 %d 次 | 不在观测点 %d 次 (%.0f%%) | 最大偏差 %.1fm | "
              "最长连续偏离 %.0fs | 首次到位 %s"
              % (r["samples"], r["bad"], 100.0 * r["bad_frac"],
                 r["off_max"], r["longest_bad_s"],
                 "未到位" if r["first_good_s"] is None else "%.0fs" % r["first_good_s"]),
              flush=True)
    if "self_meas_n" in r:
        print("[watch] 瞬移台自计 %d 次 | 不在观测点 %d 次 (%.0f%%) | 最大偏差 %.1fm | "
              "最长连续偏离 %.0fs"
              % (r["self_meas_n"], r["self_bad_cnt"],
                 100.0 * r["self_bad_cnt"] / max(1, r["self_meas_n"]),
                 r["self_off_max"], r["self_long_bad_s"]), flush=True)
    # 瞄准误差：位置钉得再牢，相机没对着目标也白搭（实测 0.9°/s 漂移 + ±170° 失控）
    if r.get("self_aim_max") is not None:
        print("[watch] 瞄准误差 最大 %.1f° | 超 15° 的秒数 %d | 末次 %.1f°"
              % (r["self_aim_max"], r.get("self_aim_bad_cnt", 0),
                 -1.0 if r.get("self_aim_deg") is None else r["self_aim_deg"]), flush=True)


if __name__ == "__main__":
    main()
