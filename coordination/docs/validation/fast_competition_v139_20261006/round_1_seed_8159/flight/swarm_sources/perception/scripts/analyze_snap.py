#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线归因：读 rec_snap.py 落盘的 jsonl，回答"这一类为什么报错"。

用法：
    python3 analyze_snap.py /tmp/snap_a2.jsonl [类别]

不指定类别时，对落盘里出现过的每一类都做一遍。

输出的是"能被否证的数据"，不是猜测。上一轮我对 brown 的判断只看了几帧
就下了结论（"被认成 red"），结果被混淆矩阵和身份对照诊断双双否掉 ——
这个脚本就是为防止同类错误而写的：**先把每帧被选中的 track 的属性量化**，
再决定改哪里。
"""
import json
import math
import sys
from collections import Counter

# 官方 actor_id_dict —— 单流类的"该报谁"以它为准
CLASS_ACTORS = {"green": [0], "blue": [1], "brown": [2], "white": [3], "red": [4, 5]}


def med(a):
    a = sorted(a)
    return a[len(a) // 2] if a else float("nan")


def q(a, p):
    a = sorted(a)
    if not a:
        return float("nan")
    return a[min(len(a) - 1, int(len(a) * p))]


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except Exception:
                pass
    return rows


def brief(dets, key):
    if key == "conf":
        return med([d.get("conf", float("nan")) for d in dets])
    if key == "s_ema":
        return med([d.get("s_ema", float("nan")) for d in dets])
    if key == "h":
        return med([d.get("h", float("nan")) for d in dets])
    if key == "sp":
        return med([d.get("sp", float("nan")) for d in dets])
    if key == "hits":
        return med([d.get("hits", float("nan")) for d in dets])
    if key == "range":
        return med([d.get("range", float("nan")) for d in dets])
    if key == "coast":
        return med([d.get("coast", float("nan")) for d in dets])
    return float("nan")


def pick_target(tag, dets):
    """这一"流"该报谁。

    ⚠ 不能用"最常靠近哪个 actor"来定 —— 误检一旦占多数，它会指向错误的目标，
    结论会整个反掉（自测时踩过：97% 静止误检让 brown 的目标被定成 a5）。
    单流类**以官方 actor_id_dict 映射为准**，多流类才退化为数据推断。
    """
    base = tag.rstrip("0123456789")
    ids = CLASS_ACTORS.get(base)
    if ids and len(ids) == 1:
        return "a%d" % ids[0], "官方映射"

    def nearest(d):
        ds = {k: v for k, v in (d.get("dist") or {}).items() if v is not None}
        return min(ds, key=ds.get) if ds else None

    hits = [nearest(d) for d in dets]
    hits = [h for h in hits if h is not None]
    # 多流类：优先用"真的命中过(≤1m)"的众数
    strict = []
    for d in dets:
        ds = {k: v for k, v in (d.get("dist") or {}).items() if v is not None}
        if ds:
            k = min(ds, key=ds.get)
            if ds[k] <= 1.0:
                strict.append(k)
    if strict:
        return Counter(strict).most_common(1)[0][0], "命中众数"
    if hits:
        return Counter(hits).most_common(1)[0][0], "最近者众数(该流从未命中, 可疑)"
    return None, "无法判定"


def analyze_cls(tag, dets, allcls):
    """tag: 'brown' 或 'red1'（red 两条流分开看）"""
    if not dets:
        return
    target, how = pick_target(tag, dets)
    if target is None:
        print("--- %s: 没有可比对的真值（落盘时 /actor_N/pose 缺失）---" % tag)
        return

    hit, miss = [], []
    for d in dets:
        dv = (d.get("dist") or {}).get(target)
        if dv is None:
            continue
        (hit if dv <= 1.0 else miss).append((d, dv))

    n = len(hit) + len(miss)
    print("\n--- %s （目标 %s，来自%s；命中 %d/%d 帧）---" % (tag, target, how, len(hit), n))
    print("  发布条目 %d  命中(≤1m) %d (%.1f%%)  超差 %d"
          % (n, len(hit), 100.0 * len(hit) / max(n, 1), len(miss)))
    for nm, grp in (("命中组", hit), ("超差组", miss)):
        if not grp:
            continue
        dd = [g[0] for g in grp]
        print("  【%s】conf %.2f | 身高 %.2fm | 净速度 %.2fm/s | hits %.0f | "
              "score_ema %.2f | 量程 %.1fm | coast %.0f"
              % (nm, brief(dd, "conf"), brief(dd, "h"), brief(dd, "sp"),
                 brief(dd, "hits"), brief(dd, "s_ema"), brief(dd, "range"),
                 brief(dd, "coast")))

    # 2) 超差组的坐标到底落在哪 —— 这是"静态误检"vs"别人的真身"的分水岭
    if miss:
        to_tgt = [g[1] for g in miss]
        to_other = []
        other_hit = 0
        for d, dv in miss:
            ds = (d.get("dist") or {})
            cands = [(v, k) for k, v in ds.items()
                     if v is not None and k != target]
            if not cands:
                continue
            v, k = min(cands)
            to_other.append(v)
            if v <= 1.0:
                other_hit += 1
        print("  超差组到本流目标的距离: 中位 %.1fm  P90 %.1fm  最小 %.1fm"
              % (med(to_tgt), q(to_tgt, 0.9), min(to_tgt)))
        if to_other:
            print("  超差组到【最近的其它 actor】的距离: 中位 %.1fm  最小 %.1fm  "
                  "≤1m 占 %.1f%%（高→「别人的真身被误分类」）"
                  % (med(to_other), min(to_other),
                     100.0 * other_hit / len(to_other)))
    # 3) 结论
    if miss and hit:
        sp_h, sp_m = brief([g[0] for g in hit], "sp"), brief([g[0] for g in miss], "sp")
        c_h, c_m = brief([g[0] for g in hit], "conf"), brief([g[0] for g in miss], "conf")
        s_h, s_m = brief([g[0] for g in hit], "s_ema"), brief([g[0] for g in miss], "s_ema")
        print("  ⇒ 判读: ", end="")
        if sp_m < 0.35 <= sp_h:
            print("超差组几乎不动（净速度 %.2f vs %.2f）→ **静止误检抢槽**；"
                  "命中组自己是在动的真人。" % (sp_m, sp_h))
        elif s_m > s_h:
            print("超差组的 score_ema 反而更高（%.2f vs %.2f）→ 选优打分把它排在了"
                  "真目标前面，需要引入'粘滞/运动优先'而不是纯打分。" % (s_m, s_h))
        elif c_m < c_h:
            print("超差组 conf 更低（%.2f vs %.2f）却仍然被选中 → 选优键有问题。"
                  % (c_m, c_h))
        else:
            print("命中/超差两组的属性差异不明显 → 更像是**遮挡导致的跟丢后外推**，"
                  "不是选优问题。")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    only = sys.argv[2] if len(sys.argv) > 2 else None
    rows = load(path)
    print("=" * 84)
    print("发布决策归因: %s   共 %d 帧" % (path, len(rows)))
    if rows:
        print("时间跨度 %.1f s（仿真）" % (rows[-1].get("t", 0) - rows[0].get("t", 0)))
    print("=" * 84)

    # 按 (cls, tag) 分开：red 的两条流要分别看
    groups = {}
    for r in rows:
        for d in r.get("dets", []):
            key = d.get("tag", d.get("cls", "?"))
            groups.setdefault(key, []).append(d)
    if not groups:
        print("落盘里没有任何 dets —— 感知可能没在发调试快照 /perception/debug_snapshot"
              "（注意：/coordination/target_report 自 2026-09-20 起是协同契约，不含 dets）")
        return 1
    print("出现过的流: %s" % ", ".join("%s(%d)" % (k, len(v)) for k, v in sorted(groups.items())))
    for tag in sorted(groups):
        if only and only not in tag:
            continue
        analyze_cls(tag, groups[tag], groups)
    print("\n" + "=" * 84)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
