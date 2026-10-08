#!/usr/bin/env python3
"""Accuracy of what we actually published vs the actor's true position.

The judge resets its 15 s counter whenever a received coordinate is off by more
than 1 m, so continuity and accuracy are separate gates. This measures the
second one on real data: every official report we sent, and every internal
observation, against the actor truth stream the independent observer sampled
via /gazebo/get_model_state.

Pairing rule matches the observer's own: linear interpolation between truth
samples whose gap is <= 0.2 s.

Run: python3 accuracy_report.py --round <round_dir> [--tag green]
"""
import argparse
import bisect
import json
import math
import os
import sys


def load_rows(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass
    return rows


def metrics(errors):
    ok = [e for e in errors if e is not None]
    if not ok:
        return dict(n=0)
    ok_sorted = sorted(ok)
    return dict(
        n=len(ok),
        mean=sum(ok) / len(ok),
        p50=ok_sorted[len(ok) // 2],
        p90=ok_sorted[min(len(ok) - 1, int(len(ok) * 0.9))],
        max=ok_sorted[-1],
        min=ok_sorted[0],
        frac_under_1m=sum(1 for e in ok if e < 1.0) / len(ok),
        n_over_1m=sum(1 for e in ok if e >= 1.0),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--round", required=True)
    ap.add_argument("--tag", default="green")
    ap.add_argument("--bridge-trace", default="")
    args = ap.parse_args()
    rnd = args.round.rstrip("/")
    name = ("independent_visual_accuracy" if args.tag == "green"
            else "independent_visual_accuracy_" + args.tag)
    path = os.path.join(rnd, "observers", name + ".jsonl")
    if not os.path.isfile(path):
        print("no observer file:", path)
        return 1
    rows = load_rows(path)
    truth = sorted([(float(r["sample"][0]), float(r["sample"][1]), float(r["sample"][2]))
                    for r in rows if r.get("kind") == "actor_truth"])
    official = [(float(r["sample"][0]), float(r["sample"][1]), float(r["sample"][2]))
                for r in rows if r.get("kind") == "official"]
    # image rows carry the whole observation payload as `sample`
    image = []
    for r in rows:
        if r.get("kind") != "image":
            continue
        s = r.get("sample")
        if isinstance(s, dict) and isinstance(s.get("xyz"), (list, tuple)):
            image.append((float(s.get("sample_s")), float(s["xyz"][0]), float(s["xyz"][1])))
        elif isinstance(s, (list, tuple)) and len(s) >= 3:
            image.append((float(s[0]), float(s[1]), float(s[2])))

    print("tag=%s  truth samples=%d  official=%d  internal image observations=%d"
          % (args.tag, len(truth), len(official), len(image)))
    if not truth:
        print("no truth; nothing to compare")
        return 1
    ts = [t[0] for t in truth]

    def err_at(t, x, y):
        i = bisect.bisect_left(ts, t)
        if i == 0 or i >= len(truth):
            return None
        a, b = truth[i - 1], truth[i]
        gap = b[0] - a[0]
        if not 0 < gap <= 0.2:
            return None
        f = (t - a[0]) / gap
        ix, iy = a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])
        return math.hypot(x - ix, y - iy)

    off_err = [err_at(t, x, y) for t, x, y in official]
    img_err = [err_at(t, x, y) for t, x, y in image]

    print()
    print("=== 官方上报坐标 vs 演员真值（裁判就是拿这个判 <1 m）===")
    print(json.dumps(metrics(off_err), ensure_ascii=False, indent=1))
    print()
    print("=== 内部观测坐标（/swarm/visual_observation）vs 真值 ===")
    print(json.dumps(metrics(img_err), ensure_ascii=False, indent=1))

    print()
    print("=== 逐条官方上报 ===")
    for (t, x, y), e in zip(official, off_err):
        print("   t=%.3f published=(%.2f, %.2f) truth_err=%s"
              % (t, x, y, ("%.3f m" % e) if e is not None else "no truth pair"))

    bt = args.bridge_trace
    if bt and os.path.isfile(bt):
        print()
        print("=== 桥发出 vs 真值（requested_xy，含外推补偿）===")
        rows = load_rows(bt)
        for r in rows:
            if r.get("kind") != "upload":
                continue
            xy = r.get("requested_xy") or []
            if len(xy) != 2:
                continue
            e = err_at(float(r["receipt_s"]), xy[0], xy[1])
            print("   t=%.3f tag=%s requested=(%.2f, %.2f) fused=(%.2f, %.2f) err=%s"
                  % (float(r["receipt_s"]), r.get("tag"), xy[0], xy[1],
                     (r.get("fused_xy") or [0, 0])[0], (r.get("fused_xy") or [0, 0])[1],
                     ("%.3f m" % e) if e is not None else "n/a"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
