#!/usr/bin/env python3
"""起飞前自检 —— 判断「当天随机图能不能安全起飞」。

背景（官方 map_generator.py 的两个已实测缺陷）：
  A. fallback 分支：create_point 连败 10 次后走 fallback，content.readlines()
     被读第二次返回空 => house.world / robocup.world 写成 0 字节。
     实测本机 2/8、VM 4/6~6/6 不等（不是每张图都会中）。
  B. 建筑压起飞点：create_point() 只检查建筑之间不重叠，**完全不检查与
     官方起飞点 (0,±3)/(3,±3) 的距离**（而行人放置有 8m 避让）。
     实测 3/12 = 25% 把 30x25 的 fast_food_93 直接盖在起飞点上。

本脚本的作用：把这两类「开局即死」在起飞**之前**查出来，并给出可用的
起飞点（或可用的 world）。比赛现场跑一次即可，不影响任何算法。

用法：
  python3 preflight_check.py --gen-dir ~/robocup_real/_genvm      # 批检
  python3 preflight_check.py --bb <black_box.txt> --world <robocup.world>
退出码：0 = 可用；2 = 起飞点被压（给出建议点）；3 = world 空；4 = 文件缺失。
"""
import argparse
import math
import os
import sys

DEFAULT_TAKEOFF = [(0.0, -3.0), (3.0, -3.0), (0.0, 0.0),
                   (3.0, 0.0), (0.0, 3.0), (3.0, 3.0)]


def load_boxes(bb_path):
    here = os.path.dirname(os.path.abspath(__file__))
    for c in (here,
              os.environ.get('RADAR_DEMO_DIR', ''),
              os.path.expanduser('~/robocup_real/_radar_test'),
              os.path.expanduser('~/robocup_real/_demo2026'),
              os.path.expanduser('~/robocup_real/_sync_in')):
        if c and os.path.isdir(c):
            sys.path.insert(0, c)
    import astar_plan as A
    return A.boxes_from_black_box(bb_path)


def blocker(boxes, p, pad=0.0):
    """返回压住 p 的障碍名（含 pad 膨胀），没有则 None。"""
    for b in boxes:
        if abs(p[0] - b[1]) <= b[3] + pad and abs(p[1] - b[2]) <= b[4] + pad:
            return b[0]
    return None


def clearance(boxes, p):
    """点到最近障碍表面的距离。"""
    d = 1e9
    for b in boxes:
        dx = max(abs(p[0] - b[1]) - b[3], 0.0)
        dy = max(abs(p[1] - b[2]) - b[4], 0.0)
        d = min(d, math.hypot(dx, dy))
    return d


def spiral_free(boxes, p, pad=0.5, step=0.5, rmax=30.0):
    """从 p 出发螺旋外扩，找第一个不压障碍的点。"""
    if blocker(boxes, p, pad) is None:
        return p
    r = step
    while r <= rmax:
        n = max(8, int(2 * math.pi * r / step))
        for i in range(n):
            a = 2 * math.pi * i / n
            q = (p[0] + r * math.cos(a), p[1] + r * math.sin(a))
            if blocker(boxes, q, pad) is None:
                return q
        r += step
    return None


def check_one(name, world, bb, takeoff=DEFAULT_TAKEOFF, quiet=False):
    """返回 dict：ok / reason / clean_pts / suggest。"""
    out = {'name': name, 'world': world, 'bb': bb}
    if not os.path.exists(world) or os.path.getsize(world) == 0:
        out['ok'] = False
        out['reason'] = 'world 0 字节或缺失（官方 fallback bug）'
        return out
    if not os.path.exists(bb) or os.path.getsize(bb) == 0:
        out['ok'] = False
        out['reason'] = 'black_box.txt 缺失/空'
        return out
    boxes = load_boxes(bb)
    out['n_box'] = len(boxes)
    clean, blocked = [], []
    for t in takeoff:
        nm = blocker(boxes, t)
        (blocked if nm else clean).append((t, nm) if nm else t)
    out['clean_pts'] = clean
    out['blocked_pts'] = blocked
    out['min_clear_takeoff'] = min(clearance(boxes, t) for t in clean) if clean else 0.0
    if clean:
        # 挑净空最大的点当推荐起飞点（净空 <1.0m 时风险偏高）
        best = max(clean, key=lambda t: clearance(boxes, t))
        bc = clearance(boxes, best)
        out['best_pt'] = (round(best[0], 1), round(best[1], 1))
        out['best_clear'] = round(bc, 2)
        out['ok'] = True
        out['reason'] = '可用（%d/%d 个起飞点干净%s）' % (
            len(clean), len(takeoff), '，⚠ 净空偏紧' if bc < 1.0 else '')
    else:
        sug = []
        for t, _nm in blocked:
            q = spiral_free(boxes, t)
            if q:
                sug.append((t, (round(q[0], 1), round(q[1], 1))))
        out['ok'] = False
        out['reason'] = '⛔ 6 个起飞点全被建筑压住（官方生成器缺陷 B）'
        out['suggest'] = sug
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gen-dir', default=os.path.expanduser('~/robocup_real/_genvm'),
                    help='批检模式：扫描该目录下 try*/ 子目录')
    ap.add_argument('--bb', help='单检模式：black_box.txt 路径')
    ap.add_argument('--world', help='单检模式：robocup.world 路径')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    if a.bb and a.world:
        items = [(a.world, a.bb)]
        single = True
    else:
        items = []
        if os.path.isdir(a.gen_dir):
            for d in sorted(os.listdir(a.gen_dir)):
                sub = os.path.join(a.gen_dir, d)
                if not os.path.isdir(sub):
                    continue
                w = os.path.join(sub, 'robocup.world')
                bb = os.path.join(sub, 'black_box.txt')
                if os.path.exists(w) or os.path.exists(bb) \
                        or os.path.exists(os.path.join(sub, 'obstacle.txt')):
                    items.append((w, bb))
        single = False

    if not items:
        print('没有可检的图（--gen-dir 下无 try*/）')
        return 4

    results = []
    for i, (w, bb) in enumerate(items):
        nm = os.path.basename(os.path.dirname(w)) if not single else os.path.basename(w)
        r = check_one(nm, w, bb)
        results.append(r)
        if a.json:
            continue
        flag = '✅' if r.get('ok') else '⛔'
        print('%s %-8s %-44s' % (flag, r['name'], r['reason']))
        if r.get('n_box'):
            print('      障碍框 %d   起飞点最小净空 %.2f m'
                  % (r['n_box'], r.get('min_clear_takeoff', 0)))
            if r.get('best_pt'):
                print('      推荐起飞点 %s（净空 %.2f m）'
                      % (r['best_pt'], r.get('best_clear', 0)))
        for t, bnm in r.get('blocked_pts', []):
            print('      被压: (%.0f,%.0f) <- %s' % (t[0], t[1], bnm))
        for old, new in r.get('suggest', []):
            print('      建议改起飞点: (%.0f,%.0f) -> %s' % (old[0], old[1], new))

    if a.json:
        import json
        print(json.dumps(results, ensure_ascii=False, indent=1, default=str))
        return 0

    nok = sum(1 for r in results if r.get('ok'))
    print()
    print('=' * 70)
    print('汇总：%d/%d 张图可正常起飞' % (nok, len(results)))
    bad = [r for r in results if not r.get('ok')]
    if bad:
        print('不可用：')
        for r in bad:
            print('  · %s  %s' % (r['name'], r['reason']))
    print('结论：%s' % ('✅ 当天随机图可用，直接飞'
                      if nok == len(results) else
                      '⚠ 有不通过项 —— 按上面「建议改起飞点」换点，或让官方重新生成'))
    return 0 if nok == len(results) else 2


if __name__ == '__main__':
    sys.exit(main())
