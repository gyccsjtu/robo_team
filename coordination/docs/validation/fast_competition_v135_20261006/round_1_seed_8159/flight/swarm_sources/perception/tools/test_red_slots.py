#coding: utf-8
"""red 双流槽位绑定的单元测试（不需要 ROS / 不需要仿真）。

为什么单独测这个：槽位绑定一旦在帧间发生互换，裁判的 red callback 就会把
"连续 15 s"计时反复清零，M5 直接零分 —— 而这类 bug 在仿真里只表现为
"命中率莫名很低"，极难定位。所以先用纯逻辑把边界钉死。

跑法：
    python test_red_slots.py
"""
import ast
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "perception_real.py")


def load_assign_slots(path):
    """用 AST 把 assign_slots 抠出来单独执行 —— 这样不必 import rospy/cv2。"""
    with io.open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    fn = None
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "assign_slots":
            fn = node
            break
    if fn is None:
        raise SystemExit("没找到 assign_slots")
    mod = ast.Module(body=[fn], type_ignores=[])
    ast.fix_missing_locations(mod)
    ns = {"math": math}          # 被抠出来的函数只用 math，不碰 rospy/cv2
    exec(compile(mod, path, "exec"), ns)
    return ns["assign_slots"]


class T(object):
    """最小 Track 替身：assign_slots 只用 id / hits / score_ema / x / y 这几个字段。"""
    def __init__(self, tid, hits=10, score=0.5, x=0.0, y=0.0):
        self.id = tid
        self.hits = hits
        self.score_ema = score
        self.x = x
        self.y = y


def main():
    assign = load_assign_slots(SRC)
    CONFIRM = 5
    fails = []

    def check(name, got, want):
        ok = got == want
        print("%-46s %s" % (name, "ok" if ok else "FAIL  got=%s want=%s" % (got, want)))
        if not ok:
            fails.append(name)

    # --- 1) 首次绑定：两个候选各占一槽，按分降序 ---
    a, b = T(1, 10, 0.80), T(2, 10, 0.70)
    slots = [None, None]
    got = assign([b, a], slots, CONFIRM)        # 故意乱序传入
    check("首次绑定：高分进槽0", [(s, t.id) for s, t in got], [(0, 1), (1, 2)])
    check("首次绑定：slots 落位", slots, [1, 2])

    # --- 2) 关键用例：下一帧分数反转，绑定必须不动 ---
    a2, b2 = T(1, 11, 0.30), T(2, 11, 0.90)     # 槽1 的分反而更高了
    got = assign([b2, a2], slots, CONFIRM)
    check("分数反转后绑定不互换（核心）", [(s, t.id) for s, t in got], [(0, 1), (1, 2)])
    check("分数反转后 slots 不变", slots, [1, 2])

    # --- 3) 槽0 的 track 消失 -> 释放并由新 track 顶上，槽1 不动 ---
    b3, c3 = T(2, 12, 0.60), T(3, 9, 0.95)
    got = assign([b3, c3], slots, CONFIRM)
    check("槽0 空出后由最高分补上", [(s, t.id) for s, t in got], [(0, 3), (1, 2)])
    check("槽0 空出后 slots", slots, [3, 2])

    # --- 4) 只剩一个候选 -> 只发一条流，另一槽保持空 ---
    got = assign([T(3, 13, 0.9)], slots, CONFIRM)
    check("单候选只占一槽", [(s, t.id) for s, t in got], [(0, 3)])
    check("单候选时另一槽为空", slots, [3, None])

    # --- 5) 三个候选不会撑破槽数 ---
    slots2 = [None, None]
    got = assign([T(1, 9, .9), T(2, 9, .8), T(3, 9, .7)], slots2, CONFIRM)
    check("候选多于槽数时只绑前 N 个", [(s, t.id) for s, t in got], [(0, 1), (1, 2)])

    # --- 6) 已确认的 track 优先于未确认的高分 track ---
    slots3 = [None, None]
    got = assign([T(7, 1, 0.99), T(8, 20, 0.50)], slots3, CONFIRM)
    check("已确认优先于未确认高分", [(s, t.id) for s, t in got], [(0, 8), (1, 7)])

    # --- 7) 同一个 track 不会被绑到两个槽 ---
    slots4 = [5, None]                          # 槽0 锁着 5 号
    got = assign([T(5, 10, 0.9)], slots4, CONFIRM)
    check("一个 track 至多占一个槽", [(s, t.id) for s, t in got], [(0, 5)])

    # --- 8) 空候选：两槽全空，且旧绑定被清理 ---
    slots5 = [5, 6]
    got = assign([], slots5, CONFIRM)
    check("无候选时全部释放", (got, slots5), ([], [None, None]))

    # --- 9) 长序列稳定性：连续 200 帧两目标交叉移动，绑定不许换 ---
    slots6 = [None, None]
    swap = 0
    prev = None
    for f in range(200):
        s1 = 0.5 + 0.4 * ((f // 3) % 2)         # 周期性让两者分数交替领先
        s2 = 0.5 + 0.4 * (1 - (f // 3) % 2)
        got = assign([T(11, 30, s1), T(12, 30, s2)], slots6, CONFIRM)
        cur = tuple(t.id for _, t in got)
        if prev is not None and cur != prev:
            swap += 1
        prev = cur
    check("200 帧交叉打分零次互换", swap, 0)

    # --- 10) 空间连续性：空槽补位优先挑"离该槽最后位置近"的，而不是分高的 ---
    #     这条是被实测逼出来的：track 重建后按分数补位会落到**另一个红衣人**身上，
    #     于是 red1 流一会儿指 actor_4、一会儿指 actor_5（实测归属 261:224 对半）。
    s = [None, None]
    xy = [(10.0, 10.0), (40.0, 10.0)]          # 两个槽的最后已知位置
    near = T(21, 10, 0.30, 10.5, 10.2)         # 离槽0 很近，但分低
    far = T(22, 10, 0.95, 50.0, 40.0)          # 分最高，但离谁都远
    got = assign([far, near], s, CONFIRM, xy, 6.0)
    check("空间连续性优先于分数（核心）", [(k, t.id) for k, t in got], [(0, 21), (1, 22)])

    # --- 11) 空间连续性只对"新补位"生效：超出半径就退回按分数 ---
    s2 = [None, None]
    xy2 = [(10.0, 10.0), (40.0, 10.0)]
    got = assign([T(31, 10, 0.90, 70.0, 60.0), T(32, 10, 0.80, 75.0, 65.0)], s2, CONFIRM, xy2, 6.0)
    check("超出半径后按分数补位", [(k, t.id) for k, t in got], [(0, 31), (1, 32)])

    # --- 12) 绑定成功后 slot_xy 要刷新到当前坐标 ---
    s3 = [None, None]
    xy3 = [None, None]
    assign([T(41, 10, 0.9, 1.0, 2.0), T(42, 10, 0.8, 3.0, 4.0)], s3, CONFIRM, xy3, 6.0)
    check("slot_xy 已刷新", [tuple(round(v, 1) for v in p) for p in xy3],
          [(1.0, 2.0), (3.0, 4.0)])

    # --- 13) 关闭空间连续性（sticky_r=0）时行为与老版本一致 ---
    s4 = [None, None]
    xy4 = [(10.0, 10.0), (40.0, 10.0)]
    got = assign([T(51, 10, 0.30, 10.5, 10.2), T(52, 10, 0.95, 50.0, 40.0)], s4, CONFIRM, xy4, 0.0)
    check("sticky_r=0 退回纯分数", [(k, t.id) for k, t in got], [(0, 52), (1, 51)])

    print("-" * 60)
    if fails:
        print("失败 %d 项: %s" % (len(fails), fails))
        return 1
    print("全部 %d 项通过" % 13)
    return 0


if __name__ == "__main__":
    sys.exit(main())
