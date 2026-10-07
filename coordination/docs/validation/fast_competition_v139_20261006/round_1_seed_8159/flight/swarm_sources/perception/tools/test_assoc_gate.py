#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""perception_real.assoc_gate 的离线单测（不 import rospy/torch，用 AST 抠函数）。

门限的取值不是拍脑袋，而是被 brown 干净轮的实测数据钉死的：
  · 真人连续跟踪：帧间世界位移 中位 0.098 m / p99 0.501 / 最大 0.577（dt≈0.11 s）
  · 含误检段：13 次 >1 m，最大 3.29 m，其中 11 次 dt≤0.15 s
所以本测试同时锁住"真人一定过得去"和"3 m 级跳变一定被拒"两侧。
"""
import ast
import os

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "perception_real.py")
tree = ast.parse(open(SRC, encoding="utf-8").read())
# 默认参数在"定义时"求值，所以必须先把模块级的常量按原样喂进命名空间。
# 直接抠模块里的常量赋值（而不是在测试里抄一遍数字），改了感知参数这里自动跟上。
WANT = {"CONFIRM_HITS", "GATE", "GATE_MIN", "GATE_MULT", "GATE_DYN"}
const_nodes = [n for n in tree.body if isinstance(n, ast.Assign)
               and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
               and n.targets[0].id in WANT]
ns = {"os": os, "float": float, "int": int, "min": min, "max": max}
exec(compile(ast.Module(body=const_nodes, type_ignores=[]), "consts", "exec"), ns)
fn = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "assoc_gate"]
assert fn, "没找到 assoc_gate"
assert WANT <= set(ns), "常量没抠全: %s" % (WANT - set(ns))
exec(compile(ast.Module(body=fn, type_ignores=[]), "assoc_gate", "exec"), ns)
assoc_gate = ns["assoc_gate"]
print("（测试读到的模块常量：%s）"
      % ", ".join("%s=%r" % (k, ns[k]) for k in sorted(WANT)))

FAIL = []
N = [0]


def check(name, got, want):
    N[0] += 1
    ok = (abs(got - want) < 1e-6) if isinstance(want, (int, float)) else (got == want)
    if not ok:
        FAIL.append("%s: got %r want %r" % (name, got, want))
    print("  %-52s %s" % (name, "OK" if ok else "FAIL got=%r want=%r" % (got, want)))


print("--- assoc_gate：已确认 track 按 dt 缩放 ---")
# 1) dt=0.11（实测中位）：门限 1.02 m —— 真人最大跳变 0.577 必须过得去
g = assoc_gate(0.11, hits=92)
check("dt=0.11 gate", round(g, 4), 1.02)
check("真人最大跳变 0.577 通过", g > 0.577, True)
# 2) C 轮那次致命的 3.1 m 跳变（dt≈0.1）必须被拒
check("误检 3.1m 跳变被拒", assoc_gate(0.10, hits=92) < 3.1, True)
check("误检 3.29m 跳变被拒", assoc_gate(0.11, hits=92) < 3.29, True)
# 3) 误检段里 1.32 m(p95) / 2.97 m(p99) 的跳变也应被拒
check("1.32m 跳变被拒", assoc_gate(0.11, hits=92) < 1.32, True)
check("2.97m 跳变被拒", assoc_gate(0.11, hits=92) < 2.97, True)
# 4) dt 变大要放宽（检出掉帧时不能把真人一起拒了）
check("dt=0.2 gate", round(assoc_gate(0.20, hits=92), 4), 1.2)
check("dt=0.5 gate", round(assoc_gate(0.50, hits=92), 4), 1.8)
# 5) 不超过固定 GATE
check("dt=5.0 封顶在 3.0", assoc_gate(5.0, hits=92), 3.0)
# 6) 未确认 track 不受影响（走固定 GATE）
check("hits=1 用固定 GATE", assoc_gate(0.11, hits=1), 3.0)
# 7) 可以整条关掉（A/B 对照）
check("GATE_DYN=0 退回固定", assoc_gate(0.11, hits=92, dyn=0), 3.0)
# 8) 下限保护：dt 极小时不塌到 0
check("dt=0.001 不低于 GATE_MIN", assoc_gate(0.001, hits=92) >= 0.8, True)

print("\n%d 项，失败 %d 项" % (N[0], len(FAIL)))
for f in FAIL:
    print("  FAIL " + f)
raise SystemExit(1 if FAIL else 0)
