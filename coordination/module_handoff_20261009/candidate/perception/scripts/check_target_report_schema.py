#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用**队友自己的严格校验器**验我方 /coordination/target_report 的载荷形状。

为什么要单独写这个：协同核心的 schema 校验是 fail-closed 的 ——
`protocol.fields()` 一旦发现字段集合不匹配就抛 `SCHEMA_FIELDS`，
而 `coordination_node._submit()` 只打一条 `rospy.logwarn` 就丢掉这条消息。
不会崩、不会报错退出、话题上照样有消息在流 —— 现象就是"核心什么都收不到、
什么任务都不分配"，极难从表面看出来。

本脚本不依赖 ROS，直接 import 队友的 coordination/protocol.py 中的 SPECS 与 fields(),
对**我方发布端真实会发出去的那个 dict** 做校验，同时对照旧形状证明缺陷真实存在。

用法：
    python3 check_target_report_schema.py
退出码 0 = 新形状通过 / 旧形状被正确拒绝。
"""
import json
import os
import sys

# 队友的 coordination 包（纯 Python，无 ROS 依赖）
def _find_coord_pkg():
    """定位队友的 coordination 包（纯 Python，无 ROS 依赖）。

    权威校验收在队友手里，所以这里不复制一份 SPECS，而是**直接 import 他们的**，
    避免"我们以为的契约"和"核心实际校验的契约"出现分叉。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    rel = os.path.join("src", "robocup_navigation", "src",
                       "robocup_navigation", "coordination")
    candidates = [os.environ.get("ROBO_REPO_COORD")]
    # 从脚本所在目录逐级向上找 robocup_repo。
    # 不写死"向上几层"：脚本在仓库里挪位置（比如从仓库根挪进 perception/scripts/）
    # 之后相对路径会静默失效，而本脚本的降级行为只是打印提示 —— 不会报错，
    # 很容易被当成"跑过了"。逐级查找能从根上避免这种假绿。
    d = here
    for _ in range(6):
        parent = os.path.dirname(d)
        if not parent or parent == d:
            break
        d = parent
        candidates.append(os.path.join(d, "robocup_repo", rel))
    candidates.append(os.path.expanduser("~/team_ws/robocup/" + rel))
    for c in candidates:
        if c and os.path.isfile(os.path.join(c, "protocol.py")):
            return os.path.abspath(c)
    raise SystemExit(
        "[!] 没找到队友的 coordination 包（protocol.py）。\n"
        "    请用环境变量指定，例如：\n"
        "    export ROBO_REPO_COORD=/path/to/robocup_repo/" + rel + "\n")


COORD_PKG = _find_coord_pkg()
sys.path.insert(0, COORD_PKG)

from protocol import SPECS, CoordinationError, fields  # noqa: E402

SPEC = SPECS["TARGET_REPORT"]


def check(name, payload):
    """按核心的调用路径校验：fields(data, SPEC)。"""
    try:
        fields(payload, SPEC)
        return True, ""
    except CoordinationError as exc:
        return False, str(exc)
    except KeyError as exc:
        return False, "MISSING_KEYS:%s" % exc


def new_shape():
    """与 perception_real.py 里 coord.publish 的 dict **逐字段一致**。"""
    tag = "red1"          # 用 red 的槽位 tag，顺带验证带数字的 id
    tk_conf = 0.83
    tk_x, tk_y = -20.5, 4.25
    TARGET_Z = float(os.environ.get("PR_TARGET_Z", "1.25"))
    return {
        "target_id": tag,
        "frame_id": "world_enu",
        "xyz": [round(tk_x, 2), round(tk_y, 2), TARGET_Z],
        "confidence": max(0.0, min(1.0, float(tk_conf))),
        "observation_id": "obs-%s-%d" % (tag, 1)}


def source_payload_keys():
    """从**真实源码**里提取 coord.publish 那个 dict 的顶层键。

    为什么要这样：如果只在测试里手抄一份 payload，源码改了测试不会知道 ——
    测试照样绿、线上照样接不上。这里直接读源码，杜绝两边漂移。

    返回 (None, None) 表示没找到源文件 —— 此时跳过本项检查，**不算失败**，
    以便本脚本在只有发布端、没有源码的机器上也能跑通前面的契约校验。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.environ.get("PR_SOURCE"),
        os.path.join(here, "..", "perception_real.py"),        # 本仓库布局
        os.path.join(here, "..", "robocup_assets", "vm", "perception_real.py"),
        os.path.expanduser("~/robocup_real/perception_real.py"),
    ]
    src = next((os.path.abspath(c) for c in candidates
                if c and os.path.isfile(c)), None)
    if src is None:
        return None, None
    with open(src, encoding="utf-8") as fh:
        text = fh.read()
    start = text.find('coord.publish(String(data=json.dumps({')
    if start < 0:
        raise AssertionError("在 %s 里找不到 coord.publish 的 dict" % src)
    # 找到该 dict 的结束：第一个 '\n            })))'（缩进 12 空格 = dict 在 for 体内）
    body = text[start:]
    end = body.find('})))')
    assert end > 0, "找不到 coord.publish dict 的结尾"
    body = body[:end]
    import re
    keys = re.findall(r'"([A-Za-z_][A-Za-z0-9_]*)":', body)
    return src, sorted(set(keys))


def old_shape():
    """2026-09-20 之前真实在发的形状（保底复现用）。"""
    return {"stamp": 1758000000.0, "cam": [0.0] * 7, "uav": "typhoon_h480_0",
            "dets": [{"cls": "red", "xyto": [0, 0], "xyz": [-20.5, 4.25]}]}


def main():
    print("校验器: %s" % os.path.abspath(COORD_PKG))
    print("TARGET_REPORT 要求字段: %s" % sorted(SPEC))

    ok_new, err_new = check("new", new_shape())
    print("\n[新形状] %s" % json.dumps(new_shape(), ensure_ascii=False, sort_keys=True))
    print("  -> %s %s" % ("通过 ✅" if ok_new else "被拒 ❌", err_new))

    ok_old, err_old = check("old", old_shape())
    print("\n[旧形状] %s" % json.dumps(old_shape(), ensure_ascii=False, sort_keys=True))
    print("  -> %s %s" % ("通过" if ok_old else "被拒（预期）✅", err_old))

    # 额外边界：confidence 越界、xyz 二维，都应被拒
    edge = []
    p = new_shape(); p["confidence"] = 1.4
    edge.append(("confidence>1", p))
    p = new_shape(); p["xyz"] = [-20.5, 4.25]
    edge.append(("xyz 只有两维", p))
    p = new_shape(); p["frame_id"] = "map"
    edge.append(("frame_id 不是 world_enu", p))
    p = new_shape(); p["extra"] = 1
    edge.append(("多余字段", p))
    print("[边界用例] 全部**应当**被拒：")
    all_rejected = True
    for name, payload in edge:
        ok, err = check(name, payload)
        if ok:
            all_rejected = False
        print("  %-22s -> %s %s" % (name, "通过（有问题！）" if ok else "被拒 ✅", err))

    # —— 最关键的一步：直接核**真实源码**里的字段集合 ——
    src, keys = source_payload_keys()
    if src is None:
        keys_ok = True
        print("\n[源码对照] 跳过 —— 没找到 perception_real.py 源码")
        print("  （可用 PR_SOURCE=/path/to/perception_real.py 指定）")
    else:
        keys_ok = keys == sorted(SPEC)
        print("\n[源码对照] %s" % src)
        print("  coord.publish 实际字段 = %s" % keys)
        print("  契约要求字段        = %s" % sorted(SPEC))
        print("  -> %s" % ("完全一致 ✅" if keys_ok
                            else "不一致 ❌（测试与源码漂移，或契约已变）"))

    good = ok_new and (not ok_old) and all_rejected and keys_ok
    print("\n结论: %s" % ("新形状符合契约，旧形状确实接不上，源码字段与契约一致 ✅"
                          if good else "有不符项 ❌"))
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
