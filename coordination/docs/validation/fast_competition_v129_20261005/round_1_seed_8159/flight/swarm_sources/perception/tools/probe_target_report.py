#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""线上流量探针：订阅 /coordination/target_report，用**队友自己的**校验器逐条验。

为什么需要它（而不是再跑一遍静态 schema 测试）：
  * 静态测试只能证明"我打算发的形状是对的"；本探针证明"**线上真实在流的**
    每一条消息都能被核心的校验器接受"——中间还隔着 json.dumps、ROS 串行化、
    发布节流、字段取值等环节，任一处漂移静态测试都看不见。
  * `coordination_node._submit()` 对不合规载荷**只打一条 logwarn 就丢**，
    不崩不退出、话题照常流。所以"话题有流量"≠"核心收到了"。本探针把
    "通过/被拒 + 拒因直方图"变成可读数字。

同时统计一个核心语义点：core 的 `_target()` 是
    obs[key] = d
    if d['confidence'] < 1.: return      # v1 consumes confirmed reports
即**只有 confidence == 1.0 的报告才会驱动任务坐标更新**，其余只进
observations 台账。本探针会把"能驱动核心的条数"单独统计出来。

用法：
    python3 probe_target_report.py [时长秒=60] [话题=/coordination/target_report]
环境变量：
    TEAM_CORE_COORD  队友 coordination 包路径（默认 ~/team_core/robocup_navigation/coordination）
    PROBE_DUMP       把通过校验的原始载荷逐行落盘（jsonl），供离线重放用
"""
import collections
import json
import os
import sys
import time

import rospy
from std_msgs.msg import String

COORD = os.environ.get(
    "TEAM_CORE_COORD",
    os.path.expanduser("~/team_core/robocup_navigation/coordination"))
sys.path.insert(0, os.path.abspath(COORD))
from protocol import SPECS, CoordinationError, fields  # noqa: E402

SPEC = SPECS["TARGET_REPORT"]


class Probe(object):
    def __init__(self, topic):
        self.topic = topic
        self.n = 0
        self.ok = 0
        self.rej = collections.Counter()
        self.rej_example = {}
        self.tids = collections.Counter()
        self.obs_ids = collections.Counter()
        self.conf = []
        self.drive = 0            # confidence >= 1.0，能真正驱动核心的条数
        self.t_first = None
        self.t_last = None
        self.gaps = []
        self.raw_example = None
        self.dump_path = os.environ.get("PROBE_DUMP", "")
        self._dump = open(self.dump_path, "w", encoding="utf-8") if self.dump_path else None
        rospy.Subscriber(topic, String, self._cb, queue_size=200)

    def _cb(self, msg):
        now = time.time()
        self.n += 1
        if self.t_first is None:
            self.t_first = now
        else:
            self.gaps.append(now - self.t_last)
        self.t_last = now

        if self.raw_example is None:
            self.raw_example = msg.data

        try:
            d = json.loads(msg.data)
        except ValueError as exc:
            self._reject("BAD_JSON: %s" % exc, msg.data)
            return
        if not isinstance(d, dict):
            self._reject("NOT_AN_OBJECT (%s)" % type(d).__name__, msg.data)
            return

        try:
            fields(d, SPEC)
        except CoordinationError as exc:
            self._reject(str(exc), msg.data)
            return
        except KeyError as exc:
            self._reject("MISSING_KEYS: %s" % exc, msg.data)
            return

        self.ok += 1
        if self._dump is not None:
            self._dump.write(json.dumps({"t": now, "data": d}, sort_keys=True) + "\n")
        self.tids[d["target_id"]] += 1
        self.obs_ids[d["observation_id"]] += 1
        c = float(d["confidence"])
        self.conf.append(c)
        if c >= 1.0:
            self.drive += 1

    def _reject(self, reason, raw):
        key = reason.split(":")[0]
        self.rej[key] += 1
        if key not in self.rej_example:
            self.rej_example[key] = (reason, raw[:200])

    def report(self):
        if self._dump is not None:
            self._dump.close()
        print("=" * 74)
        print("话题: %s" % self.topic)
        if self.dump_path:
            print("流量落盘: %s" % self.dump_path)
        print("校验器: %s" % os.path.abspath(COORD))
        print("契约字段: %s" % sorted(SPEC))
        print("-" * 74)
        print("收到消息        : %d 条" % self.n)
        if self.n == 0:
            print("!! 话题上没有任何消息 —— 感知没在发（PR_COORD_ON 关着？话题名不对？）")
            return 2
        span = (self.t_last - self.t_first) if self.t_first else 0.0
        print("时间跨度        : %.1f s  ⇒  %s Hz" % (
            span, ("%.2f" % (self.n / span)) if span > 0.5 else "n/a"))
        if self.gaps:
            sg = sorted(self.gaps)
            print("间隔 中位/最大  : %.3f / %.3f s   （官方广播要求 <1s）" % (
                sg[len(sg) // 2], sg[-1]))
        print("**契约校验通过** : %d 条（%.1f%%）" % (
            self.ok, 100.0 * self.ok / max(1, self.n)))
        print("被拒            : %d 条" % sum(self.rej.values()))
        for k, v in self.rej.most_common():
            reason, raw = self.rej_example[k]
            print("    [%s] x%d  例: %s" % (k, v, reason[:110]))
            print("        原始载荷: %s" % raw)
        if self.ok:
            print("-" * 74)
            print("唯一 target_id  : %d 个  %s" % (len(self.tids), dict(self.tids)))
            uniq = len(self.obs_ids)
            print("observation_id  : %d 个唯一 / %d 条  ⇒ %s" % (
                uniq, self.ok,
                "**每条都不同**（核心的重复观测去重形同虚设，observations 台账会无界增长）"
                if uniq == self.ok else "有重复（幂等去重生效）"))
            cs = sorted(self.conf)
            print("confidence      : min %.3f / 中位 %.3f / max %.3f" % (
                cs[0], cs[len(cs) // 2], cs[-1]))
            print("**能驱动核心**   : %d 条（confidence >= 1.0）" % self.drive)
            if self.drive == 0:
                print("    ⚠ core._target() 是 `if d['confidence'] < 1.: return` ⇒")
                print("      当前所有报告**只会进 observations 台账，不会更新任务坐标**。")
                print("      若期望核心按目标位置行动，需由上游对'已确认'目标发 1.0，")
                print("      或与队友确认该语义。")
        return 0 if (sum(self.rej.values()) == 0 and self.ok > 0) else 1


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    topic = sys.argv[2] if len(sys.argv) > 2 else "/coordination/target_report"
    rospy.init_node("probe_target_report", anonymous=True)
    p = Probe(topic)
    print("探针已订阅 %s，观察 %.0f s ..." % (topic, secs))
    rospy.sleep(secs)
    rc = p.report()
    sys.exit(rc)


if __name__ == "__main__":
    main()
