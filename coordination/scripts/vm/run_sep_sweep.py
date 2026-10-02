#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SEP_TARGET_M 扫参：一个配置一个进程，输出到自己的文件，互不干扰。

命令行：python run_sep_sweep.py <sep> <outfile> [seed_lo] [seed_hi]
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

sep = float(sys.argv[1])
out = sys.argv[2]
lo = int(sys.argv[3]) if len(sys.argv) > 3 else 1
hi = int(sys.argv[4]) if len(sys.argv) > 4 else 10

import mission_time as MT
MT.SEP_TARGET_M = sep
MT.SEP_PUSH_MAX_DEG = 30.0
import probe_form as PF
PF.SLOT_MODE = "chase"

# 把汇总重定向到独立文件（stdout 里 grep 会缓冲，看不到中间结果）
with open(out, "w", encoding="utf-8") as fh:
    real = sys.stdout
    sys.stdout = fh
    try:
        print("########## SEP_TARGET_M = %.1f  seeds %d..%d ##########"
              % (sep, lo, hi), flush=True)
        PF.metrics(list(range(lo, hi + 1)))
        sys.stdout = real
        print("SEP=%.1f 完成 -> %s" % (sep, out), flush=True)
    finally:
        sys.stdout = real
