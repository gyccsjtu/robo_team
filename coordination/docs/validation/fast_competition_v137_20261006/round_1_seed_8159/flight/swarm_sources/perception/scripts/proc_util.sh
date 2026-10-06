#!/bin/bash
# ============================================================================
# 进程匹配工具（给 m3_ann_run.sh / m5_dual_run.sh source）
#
# 为什么不能简单用 pgrep/pkill -f
# -------------------------------
# · `pkill -f "perception_real.py"` 会匹配到**远程 SSH 自己的命令行**
#   （bash -c 那一长串里就含这个字符串）→ 自杀，后面命令全不执行，
#   表现为"exit 1、零输出"。
# · 改成按 `ps -o comm` 过滤 `$2=="python3"` 也不行：**PyTorch 会把进程名
#   改成 `pt_main_thread`**，于是 kill 静默不匹配、清理校验还报"0 个残留"，
#   结果一轮里跑了 **3 个 perception_real**（load average 26，帧率掉到 0.5Hz）。
#
# 正确判据
# --------
# 读 /proc/<pid>/cmdline，**第一个 token 必须是 python3**（shell 包装进程的
# 第一个 token 是 bash/ssh，天然被排除），再在整行里找关键字。
# 这样既不会自杀，也不怕进程改名。
# ============================================================================

_pu_pids() {   # _pu_pids <关键字>  -> 每行一个 pid
  local pat="$1" p cl
  for p in /proc/[0-9]*; do
    [ -r "$p/cmdline" ] || continue
    cl=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null) || continue
    case "$cl" in
      python3\ *|*/python3\ *|python3|*/python3) ;;
      *) continue ;;                      # 第一个 token 不是 python3 -> 跳过
    esac
    case "$cl" in *"$pat"*) echo "${p#/proc/}" ;; esac
  done
}

count_pat() {  # count_pat <关键字> -> 数量
  local n=0 x
  for x in $(_pu_pids "$1"); do n=$((n + 1)); done
  echo "$n"
}

list_pat() {   # list_pat <关键字> -> "pid args" 行
  local x cl
  for x in $(_pu_pids "$1"); do
    cl=$(tr '\0' ' ' < "/proc/$x/cmdline" 2>/dev/null)
    echo "  $x $cl"
  done
}

kill_pat() {   # kill_pat <关键字> -> 杀掉匹配的 python3 进程（先 TERM）
  local x
  for x in $(_pu_pids "$1"); do
    kill "$x" 2>/dev/null
  done
  return 0
}

kill_pat_hard() {  # 等 3 秒仍活着就 KILL
  local pat="$1" i
  kill_pat "$pat"
  for i in 1 2 3; do
    sleep 1
    [ "$(count_pat "$pat")" = "0" ] && return 0
  done
  for x in $(_pu_pids "$pat"); do kill -9 "$x" 2>/dev/null; done
  return 0
}
