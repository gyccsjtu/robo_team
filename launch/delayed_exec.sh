#!/bin/bash
# 延迟执行包装器：delayed_exec.sh <秒> <命令...>
# 用途：串行化 spawn_model 调用，规避 Gazebo 11.15.1 ODE 引擎
#      在并发插入多个带碰撞模型时的段错误（gzserver SIGSEGV in libgazebo_ode.so.11）。
# 用法（launch-prefix）：launch-prefix="/path/to/delayed_exec.sh 6"
DELAY="$1"; shift
sleep "$DELAY"
exec "$@"
