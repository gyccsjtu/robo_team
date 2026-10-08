# hard_cap watcher 存活路径修复（2026-10-07）

## 问题

`run_match.sh start` 启动比赛后，5min hard_cap 由一个后台 watcher 子 shell 触发自动 stop。原实现：

```bash
( sleep $MATCH_HARD_CAP; stop_group ... ) &
disown
```

`disown` 只解除 job table 归属，**子 shell 仍与父进程同 PGID**。用户在 5min 内 Ctrl-C 主控 → SIGINT 发到整个 PGID → 子 shell 的 sleep 收到 SIGINT 退出 → hard_cap 永远不触发 → 6 机残跑 + 拿不到 [FINAL]。

实测（v18 局 logs_20261007_213201）：用户 Ctrl-C 主控 → score_cal ROSInterruptException → 6 个 control_actor + 6 个 swarm_agent 仍存活写日志 5min+，judge.log 末尾无 `[FINAL]`。

## 修复

`run_match.sh` 1073-1121 段：把 `( ... ) & disown` 改成 `setsid -f bash -c 'trap "" INT TERM HUP; ...'`，把 watcher 丢进新 session：

- `setsid -f` 创建新 session + 新 PGID，主控 Ctrl-C 的 SIGINT 触不到
- `trap "" INT TERM HUP` 进一步屏蔽信号（双保险）
- 晚审计同模式也修了（晚审计日志改为 append 到 `$LOGDIR/start.log`）

## 端到端验证

`/tmp/test_hardcap.sh`（已清理）：mock `$REG` + mock 业务（sleep 60）+ setsid watcher（hard_cap=8s）：

```
[21:57:09] 父 PID=2310857 PGID=2310854 mock_business PGID=2310854 mock PID=2310919
[21:57:09] watcher 已 setsid 启动
[21:57:12] 父准备 Ctrl-C（kill -INT 自己）
===== 父进程已退，验证 watcher 表现 =====
[21:57:17] [watcher] hard_cap 8s 到，TERM mock_business PGID=2310854
[21:57:45] [OK] mock 业务（sleep 60）已被 watcher TERM ✓
[21:57:45] (watcher 已正常退场 ✓)
```

父 Ctrl-C 后 5s（= hard_cap 8s - 父等了 3s）watcher 准时触发，TERM mock 业务 + 自己正常退场。

## 后续

- 下次仿真（v18.1）建议：用户中途随时 Ctrl-C 看分（ros 死了不影响 hard_cap watcher），5min 到点自动 stop + 写 [FINAL]
- 已不回归：bash -n 语法过、setsid 命令存在、`/usr/bin/setsid` 可用
