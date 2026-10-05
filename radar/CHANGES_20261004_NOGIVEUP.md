# 比赛防弃赛改造清单（2026-10-04 · radar_avoid.py）

> 主题：**把"启动期拦截 / 一失败就退出 / 靠重新启动"这类在正式比赛里无效的
> 防守逻辑全部换掉**。正式比赛的两条硬约束决定了它们无效：
> ① **一旦起飞就无法重开**（进程退出 = 直接弃赛）；
> ② **比赛限时**（所以替代方案不能是"无限等待/无限重试"，必须**有界**）。
>
> 本清单对应的**已验证实现**在
> `C:\Users\guo\Desktop\robocup\_radar_deliver\radar\radar_avoid.py`
> （备份：`radar_avoid.py.bak_pre_antiexit_20261004`，即本次改造之前）。
>
> 配套测试：`test_low_alt_opt.py`（D 段验"不拦启动"、E 段静态防回归）。

---

## 判据：什么算"比赛里没用的防守"

| 类别 | 为什么无效 | 处置 |
|---|---|---|
| **A 启动期参数拦截**（自己设的参数自己拦） | 参数在起飞前由人设定，运行期不会再变；拦在启动既拦不住既成事实，又可能在"必须起飞"的窗口里把整场比赛废掉 | **删**，后果移交运行期 |
| **B 一次性失败就退出/放弃**（等不到就 return 1 / 拒绝起飞） | 比赛不可重开 ⇒ 等于弃赛。而这些失败多是**环境快慢**（MAVROS 晚起、EKF 未收敛、参数表未就绪），会自愈 | 改**有界预算内持续重试 + 自愈**，仍失败才降级/放弃 |
| **C 静默失效无人管**（心跳线程死了没人知道） | 心跳是全脚本唯一 setpoint 发布点，死了必然掉 OFFBOARD；现场无人干预 ⇒ 必须自愈 | 加**自助重启** |
| **D 无限等待**（本次明确禁止） | 比赛限时，无限等待会把比赛时间耗光 | **不采用**；一切等待/重试都设**有界预算** |

---

## 改动清单（共 10 项，均在 `radar/radar_avoid.py`）

### 1. `set_failsafe_params` —— 固定 3 轮 → 时间预算内持续重试

**签名**：`def set_failsafe_params(self, budget=90.0)`（原无参）

- **旧**：`for attempt in range(1, 4)`，3 轮读回不一致即 `return False`。
- **新**：在 `budget` 秒内**持续重试**；参数服务不出现时也不再"固定 30s 就放弃"，
  改为 2s 一次轮询到预算用尽。返回 False 只表示"预算内未确认生效"，由调用方决定。
- **理由**：参数服务晚到 / 参数表未就绪都不是弃赛理由。

### 2. `main` —— 不再"failsafe 参数未生效就拒绝起飞"

- **旧**：
  ```python
  if not pilot.set_failsafe_params():
      rospy.logerr("[radar] 飞控 failsafe 参数未生效 ⇒ 拒绝起飞")
      pilot.stop()
      return 1
  ```
- **新**：`if not pilot.set_failsafe_params():` → **告警后继续**（不 return）。
  并注明风险：若始终失败，RC 失联时 PX4 可能按 `NAV_RCL_ACT` 把飞机踢出 OFFBOARD。
- **运行时替代**：OFFBOARD 重试循环**每 10s 补设一次**参数（见第 5 项）。

### 3. `wait_stable` —— 超时拒绝起飞 → 分级放宽 + 超预算放行

- **签名**：`def wait_stable(self, tol=0.15, window=6, timeout=30.0, tol_max=1.0)`
- **旧**：`timeout` 到即 `return False`（main 据此拒绝起飞）。
- **新**：① 用 `tol` 等到 `timeout`；② 未达 ⇒ 放宽容差到 `tol_max` 继续等；
  ③ 再等 `2×timeout` 仍未达 ⇒ **告警放行**。只有进程关闭才返回 False。
- **理由**：静止判据是为提高质量，不该成为弃赛理由；风险由真值位姿 + 实时
  origin 换算吸收。

### 4. `takeoff` ①：EKF 收敛等待 —— 超时拒绝 → 超时告警放行

- **旧**：`if time.time() - t_ekf > 90.0: ... return False`
- **新**：同样在 90s（**原值不变**）后，改为 `break`（放行）并告警。
- **理由**：90s 仍不收敛基本都是环境问题，继续干等没用；放行风险由起飞后
  `nav_hold` 悬停熬 PX4 `nav_test` 危险窗口兜底。

### 5. `takeoff` ②：OFFBOARD —— 90s 放弃 → 150s + 每 10s 补设参数/复查心跳

- **旧**：`if time.time() - t_off > 90.0: break`（随后 `return False`）
- **新**：预算 150s；循环内**每 10s**：调 `set_failsafe_params(budget=5.0)`
  补设参数 + 若 `_sp_sent<=0` 则 `restart_heartbeat()`。
- **理由**："参数没生效"和"心跳死掉"是切不进 OFFBOARD 的两大真因，
  与其干等，不如**边等边修**。

### 6. `takeoff` ③：arm —— 30s → 90s

- **旧**：`if time.time() - t_arm > 30.0: break` → `return False`
- **新**：90s（有界延长）。

### 7. `takeoff` ④：爬升 + 悬停掉锁 —— 直接失败 → 自助重解锁

- **旧**：`if not self.armed(): rospy.logerr(...); return False`（爬升段与
  悬停段各一处）
- **新**：每 10s 尝试 `srv_mode("OFFBOARD")` + `srv_arm(True)` 重解锁，
  在预算（爬升 90s / 悬停 `nav_hold`）内继续；不再一掉锁就判失败。

### 8. `main` 起飞重试 —— 固定 3 次 → 有界预算内持续重试

- **旧**：`TAKEOFF_RETRIES = 3`，3 次失败 `return 1`。
- **新**：`TAKEOFF_BUDGET = 300.0`，预算内持续重试（每次失败 sleep 8s）。
- **理由**：不轻易弃赛，但**比赛限时** ⇒ 不能无限试。

### 9. 心跳活性检测 —— 只告警 → **自助重启线程**

- **新增方法** `def restart_heartbeat(self)`：
  `_alive=False` → `join(0.5)` 旧线程 → 重建；已 `stop()` 或进程关闭时拒绝；
  最小重启间隔 3s（防空转）。安全性依据：`_sp_loop` **只在锁内取引用、
  锁外 publish**，不存在持锁阻塞。
- **调用点 3 处**：
  ① `step_toward` 每步检测：`_sp_sent` 停滞 >1s ⇒ 重启；
  ② `takeoff` EKF 段后 `_sp_sent<=0` ⇒ 重启 + 5s 观察窗；
  ③ `takeoff` OFFBOARD 循环每 10s 复查 ⇒ 仍 0 帧再重启。
- **配套**：`__init__` 增 `self._stopped=False`；`stop()` 置 True（收尾后
  不允许再拉起设定点流）。

### 10. `main` 就绪等待 —— **保留**（60s → 90s）

- **保留** `if not pilot.wait_ready(90.0): return 1`，**不改成无限等待**。
- **理由（与"参数拦截"性质不同）**：位姿 / 状态 / 雷达任一缺失时，后面的
  `step_toward` / `takeoff` **没有任何可用输入（会直接抛异常）**，根本飞不了；
  早退反而给人工留出干预时间。且比赛限时 ⇒ 不能无限等。
- 顺带重构：把"缺哪一项"抽成 `ready_missing()`（含雷达连接数诊断），
  超时日志更精确。

---

## 明确**保留不动**的（附理由）

| 逻辑 | 位置 | 为什么不是"没用的防守" |
|---|---|---|
| 航点超时 ⇒ **跳下一个** | 主循环 `_limit` | 是"继续任务"，不是弃赛 |
| 雷达陈旧 >2s ⇒ **原地悬停等恢复** | 主循环 `scan_age()` | 等待有界于比赛时限，数据一到立刻恢复 |
| AUTO.LAND 收尾超时 ⇒ 告警"请人工接管" | 主循环末尾 | 任务已完成，进程本就要结束 |
| 配置缺失 ⇒ `return 1` | `--online-map` 缺 `--start/--goal`；找不到 `astar_plan.py`；无目标点 | **配置错误**，运行时无法替代；比赛前必须配好 |
| `detect_origin` 离散超限 ⇒ 拒绝采用 | 已废弃函数 | **不在主流程**（现为实时 origin），仅诊断保留 |
| `_sp_loop` 循环体整圈兜异常 | 心跳线程 | 与"退出"相反 —— 保证线程不因单帧异常而死 |

---

## 验收（施工完成后逐条执行）

| # | 命令 | 预期 |
|---|---|---|
| 1 | `python3 test_low_alt_opt.py` | Ran 24 checks, 0 failed |
| 2 | `python3 test_radar_offline.py` | 总计 35 项，失败 0 项 |
| 3 | `python3 test_target_follow.py` | 总计 31 项，失败 0 项 |
| 4 | `python3 sim_radar_closed_loop.py` | 5 场景到达、碰撞帧 0、最小净空 >0.80 |
| 5 | `python3 online_planner.py --selftest` | 全部通过 PASS |
| 6 | `python3 occupancy_online.py --selftest` | 全部通过 PASS |
| 7 | `RADAR_DEMO_DIR=<_demo2026> python3 test_two_layer.py` | 两级集成 4/4 |
| 8 | `python3 random_map_fly.py --gen-dir <基线 _gen_20260929> --n-route 18 --seed 11` | 108/108、0 碰撞、最小净空 ≈1.079 m（与基线一致 ⇒ 零回归） |

关键判定：
- **#1 的 E 段**：`ap.error` 不出现、`TAKEOFF_RETRIES` 不出现、
  `restart_heartbeat` 定义 1 处+调用 ≥3 处 —— 防止有人把守卫/固定次数改回来。
- **#1 的 D 段**：`--alt 2.0/2.45/2.8` 都走到 `wait_ready` 而不抛
  `SystemExit(2)` —— 证明"参数层拦截"确实不存在。
- **#8 净空数字必须与基线一致**：本次改动全在 `main` / `takeoff` / 心跳线程，
  离线链路（`subgoal_from_scan` 等）一行未动。

---

## 已实测结果（2026-10-04 实现版）

| 套件 | 结果 |
|---|---|
| test_low_alt_opt（含新增 D/E 段） | **24/24** |
| test_radar_offline | 35/35 |
| test_target_follow | 31/31 |
| test_two_layer（WSL + RADAR_DEMO_DIR） | 4/4 |
| sim_radar_closed_loop | 5/5（最小净空 0.964 m） |
| online_planner / occupancy selftest | PASS / PASS |
| random_map_fly 基线 | 108/108、0 碰撞、最小净空 **1.079 m**（与改动前基线逐位一致 ⇒ 零回归） |

## 已知边界

- 本次是"**不弃赛**"改造，**不解决** 2D 单平面雷达的低空盲带（悬挑物无解），
  那一半见 `CHANGES_20261004_ALTOPT.md`。
- 起飞阶段的**有界预算值**（就绪 90s / EKF 90s / OFFBOARD 150s / arm 90s /
  爬升 90s / 起飞总 300s）是按"单次起飞正常几秒、最坏不超过数分钟"取的；
  正式比赛若确认有更长窗口，按现场时限等比放大即可（都是常数，集中可改）。
