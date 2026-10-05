# 起飞等待链的时钟语义修正（2026-10-04 十六次修正）

**目标文件**：`radar_avoid.py`（同目录）。**备份**：`radar_avoid.py.bak_pre_takeoffclock_20261004`。
**新测试**：`test_low_alt_opt.py` 新增 F 段（7 项）。

---

## 0. 症状 → 根因（先纠正一份错误结论）

**症状**：起飞过程在仿真机上要 ~80 墙钟秒，其中"原地悬停"看起来要等三分多钟。

**此前（十五次修正时）的错误判断**：以为 `nav_hold=30` 走 ROS 仿真钟 ⇒
30 仿真秒 = RTF 0.15 下约 200 墙上秒。
**实测核实**：`nav_hold` 的循环条件用的是 `time.time()`（**墙钟**），
`time.time() - t_hold < self.nav_hold` ⇒ 本来就是 30 **真实**秒。上面那条是错的。

**真实根因有两条**：

| # | 根因 | 证据 | 后果 |
|---|---|---|---|
| ① | 悬停从"**到达巡航高度**"起算，而 PX4 的窗口是从"**离地**"起算 | PX4 `LandDetector.cpp:163` 在"检测到离地"打点 `takeoff_time`；`Commander.cpp:4151` 要求 `hrt_elapsed_time(&takeoff_time) > 30_s` | 爬升那一段（RTF=0.15 时十余墙钟秒）被**重复计一遍**，白等 |
| ② | **重试间隔 / 参数沉降**用 `rospy.sleep`（仿真钟），而 PX4 的模式切换、参数写回、`COM_OF_LOSS_T` 失联判定全走硬件 hrt（真实钟） | `gazebo_ros_api_plugin.cpp:215` 置 `/use_sim_time=true`；`rospy.sleep(0.5)` 在 RTF=0.15 下实睡 3.3s | 重试间隔被凭空放大 6 倍；且 `/clock` 停发（Gazebo 暂停）时 `rospy.sleep` **永久阻塞**，连墙钟预算都再也检查不到 |

**必须保留仿真钟的场合（本文件最重要的一节，禁止改动）**：
`takeoff ①` 的 EKF 健康采样（`rospy.sleep(1.0/CTRL_HZ)`）与 `wait_stable` 的
`rospy.sleep(0.25)` **必须继续走仿真钟**。理由：PX4 SITL 按**仿真时间**积分
传感器，"连续 40 帧 EKF 健康 / 6 帧真值静止"度量的是**物理稳定窗口**。
把它改成墙钟只会让循环更快读到**同一份**旧数据 ⇒ 判据被削弱成假通过
（RTF=0.15 时 40 帧会退化成 6 个不同样本）。F 段最后一条断言就是在防这个。

---

## 改动 1：新增墙钟睡眠助手（`RadarPilot._wall_sleep`）

**锚点**：`def armed(self):` 之后、`def wait_stable` 之前。

```python
    def _wall_sleep(self, sec):
        """**墙钟**睡眠（不受 `/use_sim_time` 影响）。2026-10-04 新增。
        ...
        """
        t_end = time.time() + max(0.0, float(sec))
        while True:
            left = t_end - time.time()
            if left <= 0.0 or rospy.is_shutdown():
                return
            time.sleep(min(0.05, left))
```

每 50ms 检查 `is_shutdown()` 与截止时间 ⇒ Gazebo 暂停也不会挂死。

## 改动 2：爬升段记录"离地时刻"

**锚点**：`takeoff()` 里竖直升循环（`while not rospy.is_shutdown() and time.time() - t0 < 90.0:`）。
在循环前加：

```python
        LIFTOFF_DZ = 0.3
        z_ground = p0[2] if p0 is not None else None
        t_lift = None
```
循环内（`c = self.pose`、`if c is None: ... continue` 之后，`reached` 判定**之前**）加：

```python
            if (t_lift is None and z_ground is not None
                    and (c[2] - z_ground) > LIFTOFF_DZ):
                t_lift = time.time()
                rospy.loginfo("[radar] 已离地（真值 z=%.2f，地面 z=%.2f）"
                              "—— nav_test 的 30s 窗口自此起算", c[2], z_ground)
```

`takeoff_time` 是 PX4 在"检测到离地"时打点的，这里用真值 z 相对地面上升
0.3 m 做等价代理。

## 改动 3：悬停改为"自离地起累计"

**锚点**：`if self.nav_hold > 0.0:` 分支。

旧：
```python
            t_hold = time.time()
            _t_rearm = 0.0
            while (not rospy.is_shutdown()
                   and time.time() - t_hold < self.nav_hold):
```
新：
```python
            t_hold = time.time()
            _ref = t_lift if t_lift is not None else t_hold
            _passed = t_hold - _ref
            rospy.loginfo("[radar] 原地悬停：nav_test 窗口须自离地累计 %.0fs"
                          "（已过 %.0fs）⇒ 再等 %.0fs。熬过前创新比率 fail 即"
                          "判 Navigation failure 降落",
                          self.nav_hold, _passed,
                          max(0.0, self.nav_hold - _passed))
            _t_rearm = 0.0
            while (not rospy.is_shutdown()
                   and time.time() - _ref < self.nav_hold):
```
结束日志同步改为 `time.time() - _ref`（自离地累计）。

## 改动 4：重试/沉降等待改墙钟（共 8 处 `rospy.sleep` → `self._wall_sleep`）

| 位置 | 旧 | 新 |
|---|---|---|
| `set_failsafe_params` 每轮 set 后 | `rospy.sleep(0.4)` | `self._wall_sleep(0.4)` |
| `set_failsafe_params` 读回不一致后 | `rospy.sleep(1.0)` | `self._wall_sleep(1.0)` |
| `takeoff ②` OFFBOARD 循环 | `rospy.sleep(0.5)` | `self._wall_sleep(0.5)` |
| `takeoff ③` arm 循环 | `rospy.sleep(0.5)` | `self._wall_sleep(0.5)` |
| `takeoff ④` 悬停循环体 | `rospy.sleep(0.2)` | `self._wall_sleep(0.2)` |
| 心跳重启观察窗 | `rospy.sleep(0.2)` | `self._wall_sleep(0.2)` |
| 爬升循环 ×2 + `c is None` 自旋 | `rospy.sleep(0.1)` | `self._wall_sleep(0.1)` |
| `wait_ready` 循环 | `rospy.sleep(0.3)` | `self._wall_sleep(0.3)` |

**不动**：`takeoff ①` 与 `wait_stable`（见上文"必须保留仿真钟"）；
主循环 `rospy.sleep(1.0/CTRL_HZ)`；`main` 里起飞重试间隔 `rospy.sleep(8.0)`
（"让飞机在**仿真时间**里落稳"，属物理过程）；收尾段驻留。

## 改动 5：`--takeoff-hold` 语义与默认值

旧：`default=30.0`，含义"到达巡航高度后原地悬停秒数"。
新：`default=32.0`，含义"**自离地起累计的真实秒数**"。
32 = 旧"到达高度后 30s"的等效值（爬升≈2 仿真秒），**RTF≈1 时行为与旧版一致**。

---

## 实测效果（RTF≈0.15 的 VM 上估算）

| 阶段 | 旧墙钟 | 新墙钟 | 说明 |
|---|---|---|---|
| set_failsafe 沉降 | ~3s | ~0.5s | 墙钟 |
| OFFBOARD/arm 重试间隔 | 3.3s/次 | 0.5s/次 | 墙钟 |
| EKF 采样（保持仿真钟） | ~13s | ~13s | **物理窗口，不可动** |
| 爬升（物理） | ~13s | ~13s | 不可动 |
| 悬停 | 30s | ~19s | 自离地累计，扣除爬升 |
| **合计** | **~80s** | **~50s** | 且计分消耗同步下降 |

**计分面（官方 `score_cal.py`：`score = 2580 − elapsed`，1 分/秒）**：
悬停按仿真秒计 ⇒ 从 30 仿真秒降到约 19 仿真秒 × RTF ≈ 3~5 分。
（此前所述"−30 分"是基于错误前提，作废。）

---

## 验收命令与判定

```bash
cd <radar 目录>
python3 test_low_alt_opt.py            # 31 项全绿（含 F 段 7 项）
python3 test_radar_offline.py          # 35/35
python3 test_target_follow.py          # 31/31
python3 sim_radar_closed_loop.py       # 5/5，最小净空 0.964m
python3 online_planner.py --selftest   # PASS
python3 occupancy_online.py --selftest # PASS
python3 random_map_fly.py --gen-dir <_gen_20260929> --n-route 18 --seed 11
                                       # 108/108，全局最小净空 1.079m（与改前同值）
# WSL：RADAR_DEMO_DIR=~/robocup_real/_demo2026 python3 test_two_layer.py  # 4/4
```

**零回归依据**：全部改动落在 `takeoff()` 与 `wait_ready()` 的**等待/日志**路径，
决策函数（`subgoal_from_scan` 及其下游）一行未动；离线链路不经过
`takeoff()`，故 `random_map_fly` 数字必须与改前**逐位相同**——实测一致。

## 已知边界

- `nav_hold` 的 30 真实秒**删不掉**：`CBRK_VELPOSERR=201607` 虽能关掉
  nav_test（`Commander.cpp:4051`），但同一 `if (run_quality_checks)` 块
  （4181–4224）是**唯一**更新 `_status_flags.local_position_valid` 的地方，
  而该标志在 OFFBOARD 准入处（4337）被检查 ⇒ 禁用后该标志恒 false
  （`_status_flags{}` 零初始化）⇒ **OFFBOARD 位置控制永远切不进**。死路。
- 本改动**未做**（另案）：失败路径预算收紧（内层最坏之和 ≈450s > 外层
  `TAKEOFF_BUDGET` 300s，外层重试形同虚设）；心跳线程墙钟化
  （`rospy.Rate(20)` 在 RTF=0.15 下真实仅 3 Hz，逼近 PX4
  `COM_OF_LOSS_T` 的 2 Hz 底线）。
