# 给协同核心一侧的消息

（草稿，可直接发。以"搜索栈一侧"的口吻。）

---

## 结论先说

我这边把两份代码都读了一遍，判断是**不需要"合并两套系统"，而是用我这边的一块补上你唯一缺的那个**：核心缺一条能绕过建筑的路线，我这边有实测过 0% 穿墙的 A\*。

核心的其余部分（冲突检测、预约、授权、验收判定）我读完认为**质量比我这边高，应该保留为权威**，我这边改为接入。

---

## 一、我发现的一个问题，想先跟你确认

`coordination_executor.py` 的 `_build_offer()` 里，路线生成是这 4 行：

```python
points = [list(start),
          [start[0], start[1], self.takeoff_alt],
          [goal[0], goal[1], self.takeoff_alt], goal]
```

这是一条**硬编码的直角折线**：竖直爬升 → 水平直飞 → 下降。之后 `_route_clearance(points, obstacles)` 检查这条直线撞不撞建筑，撞了就把 `static_safe` 置 `False`，核心据此不放行。

我看 `config/coordination/six_uav_gazebo.json` 里配的是：

```json
"clearance_source": "empty_world",
"empty_world_clearance_m": 25.0,
```

也就是**在空世界里测试**，`clearance` 直接给常量 25，所以这条路线的 `static_safe` 恒为真。空世界里没问题，但**换到真实城市场景（`training_city_full_s7`，38 栋建筑）后，这条直线大概率撞建筑** → `static_safe=False` → 核心拒绝授权 → 飞机飞不出去。

**这也能解释 `docs/wb_multi_uav_bringup.md` 里那句**：

> 执行器发出 ROUTE_OFFER ✅ `clearance=25m, static_safe=true, grid_safe=true`（证据来自配置源）

`clearance=25m` 正是 `empty_world_clearance_m` 这个常量，不是真实测出来的。

**所以我的问题 1**：真实地图下，这个路线生成是否需要替换成绕障路线？我这边已经跑通 A\*（8 邻接 + 障碍膨胀 0.5m + Catmull-Rom 平滑 + lookahead 跟踪），实测轨迹穿墙率 0/196 采样点，可以直接用于生成 `ROUTE_OFFER` 的 `points`。

---

## 二、我这边已解决的一个问题，可能对你有用

`wb_multi_uav_bringup.md` 里 OFFBOARD 标的是 ❌。我这边查到了**另一条独立的根因**，跟 RC failsafe 不是同一件事（你修的 `COM_RCL_EXCEPT=7` 是对的，但那只解决了一半）。

**根因**：`config/multi_uav/fleet.yaml` 原来的出生点 `start_xy: [-20, 0]`，六机一排里 **4/6 架落在建筑内或贴得太近**：

```
✗ uav_2 (-12.0, 0.0)  净空 -3.40 m  撞 building_1171
✗ uav_3 ( -4.0, 0.0)  净空  1.99 m  撞 building_1169
✗ uav_4 (  4.0, 0.0)  净空 -1.49 m  撞 building_1169
✗ uav_6 ( 20.0, 0.0)  净空 -0.77 m  撞 building_1344
```

**因果链**（每步都有实测证据）：

1. 模型卡在建筑碰撞体内 → Gazebo 物理引擎持续推挤
2. → 陀螺仪报**假角速度**（实测 0.34 rad/s，正常静止应 < 0.005）
3. → PX4 报 `WARN [ekf2] primary EKF changed N (gyro fault)`
4. → EKF 不再融合位置（`estimator_status` 的 `pos_horiz_abs_status_flag=False`）
5. → `system_status` 卡在 3(STANDBY)，不升 4(ACTIVE)
6. → **PX4 静默拒绝 OFFBOARD**：`set_mode` 返回 `mode_sent: True` 但模式不变，**且不产生任何 statustext**

第 6 步的"静默"特性是它难查的原因 —— 看 statustext、看 mode 返回值都得不到线索，得从陀螺仪原始值（`/uav_N/mavros/imu/data` 的 `angular_velocity`）和 EKF 标志位反推。

**我已做的**：

- `fleet.yaml` 出生点改为 `start_xy: [-18, 45]`，最小净空 14.0 m
- 新增 `scripts/vm/check_spawn_points.py`：读 fleet.yaml + 地图 metadata 校验，`--find` 可搜索安全排，**不安全时退出码 1**（可挂到启动前自检）

建议把出生点校验加到 `start_multi_uav.sh` 前置。换地图或换随机布局后必须重跑。

---

## 三、我读完核心实现后的判断（说明我为什么建议以你为权威）

我核查了实现，不只是读接口文档。三点确认：

- `_conflict()` 遍历所有未 `RELEASED` 的预约 + 所有在飞飞机，用 `route_distance()`（**线段-线段**最短距离，非点对点）比对 `min_separation_m + 2*tracking_bound_m`
- `matching()` 是位掩码 DP 的最大基数 + 最小代价匹配，注释与实现一致："缺失的 cost 边是不可行，**绝不用直线距离替代**"
- `verdict.py` 基于**独立 monitor report** 判定，证据不全时返回 `ABSTAIN` 而非 `PASS`

`tests/test_coordination.py` **34 个测试全过**，且测的是安全属性本身（`test_claimed_pass_with_collision_fails`、`test_unknown_run_or_missing_reservation_never_flys`、`test_missing_clearance_cannot_pass`）。

这些我都认可，所以我不建议重写核心。

---

## 四、我建议的接入方式

核心的飞行链路（位置控制 + 授权 + OFFBOARD）**一行不用改**，我这边做三件事：

| 我这边 | 动作 | 说明 |
|---|---|---|
| A\* 绕障 | **替换 `_build_offer()` 的路线生成** | 就那 4 行，工作量小 |
| 目标检测（几何判定 + 视线遮挡，已验证） | 输出改为 `TARGET_REPORT` | 字段天然对应 |
| `TargetTracker`（规则 4/5 计时） | 保留算法，**锁的权威交给核心** | 计时在我，锁在你 |
| `TaskAllocator` 拍卖 | 降级为**优先级生成器** | 只输出"哪些格要搜、多急" |
| `swarm_manager` / `LeaseManager` | **退役** | 你的 `TASK_ASSIGN` + `lease_s` 已覆盖 |
| `target_sim_node`（6 名恐怖分子） | 保留 | 核心不模拟恐怖分子 |
| `check_spawn_points.py` | 保留并前置 | 见第二节 |

数据流变成：

```
Coordinator（安全权威）
   ↑ VEHICLE_STATE / TARGET_REPORT / ROUTE_OFFER / ACK
   ↓ TASK_ASSIGN / ROUTE_GRANT / TARGET_LOCK / RESERVATION
executor + 搜索栈能力
   · 位置控制飞行（executor 原有）
   · A* 绕障 → 生成 ROUTE_OFFER 的 points
   · OFFBOARD 时序（含出生点校验）
   · 几何判定 → TARGET_REPORT
```

---

## 五、想请你确认的问题

**关键的三个：**

1. **真实地图下，`_build_offer()` 的路线生成需要替换成绕障路线吗？** 我这边 A\* 可以直接接（实测 0% 穿墙）。如果可以，我改这个函数就行。

2. **`TASK_ASSIGN` 的 task 能直接用我的搜索格吗？** 我的覆盖栅格是 200×100m 分成 20×10 个 10m 格，想把它作为 task 来源。是否有格式或粒度上的要求？

3. **`TARGET_REPORT` 的 `observation_id` 是否要求全局唯一？** 我打算用「无人机 ID + 帧序号」生成，可以吗？

**另外三个细节：**

4. 控制方式我看到 executor 用 `setpoint_position/local`，我这边是速度控制。**统一到位置控制我没有异议** —— 我的理解是核心的安全证明依赖"实际飞行 = 授权路线"（`_conflict` 拿 `ROUTE_GRANT` 的 points 去比对预约），执行器自主绕道会让证明失效。确认下这个理解对不对。

5. `COM_RCL_EXCEPT` 你用 7、我用 4，**统一到 7** 可以吗？

6. `grid_source=static_substitute` 我理解是**开发替身、比赛不能用**（文档 §4 也是这么写的）。那比赛时这个字段计划怎么填？

---

## 六、我建议的第一步（最小验证）

不改任何现有代码，只把 A\* 路线塞进 `_build_offer()`，然后在 `training_city_full_s7` 上跑 2 机，看：

```
A* 路线 → ROUTE_OFFER → 核心算 clearance → static_safe=True
       → ROUTE_GRANT → executor 位置控制 → 真起飞
```

这样能一次性验证整条链路是否成立。**通了后面就是体力活；不通也能早发现。**

---

附：我这边已有的验证命令，你可以直接跑

```bash
# 出生点安全自检
ROBOCUP_WORKSPACE=$PWD python3 scripts/vm/check_spawn_points.py

# 避障飞行验证（穿墙率 / 贴墙距离 / 高度稳定性 / 覆盖率）
python3 src/robocup_swarm/scripts/verify_swarm_avoidance.py --duration 40

# 核心测试
python3 tests/test_coordination.py
```

详细分析我也写进仓库了：`docs/coordination_integration_analysis.md`。
