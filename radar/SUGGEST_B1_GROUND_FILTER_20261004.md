# B1 补充建议：地面回波剔除（可直接转交队友）

> 对象：`coordination/src/robocup_swarm/scripts/radar_velocity_guard.py`
> 与 `swarm_agent.py` 的 `_radar_guard_velocity`（瞬时守卫）+ `_scan_cb`（地图喂入）
> 日期：2026-10-04　｜　性质：**补充**，不动队友现有逻辑
> **状态（10-04 17:20）：已在本地 `D:/robocup/robo_team` 实施完毕，未 commit、未 push。**
> 补丁：`b1_ground_filter_20261004.patch`（3 文件 +280/−5）　｜　测试：`test_radar_velocity_guard.py` **23/23** 全绿
>
> **含两个站点**：B1 = 瞬时守卫前净化（`_radar_guard_velocity`）；
> **B1b = 地图喂入前净化（`_scan_cb → ObservedMap.feed`）**。理由见 §4.4——只做 B1 是"治一半"。

---

## 0. 聊天可直接转发版（短）

> 发现一处可以让 `radar_velocity_guard` 更准的地方：2D 雷达是随机体刚性转动的，飞机一俯仰（爬升、急转、制动都会有），扫描面跟着倾斜，就有光束**打到地面**，这些地面回波被 `corridor_clearance()` 当成"前方有障碍"。
>
> 实测（直接跑你的 `guard_velocity`，合成扫描）：层高 2.8m、俯仰 12°、3.5 m/s、**开阔空域无任何障碍** → 因为地面回波 clear 只剩 12.57m < 刹车距离 14.0m → 返回 **`BRAKING_REQUIRED`，"速度归零"**。俯仰越大越严重（20° 时 3.0 m/s 就急停）。
>
> 修法：在调用 `guard_velocity` 之前先把"几何上必然打地面"的束剔掉（置 `inf`）。判据是纯几何的：射线世界系 z 分量 `d_z = R20·cos a + R21·sin a`（R20/R21 = 机体→世界旋转矩阵第三行），`d_z<0` 时地面交点斜距 `r_gnd = -z_sensor/d_z`，读数落在容差内即判为地面回波。
>
> **要改两个地方**（同一个净化函数调两次）：① 调用 `guard_velocity` 之前（`_radar_guard_velocity`）；② **喂 `ObservedMap.feed()` 之前（`_scan_cb`）**——否则地面回波会在 2D 栅格上累积成幻影墙，污染 `_grid_guard_velocity` / `online_planner.command_clear` 这两条主守卫。
>
> **水平飞行时它一个束都不剔（已断言逐位一致）**，所以对现有行为零影响；**正前方有真墙时过滤前后行为完全一致**（已验证 8m 墙例子），不会削弱真障碍。代码我这边有跑过回归的成品，可以直接搬过来。

---

## 1. 机理（三行公式）

雷达束在机体系的角度为 `a`，机体系单位向量 `(cos a, sin a, 0)` 经旋转矩阵 `R`（机体→世界）后，世界系 z 分量为：

```
d_z = R20·cos a + R21·sin a        R20 = 2(xz − wy)   R21 = 2(yz + wx)
```

`d_z < 0` ⇒ 该束朝下。它与地面（z=0）的交点**斜距**：

```
r_gnd = −z_sensor / d_z            z_sensor = local_z + 0.080（SDF 雷达安装高度）
```

读数 `r` 落在 `r_gnd` 容差内 ⇒ **几何上必然打的是地面** ⇒ 置 `inf`（= 该方向通）。

纯俯仰 `θ` 时 `d_z ≈ −sinθ·cos a`，故 `r_gnd = z_sensor / (sinθ·|cos a|)`：
**`|cos a|` 最大 ⇒ `r_gnd` 最小 ⇒ 地面回波最先出现在正前方（a≈0）**——恰好就是守卫最在意的那条走廊。

### 门槛表（量程 `range_max = 20 m`，队友 SDF 实测）

地面回波进入视野的条件是 `r_gnd ≤ 20`，即 `sinθ ≥ z_sensor / 20`：

| 高度层 | z_sensor | 触发俯仰 | 该俯仰下正前方地面斜距 |
|---|---|---|---|
| 2.8 m | 2.880 m | **8.3°** | 20.0 m |
| 3.4 m | 3.480 m | 10.0° | 20.0 m |
| 4.0 m | 4.080 m | 11.8° | 20.0 m |
| 4.6 m | 4.680 m | 13.5° | 20.0 m |

> **最低层最危险**：2.8m 层只要倾斜超过 **8.3°**，地面回波就必然落进量程内；
> 而高度越高、`r_gnd` 越大、越容易被 20m 量程截掉（4.6m 层 12° 时**一束都不剔**）。
> 这与"低空（2.45m）撞房"的场景方向一致。

---

## 2. 实测证据（直接调用队友未改动的 `guard_velocity`）

合成扫描：512 束、±π、量程 `[0.5, 20]`；实测速度 = 请求速度；`radius=1.2 / latency=0.5 / brake_accel=0.5`。

| 用例 | 剔除束数 | 过滤前 | 过滤后 |
|---|---|---|---|
| C0 水平 0°、开阔、v=3.5 | **0** | CLEAR clear=18.80 | CLEAR clear=18.80 |
| C1 **俯仰12° + 正前真墙 8m**、v=3.5 | 122 | BRAKING_REQUIRED clear=6.75 | BRAKING_REQUIRED clear=6.75 |
| C2 俯仰 **12°** 开阔无墙、v=3.5 | 131 | **BRAKING_REQUIRED** clear=12.57 | **CLEAR** clear=18.80 |
| C2 俯仰 14° 开阔无墙、v=3.5 | 153 | BRAKING_REQUIRED clear=10.63 | CLEAR clear=18.80 |
| C2 俯仰 20° 开阔无墙、v=3.5 | 185 | BRAKING_REQUIRED clear=7.17 | CLEAR clear=18.80 |
| C3 俯仰 20° 开阔无墙、v=3.0 | 185 | **BRAKING_REQUIRED** clear=7.17 | **CLEAR** clear=18.80 |
| C4 层高 4.6m 俯仰12° 开阔、v=3.5 | **0** | CLEAR clear=18.80 | CLEAR clear=18.80 |

**关键读法**：
- **C0 = 零回归证明**：水平飞行时剔除 0 束，返回的 `velocity_xy` 与 `reason` 逐位一致。
- **C1 = 不误伤证明**：正前方有真墙时，过滤前后行为**完全相同**（墙 8m 主导，clear 都是 6.75）。过滤器没有把真障碍"看漏"。
- **C2/C3 = 收益**：开阔空域 + 中等俯仰 ⇒ 过滤前**直接急停**（`0,0`），过滤后正常 `CLEAR`。

---

### 2.1 全高度层扫描（v=3.5 m/s，开阔空域**无任何真障碍**）

| 层高 | 门槛 | 俯仰 5° | 8° | 10° | 12° | 15° | 20° |
|---|---|---|---|---|---|---|---|
| **2.8 m** | 8.3° | 0 CLEAR | 0 CLEAR | 97 CLEAR | **131 → 修好假急停** | 159 → 修好 | 185 → 修好 |
| **3.4 m** | 10.0° | 0 | 0 | 0 | 95 CLEAR | 135 → 修好 | 169 → 修好 |
| **4.0 m** | 11.8° | 0 | 0 | 0 | 31 CLEAR | 109 CLEAR | 151 → 修好 |
| **4.6 m** | 13.5° | 0 | 0 | 0 | **0（完全不动作）** | 71 CLEAR | 133 → 修好 |

（"→ 修好"= 过滤前 `BRAKING_REQUIRED` 假急停，过滤后 `CLEAR`）

**三条可用结论**：

1. **没有任何一行"过滤后比过滤前差"** —— 全表 28 格，过滤后的结果要么与过滤前**完全相同**，要么是**修掉一个假急停**。
2. **动作量随高度单调递减**：12° 时按 2.8→3.4→4.0→4.6m 依次剔除 **131 / 95 / 31 / 0** 束。**最高层完全不动作**。
3. **门槛以下恒为 0 束**：8.3° / 10.0° / 11.8° / 13.5° 是各层的物理门槛（`sinθ ≥ z/20`），低于门槛时地面回波根本没进量程，过滤器连一个字节都不改。

⇒ **各层统一启用，风险一致且不高于最低层；高层天然更安全（动作更少），不存在"高层反而更危险"的情形。**

### 2.2 ★ 当前比赛配置下的收益（`local ALT=2.2m`，雷达离地 **2.28m**）

> 依据：`coordination/scripts/city_swarm_run.py:81`
> `ALT_BASE='2.2',ALT_NLAYER='1',ALT_TARGET_CAP='2.2',CLIMB_DONE_ALT='2.0'`
> —— **六机同层，local 2.2m ⇒ 世界约 2.5~3.0m**（估计器偏差），与用户口径一致。

| 俯仰 | 剔除束数 | v=2.0 | v=2.6（FLEE_CHASE） | **v=3.0（SWARM_MAX）** |
|---|---|---|---|---|
| ≤6° | 0 | 通行 | 通行 | 通行 |
| 8° | 99 | 通行 | 通行 | 通行 |
| 10° | 139 | 通行 | 通行 | 通行 |
| **12°** | 161 | 通行 | 通行 | **急停 → 通行** |
| 14° | 177 | 通行 | 通行 | **急停 → 通行** |
| 16° | 187 | 通行 | **急停 → 通行** | **急停 → 通行** |
| 18–20° | 195–201 | 通行 | **急停 → 通行** | **急停 → 通行** |

**门槛公式（各速度下被地面回波打成假急停的俯仰）**：

| 速度 | 需刹停距离 `vτ + v²/2a` | 假急停门槛 `asin(z/(req+radius+beam))` |
|---|---|---|
| 2.0 m/s | 5.0 m | 21.4° |
| 2.6 m/s | 8.1 m | **14.2°** |
| 3.0 m/s | 10.5 m | **11.2°** |

⇒ **2.2m 巡航 + 3.0 m/s 时，俯仰只要超过 11.2° 就会被打成假急停**（地面回波更是从 **6.5°** 起就进量程）。
而 `MAX_ACC=2.5 m/s²` 下满加速/满减速的稳态俯仰角 `atan(2.5/9.81) ≈ 14.3°` —— **正好越过 11.2° 门槛**。

⚠️ **方向性（重要，别误读）**：地面回波落在**哪一侧**取决于倾斜方向。
- **低头**（前向加速）⇒ 地面带落在 **a≈0 前方** ⇒ 污染守卫最在意的前向走廊 ⇒ **这才是受害者**；
- **抬头**（减速）⇒ 地面带落在 a≈180° 后方 ⇒ `along<0` 被忽略 ⇒ 无害；
- **横滚**（转弯）⇒ 地面带落在 **a≈±90° 两侧** ⇒ `across > inflated` 被忽略 ⇒ 基本无害。

⇒ 真正的风险窗口 = **高速（≈3.0 m/s）+ 低头（前向硬加速或俯冲瞬态）**。属**瞬态**而非持续，但每次命中都白扣时间。

> **验证注意**：`docs/validation/shared_gpu_run*` 历史快照用的是 `ALT_BASE='4.5'` 单层 —— 在 4.5m 下门槛高达 13.2°，**B1 几乎不动作**，所以**用旧快照复跑看不到收益**，必须在当前 2.2m 配置下验证。

---

## 3. 为什么是零风险

1. **水平飞行完全不动作**：`d_z ≥ GROUND_MIN_DZ (−0.05)` 的束一律原样保留；不倾斜就没有 `d_z < 0` 的束 ⇒ 剔除数恒为 0（C0 已断言）。
2. **不削弱真障碍**：容差 `tol = 0.6 + 0.06·r_gnd`，换算成"高度窗口"约 `tol·sinθ ≈ 0.3~0.4 m`。也就是**只有高度低于约 0.3m、且正好躺在预测地面截距上的小物件**才可能被误剔。墙、人（约 1.7m）这类真障碍在更近的距离就被截住了，不受影响（C1 已验证）。
3. **NaN / inf 一律原样保留**：不做任何"猜测性"过滤，语义保守。
4. **`z_sensor ≤ 0.5` 时不启用**：起飞前 / 位姿未就绪时自动失效。

---

## 4. 落地改动（2 个文件，3 处；`guard_velocity` 本体一字不改）

### 4.1 新增函数 —— 追加到 `radar_velocity_guard.py`（该模块本就是 ROS-free 雷达数学，同族放置）

```python
# ---- 地面回波剔除 ----------------------------------------------------------
MOUNT_Z = 0.080            # 雷达相对机体的安装高度（SDF pose z）
GROUND_TOL_ABS = 0.6       # 地面截距判定绝对容差 m
GROUND_TOL_REL = 0.06      # 相对容差（远处放宽）
GROUND_MIN_DZ = -0.05      # d_z 负过此值才算"朝下方"（约 2.9°）


def filter_ground_beams(ranges, angle_min, angle_increment,
                        r20, r21, z_sensor,
                        tol_abs=GROUND_TOL_ABS, tol_rel=GROUND_TOL_REL):
    """剔除几何上必然打到地面的雷达束（机体倾斜时的地面回波）。

    纯几何、无 ROS 依赖。返回 (新 ranges 列表, 剔除束数)。
    r20/r21 = 机体->世界旋转矩阵第三行；z_sensor = 雷达离地高度
    = local_z + MOUNT_Z。射线世界系 z 分量 d_z = r20*cos(a)+r21*sin(a)；
    d_z<0 的束朝下，地面交点斜距 r_gnd = -z_sensor/d_z；读数落在容差内
    即判为地面回波，置 inf（语义 = 该方向通，与无回波一致）。
    水平/上仰束、NaN、inf 一律原样保留；z_sensor<=0.5 时不启用（起飞前）。
    """
    out = list(ranges)
    n_gnd = 0
    if z_sensor <= 0.5:
        return out, 0
    for i in range(len(out)):
        r = float(out[i])
        if not math.isfinite(r):
            continue                       # NaN / +-inf 原样保留
        a = angle_min + i * angle_increment
        dz = r20 * math.cos(a) + r21 * math.sin(a)
        if dz >= GROUND_MIN_DZ:
            continue                       # 不朝下 => 不可能是地面回波
        r_gnd = -z_sensor / dz
        if abs(r - r_gnd) <= tol_abs + tol_rel * r_gnd:
            out[i] = float('inf')
            n_gnd += 1
    return out, n_gnd
```

### 4.2 `swarm_agent.py` —— 记录旋转矩阵第三行

`local_position` 回调里已有 `q = msg.pose.orientation`（约 :615），紧接 yaw 之后加一行：

```python
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 机体->世界旋转矩阵第三行（地面回波剔除用）
        self._zrow = (2.0 * (q.x * q.z - q.w * q.y),
                      2.0 * (q.y * q.z + q.w * q.x))
```

初始化处（约 :344，`self.yaw = 0.0` 附近）加：`self._zrow = (0.0, 0.0)`

### 4.3 `_radar_guard_velocity`（约 :1156-1177）—— 调用前过滤

```python
from radar_velocity_guard import guard_velocity, filter_ground_beams, MOUNT_Z

    def _radar_guard_velocity(self, vx, vy):
        ...
        ranges = scan.ranges
        if self.local_z is not None:
            ranges, n_gnd = filter_ground_beams(
                ranges, scan.angle_min, scan.angle_increment,
                self._zrow[0], self._zrow[1], self.local_z + MOUNT_Z)
            if n_gnd:
                rospy.loginfo_throttle(3., '[%s] ground beams filtered=%d',
                                       self.uav_id, n_gnd)
        result = guard_velocity(
            (vx, vy), velocity[:2], self.yaw, ranges,        # <== 这里由 scan.ranges 改为 ranges
            scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            scan.header.stamp.to_sec(), now, max_age=RADAR_FRESH_S,
            radius=RADAR_STOP_R, latency=RADAR_LATENCY_S, brake_accel=RADAR_BRAKE_MPS2)
```

> 备注：`filter_ground_beams` 是纯函数，可以按需改成"只算不生效"先跑一轮基线，
> 统计 `n_gnd` 的触发占比再决定是否接线。

### 4.4 ★ B1b —— `_scan_cb` 地图喂入前**同样**净化（关键，别漏）

**只做 4.3 是治一半。** 因为扫描帧有**两条下游**：

```
                          ┌─→ _radar_guard_velocity → guard_velocity      (瞬时守卫)  ← 4.3 治这里
scan → _scan_cb(msg.ranges)
                          └─→ ObservedMap.feed(msg.ranges) → _online_safe_grid
                                    → _grid_guard_velocity / online_planner.command_clear   (栅格主守卫) ← 原样污染!
```

`_scan_cb:644` 把**未过滤的 `msg.ranges`** 直接 `observed.feed(...)`。地面回波会在 2D 栅格上标出**幻影占据格**，并在 `max_age` 内持续存在 ⇒ 污染 `_online_safe_grid` ⇒ **`_grid_guard_velocity`（速度缩放）与 `_online_planner.command_clear`（发布前最后一道闸）都可能莫名减速 / `ONLINE_SPACE_UNKNOWN` 急停**。而这两条恰恰是**主守卫**（瞬时雷达只是 `RADAR_FRESH_S` 内的快照）。

改法（`_scan_cb`，`observed.feed` 之前）：

```python
            map_ranges = msg.ranges
            _zl = getattr(self, 'local_z', None)
            _zr = getattr(self, '_zrow', None)
            if _zl is not None and _zr is not None:
                map_ranges = filter_ground_beams(
                    map_ranges, msg.angle_min, msg.angle_increment,
                    _zr[0], _zr[1], _zl + MOUNT_Z)[0]
            if not observed.feed(map_ranges, position, self.yaw, msg.angle_min,
                                 msg.angle_increment, msg.range_min, msg.range_max,
                                 self._scan_t, self._local_prev_t, now):
                return
```

- **水平飞行剔除 0 束** ⇒ 喂进地图的 ranges 与历史**逐位一致**，零回归（已由新增用例断言）。
- 真障碍（墙/人）在更近处被截住，不在容差内 ⇒ 地图照常标注，不削弱。
- 该段落在 `_scan_cb` 的**早退之后**（`_online_map is None` 时不会到达），因此**既有 `AdapterTests` 完全不受影响**（`test_delayed_scan_is_not_refreshed_by_arrival` 用的是无 `_online_map` 的裸对象，走早退分支）。
- 新增用例 `test_scan_callback_feeds_ground_filtered_ranges_to_map` 直接断言：倾斜帧喂入的 ranges 全被净化、水平帧逐位一致。

---

## 5. 建议的验收断言

| # | 断言 | 依据 |
|---|---|---|
| A1 | **水平飞行时过滤后 ranges 与原始逐位相同**（`n_gnd == 0`） | C0，零回归 |
| A2 | **正前方真墙场景过滤前后 `reason`/`velocity_xy` 完全一致** | C1，不误伤 |
| A3 | 复跑 `shared_gpu_six_complete_20261004` 等价场景，逐项无回归 | 队友既有验证快照 |
| A4 | 俯仰 >12° 开阔场的 `BRAKING_REQUIRED` 计数显著下降 | C2/C3 |

---

## 6. 与其它空白的关系（10-04 更正）

- **B2（高度盲带）**：B1 解决"地面回波进走廊"，B2 是"机顶贴装导致下方 0.331m 看不到"。两者**互补**，不重叠。
- **B3（多帧记忆进速度守门）—— 核查后撤回，你们早已实现**：
  `_grid_guard_velocity:1202` 在有 `_online_planner` 时走 `_online_safe_grid` + `planner.command_clear()`，
  而 `_online_safe_grid` 来自 `_scan_cb:644 observed.feed(...)` → `_compute_plan:1603 planner.grid(frozen,...)` → `:1617 self._online_safe_grid = grid`。
  **多帧地图确实已经在速度闸里**，且用的是**与 `guard_velocity` 完全相同的刹车距离模型**（radius 1.2 / latency .5 / brake .5）。⇒ **不是空白，无需补**。
  （B1b 的作用正是保护这条已有链路不被地面回波污染。）
- **B4（守卫链顺序）—— 核查后基本撤回**：实际顺序为
  `… → _radar_guard_velocity:1451 → _gate → _friend_guard_velocity(ORCA):1454 → route_gate.command_clear:1458 → online_planner.command_clear:1465 → 发布`。
  ORCA 改向**之后确实有复核**（route_gate + online_planner），只不过是**多帧栅格**而非瞬时帧——而栅格是同一批扫描累积的**超集**，比瞬时帧更可靠。⇒ 原先担心的窗口基本不存在，降级为无需改动。
- **B5（多处急停）**：注意 `guard_velocity` 对 **NaN** 返回 `UNKNOWN_RANGE` 并**急停**。B1 保留 NaN 不动，故不改变该行为——这是 B5 该处理的一条（真空白，低优先）。

## 7. 实施记录（本地，未 commit / 未 push）

分支 `codex/radar-coordination-20261003` @ `81d1698`，改动前工作区干净。

| 文件 | 变更 | 内容 |
|---|---|---|
| `radar_velocity_guard.py` | +62 | 新增 `filter_ground_beams()` 与 `MOUNT_Z` / `GROUND_*` 常量；残留注释更正（原指向 B3，B3 已存在）。**`guard_velocity` 本体一字未改** |
| `swarm_agent.py` | +33 −4 | ① `:41` import 增补；② `:343` 新增 `self._zrow` 初始化；③ `local_position` 回调附 `_zrow`；④ `_radar_guard_velocity` 调用前净化（B1）；⑤ **`_scan_cb` 地图喂入前净化（B1b）** |
| `test_radar_velocity_guard.py` | +184 −2 | 新增 `GroundBeamFilterTests`（6 例）+ 适配器接线回归 2 例（瞬时守卫 + 地图喂入）；测试 env 补 `filter_ground_beams` / `MOUNT_Z` |

**防御性设计**（关键）：两处净化都用
`getattr(self, 'local_z', None)` 与 `getattr(self, '_zrow', None)`。
姿态未知（`_zrow is None`）→ 整段跳过 ⇒ **不给既有测试/其它调用方引入任何新符号依赖**；
`_zrow = (0,0)` 时 `dz ≡ 0 ≥ GROUND_MIN_DZ` ⇒ 同样一束不剔。

**测试结果**：

| 套件 | 结果 |
|---|---|
| `test_radar_velocity_guard`（23 例） | **23/23 OK**（原 15 例全绿 + 新增 8 例） |
| `coordination/tests` 全量 | 改动后 **393 ran / 1F / 6E / 3skip**；基线（`git stash` 后）**385 ran / 1F / 6E / 3skip** —— **失败清单逐条相同，零新增失败**。7 项失败均为环境因素（缺 `rospy`、Windows 路径分隔符） |

备份（仅新增，未删任何文件）：
`radar_velocity_guard.py.bak_pre_b1_20261004`、`swarm_agent.py.bak_pre_b1_20261004`、`test_radar_velocity_guard.py.bak_pre_b1_20261004`

---

## 8. 残留风险（10-04 更正：**已被既有链路覆盖**）

**强倾斜时，正前方光束物理上先触地，瞬时雷达看不见更远的地物。**

- 倾斜 `θ` 时，正前方有效前向地平线收缩到 `z_sensor / tan θ`：
  2.8m 层 12° ⇒ **13.5m**；当前 **2.2m 配置**（z_sensor 2.28m）12° ⇒ **10.7m**（原本 20m）。
- 一堵 **15m 外**的墙，在 12° 姿态下**根本没有回波** —— 这是 2D 单平面雷达的物理限制，与过滤无关。
- "过滤前"那个地面回波（在 ~13.85m）**碰巧**让瞬时守卫保持在 `BRAKING_REQUIRED`，形成一种**偶然的保护**；过滤后这层偶然保护消失。

**为什么仍判定为净收益**：过滤前该守卫的行为等价于"**只要倾斜 >12°，无论前方有没有东西一律急停**"（开阔空域同样急停）——它无法区分，不构成真正保护，且代价是每次倾斜都白扣分。

**为什么这条残留不再是问题（关键更正）**：

1. 瞬时雷达**只是两道防线之一**。正前方 15m 的墙，**倾斜之前**就已经被雷达看到并喂进 `ObservedMap`；
   只要它在 `max_age`（默认 3s）内，`_online_safe_grid` 上就**永久有这座墙**，
   `_grid_guard_velocity` 与 `online_planner.command_clear` 照样会拦住 —— **多帧记忆天然补上了瞬时盲锥**。
2. 倾斜是**瞬态**：机体回正后光束立刻扫上去，墙马上重新可见；3.5 m/s 下回正 0.5s 仅前进 ~1.8m，相对 15m 余量充足。
3. **B1b 的意义正在于此**：保证喂进这张地图的每一帧都**不含幻影地面格**，
   否则上面第 1 条会被污染（地图上凭空多出墙 ⇒ 反而误停）。

⇒ 结论：**"多帧记忆并入速度守门"不需要我们新做——它已经是队友的既有能力（B3 撤回）；B1 + B1b 保证这条既有链路不被地面回波污染。** 两者合起来，这条残留即闭合。

---
*第 2 / 2.1 节数据由直接调用（改动前）`guard_velocity` 的合成扫描产出；第 7 节为本地实施记录。*
