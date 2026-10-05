# 给队友的雷达层补充建议（2026-10-04）

> **来源**：本地 `radar_avoid.py` 的实测积累。
> **原则**：不重复你们已有的能力，只补空白。你们已有的（**我们不再动**）：
> `radar_velocity_guard` 走廊法速度守门、`radar_observed_map` 多帧记忆、
> `online_radar_planner` 在线规划、`fleet_motion_guard` 机间避让。
>
> 核查对象：`coordination/src/robocup_swarm/scripts/swarm_agent.py`（2575 行）。

---

## B1 ★★★ 地面回波剔除 —— 你们目前**完全没有**

| 项 | 内容 |
|----|------|
| **现象** | 2D 雷达是**随机体刚性转动**的。机体一倾斜（pitch/roll），扫描面就跟着倾斜 ⇒ **一部分光束打到地面** ⇒ 这些地面回波进入 `guard_velocity` 的 `corridor_clearance()`，被当成"前方有障碍" ⇒ **误减速**，严重时 `BRAKING_REQUIRED` 直接返回 `(0,0)` **急停**。 |
| **证据** | `grep -n "ground\|r20\|r21\|attitude\|pitch\|roll\|tilt" swarm_agent.py` ⇒ 除第 141 行一条注释外 **0 命中**，无任何姿态补偿。 |
| **触发场景** | 你们**大倾角**的时机不少：起飞爬升（`:141-143` 注释自己提到"速度环大倾角"）、ORCA 急转、制动减速。 |
| **我方现成资产** | `filter_ground_beams(msg, r20, r21, z_sensor)` —— `radar_avoid.py:378-410`，已接入实飞并有回归。 |

**算法（逐束几何剔除，非启发式）**

```
d_z  = r20·cos(a) + r21·sin(a)        # 该束在世界系的 z 分量
       (r20, r21 = 机体→世界旋转矩阵第三行的前两个元素，由四元数推得)
d_z >= -0.05  ⇒ 上仰/近水平束，原样保留（绝不动）
d_z <  -0.05  ⇒ r_ground = -z_sensor / d_z   # 与地面的交点斜距
                 |r - r_ground| <= 容差 ⇒ 判为地面回波 ⇒ ranges[i] = inf
容差 = 0.6 + 0.06·r_ground   （近处紧、远处松）
z_sensor = local_z + 0.08    （传感器离地高度 = 高度 + 雷达安装 z）
```

**为什么安全**：只在 `d_z < 0`（射线确实朝下）**且**读数落在预测地面截距容差内才剔除；
水平飞行时 `d_z≈0` ⇒ 过滤器自然不动作 ⇒ **与你们既有行为零冲突**。

**接法**（两处调用，同一个纯函数）：

1. **瞬时守卫**：`_radar_guard_velocity()` 里，把 `scan.ranges` 过滤后再传给 `guard_velocity()`；
2. **★ 地图喂入**：`_scan_cb()` 里，`observed.feed(msg.ranges, …)` 改用过滤后的 ranges ——
   否则地面回波会在 2D 栅格上累积成**幻影墙**，污染 `_grid_guard_velocity` 与 `online_planner.command_clear` 这两条**主守卫**。

姿态四元数从你们已订阅的 `/mavros/local_position/pose` 取即可（与 `self.yaw` 同源，`self.local_z` 同高度源）；
旋转矩阵第三行 `_zrow` 只需在 `local_position` 回调里多存一行。

---

## B2 ★★ 高度盲带 —— **已实锤的失效模式**，但**修法必须改**（10-04 二次更正）

`guard_velocity(request, measured, yaw, ranges, ...)` 全是 2D 量 ⇒ 对"低空盲带"**零感知**。
这一条**不是推测**：你们自己的 `validation/shared_gpu_run10_teammate_v115/fast_food_contact_geometry.json`
就是这次失效的**现场切片**。

### 实锤证据链

| 量 | 值 | 来源 |
|---|---|---|
| 接触发生仿真时刻 | 2127.020 s（6 号机 × `fast_food_93`） | `docs/teammate_radar_integration_20261004.md` |
| 模型 link 世界位置 | `[8.0, -27.0, 0.0]`，网格 unit=inch，scale `(3,3,2)` | 同上 json |
| **水平切片 2.4 m** | 最近表面 **0.011 m** ⇒ 该高度**有实体** | json `horizontal_slice_analysis` |
| **水平切片 2.95 m** | 最近表面 **0.217 m** ⇒ 仍有实体 | 同上 |
| **水平切片 3.0 m 及以上** | 最近表面 **5.707 m** ⇒ **该 XY 附近没有实体了** | 同上 |
| ⇒ 该处挑檐**顶面 ≈ 2.95 ~ 3.0 m** | | 由切片夹逼 |
| 当时机体中心（world） | **3.15 m** | 你们文档原文 |
| 当时**雷达扫描面** | 3.15 + 0.080 = **3.230 m** ⇒ **在挑檐顶之上** | SDF 机顶贴装 |
| 当时**起落架最低点** | 3.15 − 0.251 = **2.899 m** ⇒ **在挑檐顶之下** | SDF left_leg z≈−0.251 |

**3.230 > 2.95 ⇒ 扫描面从挑檐上方掠过 ⇒ 该方向回波为零；2.899 < 2.95 ⇒ 腿撞进去。**
完全对上。这就是 `[alt−0.251, alt+0.080)` 这条 **0.331 m 盲带**的标准症状。

### 为什么"加水平余量 / 给 radius 加高度项"都**无效**

1. **隐形物在雷达里没有回波**——不是"回波被噪声淹没"，是 `ranges` 那个方向**压根没有数**。
   余量、滤波、`radius` 放大，全都作用在"已检测到的返回"上；对**没有返回**的方向不起作用。
2. 给 `guard_velocity.radius` 加高度项会**连带污染 A\***：同一个常量 `1.2` 被
   `OnlinePlanner.__init__(radius=1.2)` 复用为 A* 膨胀半径（`online_radar_planner.py:137
   radius_cells = ceil(radius/resolution)`）。1.2 → 1.55 会让窄通道全部被判不可通行。
3. 高倾斜（俯仰）有时能把扫描面甩到挑檐下方、从而"看得见"，但那是**偶发**的，
   且与 B1 修的"倾斜导致地面回波"是同一枚硬币——不能当机制用。

⇒ **运行时无解。** 这是"单平面 2D 传感器 + 机顶贴装"的物理盲区，不是算法缺口。

### 唯一可控变量：巡航高度 / 场景先验

**按你们当前生效配置**（`coordination/scripts/city_swarm_run.py:81`
`ALT_BASE='2.2', ALT_NLAYER='1', ALT_TARGET_CAP='2.2', CLIMB_DONE_ALT='2.0'`
⇒ **六机同层，local 2.2 m**，世界约 2.5~3.0 m）：

| 高度 | 盲带区间（**顶面落此即隐形**） | 结果 |
|--------|------------------------------|------|
| local 2.2 m（当前） | [1.949, 2.280) | fast_food 挑檐(local≈2.60) 已在盲带**之上** ⇒ 可见 ✔ |
| local 2.8 m（v1.15） | [2.549, 2.880) | 挑檐(local≈2.60) **落进盲带** ⇒ 隐形 ✘ **撞机** |
| local 4.5 m（旧快照） | [4.249, 4.580) | 平移，仍旧存在 |

⇒ 你们"降到 local 2.2 m"**确实把 fast_food 赶出盲带了**，判断正确。
**但盲带只平移、不消除**：换成任何顶面落在 `[1.949, 2.280)`（local）的物体，同样会隐形。

### 建议（B2 的**正确形态**）

**B2 = 高度体检 + 高度纪律，不是守卫改造。** 我方交付 `scene_band_audit.py`：

```bash
# 在装了 Gazebo 模型缓存的机器上（VM / WSL）跑：
python scene_band_audit.py <你当轮的 connectivity.world> \
    --model-root ~/.gazebo/models \
    --model-root /root/robocup_resources/models \
    --alt-world 2.55          # = local 2.2 + 约 0.35 世界偏移
```

它会列出：**顶面落入盲带的对象（必撞、雷达看不见）**、**顶面距盲带下沿 margin 内的对象（预警）**、
以及**未能定高**的（提示加 `--model-root` 重跑）。

**待你们确认的头号嫌疑**（我方无 VM 模型缓存，未能定高）：

| 模型 | 数量 | 疑点 |
|---|---|---|
| `stop_sign_*` | **6** | 杆+牌总高约 2.4~2.7 m；若顶面落进 **local [1.949, 2.280)** ⇒ **6 个隐形杆** |
| `rover_static_*`（polaris） | 20+ | 实测最高 mesh 位姿 z≈**1.94 m**（含顶部传感器）⇒ 距盲带下沿仅约 0.4 m |
| `fast_food_93` | 1 | 挑檐 2.95~3.0 m（**已实测定高**）⇒ 2.2m 配置下安全 |
| `Shelves high 2` | 3 | 碰撞 mesh 0→**6.04 m**（已由我方 DAE 解析算出）⇒ 永不入带 |
| `Shelves high` | 2 | 碰撞 mesh 仅 **0→0.12 m**（占位碰撞体，与 7.2 MB 视觉网格不符）⇒ 实际**无碰撞体积**，飞机会穿模 |

> `Shelves high` 那条顺带提醒：它的 `shelves_high_collision.dae` 高只有 **0.093 m**，
> 而视觉网格 `shelves_high.dae` 有 7.2 MB ⇒ **碰撞代理形同虚设**。若裁判世界同款，
> 靠它当障碍是不可靠的（反过来，你们也别指望它会拦住谁）。

**高度纪律**：一旦体检报出隐形对象，只有两条路——
① 换巡航高度把该高度带让开；② 把该物体的 footprint 当**已知几何**注入规划器做水平绕行。
（②需要场景先验，且必须用**独立于 `1.2` 的常量**，否则会污染 A* 膨胀。）

---

## B3 ✕ 多帧记忆进速度守门 —— **核查后撤回：你们早已实现**

~~`_radar_guard_velocity()` 只传当前帧 `scan.ranges`~~ —— 这只是**两道防线之一**。
另一道是**栅格主守卫**，它已经吃多帧地图：

```
_scan_cb:644   observed.feed(msg.ranges, …)              # 雷达帧 → ObservedMap（多帧累积）
_compute_plan:1603  grid = planner.grid(frozen, …)       # ObservedMap 快照 → 安全栅格
:1617          self._online_safe_grid = grid
_grid_guard_velocity:1202  planner.command_clear(self._online_safe_grid, …)   # 沿方向逐步降速
_send_vel:1465             self._online_planner.command_clear(…)             # 发布前最后一道闸
```

`command_clear` 用的是**与 `guard_velocity` 完全相同的刹车距离模型**（radius 1.2 / latency .5 / brake .5），
并且 `_grid_guard_velocity` 还做 1.0→0.8→0.6→0.4→0.2→停 的**渐进降速**。⇒ **多帧记忆确实已在速度闸里，不是空白。**
**我们不再动它**；B1（含 B1b）只负责保证喂进这张地图的帧**不含幻影地面格**。

---

## B4 ✕ 守卫链顺序 —— **核查后基本撤回**

实测顺序（`:1451-1467`）：

```
_grid/map_guard(:1372) → 加速度限幅 → 高度护栏
  → _radar_guard_velocity(:1451) → _gate(:1452) → _friend_guard_velocity(ORCA)(:1454)
  → route_gate.command_clear(:1458) → online_planner.command_clear(:1465) → 发布
```

ORCA 改向**之后确实有复核**，只不过是**多帧栅格**而非瞬时帧。而栅格是同一批扫描累积的**超集**
（`scan → ObservedMap → grid`），比单帧更可靠 ⇒ 原先担心的"ORCA 把速度推向瞬时可见、栅格未覆盖的墙"窗口**基本不存在**。
⇒ 降级为**无需改动**。（若日后发现地图刷新空窗期过长，再考虑 ORCA 后补一次瞬时复核；当前证据不支持。）

---

## B5 ☆ 多处"急停"（返回 `0,0`）

`_radar_guard_velocity`（雷达陈旧/缺输入）、`_grid_guard_velocity`（图缺失）、
`route_gate`、`online_planner` 都在条件不满足时返回 `(0,0)` **急停**。

- ⇒ 雷达或图**稍有抖动就可能"集体卡死"**（比赛里表现为飞机莫名不动）。
- **建议（可选）**：改为"**限速 + 保持上次安全方向**"，或设一个很小的蠕行速度下限。

---

## 优先级（10-04 更正后）

**B1（含 B1b） > B2 > B5**　　|　　~~B3 撤回（已实现）~~　　~~B4 撤回（已覆盖）~~

其中 **B1 最干净、最可交付**：我方已有经过回归的成品函数，接进你们链路是**两处调用点**的事
（瞬时守卫 + 地图喂入，见下）。

> **B1 已出详版 + 实测数据（2026-10-04 17:20 更新）**：见 `SUGGEST_B1_GROUND_FILTER_20261004.md`。
> 直接调用你们**未改动**的 `guard_velocity` 合成扫描验证：层高 2.8m、俯仰 12°、3.5 m/s、开阔无墙 ⇒ 过滤前返回 **`BRAKING_REQUIRED`(0,0)**，过滤后 `CLEAR`；
> 水平飞行剔除 0 束（逐位一致，零回归）；正前方 8m 真墙过滤前后行为完全一致（不误伤）。含门槛表与可粘贴的改动代码。
> **已按你们真实配置（`ALT_BASE=2.2`）重算**：地面回波从 **6.5°** 起进量程，3.0 m/s 时俯仰 >**11.2°** 即假急停。

*本文件为建议，未改任何源码（B1/B1b 的改动在本地 `D:/robocup/robo_team`，未 commit/push，见详版 §7）。*
