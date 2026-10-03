# UAV 单机全链路改动说明（感知 → 建轨 → 飞控 → 雷达避障）

日期：2026-10-03
分支：`uav-single-link-20261003`（基于 `main`，未合并 `codex/radar-coordination-20261003`）
涉及文件：改 `yolo_target_bridge.py`、`perception_real.py`、`swarm_agent.py`；
新增 `launch/robocup_single.launch`、`launch/single_vehicle_spawn_xtd_delayed.launch`、
`run_match_single.sh`、`scripts/yolo_flyer.py`、`scripts/radar_guard.py`、
`scripts/verify_score.py`、`scripts/verify_range.py`

> 本分支只做**单机**链路。协同层（`codex/radar-coordination-20261003` 那条）一行未动。

---

## 0. 三个必须知道的硬 bug（不修，红球永远 0 分）

| # | 文件 | 问题 | 后果 |
|---|---|---|---|
| 1 | `yolo_target_bridge.py` | 发出去的 `am.cls` 是 `'red1'/'red2'`，而官方 `actor_id_dict` 的键是 **`'red'`** | 两个 red 回调都匹配不到 ⇒ 红球**永不消除** |
| 2 | `yolo_target_bridge.py` | 红球 `TAG_TO_TID` 映射反了（原 `red1→t4 / red2→t5`，官方是 `red1→actor_5 / red2→actor_4`） | `set_left()` 把"红球被消除"记到**错的流**，补发错误 `eliminated`，活着的球被摘掉 |
| 3 | `perception_real.py` | `observation_id = "obs-<tag>-<seq>"`，`seq` 是**每机本地计数** | 六机会撞同名 id ⇒ `OBSERVATION_CONFLICT`，**整条观测被拒收** |

**修法**：1 用 `OFFICIAL_CLS_OF_TAG` 映射；2 用 `ACTOR_INDEX_OF_TAG` 修正；3 给 `observation_id` 加 UAV 前缀。

---

## 1. `yolo_target_bridge.py`（+431）

- 上面 1、2 两个 bug。
- **播报距离闸门补上滞回**：原代码 `GATE_RETRY_S` 从未被引用、`_gate_hold` 只写不读 —— **滞回其实从未生效**。现为 12 m 进 / 15 m 出。
- **播报保持器**：防止判断成立期间出现 >1 s 静默（官方要求间隔 ≤1 s）。
- **红球双流 `RED_DUAL`**：同一坐标同时向两条 red 流播报（两个红球在视觉上不可分，见 §5）。

## 2. `perception_real.py`（+116）

- **相机高度标定 `PZ_SCALE=0.941`**：测距 rng 存在系统性偏长 6.4%。
- **近目标"脚被裁"**：改用身高反推测距（曾被报成 7.09 m → 10.61 m）。
- **远距离小框 AR 放宽**：白球 19×46 px 的小框被几何门拒掉 ⇒ 2996 次检测 0 条 track。
- **静止例外**：本套 world 里 actor 默认不动，原运动门会把它们全部挡掉。
- 对外发布做平滑。

## 3. `swarm_agent.py`（+107）

- **`FLIGHT_OUTPUT=pos` 位置设定点通道**：纯速度通道下垂向跟踪成功率只有约 20%，且地面 `vz>0` 根本起不来（飞机趴地）。
- 确认被重置时**先判"够不着"**，不再一律当成"播报被拒"而退避 60 s。

## 4. 新增文件

| 文件 | 作用 |
|---|---|
| `scripts/yolo_flyer.py` | 我方飞行器：起飞 → 搜索 → 跟踪 → 报点。**不订阅任何真值话题** |
| `scripts/radar_guard.py` | 雷达避险层：**只修正目标点，不发布任何 setpoint**（保持"单一 setpoint 发布者"铁律） |
| `launch/robocup_single.launch` | 单机起场。**只把 `sdf` 换成 `typhoon_h480_lidar`，`vehicle` 不动 ⇒ 话题名全不变**，只多一路 `/typhoon_h480_0/scan` |
| `launch/single_vehicle_spawn_xtd_delayed.launch` | 单机延时 spawn（`spawn_delay=45`，按**仿真时间**计） |
| `run_match_single.sh` | 单机编排：`./run_match_single.sh yolo 7 2.5 0.9`（检测器 / keep / **高度** / 速度） |
| `scripts/verify_score.py`、`verify_range.py` | 判定与测距自检工具 |

### 高度与巡逻参数（`yolo_flyer.py`）

- `ALT_DEFAULT = 2.5` m，代码硬顶 5.0 m；官方红线是 **>6 m 直接 score=0**。
- 定 2.5 m 的依据：**3.6 m 时白球 conf 掉到 0.265**（低于 0.40 阈值 ⇒ 漏检），3.0 m 时为 0.689。
- 改高度不用改代码：第三个位置参数即是。

---

## 5. 传下去之前必须叮嘱三件事

1. **`BRIDGE_NEW_TRACK_CONF` 保持 0.7，不要降。**
   该常量注释里记的"3 条 0.43~0.61 的 red1 误检建出鬼影轨"正是它治的病。
   我方单机脚本一度降到 0.35，结果红色场景物（conf 0.30~0.56）全部能建轨、飞机去追**消不掉的假目标**。本分支已改回 0.7 并加注释禁止再降。
2. **`PR_PZ_SCALE=0.941` 是按这一台相机标定的**，换机型 / 改安装位置必须重新标定。
3. 感知上报的 `confidence` 是**原始 YOLO conf**，不是含身高/运动因子的 `score`；评估建轨阈值以原始 conf 为准。

### 红球纪律

两个红球（`actor_4` / `actor_5`）在本世界视觉上**完全不可分**。
因此：一次只盯一个球，坐标同一时刻向 red1 与 red2 两条流同发（`red1→actor5`、`red2→actor4`）。

---

## 6. 未验证项（别当已验证的用）

- **红球双流 `RED_DUAL`**：10-03 单机跑通那轮走的是"一次盯一个"的旧路径，双流没上机。
- **雷达 `--peers` 六机图互联**（在 `Zhibanyuan/robocup-yolo`，不在本仓）：单机回归不受影响，六机没跑过。
- **自主巡逻 `SEARCH_PTS`**：代码齐全（15 个点覆盖 x∈[-40,120]×y∈[-45,45]，贪心最近邻排序，每点原地自转一圈），但受下面 §7 影响**从未被真正进入过**。

## 7. 已知环境问题（非本分支引入）

- **官方原版 actor 插件在 Gazebo 11 下无法定位 actor**：`ActorPluginRos.cpp` 用 `actor->WorldPose()`，而 Gazebo 11 对 Actor 恒返回 0，导致每帧被 `SetWorldPose(init_pose)` 按回，6 个 actor **全叠在原点 (0.41, 0.02)**。这是唯一需要改动 actor 插件的原因。
- **感知对红色小物体过敏**：红色消防栓 / STOP 牌 / 加油站红标会让单帧 `n_det` 冲到 1000+。只烧算力，不直接扣分；真正的危害是误建轨后被追。处置即 §5 第 1 条。

---

## 8. 单机全链路实测（2026-10-03，官方随机地图，零注入）

- 官方 `map_generator.py` 生成随机图（80110 行，black_box 47 项）。
- 机型 `typhoon_h480_lidar`：`/typhoon_h480_0/scan` 231~234 Hz。
- 雷达避险层：1100 次调用 → 改向 454 / 透传 646 / 异常 0，最大横向修正 1.07 m。
- **完整闭环（首次，零人工注入）**：`left_actors [0..5] → [0,1]`、`score = 1175`、6× tracking success、4 次消除。
  对账：`6×50 + 6×80 + 4×100 − 5.1 = 1174.9` ✓

---

## 9. 从队友分支搬入的改动（2026-10-03 晚）

来源两个队友分支，都是**已经他们自己实测过**的东西，只搬实现、不改逻辑：

| 来源分支 | 搬了什么 |
|---|---|
| `gyccsjtu/robo_team` → `codex/radar-coordination-20261003` | 飞控参数读回校验、六机相机端口去冲突 |
| `Zhibanyuan/robocup-yolo` → `mavros-port-iris` | 裁判上报确认门、协同上报 confidence 语义 |

### 9.1 `perception_real.py`：协同上报 confidence 改成「方案 A」（默认已开）

协同核心 `coordination/core.py::_target()` 第一句是 `if d['confidence'] < 1.: return`，
它的约定是**上游负责确认判定、只有已确认目标才发 1.0**。

- 我方此前是**无条件发 1.0**，比队友还激进：连未确认的误检也会被核心直接吃下去
  （叠上"红色过敏"就是让全队去追一个消不掉的假目标）。
- 现在：`PR_COORD_CONF_ONE=1`（**默认**）⇒ 已确认 track（`hits >= CONFIRM_HITS`）
  发 1.0，未确认的照旧发真实 YOLO 置信度，核心自然忽略。

⚠️ 原始 conf 仍原样留在调试快照与裁判侧，离线复盘不受影响。

### 9.2 `perception_real.py`：新增裁判上报「确认门」（默认关）

官方 `score_cal.py` 只比 x,y 误差（<1m）与上报间隔（<1s），**完全不看 confidence**。
未确认 track 大多是活不过几秒的误检，一旦发到 `/actor_*_info`，就会以 20~40m 的
误差把裁判 15s 连续计时**清零**（清零 = 永远消除不了）。

- `PR_ACTOR_CONFIRM_ONLY=1` ⇒ 仅已确认 track 上报；`0`（**默认**）= 旧行为。
- 与已有的 `ACTOR_PUB_RANGE` 是互补两道门：距离门挡"远而持续"的静态误检
  （灯柱在 17~23m，真目标 6~9m），确认门挡"近而短命"的误检（靠距离切不开）。

### 9.3 `fcu_configuration.py`（新文件）+ `swarm_agent._configure_fcu`：参数读回

老实现只 `set`、不看返回值。MAVROS 参数表未就绪时 `param/set` 会被**静默拒绝**，
飞机带着未生效的 `NAV_RCL_ACT`/`COM_RCL_EXCEPT` 继续 arm/OFFBOARD，最后在 RC 失联
failsafe 上被拦 —— 表象是"解锁失败"，根因在几百行之前。

现在 `pull → set → get` 读回验证，三轮不一致抛 `FCU_CONFIGURATION_NOT_VERIFIED`
终止启动。**宁可引起不来，也不要带着未生效参数上天。**

### 9.4 `scripts/patch_px4_camera_ports.py`（新增，六机前才需要）

`6011_typhoon_h480.post` 里相机 MAVLink 端口写死 `14558/14530`，六机同跑时
**互相抢绑**，表现是相机流时断时续，而 PX4 照常启动、不报错（又一个静默失效）。

用法：只把端口改成 `14600+instance` / `14630+instance`，**改的是该轮私有 etc 副本，
原构建文件一字不动**。单机跑法零影响。

```bash
python3 scripts/patch_px4_camera_ports.py \
    /data/PX4_Firmware/build/px4_sitl_default /tmp/robocup_six/px4_etc
# 之后 px4 -d /tmp/robocup_six/px4_etc -s etc/init.d-posix/rcS -i <n> -w <work>
```

### 9.5 这几处的验证状态

- 三处代码改动：`ast.parse` 语法校验通过；**尚未上机复跑**（VM 上那份还没同步）。
- 端口脚本：逻辑照搬队友已验证实现，**未在本机六机场景执行过**。

