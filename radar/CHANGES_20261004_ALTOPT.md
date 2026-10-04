# 低空保守化 + 高度看门狗 改动画移交清单（2026-10-04 十五次修正）

> 用途：交给另一个模型/另一台机器**照单施工**。每项给出文件、定位锚点、
> 旧代码、新代码、理由。施工完后按文末"验收"逐条跑，全绿才算完。
>
> 本清单对应的**已验证实现**在：
> `C:\Users\guo\Desktop\robocup\_radar_deliver\radar\radar_avoid.py`
> （备份：`radar_avoid.py.bak_pre_altopt_20261004`）。如允许直接拷贝，
> 以该文件为准，本清单用于评审与复现。

## 背景（为什么改）

队友用本模块在 **2.45 m 巡航撞房屋一次**。代码层根因：

1. **避障对高度零感知**：`scan_to_obstacles` 只用 `mount[0], mount[1]`
   （源码注释原文"仅 mx,my 参与 XY 障碍投影，z 不参与运算"），
   `BODY_RADIUS=0.45` 是水平圆盘，全脚本无任何高度阈值比较。
2. **2D 单平面雷达存在低空盲带**：扫描面 = 机体高度 + mount_z(+0.08)，
   2.45 m ⇒ 扫描面 ≈2.53 m；机体（含起落架/桨盘）实占约 2.10–2.65 m ⇒
   房屋附属结构（院墙/台阶/门廊/雨棚/檐口下沿）在 2.10–2.53 m 高度带
   对雷达完全隐形，却在机体包络内。全部历史验证在扫描面 5.58 m 层做。
3. **高度只发不读**：`step_toward(..., alt)` 把 alt 当常量下发，
   没有任何 |实测z − 期望z| 比较，掉高无人知晓。

改法 = 低空时加大水平余量 + 高度看门狗 + 心跳活性检测。
**所有改动走"缺省不改变行为"框架**：新参数缺省时与历史版本逐字节一致。

> ⚠️ 曾经设计过第 4 项"启动期 `--alt` 下限守卫（<2.4m 拒绝启动）"，
> **已作废删除**。原因：正式比赛一旦起飞就无法重开，巡航高度是起飞前
> 由人设定的，运行期不会再变 —— 拦在启动既拦不住既成事实，又可能在
> "必须起飞"的窗口里把整场比赛废掉。低空的后果一律改由**运行期**承接
> （改动 1/2/3/4 的 clearance_bonus + 高度看门狗），二者都不依赖任何
> 启动参数。**本清单不含任何启动期拒绝/告警逻辑。**

## 硬约束（施工时绝对不允许动）

- `GAP_DEV_MAX = 70°` 一行都不许碰（三次放宽全部严重回归，有案可查）。
- `find_free_gap` / `lateral_mins_fullres` / `scan_to_obstacles` 的
  投影链与判据内部一行都不许改，只能在**调用处**传参。
- 不许改 `RANGE_MIN/RANGE_MAX/BEAM_COUNT`（规则要求不得改模型参数）。
- 所有新参数必须缺省兼容：不传 = 旧行为。

---

## 改动 1：新增常量与纯函数 `clearance_bonus`

**文件**：`radar/radar_avoid.py`
**锚点**：`CLEARANCE = BODY_RADIUS + SAFE_GAP`（约 104 行）之后。

旧代码（锚点）：
```python
SAFE_GAP = 0.35             # 附加安全间隙（控制超调 + 定位误差）
# 有效净空：中心到障碍表面必须大于此值
CLEARANCE = BODY_RADIUS + SAFE_GAP
```

新代码（追加在锚点之后）：
```python
SAFE_GAP = 0.35             # 附加安全间隙（控制超调 + 定位误差）
# 有效净空：中心到障碍表面必须大于此值
CLEARANCE = BODY_RADIUS + SAFE_GAP

# ---- 扫描面高度 → 附加余量（2026-10-04 十五次修正：低空保守化）----
# 兼容性（必须遵守）：clearance_bonus(None) == 0，且 scan_z >= SCAN_Z_REF
# 时恒为 0 ⇒ 不传 scan_z / 在历史高度层调用，行为与历史版本逐字节一致。
SCAN_Z_REF = 4.0            # 扫描面 ≥ 此高度：历史验证层，补偿恒为 0
SCAN_Z_FLOOR = 2.0          # 扫描面 ≤ 此高度：补偿取满
ALT_BONUS_MAX = 0.4         # 低空最大附加水平余量 m（总余量 0.8→1.2）

def clearance_bonus(scan_z):
    """扫描面高度（世界系 m）→ 附加水平安全余量（纯函数）。

    scan_z >= SCAN_Z_REF → 0.0（零回归保证）；
    SCAN_Z_FLOOR ~ SCAN_Z_REF 之间线性插值；
    scan_z <= SCAN_Z_FLOOR → ALT_BONUS_MAX。
    """
    if scan_z is None or scan_z >= SCAN_Z_REF:
        return 0.0
    if scan_z <= SCAN_Z_FLOOR:
        return ALT_BONUS_MAX
    return ALT_BONUS_MAX * (SCAN_Z_REF - scan_z) / (SCAN_Z_REF - SCAN_Z_FLOOR)
```

## 改动 2：`lateral_guard_shift` 增加 clearance 覆盖参数

**锚点**：`def lateral_guard_shift(sub_x, sub_y, lat_x, lat_y,`（约 526 行）。

旧代码：
```python
def lateral_guard_shift(sub_x, sub_y, lat_x, lat_y,
                        left_min, right_min, dilute, amt_cap):
    """侧向守门动作（2026-10-03 从 subgoal_from_scan 内联逻辑抽出）。
    ...（docstring 保持原文）...
    dilute = 前瞻稀释补偿系数；amt_cap = 单周期修正幅度上限。
    """
    want_lat = CLEARANCE + SAFE_GAP          # 理想：两侧各留这么多
```

新代码：
```python
def lateral_guard_shift(sub_x, sub_y, lat_x, lat_y,
                        left_min, right_min, dilute, amt_cap,
                        clearance=None):
    """侧向守门动作（2026-10-03 从 subgoal_from_scan 内联逻辑抽出）。

    纯函数：输入当前子目标与两侧横向净空，返回修正后的 (sub_x, sub_y)。
    语义与原内联实现逐行一致：
      · 两侧都有障碍 ⇒ 通道居中；通道太窄退到"安全底线"；
      · 只有单侧 ⇒ 推开到 want_lat（上限 INFLUENCE）；
      · 两侧皆无 ⇒ 原样返回。
    dilute = 前瞻稀释补偿系数；amt_cap = 单周期修正幅度上限。
    clearance: 可选的有效净空覆盖（默认 CLEARANCE）。2026-10-04 新增：
      低空保守化时由调用方传入 CLEARANCE+clearance_bonus(scan_z)；
      None ⇒ 与历史行为逐字节一致。
    """
    base_clr = CLEARANCE if clearance is None else clearance
    want_lat = base_clr + SAFE_GAP      # 理想：两侧各留这么多
```

函数体内其余代码一行不动。

## 改动 3：`subgoal_from_scan` 增加 scan_z 参数并接线（核心）

**锚点 A**：函数签名 + docstring（约 1215 行）。

旧代码：
```python
def subgoal_from_scan(msg, yaw, pos_enu, goal_enu, mount,
                      stride=1, max_range=None, commit=None, attitude=None):
```

新代码：
```python
def subgoal_from_scan(msg, yaw, pos_enu, goal_enu, mount,
                      stride=1, max_range=None, commit=None, attitude=None,
                      scan_z=None):
```
并在 docstring 中 attitude 段落之后追加说明段：
```
    🔴🔴 2026-10-04 新增 scan_z 参数（扫描面世界高度，低空保守化）：
    传入了就在 CLEARANCE 上叠加 clearance_bonus(scan_z)，
    影响 need_swerve 判据 / look 前推 / 势场权重分母 / find_free_gap 的
    空/堵阈值 / 侧向守门的 want_lat。None（离线仿真与历史调用方默认）
    ⇒ bonus=0 ⇒ 与历史版本逐字节一致，全部回归零影响。
```

**锚点 B**：地面过滤块之后（约 1295 行）。

旧代码：
```python
        msg = _ScanView(msg, fr)

    cur = pos_enu
```

新代码：
```python
        msg = _ScanView(msg, fr)

    # ---- 低空保守化（2026-10-04）：扫描面越低，附加水平余量越大 ----
    # scan_z=None（历史调用方/离线仿真默认）⇒ clr == CLEARANCE，逐字节不变。
    clr = CLEARANCE + clearance_bonus(scan_z)

    cur = pos_enu
```

**锚点 C**：三处 `CLEARANCE` → `clr`（只改这三处，全文件搜索
`BLOCK_GAP + CLEARANCE`、`gap_r - CLEARANCE`、`INFLUENCE - CLEARANCE)`
各出现一次）：
```python
    need_swerve = (front_hit is not None and
                   front_hit < BLOCK_GAP + clr)          # 原 CLEARANCE
...
        look = gap_r - clr                              # 原 CLEARANCE
...
                    wgt = (INFLUENCE - qd) / max(
                        1e-6, INFLUENCE - clr)          # 原 CLEARANCE)
```

**锚点 D**：`find_free_gap` 调用（约 1435 行）。

旧代码：
```python
    gap_ang, gap_r, gap_w = find_free_gap(msg, yaw, side=side, want_ang=want,
                                          stride=GAP_STRIDE, heading=heading)
```

新代码：
```python
    gap_ang, gap_r, gap_w = find_free_gap(msg, yaw, side=side, want_ang=want,
                                          stride=GAP_STRIDE, heading=heading,
                                          clearance=clr + SAFE_GAP)
```

**锚点 E**：两处 `lateral_guard_shift` 调用（约 1625、1645 行），各加
`clearance=clr`：
```python
                sub_x, sub_y = lateral_guard_shift(
                    sub_x, sub_y, lat_x, lat_y,
                    left_min, right_min, _dilute, _amt_cap,
                    clearance=clr)
...
                    sub_x, sub_y = lateral_guard_shift(
                        sub_x, sub_y, lat_x, lat_y,
                        l_min, r_min, _dilute, amt_cap,
                        clearance=clr)
```

## 改动 4：`step_toward` —— 高度看门狗 + 心跳活性检测 + scan_z 传递

**锚点**：`def step_toward(self, tx, ty, alt, lookahead):`（约 2420 行）
函数体开头。

旧代码：
```python
        cur = self.pose
        hd = self.heading
        if cur is None:
            return 0.0, 0.0, 0.0, None
        dist = math.hypot(tx - cur[0], ty - cur[1])
        if dist < 1e-6:
            self.set_sp(cur[0], cur[1], alt, hd)
            return cur[0], cur[1], 0.0, None

        sub_x, sub_y, shift, nearest, blocked = subgoal_from_scan(
            self.scan, hd, cur, (tx, ty), self.mount, stride=self.stride,
            commit=self.commit, attitude=self.attitude)
```

新代码：
```python
        cur = self.pose
        hd = self.heading
        if cur is None:
            return 0.0, 0.0, 0.0, None

        # ---- 高度看门狗（2026-10-04 十五次修正）----
        # 此前 alt 是只发不读的常量，全脚本没有任何 |实测z − 期望z| 的比较
        # ⇒ 掉高 0.35m 无人知晓，而避障对高度零感知，继续按巡航高度做水平
        # 决策 ⇒ 机体钻进 2.10–2.53m 的低空盲带（院墙/台阶/雨棚/檐口不可见）。
        # 做法：偏差超 tol 连续 alt_watch_frames 帧 ⇒ 暂停水平推进，只发
        # (cur.x, cur.y, alt) 原地纠高度，并告警。alt_tol<=0 ⇒ 关闭。
        if self.alt_tol > 0.0 and len(cur) >= 3:
            if abs(cur[2] - alt) > self.alt_tol:
                self._alt_dev_frames += 1
                if self._alt_dev_frames >= self.alt_watch_frames:
                    rospy.logerr_throttle(
                        2.0, "[radar] 高度看门狗：实测 %.2f 期望 %.2f（偏差 %.2f > tol %.2f，"
                             "连续 %d 帧）⇒ 暂停水平推进，先纠高度",
                        cur[2], alt, cur[2] - alt, self.alt_tol,
                        self._alt_dev_frames)
                    self.set_sp(cur[0], cur[1], alt, hd)
                    return cur[0], cur[1], 0.0, None
            else:
                self._alt_dev_frames = 0

        # ---- 心跳活性检测（2026-10-04 十五次修正）----
        # _sp_loop 已兜异常（十四次修正），但"线程活着却发不出去"仍无检测。
        _now = time.time()
        _cnt = self._sp_sent
        if _cnt == self._hb_last[1]:
            if _now - self._hb_last[0] > 1.0:
                rospy.logerr_throttle(
                    5.0, "[radar] 设定点心跳疑似停滞：_sp_sent=%d 已 %.1fs 未增长",
                    _cnt, _now - self._hb_last[0])
        else:
            self._hb_last = (_now, _cnt)

        dist = math.hypot(tx - cur[0], ty - cur[1])
        if dist < 1e-6:
            self.set_sp(cur[0], cur[1], alt, hd)
            return cur[0], cur[1], 0.0, None

        # 扫描面世界高度 = 位姿 z + 雷达安装高度 mount[2]（缺省 0.08）。
        _scan_z = cur[2] + self.mount[2] if len(cur) >= 3 else None
        sub_x, sub_y, shift, nearest, blocked = subgoal_from_scan(
            self.scan, hd, cur, (tx, ty), self.mount, stride=self.stride,
            commit=self.commit, attitude=self.attitude, scan_z=_scan_z)
```

## 改动 5：`RadarPilot.__init__` 增加参数与状态

**锚点**：`def __init__(self, uav, dry_run=False, ns=None, scan_topic=None,`
（约 1700 行）。

旧代码：
```python
    def __init__(self, uav, dry_run=False, ns=None, scan_topic=None,
                 pose_source='gazebo', nav_hold=30.0):
```

新代码：
```python
    def __init__(self, uav, dry_run=False, ns=None, scan_topic=None,
                 pose_source='gazebo', nav_hold=30.0, alt_tol=0.15,
                 alt_watch_frames=15):
```

**锚点**：`self.pose_source = pose_source`（约 1745 行）。

旧代码：
```python
        self.uav = uav
        self.dry = dry_run
        self.pose_source = pose_source
```

新代码：
```python
        self.uav = uav
        self.dry = dry_run
        self.pose_source = pose_source
        # 高度看门狗（2026-10-04 十五次修正）：|实测z − 期望alt| 超 tol
        # 连续 alt_watch_frames 帧 ⇒ 暂停水平推进先纠高度。tol<=0 关闭。
        self.alt_tol = float(alt_tol)
        self.alt_watch_frames = int(alt_watch_frames)
        self._alt_dev_frames = 0
        self._hb_last = (0.0, -1)      # 心跳活性检测：(时间, _sp_sent)
```

## 改动 6：`main` —— 新增 `--alt-tol / --alt-watch-frames`（**无任何启动期拦截**）

**锚点 A**：`ap.add_argument('--alt', ...)`（约 2900 行）。

旧代码：
```python
    ap.add_argument('--alt', type=float, default=2.8,
                    help='巡航高度 m（世界系）。比赛实测 2.5~3m，默认取中值 '
                         '2.8；规则红线是高度 >6m 记 0 分')
```

新代码：
```python
    ap.add_argument('--alt', type=float, default=2.8,
                    help='巡航高度 m（世界系）。比赛实测 2.5~3m，默认取中值 '
                         '2.8；规则红线是高度 >6m 记 0 分。高度越低，"雷达'
                         '看不见却在机体包络内"的盲带越宽 —— 本脚本不拦截'
                         '任何启动参数（比赛不可重开）；低空后果由运行期'
                         '承接：clearance_bonus 按 scan_z 自动加大净空 + '
                         '高度看门狗守住实际高度。建议 ≥2.8m')
    ap.add_argument('--alt-tol', type=float, default=0.15, dest='alt_tol',
                    help='高度看门狗容差 m（默认 0.15）：|实测z − 巡航alt| 超此值'
                         '连续 --alt-watch-frames 帧 ⇒ 暂停水平推进先纠高度。'
                         '<=0 关闭（旧行为）')
    ap.add_argument('--alt-watch-frames', type=int, default=15,
                    dest='alt_watch_frames',
                    help='高度看门狗连续超差帧数（默认 15，主循环 20Hz ⇒ 0.75s）')
```

**锚点 B**：`args = ap.parse_args()` 之后（约 2960 行）。

旧代码：
```python
    args = ap.parse_args()

    rospy.init_node('radar_avoid', anonymous=True)

    pilot = RadarPilot(args.uav, dry_run=args.dry_run,
                       ns=args.ns, scan_topic=args.scan_topic,
                       pose_source=args.pose_source,
                       nav_hold=args.takeoff_hold)
```

新代码：
```python
    args = ap.parse_args()

    # ---- 2026-10-04：启动期高度守卫已移除（比赛不可重开 ⇒ 拦截无意义）----
    # 原设计 --alt<2.4 拒绝启动、2.4~2.6 告警 —— 实为无效代码：正式比赛一旦
    # 起飞就无法重开，参数在起飞前由人设定，运行期不会再变；拦在启动既拦不住
    # 既成事实，又可能在"必须起飞"的窗口里把飞机废掉。低空后果一律改由运行期
    # 承接：①高度看门狗 ②clearance_bonus 按 scan_z 自动加大净空。
    rospy.init_node('radar_avoid', anonymous=True)

    pilot = RadarPilot(args.uav, dry_run=args.dry_run,
                       ns=args.ns, scan_topic=args.scan_topic,
                       pose_source=args.pose_source,
                       nav_hold=args.takeoff_hold,
                       alt_tol=args.alt_tol,
                       alt_watch_frames=args.alt_watch_frames)
```

**不要**加入任何形式的 `ap.error(...)` / `sys.exit(...)` / 启动期 `logwarn` 门槛
判定 —— 这是本清单的硬性否定项。

## 改动 7：`test_radar_offline.py` 的 install_stubs 补 EstimatorStatus

**理由**：`radar_avoid.py` 顶部 `from mavros_msgs.msg import EstimatorStatus, ...`，
打桩缺这个类会导致所有离线测试 import 失败。

**锚点**（test_radar_offline.py 约 85 行）：

旧代码：
```python
    mvm.ParamValue = _Simple
    mvm.State = _Simple
```

新代码：
```python
    mvm.ParamValue = _Simple
    mvm.State = _Simple
    mvm.EstimatorStatus = _Simple   # 2026-10-04：radar_avoid 引入 EKF 健康判据
```

## 改动 8：新增回归测试 `test_low_alt_opt.py`

**新文件**：与 radar_avoid.py 同目录。完整内容直接从
`C:\Users\guo\Desktop\robocup\_radar_deliver\radar\test_low_alt_opt.py`
拷贝（19 项检查：clearance_bonus 纯函数 8 项、subgoal_from_scan 端到端
兼容+生效 5 项、lateral_guard_shift 3 项、**启动期不拦截** 3 项）。
D 段用假 `RadarPilot` 打桩，验证 `--alt 2.0/2.45/2.8` 一律走到 `wait_ready`
（返回 1）而**不**抛 `SystemExit(2)` —— 若哪天有人把守卫加回来，这条会红。

---

## 验收（施工完成后逐条执行，全部通过才算完）

在 radar/ 目录下（Windows 或 WSL 均可，纯 Python 不需要 ROS）：

| # | 命令 | 预期 |
|---|---|---|
| 1 | `python3 test_low_alt_opt.py` | Ran 19 checks, 0 failed |
| 2 | `python3 test_radar_offline.py` | 总计 35 项，失败 0 项 |
| 3 | `python3 test_target_follow.py` | 总计 31 项，失败 0 项 |
| 4 | `python3 sim_radar_closed_loop.py` | 5 场景全部到达、碰撞帧 0、最小净空 >0.80 |
| 5 | `python3 online_planner.py --selftest` | 全部通过 PASS |
| 6 | `python3 occupancy_online.py --selftest` | 全部通过 PASS |
| 7 | `RADAR_DEMO_DIR=<含 astar_plan.py 的 _demo2026 目录> python3 test_two_layer.py` | 两级集成 4/4 通过 |
| 8 | `python3 random_map_fly.py --gen-dir <基线 _gen_20260929> --n-route 18 --seed 11` | 108/108 到达、0 碰撞、全局最小净空 ≈1.079 m（与改动前基线一致 ⇒ 零回归） |

关键判定标准：
- **#1 的"兼容"项**：`scan_z=None / 4.0 / 5.5` 三者输出必须逐字节相同
  （这是零回归的证明）。
- **#1 的"生效"项**：同一帧墙在扫描面 2.45 m 时子目标必须比 5.5 m 时
  离墙更远（实测 2.200 > 1.956 m）。
- **#8 的净空数字必须与改动前基线一致**（1.079 m）：本改动只应在
  传入 scan_z 时（即实飞 step_toward 路径）生效，离线链路缺省不传，
  任何数字漂移都说明接线有误。

## 已实测结果（2026-10-04 本清单实现版）

| 套件 | 结果 |
|---|---|
| test_low_alt_opt | 19/19 |
| test_radar_offline | 35/35 |
| test_target_follow | 31/31 |
| test_two_layer（WSL + RADAR_DEMO_DIR） | 4/4 |
| sim_radar_closed_loop | 5/5（最小净空 0.964 m） |
| online_planner / occupancy selftest | PASS / PASS |
| random_map_fly 基线 | 108/108、0 碰撞、最小净空 1.079 m（与基线一致） |

## 已知边界（不要指望这次改动解决）

- **悬挑物（雨棚/檐口下沿/挑檐）对任何单平面 2D 雷达无解**；
  本清单只消"平面以下"盲带的暴露面（低空余量加大），结构性正解是
  把雷达安装位置下移到起落架下沿（另案，需全量重测，不在本次范围）。
- 队友那次碰撞的**根因定案**仍需要碰撞瞬间的接触记录 + 扫描帧 dump；
  本清单是"无论根因是 2D 盲带还是掉高，都先把这两类洞补上"。
