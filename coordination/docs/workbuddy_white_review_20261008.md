# 白色人物识别诊断（WorkBuddy → Codex 复核）

> ## ⚠️ 本报告部分结论已被 `workbuddy_white_visibility_20261008.md` 更正
> 依据 `workbuddy_white_review_codex_20261008.md` 的复核，本报告以下内容**作废**：
> 摘要与正文身高数字矛盾（实为 1.065711 / 2.096977 / 2.026639）、"无一落在 1.4–2.1 m 成人带"
> （其中两条在带内）、"两条独立判据"（实为同源）、"150 m 落差可证明框不是目标"（降级）、
> "真人从未进入有效观测"（应改为"入画但过远"）、"45 帧模型零输出"（仅 3 条有独立人体字段）、
> 机号"uav_2/uav_4"（应为 uav_3/uav_5）。**以 `workbuddy_white_visibility_20261008.md` 为准。**
> 仍然成立：单张留存白图颜色 conf 0.5388、person 模型 conf 0.25→0.01 全 0 框、
> 白色观察器原图坐标与官方上报均为 0。

> 任务单：`WORKBUDDY_TASK_20261008.md`。**离线只读**，未启动/停止任何仿真进程，
> 未改 `perception_real.py` / `visual_observation.py` / 桥 / manager / agent / 路线与 STOP 核心。
> 数据源：WSL `/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159`
> （run_id `e2052eb7-090c-4327-adbd-02fca4fb0eb2`，执行提交 `462fa8e`，40Hz，seed 8159）
> 与 `coordination/docs/validation/legacy_chain_v140_20261008`。诊断真值仅离线核验，未进入控制。

---

## 0. 结论摘要（三句话）

1. **白色候选的拒绝点是人体关联层**：全部白框 `n_person_boxes=0`、`person_iou=0.0`、
   `reject_reason=no_person_overlap` —— 该帧 **person 模型整图零输出**（conf 从 0.25 扫到
   **0.01 仍为 0 框**），所以**不是门限问题**。
2. **但本次全部被拒白框都不是白色真人**：隐含身高 **1.08 / 2.27 / 2.42 m**（真值 1.75 m），
   且首帧相机距白色真人真值 **150.1 m**（白框报告 range 仅 18.85 m）。
   ⇒ **拒绝对了**，不是"漏掉真人"。
3. **真缺口在更前一层：白色真人从未进入有效观测**。整轮 white 观察器
   `original_image_coordinates.total = 0`、`official_report_coordinates.total = 0`，
   而白色真人真值样本 **2303 条**（活跃至 2069.27 s）。
   ⇒ 这是**发现/接近**的缺口，不是"人体模型对真人失明"。

---

## 1. 原图索引与证据文件

| 项 | 路径（WSL 根 `/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159`） |
|---|---|
| 白框失败原图（唯一一张） | `flight/algorithm/evidence_typhoon_h480_2_white_fail/frame_000001_1791448706145.png`（173656 B） |
| 白框失败索引 | `flight/algorithm/evidence_typhoon_h480_2_white_fail/index.jsonl` |
| 六机感知事件流 | `flight/algorithm/perception_typhoon_h480_{0..5}.csv` |
| 相机位姿/内参 | `flight/algorithm/search_camera_frames.jsonl`（1827 行；fx=fy=205.4696, cx=320.5, cy=180.5, 640×360） |
| 白色独立观察器（真值） | `observers/independent_visual_accuracy_white.json` / `.jsonl`（2303 条 `actor_truth`，格式 `[t, x, y, z]`） |
| 观察器汇总 | `observers/independent_visual_accuracy.json` |
| 白框 → 桥 | `flight/algorithm/bridge_trace.jsonl` |

**缺失（明确标注）**：本轮**没有任何"近处白色真人"的原图**。
上述唯一白图（h480_2）经核对**不含白色真人**（§2.3）。
⇒ 现有证据**无法判定** person 模型对真白人是否失明。

## 2. 已验证结论

### 2.1 拒绝位置：颜色层通过 → 人体关联层拒绝 → 轨迹层重复拒

以 h480_2 那张白图（`image_stamp=1999.208`，`path=shared`）为例：

| 层 | 记录 | 值 |
|---|---|---|
| 颜色 | `evidence_..._white_fail/index.jsonl` | `cls=white, color_conf=0.5388`（> 0.40 颜色门，通过）|
| 人体 | 同上 | `n_person_boxes=0`、`person_iou=0.0`、`person_conf=null` ⇒ `reject_reason=no_person_overlap` |
| 轨迹 | `perception_typhoon_h480_4.csv` | `person_hits=0`、`person_gate_allowed=False`、`source=verdict_hits`、同一框重复 **42 次** |

### 2.2 person 模型在该帧整图零输出（conf 扫描，非门限问题）★

对同一张原图用**真实权重**重放（`yolo11n_person.pt`，SHA `0ebbc80d…`）：

```
person model conf sweep (classes=[0]):
  conf>=0.25 -> 0 boxes
  conf>=0.10 -> 0 boxes   ← 生产值
  conf>=0.05 -> 0 boxes
  conf>=0.02 -> 0 boxes
  conf>=0.01 -> 0 boxes
```

⇒ **把人体 conf 降到 0.01 也不可能救这个框**。门限不是病灶。

### 2.3 但被拒白框不是真人（几何独立判定，不依赖投影语义）

- 白框像素 `w=7.5 px, h=11.7 px`，宽高比 **1.56**（成人直立框通常 >2）；
- 隐含身高 = `h_px × range / fx = 11.7 × 18.85 / 205.47 = **1.07 m**`（成人参考 1.75 m）；
- **独立距离核对**：该帧相机 `camera_xyz=(-37.583, 11.268, 2.285)`，
  白色真人真值同时刻 `(101.67, -44.70, 3.00)` ⇒ **直线距离 150.1 m**，
  而白框报告 `range_m=18.85` ⇒ 二者不可能是同一物体。

全轮 45 条 white 行（跨六机）的隐含身高：

| 来源 | 框数 | conf | range | 隐含身高 | 判定 |
|---|---|---|---|---|---|
| uav_2 | 1 | 0.538 | 18.85 m | **1.08 m** | 非成人 |
| uav_4 | 1 | 0.656 | 27.18 m | **2.10 m** | 非成人 |
| uav_4 | 1 (+42 重复) | 0.436 | 26.77 m | **2.03 m** | 非成人 |

⇒ **无一落在 1.4–2.1 m 的成人带内**（最近的是 2.10 m，仍偏高）；
且 uav_4 的框 42 帧 UV 恒定（`(221.2, 207.4)` 不变）⇒ **静止物**。

### 2.4 整轮 person 证明通过情况（对照）

`yolo_detection` 行中 `person_frame_verified=True` 的：**blue 163 条、green 1 条、white 0 条**。
⇒ 人体通道本身在工作（蓝/绿能过），**白色零通过**，但白色候选也都是非真人物体。

### 2.5 白色真人在场但从未被上报

`independent_visual_accuracy_white.json`：
```
actor_truth_samples: 2303,  truth_time_bounds: {3: [1936.592, 2069.266]}
original_image_coordinates: {total: 0, aligned: 0, missing: 0}
official_report_coordinates: {total: 0, aligned: 0, missing: 0}
```

## 3. 推测（未验证，需新证据）

1. **推测**：白色真人在本轮可能始终处于"过远/不在视野/被遮挡"状态，
   因此从未产生合格颜色框；h480_4 在 26.8 m 处看到的 2.0–2.1 m 白色物
   可能是建筑边白色构件或相邻物体，而非真人。
   - 反驳它的证据需要：**带真人在近处（≤12 m）的 white 原图**。
2. **推测**：uav_4 那 42 条同帧重复 `track_reject` 属"同一旧原图反复处理"，
   与 N12 日志分层的去重目标同类，但本轮**不影响判定**（都是拒）。
3. **推测（与 N13 报告的关系，需更正）**：N13 据 uav_6_white_002.png 单帧
   推断"person 模型对白色仿真小人整图失明"。**本报告不支持该推广**：
   该帧的框同样可能不是真人（当时未做几何核对）。
   ⇒ N13 §3 关于 white 的表述应降级为"待核对身份的单帧现象"。

## 4. 缺证（必须先补，不能靠推断）

| # | 缺什么 | 为什么要它 |
|---|---|---|
| G1 | **近处（≤12 m）白色真人原图**（至少 3 张，含真值时刻） | 唯一能回答"person 模型对真白人是否失明"的证据；现有白图全非真人 |
| G2 | 白色真人真值轨迹 ↔ 相机位姿的**逐帧可见性**（在哪几帧进过画面/什么距离） | 判定缺口属"从未看见"还是"看见了没识出" |
| G3 | `camera_rotation` 的**约定说明**（world→cam 还是 cam→world，行优先？） | 本次投影因约定未定而未采用；需与既有投影工具对齐后重算 |
| G4 | h480_4 `white0` 轨道（42 帧）对应的原图 | 判断 2.03 m 白色物是什么；本轮未保存 |

## 5. 候选修法（最多两个，均**未实施**）

### 候选 A（推荐，属 Codex 域）：白色沿用"运动身份"通道，不要求本帧人体证明
- **依据**：本轮全部白框均为**静止**（UV 恒定 42 帧），且非真人 ⇒ 静态干扰本身已被
  "无运动"天然区分；v1.41 已为 blue 设计 `motion_identity_verified`（schema5）。
- **做法**：把该入口扩展到 white，**但必须绑定运动证据**（原图位移/运动身份），
  静止白物一律不得进入（防止放行静态装饰，符合任务单要求）。
- **受影响文件（由 Codex 维护，WB 不改）**：`visual_observation.py`（wire schema5 扩展）、
  `yolo_target_bridge.py`、`robo_team/coordination/src/robocup_swarm/scripts/`（manager/agent 的候选消费）。
- **前置**：需 G1/G2，否则无法验证收益。

### 候选 B（保守，零风险）：维持现门，把白色缺口归入"搜索/接近"策略而非感知门限
- **依据**：本轮所有白框的拒绝都是**正确拒绝**；感知层没有放行错误，
  也没有漏掉真人（真人从未进入有效距离）。
- **做法**：不改感知门限、不加白盒通道；由 Codex 在搜索/接近策略层决定是否
  为白色增设低可信导航线索（类比"远距候选单列"，见 `predata_comparison_20261008.md` 建议 1）。
- **受影响文件**：无感知侧改动。

> **明确不建议**：统一降低 person conf / 提高颜色 conf / 放宽 IoU 门。
> 本轮证据显示这些既不解决白框（person 整图零输出），也无助真人（真人未进画面）。

## 6. 可重现命令

```bash
# 1) 白框失败索引（唯一白图记录）
cat /root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159/\
flight/algorithm/evidence_typhoon_h480_2_white_fail/index.jsonl

# 2) person 模型 conf 扫描（真权重，CUDA）
#    见 D:/a/.robocup/tmp/white_replay.sh（本次诊断脚本，可重跑）
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-20.04 -u root -- \
  bash /mnt/d/a/.robocup/tmp/white_replay.sh

# 3) 全轮 white 行 + 隐含身高（几何独立判定）
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-20.04 -u root -- \
  bash /mnt/d/a/.robocup/tmp/white_batch.sh

# 4) 白框拒绝链字段（person_hits/gate/reject_reason）
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-20.04 -u root -- \
  bash /mnt/d/a/.robocup/tmp/white_chain.sh
```

模型 SHA（本轮实际使用，取自 `city_swarm_run.py` 注入与权重文件）：
颜色 `9d0c1fe2b6370a40eb274573de06ac4e36c81eb7a39149dd1ac4aff0ed6990c2`；
人体 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`。

**边界声明**：本报告全部结论来自**离线重放与归档字段**，不代表实体轮次已复现真人漏检，
不声称白框身份已由人工目视确认（G1 缺失）。等待 Codex 复核后再决定是否整合生产代码。