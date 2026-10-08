# WORKBUDDY 批量任务交付（白色证据复核与独立补图）

> 任务单：`WORKBUDDY_BATCH_TASK_20261008.md`。**离线**：未启动或停止任何仿真进程（实测 0 个
> gzserver/px4/validate_fast 进程）。**未改生产核心**：`perception_real.py`、`visual_observation.py`、
> 桥、manager、agent、裁判、人物、传感器、WSL 配置均未改动（见 §6 自审）。
> 只新增/修改：诊断工具、独立只读观察器、测试、报告。

---

## 1. 8 与 34 差异的根因（已定位并修复）

**根因是输入索引的一个解析缺陷，不是投影点高度。**

`white_visibility_audit2.py::saved_images` 用
```python
model = os.path.basename(d).split('_')[2]        # 'evidence_typhoon_h480_2' -> 'h480'
uav = 'uav_%d' % (int(model.split('h480_')[1]) + 1)   # IndexError -> '?'
```
`split('_')[2]` 得到的是 **`h480`**（目录名里 `h480` 与 `2` 被下划线分开），
`split('h480_')[1]` 必然 IndexError，异常分支把机号写成 `'?'`。
后果：**evidence 通道的 76 张原图全部无法与同机搜索记录匹配**，只剩 `observers/frames` 的 8 张。

**修复**：机号改为
1. 优先 `camera_binding.image_topic`（`/uav_<N>/…`），退化到 `image_header.frame_id`；
2. 与目录 `typhoon_h480_<M>` → `uav_<M+1>` **交叉核对**；
3. 冲突则**记录并拒绝**（`BINDING_DIR_CONFLICT`），**不猜机号**；
4. 无 binding 时才允许目录回退，并在记录里标注 `id_source`。

实测：目录名与 binding 在本轮**全部一致**（`evidence_typhoon_h480_2` ↔ `/uav_3/…`），
拒绝 0 条。

**高度差异的量化**（任务单要求）：统一真值选择后分别算 z=0.875 与 z=1.0 的 FOV 集合 ——
**两者完全相同（各 34 帧，交集 34，差集 0）**。⇒ 中点高度**不是**差异来源。

## 2. 修正后的数量

| 项 | 值 |
|---|---|
| 搜索相机记录 | **1827**（唯一 (uav,image_s)：1827，重复 0）|
| 真值行 | 2303 全部 `kind=actor_truth` 且 `sample[3]==3`（过滤掉 0 条；其他 sample 不是白色真值）|
| 有文件的原图记录 | **94** |
| 唯一原图（同机+时刻） | **85**（重复索引 **9** 条：同一图被多条记录引用）|
| 与搜索记录精确对齐的唯一原图 | **69** |
| **in-FOV 且有精确原图（z=0.875）** | **34** ✅ 与你的 34 一致 |
| in-FOV（z=1.00） | **34**（与上完全相同）|
| 拒绝的原图记录 | **0** |

**集合对比（旧工具 vs 修正后）**：`old only = []`、`common = 8`、
`new only = 26` —— 旧 8 帧**完全包含**在新 34 帧内，新增的 26 帧全部来自修复掉的 evidence 来源。

## 3. 完整离线重放（34 帧）

模型（绝对路径 + SHA 见 §5），一次加载批量推理；未改任何门限。

**逐图记录**：`white_replay3_v140.json`（34 条，含 uav / image_s / sample_s / 原图 SHA256 /
真值时间差 / 假设与投影点 / 距离 / 预期像素高 / **全部**颜色框与人体框坐标+conf /
每颜色框对所有人体框的 IoU / 衣服验证结果 / 理论位置白色像素比）。
**有界标注图**：`annotated/` 共 **34 张**（红叉+红框＝理论投影位置；黄框＝颜色检测；
绿框＝人体检测），供人工目视。

### 3.1 已验证事实

| 事实 | 数据 |
|---|---|
| **颜色模型在这 34 帧零 white 框** | **0 / 34** |
| 同期其他类正常出框 | blue **27**、green **32**、red **20**、brown **1** |
| 全部帧距离 | 57.3 – 177.7 m（预期像素高 2.2 – 9.0 px）|
| 理论位置白色像素比 | min 0.000 / p50 **0.025** / max 0.121；**15/34 为 0**；仅 4/34 ≥0.10 |
| 最近颜色框（任意类）距理论点 | min 11.8 / p50 50.2 / max 521.5 px |
| 人体框落在理论点 12 px 内 | **0 / 34**（最近 12.2 px）|
| 人体框像素高 | 9 – 32 px（理论目标 2 – 9 px，**大 2–16 倍**）|
| 关联通过（IoU≥0.25） | 28 条：blue 16 / green 6 / red 6，**无 white** |

### 3.2 受限推测（有支持但不充分）

- 白色真人在这些帧**可能太小、未渲染或被遮挡**：支持点是"理论位置白色像素比中位仅 0.025、
  15 帧为 0"，且人体框均不在该位置（那些是更近的其他人）。
- **不能据此排除颜色模型对低对比白目标漏检**：像素比例低也可能来自仿真白衣服的光照/色偏
  （我的白色阈值 S<70,V>150 只是代理指标），或投影假设偏差。

### 3.3 缺证

- **近处（≤22 m）白色真人原图**：本轮 in-FOV 全部 ≥57.3 m，22 m 内 **0 帧**
  ⇒ **无法评价近处识别能力**（任务单明确要求此时如实说明，且不继续扫无意义 conf）。
- **渲染可见性证明**：几何入 FOV ≠ 实际渲染可见（无该帧渲染像素绑定）。
- **人工目视身份确认**：标注图 34 张已生成，需由人查看判定"是否看得见人物、是否建筑遮挡"；
  我不能替代人工目视。
- 结论**仅覆盖这 1827 条搜索记录**，不代表六机全程视野。
- 未把 4 px / 16 px 等经验值规定为赛事判据或生产门限。

## 4. 独立只读补图工具（已实现，**未实跑**）

`coordination/scripts/white_evidence_observer.py`（脚本 SHA256 `70bc1fdb…d28850`）

| 要求 | 落实 |
|---|---|
| 默认不开启、参数化 | 无 `--enable` 时只打印配置退出（已实测）；`--truth-model/--min-expected-px/--max-range-m/--per-uav-per-sec/--total-cap/--out-dir/--result-file` 等 |
| **不发布任何东西** | 源码内**零 `Publisher`/`publish(`/`ServiceProxy`**（测试断言）；不调用任何飞控服务 |
| 真值不进感知/控制 | 真值仅参与"是否存图"的判断；工具在 `perception_real.py` 之外，未被告知给它任何真值；补图结果不进任何话题 |
| 同一原图时刻对齐 | `pair_tol_s`（默认 0.02 s）校验位姿时刻与图像时刻；**超龄位姿拒绝**（`pose_max_age_s=0.5`），绝不"新位姿配旧图" |
| 优先近处/大像素 | `min_expected_px` / `max_range_m` 为**采样策略参数**（注释与报告均声明非赛事判据）|
| 有界 | 每机每秒 ≤1 图、全轮 ≤120 图、内存队列 `deque(maxlen=64)`、同机同时刻去重 |
| 输出 | PNG + `index.jsonl`（含参数、脚本 SHA、配对时间差、真值触发标记、非控制用途声明）|
| 干净退出 | `result_finished()`（仅在 ENDED/FAILED/STOPPED_AFTER_CONTACT 时退出；**文件不存在或不可读时继续等待**）+ `rospy.is_shutdown()` + SIGINT/SIGTERM 处理器 |
| 示例命令只写文档 | 见文件 docstring；**本次未实际订阅任何运行中仿真** |

## 5. 命令、测试与 SHA

```bash
V=/mnt/d/a/.robocup/robo_team/coordination/docs/validation/white_visibility_20261008
R=/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159

# 1) 修正版审计（binding 优先 + 目录交叉核对 + actor_truth/actor_id 过滤 + 双高度 + 集合对比）
python3 $V/white_audit3.py --round $R \
        --json-out $V/white_audit3_v140.json \
        --old-json $V/white_visibility_v140_codex.json

# 2) 34 帧完整重放（真权重 + CUDA，生成标注图）
bash $V/run_replay3.sh

# 3) 补图工具自检（不启用、不订阅）
python3 coordination/scripts/white_evidence_observer.py            # 打印配置并退出

# 4) 边界测试
python3 coordination/tests/test_white_tools.py
```

**测试结果**：`coordination/tests/test_white_tools.py` —— **27 项全过**（0 失败 0 跳过）。
覆盖：两个 evidence 目录机号正确、binding↔目录冲突拒绝、混合 truth 过滤、
文件缺失、重复原图、同刻不同飞机、错误处理时刻、超时/过期配对、采集数量上限、
result 终止（含"不存在/不可读/RUNNING 不停机"）、信号退出与无发布自审、
JSON 可序列化（含 numpy 标量）。另有既有回归：`test_person_verifier_rich` 18 项、
`test_evidence_capture` 6 项、`test_frame_probe` 均通过。

**模型 SHA**：颜色 `9d0c1fe2b6370a40eb274573de06ac4e36c81eb7a39149dd1ac4aff0ed6990c2`；
人体 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`。
**运行环境**：WSL `/root/robo_team_build/vision_cuda_20261003/bin/python`，GPU device 0。
**大产物**：34 张标注图与 JSON 已入仓；完整 WSL 运行根见 §0 路径。

## 6. 逐项勾检（任务单四节）

**一、先修输入与原图索引**
- [x] 修 `saved_images` 机号解析缺陷
- [x] binding `image_topic` 优先 + 目录交叉核对；冲突拒绝不猜；缺 binding 才回退且标注来源
- [x] 只取 `kind=actor_truth` 且 `sample[3]==3`
- [x] 显式声明地面 Z=0 / 参考身高 1.75 m 假设
- [x] image_s 定时；同机 + ≤0.01 s + 文件存在；邻图/处理时刻不补原图
- [x] 每条记录含路径、文件 SHA256、来源索引、机号、两时间及差值
- [x] 唯一原图/唯一搜索记录清单 + 记录数/唯一数/重复数
- [x] 与旧工具集合对比（共同/仅旧/仅新 + 原因）
- [x] 同真值选择下 z=0.875 与 z=1.0 的 FOV 集合与差异量化
- [x] 保留历史输出，新结果另存（`white_audit3_v140.json`）

**二、完整离线重放**
- [x] 全部 in-FOV 且有精确原图的唯一图片（34）
- [x] 颜色/人体/衣服，未改门限；记录模型路径/SHA/提交/配置/环境/命令
- [x] 批量一次加载模型（未反复起服务/仿真）
- [x] 逐图输出全部要求字段
- [x] 有界标注图，区分理论投影标记与真实检测框（34 张）
- [x] 三类结论：已验证事实 / 受限推测 / 缺证
- [x] 结论限定搜索记录；未把经验像素值当判据；近处无样本时明确说明

**三、独立只读补图工具**
- [x] 独立文件、默认关、参数化、只读、不发布、不调飞控、不改 perception_real
- [x] 同刻对齐、超龄/缺失/非法变换不保存；优先近处/大像素（策略参数）
- [x] 每秒/总数/队列有界，去重，失败记录
- [x] PNG/JSONL/参数/SHA/命令，投影假设与配对时间差、非控制用途
- [x] result-file/is_shutdown/SIGINT/SIGTERM 干净退出，容忍结果文件缺失或旧轮
- [x] 示例命令只写文档，**本次未实跑**

**四、自审与交付**
- [x] 边界测试 27 项（含任务单列出的各项）
- [x] 不引用控制发布/飞控服务（源码断言）；未修改生产文件
- [x] JSON 能序列化真实模型结果（Tensor/NumPy → 普通值）
- [x] 本报告 + 可复现脚本 + 输入索引 + 标注图 + 结果
- [x] 生产文件未动、仿真未启动（实测进程数 0）

**未完成 / 不在本单**：实体轮验证（由 Codex 统一安排）；近处白色样本采集（需下一轮）。

## 7. 后续建议（最多两项）

1. **把补图观察器纳入下一轮实体轮**（由 Codex 调度）：它是当前唯一能产出"白色真人在近处
   （≤22 m、≥16 px）原图"的途径；没有这类样本，识别能力**无法评价**，任何门限结论都是猜。
2. **白色接近由搜索/接近策略解决**（Codex 域）：本轮 34 帧全部 ≥57.3 m，
   说明白色**从未进入可用观测距离**；感知侧不动门限。

> 边界：全部结论来自离线归档与构造测试，不代表实体轮已复现真人漏检；未做人工目视结论；
> 未启动仿真；补图工具未运行。等待 Codex 复核。

---

## 8. 交付后自纠：补图观察器有一条**致命缺陷**（2026-10-08 晚，离线回测发现）

上面 §5 说观察器"已实现、未实跑"。既然它需要活仿真才能收话题，我改用**离线回测**
（`validation/white_visibility_20261008/observer_backtest.py`）把 v1.40 的 1827 条搜索相机记录
喂给观察器**真实的判定函数**，逐条问"这帧会不会被存下来"。结果：

| 契约 | 实存帧数（1827 条记录） | 首条拒绝原因 |
|---|---|---|
| **v1（我原来交付的）** | **0** | `pose_image_time_mismatch` × 1827 |
| **production_mirror（修正后）** | 643 合格，产量帽下实存 **120** | — |

**两个缺陷，都是设计错，不是实现笔误：**

1. **配对契约错**：v1 等 `/uav_N/mavros/local_position/pose`，要求**最新位姿的时间戳**与图像
   时间戳相差 ≤ `pair_tol_s = 0.02 s`。实测图像延迟 **min 0.216 / p50 0.472 / max 0.884 s**
   ⇒ 100% 被拒，一张都存不下。
2. **消息类型错**：v1 把该话题当 `nav_msgs/Odometry` 订阅；栈内实际是 `PoseStamped`
   （`six_radar_connectivity.py:273`、`coordination_executor.py:188` 均为 PoseStamped）
   ⇒ 即使改了容差，回调也收不到。
3. **陈旧判据错**：v1 用 `wall_now - pose_stamp <= pose_max_age_s` 判陈旧，这**无法区分**
   "这一刻没有位姿" 和 "这张原图太旧"。真正的保护应该是**原图年龄**。

**修法 = 照抄生产节点的既有做法**（`perception_real.py:1221-1234`）：图像到达时调
`/gazebo/get_link_state(<model>::cgo3_camera_link, "world")` 拉当前相机位姿，加
`R @ CAM_OFF_BL`（`0,0,-0.162`，与 `city_swarm_run.py:351` 同值），并且**只以原图年龄
≤ 1.0 s 为闸门**（与生产同值）。位姿拉取时刻晚于成像时刻，其位置误差按行记录为
`pose_pull_delay_s`。`--pose-source topic` 保留给没有该服务的环境，并改为**按最近时间戳
在历史里查**（不是"最新位姿"）。

**交叉校验**：修正后回测中 `behind_camera 789 + outside_image 245 = 1034`，
反推入画帧 1827 − 1034 = **793**，与独立审计工具 `white_audit3.py` 的 793 **完全一致**
（两个互不依赖的实现互相印证）。

**读-only 边界仍然成立**：观察器仍然零 Publisher、无飞控服务调用；新增的
`/gazebo/get_link_state` 是**唯一的**服务调用，只读，且**生产感知自己就在用同一个调用**
（测试 `test_observer_calls_exactly_one_readonly_gazebo_service` 断言全文件服务调用数 = 1）。

**测试**：`coordination/tests/test_white_tools.py` 由 27 → **38 项全过**，
其中新增的关键回归是 `test_measured_latency_does_not_block_capture`（0.472 s 延迟必须仍可采）
和 `test_pose_history_picks_nearest_stamp_not_newest`。
**注意一处契约变更**：原 `test_pose_too_old_rejected` 断言的是被我判定为错误的旧契约，
已改为 `test_old_original_rejected`（断言 `image_too_old`），`pose_too_old` 单独保留为
"话题模式 + 图像年龄门放宽"下的次级保护。

**教训**（已写入 `memory/rules/traps.md`）：**纯函数的单测全绿，完全不能证明 I/O 契约成立。**
27 项测试覆盖了判定函数，却没有任何一项能发现"话题类型错、容差 0.02 s 与现实 0.47 s 不符"。
**对照真实归档跑一遍判定函数，成本几分钟，能挡住一类本来只在实跑时才会暴露的错。**

**仍未做**：观察器**没有在活仿真里跑过**。要产出近处白色原图，必须先有一轮实体轮；
我没有启动仿真（本轮任务明确禁止，且 Codex 正在同一台 WSL 上做搜索/接近链）。
可选的两种跑法见 §7-1。