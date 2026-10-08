# N12 感知证据链扩容 · 实施方案（待陈批准后动手）

> 任务：`improve_goal.py` N12（Codex 交接 `ef55251` 第 1 步）
> 原则：**只加记录，不改判定**——所有现有阈值/权重/门限/裁判/官方件零改动，
> `filter_boxes` 的 retained 结果、`frame_verified` 的判定输出必须与旧代码 bit-exact。
> 接口边界：遵守 `YOLO_PERCEPTION_HANDOFF_20261008.md` 全部约束（视觉v3、人体证明绑定具体颜色框、
> 客户端不加载 CUDA、六机统一快照）。

---

## 0. 现状（为什么必须改）

`person_verifier.py:76-92` 的证明生成只留下 `{cls, xyxy}`：

```python
matched = any(overlap(box.xyxy[0], person) >= self.minimum_overlap for person in people)
#                          ↑ 哪个人体框、IoU 多少、人体 conf 多少 —— 全部丢弃
...
self.verified_boxes.append(dict(cls=key[0], xyxy=list(key[1])))   # ← 只有这两项
```

出事后的三种可能——**人体误检 / 错误框关联 / 颜色区域误判**——在现有记录里**不可分**。
这正是交接 P0 第 2、3 条：13 条 ≤22 m 绿色候选 `person_frame_verified=true` 却全大误差，
无法归因；失败原图也没保存。

---

## 1. 改动清单（4 个文件 + 2 个新文件）

### 1.1 `perception/person_verifier.py` —— 证明结构扩容（核心）

`verified_boxes` 每条从 2 字段扩为 9 字段：

```python
dict(cls=..., xyxy=[...],              # 原有，frame_verified 的 key，不动
     person_xyxy=[...],                # ★ 支持本条证明的人体框（哪个框）
     person_conf=0.41,                 # ★ 该人体框 conf
     person_iou=0.62,                  # ★ 实际 IoU（不是"过没过 0.25"）
     n_person_boxes=3,                 # 本图人体框总数（区分"图里没人"vs"有人没重叠"）
     shirt_fractions={...} or None,    # 颜色比例（torso_fractions 原始值）
     reject_reason=None)               # 见下
```

新增 `self.rejected_boxes`（被拒候选的对称记录）：

```python
dict(cls=..., xyxy=[...], color_conf=0.75,      # 颜色模型 conf
     n_person_boxes=0, best_iou=0.0,            # 人体框数 + 与所有人体的最佳 IoU
     reject_reason='no_person_overlap')         # 枚举见下
```

`reject_reason` 枚举（**只是记录，不影响任何过滤**）：
- `no_person_overlap`：颜色框与人体的最大 IoU < 0.25
- `person_overlap_but_not_proof_class`：重叠了但该类不在 proof_classes
- `shirt_unsupported`：有重叠但衣服颜色不支持
- `not_in_classes`：该类根本不做人体验证

**判定零改动的保证**：
- `frame_verified()`（:20-23）只比对 `detection_key(cls, xyxy)`——新字段是 dict 里的附加 key，
  旧逻辑对新结构**天然工作**（`r.get('cls')`/`r.get('xyxy')` 照常取到）。
- `filter_boxes` 的 `retained` 计算一行不改；只在循环里**多记录**，不多判断。
- 兼容测试：新旧结构各喂给 `frame_verified`，输出必须一致（写进单测）。

### 1.2 `perception/shared_inference_service.py` —— 协议 v3

- `verification_version: 2 → 3`
- `verified_person_boxes`：直接转发 1.1 的新结构（服务端本来就在 GPU_LOCK 内
  `list(_PERSON_VERIFIER.verified_boxes)` 拷贝，:171，新结构同样浅拷贝安全）
- 新增 `verified_person_rejected=_PERSON_VERIFIER.rejected_boxes`
- **兼容**：客户端按 version 门控消费；v2 客户端读 v3 响应只是多几个被忽略的 key，
  `verified_person_boxes` 里多出的 dict 字段对 `frame_verified` 无害（它只取 cls/xyxy）。

### 1.3 `perception/shared_inference_client.py` —— 门控扩到 v3

- `version in (1,2)` → `(1,2,3)`
- 新增透传 `verified_person_rejected`（v3 才有，否则 `[]`）
- **客户端继续不 import torch/ultralytics/CUDA**（现文件只有 cv2/numpy，保持）

### 1.4 `perception/perception_real.py` —— 三处消费 + 证据落盘

**(a) 证明消费不变**：:1326 的 `frame_verified(_person_boxes, cid, [x1,y1,x2,y2])` 一行不动。
从 details 里按 key 查出当前框的支持信息（新增一个 `_person_detail()` 查找函数）。

**(b) CSV 加列**（`perception_uav_N.csv` 的 dets/track_reject 行）：
`person_iou`、`person_conf`、`n_person_boxes`、`reject_reason`（无证明时为空）、`img_stamp`。
**前置核查已完成（2026-10-08 12:26）**：
- `audit_fast_city.py:74` 与 `replay_image_motion.py:28` 均为 `csv.DictReader`（**按列名**）⇒ 加列安全，
  旧归档读新代码 / 新归档读旧工具都只是"新列为 None/缺失"，无位置错位风险。
- `csv_logger.py` 的行为：表头不匹配时**自动把旧文件改名 `.legacy.<ts>` 另起新文件**，不覆盖、不混写
  ⇒ 同一轮内表头恒一致；跨轮归档各自带自己的表头，DictReader 通吃。
- 结论：**列追加无兼容风险**，回放工具（replay_green_white 等）只读旧列名，不受影响。

**(c) 有界失败原图采集（新文件 `perception/evidence_capture.py`）**：

触发条件（二者其一）：
- 颜色框 conf ≥ 0.40（过了颜色门）但 `person_frame_proof=False` 且 range ≤ 22 m
  ——"本该成为合格观测却失败"的可疑候选
- `person_frame_proof=True` 但该图稍后被判为高误差（回放时才能知道，运行时先存再说）

每条证据 = 一个 PNG + 一行 JSONL：
```
evidence_<uav>/frame_<seq>_<img_stamp>.png
evidence_<uav>/index.jsonl: {uav, img_stamp, seq, color_cls, color_conf, color_xyxy,
  person_boxes:[[xyxy,conf],...], person_iou, shirt_fractions, reject_reason,
  color_model_sha, person_model_sha, pr_conf, pr_person_conf, pr_person_iou, run_id}
```

**有界**（三重上限，仿 `capture_visual_frames.py` 的先例）：
- 每 uav 每分钟 ≤ 6 条（令牌桶）
- 每 uav 总量 ≤ 300 条，超限丢最旧索引行但保留文件（或直接停采，取简单者）
- 磁盘写入异常只打日志，**绝不抛进检测主循环**（frame_probe 同款 try 兜底）

开关：`PR_EVIDENCE_CAPTURE`（默认 `1` 开；`0` 完全关闭回到现状）。
文件名**不含** `perception` 前缀（避开 audit 通配，frame_probe 已踩过这个坑）。

**(d) 原图标识去重**：dets/reject 行加 `img_stamp` 列（frame_probe 已有，事件流补上），
使"同一旧原图被处理 N 次"在离线可数。

### 1.5 新测试 `perception/tests/test_person_verifier_rich.py`

覆盖：
1. **判定不变性**：同一输入下新旧 `verified_boxes` 的 `(cls,xyxy)` 集合 bit-exact 相等；
   `retained` 相等
2. 新字段正确性：构造已知 IoU/conf 的假框，断言 `person_iou`/`person_conf`/`n_person_boxes`
3. `reject_reason` 四种枚举各触发一次
4. **新旧结构互操作**：`frame_verified` 分别吃 v2 结构（旧 dict）与 v3 结构（新 dict），结果一致
5. 协议兼容：client 对 version=2/3 响应的字段取舍正确；v3 多余字段不炸 v2 消费者
6. 证据采集：令牌桶限速、总量上限、写盘异常不抛出、开关关闭零产出
7. 全部测试**列失败原因**，不只报总数（交接的硬要求）

---

## 2. 验证方案（不起新实体轮）

| 步骤 | 内容 | 通过标准 |
|---|---|---|
| V1 | 新单测全绿 | ≥ 20 项，逐项有名有失败原因 |
| V2 | 既有测试回归 | `perception/tests/test_shared_inference.py` + `test_frame_probe.py` 全过 |
| V3 | **历史回放 bit-exact** | 用 v139/N8 归档的 `perception_uav_N.csv` 跑 `replay_green_white.py --self-test`（它消费旧表头），新代码判定输出与归档一致 |
| V4 | 留存 12 图离线重推（WSL GPU） | `uav_2_brown_001.png` 等：新字段能区分"人体误检/框关联错/颜色区域误判"三种失败 —— **这是 N12 的验收主问题** |
| V5 | 双路径一致性 | 同一图分别走本地 PersonVerifier 与 shared service（WSL 起服务），证明字段一致 |
| V6 | 快照纪律 | `source_manifest.json` 纳入新文件；`PR_EVIDENCE_CAPTURE` 进 `city_swarm_run.py` 的 manifest 记录 |

V4/V5 需要 WSL GPU 做一次推理（非整轮仿真，几分钟），属"离线验证"不占实体轮预算。

---

## 3. 明确不做（本轮）

- 不改任何 conf / IoU / 门限 / 权重 / 22 m / 12 m
- 不修 `jersey_color` 的映射逻辑（那是 N13）
- 不动 `dwa_avoidance` / `single_uav_avoidance`（不在主链）
- 不动裁判、actor、传感器参数、官方 vendor 目录
- 不起六机仿真轮（交接第 1 步明确"不先做新实体轮"）
- 白色证明、brown 激活门的判定语义不动（只把它们**为什么失败**记下来）

## 4. 交回物（完成后）

修改文件清单 + 提交号、`PR_EVIDENCE_CAPTURE` 生效入口、metadata schema v3 兼容说明
（v2 消费者零影响）、V1–V6 结果表、12 图的新字段归因表、剩余问题清单。
实体六机验证由项目统一安排（复用 `validate_fast_city` 入口），不另造 run 系列。

---

## 5. 风险与对策

| 风险 | 对策 |
|---|---|
| CSV 加列破坏现有读取器 | 先审计 audit_fast_city/replay_image_motion 的读法；列只追加在行尾；V3 回放兜底 |
| 证据采集磁盘爆炸 | 三重上限 + 默认开但可一键关 + 写盘 try 兜底 |
| shared 协议变更引发六机不一致 | version 门控（v1/v2 行为完全不变）；六机+服务统一快照才上实体轮 |
| `shirt_fractions` 可能较大 | 只存 torso_fractions 的原始 dict（≤8 个键），不存图块 |
| 改动悄悄影响判定 | V1-1 与 V3 的 bit-exact 断言是硬门，不一致即回退 |
