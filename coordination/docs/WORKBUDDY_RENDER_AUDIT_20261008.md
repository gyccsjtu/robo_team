# 相机帧率专项审计（第一步：查清 + 提升方案）

> 提交人：WorkBuddy。来源：全链路审计 `WORKBUDDY_FULL_AUDIT_20261008.md` 的问题 C1。
> **本轮六机验证仍在运行中**（`wb_v141_validation3_20261008T104200Z`），本次审计**全部为只读**
> —— 未改动任何生产代码、未启动/停止任何仿真、未调整任何阈值。
> 所有实验为离线微基准（`glxinfo` / `glxgears`），耗时数秒，不与在飞轮争抢关键资源。

---

## 0. 一句话结论

**不是"配置低"，是"渲染跟不上"。**

本机相机配置（`update_rate=10`、640×360、`horizontal_fov=2.0`）与官方平台自带模型
**逐字节相同**。低帧率的真因是：**本机被迫使用 CPU 软件渲染**，而软件渲染在
"城市场景 + 6 台相机"这个负载下吞吐不足，只能交付配置值的 25%。

至于"为什么被迫"——**WSL 的 GPU 渲染路径存在一个已知缺陷（动画人物不可见），
我们此前主动退回软渲染**。这是一个有记录的、理性的降级选择，不是疏忽。

---

## 1. 定性：配置低 vs 渲染跟不上

### 1.1 配置与官方一致（决定性证据）

| 文件 | SHA256 |
|---|---|
| 上游 `gitee:robin_shaun/XTDrone` `sitl_config/models/typhoon_h480/typhoon_h480.sdf` | `1346f71a33130e3f5634b1513cc5598d1dc2693fdf30d13c2cf9dda2ef2cd29e` |
| 本机 `/root/third_party_official/XTDrone_official/sitl_config/models/typhoon_h480/typhoon_h480.sdf` | `1346f71a33130e3f5634b1513cc5598d1dc2693fdf30d13c2cf9dda2ef2cd29e` |

**逐字节相同。** 相机段实测值（本机 = 官方）：

```
<sensor name="camera" type="camera">
  <pose>0.0 0 -0.162 0 0 0</pose>
  <camera>
    <horizontal_fov>2.0</horizontal_fov>
    <image><format>R8G8B8</format><width>640</width><height>360</height></image>
    <clip><near>0.05</near><far>15000</far></clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>10</update_rate>          <!-- 10 Hz，仿真时间 -->
  <visualize>false</visualize>
</sensor>
```

⇒ **"是不是我们偷偷调低了？"——否。** `10 Hz / 640×360 / fov 2.0` 就是平台自带值。

### 1.2 实测帧率只有配置的 25%

| 项 | 值 |
|---|---|
| 配置 `update_rate` | **10 Hz（仿真时间）** |
| 实测（按仿真时钟，`rate_probe.py`） | **2.52 Hz** |
| 达成率 | **25%** |

（`frame_probe` 独立测得 2.30 Hz，同一量级。）

⇒ **渲染跟不上配置。**

---

## 2. 根因链

### 2.1 本机 GL 后端实测（`glxinfo -B`）

| 启动条件 | 实际渲染器 |
|---|---|
| `LIBGL_ALWAYS_SOFTWARE=1`（**当前实跑**） | **llvmpipe (LLVM 12.0.0, 256 bits)** ← 软件 |
| 不设该变量 | **D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)** ← GPU，**可用** |
| `MESA_LOADER_DRIVER_OVERRIDE=d3d12` | D3D12 (RTX 4060) |
| `MESA_LOADER_DRIVER_OVERRIDE=zink` | D3D12（zink 不可用，回落） |

硬件与驱动：`/dev/dxg` 存在、`libd3d12.so` 存在、`nvidia-smi` 报告
`RTX 4060 Laptop GPU / 驱动 592.00 / 利用率 0%`。Mesa 21.2.6（Ubuntu 20.04），
Vulkan ICD 仅 `intel / radeon / lavapipe`（**无 dzn**，lavapipe 也是软件）。

⇒ **结论：本机只有两条能走的路** ——（a）CPU 软渲染 llvmpipe（正确但慢）、
（b）GPU 的 Mesa **D3D12** 后端（快但有一个视觉缺陷）。`zink` 无可用 Vulkan 后端。

### 2.2 为什么当前走软渲染（有记录的主动选择）

`coordination/scripts/six_radar_connectivity.py:181-183`：

```python
# WSL's Mesa D3D12 backend rendered animated people invisible in the
# original camera. Keep stock sensors; select a working renderer.
env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
```

**注释写明了两件事**：
1. D3D12 后端让**动画人物**（animated people）在原相机里**不可见** —— 这是致命缺陷；
2. 我们的应对是"保持传感器原样、换一个能用的渲染器" —— 即**合法降级**，没有去动模型。

这是**全仓库唯一**设置该变量的位置（已全库 grep 确认）。启动链唯一：
```
validate_fast_city.py:66
  → run_city_swarm.sh:16
      → prepare_competition_scene.py  （生成场景）
      → exec python3 six_radar_connectivity.py   ← 此处设 LIBGL_ALWAYS_SOFTWARE=1，并起 gzserver
```

### 2.3 人物到底是什么（"animated"是关键词）

场景 world 里 6 个目标全部是 **Gazebo `<actor>` + `<animation>` + Collada 骨骼动画**：

```xml
<actor name="actor_0"> ... <animation name=...>
  <filename>walker/walk_0.dae</filename>
```

模型位于 `XTDrone_official/sitl_config/models/walker/`：`walk_0..5.dae`、`casual_female.dae`。

⇒ **注释特意写 "animated"**：骨骼动画走的是 Ogre 的 Skeleton/Animation 路径，
比静态 mesh 更容易踩到后端兼容性问题。**这为"可修"留下了线索**（见 §5-A）。

### 2.4 软渲染为什么只能出 2.5 Hz（关键定量）

**先看软渲染本身有多快（`glxgears`，离屏同尺寸对照）：**

| 分辨率 | 条件 | FPS | 单帧 |
|---|---|---|---|
| 300×300 | llvmpipe 1 线程 | **525 / 553** | ~1.9 ms |
| 300×300 | llvmpipe 默认线程 | 376 / 368 | ~2.7 ms |
| 300×300 | llvmpipe 16 线程 | 421 / 394 | ~2.5 ms |
| 300×300 | **GPU D3D12** | 58 / 69 | ~15 ms |
| **640×360**（目标分辨率） | llvmpipe 1 线程 | **240 / 284** | **~3.5–4.2 ms** |
| **640×360** | llvmpipe 16 线程 | 227 / 210 | ~4.4 ms |
| **640×360** | **GPU D3D12** | 52 / 56 | ~18–19 ms |

**再看城市场景的实际成本（本轮归档反推）：**

- sensor 周期 = 0.1 仿真秒 = **0.625 s 墙钟**（RTF 0.158）
- 实测每台机每 **0.4 仿真秒**才出一帧（= 2.5 Hz），即每 **2.5 s 墙钟**渲染 6 帧
- ⇒ **单帧城市场景 ≈ 416 ms**（6 台相机分摊）

**对照：同样 640×360，llvmpipe 渲染简单场景只要 ~4 ms。差距约 100 倍。**

⇒ **瓶颈**确定**不在像素填充**：
1. llvmpipe 的 fragment 阶段已经很快（4 ms/帧），所以**加线程无收益**（实测 16 线程 421 FPS < 单线程 525 FPS，调度开销大于收益）；
2. 100 倍的差距只能来自**顶点处理 / 场景遍历 / draw call 提交 / 阴影 pass** —— 这些在 llvmpipe 里**几乎不并行**，且**按相机数量线性放大**（6 台相机各自 cull + 提交整个城市场景）。

### 2.5 这不是 CPU 饥饿（推翻我先前的表述）

| 指标 | 实测 |
|---|---|
| gzserver 主线程 CPU 时间 | 20:53（1253 s），占进程生命期 64% |
| 次要线程 | 4:00（240 s） |
| **合计 ≈ 0.82 核**（进程已运行 1813 s） | |
| CPU 亲和性 | 无限制（0–19 全可用） |
| `OMP_NUM_THREADS` / `LP_NUM_THREADS` | **未设置** |

⇒ gzserver 平均只用 **0.82 核**。**它不是抢不到 CPU，而是渲染路径本身吞吐不足**
（单线程软件光栅化 + 6 相机串行）。
⇒ 这一条**修正**了我在全链路审计里"CPU 饱和导致掉帧"的表述。

### 2.6 这个问题已经被缓解过一次（有据可查）

`flight/development_physics.json`：

```json
{"update_rate": 40.0, "time_step": 0.004, "sensor_parameters_changed": false,
 "reason": "City software rendering produced image age 1.3-1.8s at 50Hz"}
```

⇒ 我们（Codex/我方）**早就诊断出"城市软渲染"是延迟来源**，并用"物理频率 50 → 40 Hz
+ `time_step` 0.004"来腾出墙钟预算，且**明确记录 `sensor_parameters_changed: false`**
（没动传感器）。实测 RTF 0.158 与设计值 0.16 吻合 ⇒ **物理确实跑在设计值上**。

**注意**：这一条同时纠正了我在全链路审计里"RTF 0.158 说明机器吃不消"的说法 ——
RTF 低是**主动设定**的，不是性能崩溃。

---

## 3. 已排除的方案（附实测，避免重复劳动）

| 方案 | 结论 | 依据 |
|---|---|---|
| **`LP_NUM_THREADS` 多线程软渲染** | **❌ 无收益** | 640×360：1 线程 240–284 FPS vs 16 线程 210–227 FPS（**更慢**）。瓶颈不在 fragment |
| **降低物理频率换墙钟预算** | **❌ 不可行** | 要让 6 相机在预算内跑完，需窗口 ≈ 2.5 s，对应 RTF ≈ 0.04 ⇒ 一轮 600 仿真秒要 **≈ 4.2 小时**墙钟 |
| **改分辨率 / 更新率 / 焦距** | **❌ 规则禁止** | 见 §4 |
| **换 zink 后端** | **❌ 不可用** | 本机 Vulkan 仅 intel/radeon/lavapipe，无 dzn；zink 回落到 D3D12 |

---

## 4. 合规边界（规则原文）

`tmp/rules_online_20261008.txt`（与官网 PDF 同源，SHA `622bb59b…`）：

> p. "6）传感器：… 相机的**视场角、分辨率、焦距**，激光雷达的扫描范围等参数
> **必须与 XTDrone 平台自带传感器保持一致，不能擅自修改**。
> 传感器的**安装角**可以自行设定，但需符合实际物理规律。
> **每次尝试前由裁判对传感器参数与安装位置进行检查**。"

> p. "2.3 … 传感器类型统一规定，除去**安装位置**可修改外，
> **任何参赛队未经允许不得改变模型参数**。"

### 因此

| 类别 | 项目 | 可否动 |
|---|---|---|
| **模型 / 传感器参数** | 分辨率、视场角、焦距、`update_rate`、`clip` | **❌ 禁止** |
| **传感器安装** | 安装角、安装位置 | ✅ 允许（需符合物理） |
| **环境 / 开发侧** | 渲染后端、`LIBGL_*`、物理频率（`update_rate`/`time_step` 经 `set_physics_properties`） | ✅ 允许（非模型参数） |
| **地图** | 由官方 `map_generator.py` 每次尝试前随机生成 | ⚠️ 改动需评估 |

⇒ **"提升相机帧率"只能从"环境/开发侧"或"渲染后端"入手，不能靠改传感器参数。**

---

## 5. 两个待验证方案（需跑实验，本轮结束后执行）

### A（高收益）：走 GPU（D3D12）并解决"人物不可见"

**为什么值得试**：
- D3D12 路径**确实可用**（`glxinfo` 已验证），且 RTX 4060 空闲（利用率 0%）
- 虽然 D3D12 每帧有 **~18 ms 固定开销**（WSL 跨 VM 提交），但若城市场景在 GPU 上
  单帧为 30–50 ms，则 6 相机 ≈ 180–300 ms < 625 ms 预算 ⇒ **理论上能跑满 10 Hz**
- 收益是**决定性的**：帧率 2.5 → 10 Hz ⇒ 图像延迟从 ~0.79 s 降到 ~0.2–0.3 s
  ⇒ **准确性与连续性两道门同时松**（见 `latency_impact.py`）

**待查**：为什么"动画人物"不可见。线索：
- 是**只有**骨骼动画不可见，还是**所有** mesh？（用静态物体做对照即可分开）
- 是材质/纹理采样问题，还是 Skeleton/Animation 的 GL 调用问题？
- 旧结论来自哪个 Mesa/WSL 版本？**当前环境是否仍然成立**（值得直接复验，不继承旧结论）

**先要做的**：写一个**最小复现**（不是六机轮）——单机 + 若干 walker actor，
D3D12 下渲染一帧，**直接看图**。

### B（保守）：保留软渲染，查清成本构成

- 城市场景单帧 416 ms 里，**阴影 pass 占多少**？（world 是 `<shadows>1</shadows>`）
- 若能确认阴影是大头，则"是否可关"的**合规性**需要单独裁定（地图由官方脚本生成）
- 对照实验：`<shadows>1` vs `<shadows>0`（**临时实验副本，不入库**）

### 最小对照实验设计（成本远低于六机轮）

| 组 | 渲染后端 | 阴影 | 测什么 |
|---|---|---|---|
| 1 | llvmpipe（基线） | 开 | 帧率 / 单帧耗时 / 人物可见性 |
| 2 | D3D12 | 开 | 同上（**核心：人物可见性**） |
| 3 | D3D12 | 临时关 | 阴影占比 |

指标：**相机仿真帧率**、**单帧渲染耗时**、**一帧原图（人物是否可见）**、图像年龄分布。
单机 + walker actor 的最小场景即可，**无需完整 ROS 控制链**。

---

## 6. ★ 战略判断（比技术方案更重要）

**这个问题很可能是"本机环境特有"，而不是比赛场景的固有属性。**

本机 = WSL2 + 笔记本 GPU（RTX 4060 Laptop）+ **我们自己设的软渲染降级**。
比赛在**组委会工作站**上运行。

⇒ **本地帧率低 ≠ 赛场帧率低。** 若赛场是原生 Linux + 正常 GL 驱动，
`LIBGL_ALWAYS_SOFTWARE` 这个降级根本不存在，延迟会小得多。

**这条直接决定"要不要为延迟放宽门限"**：
`EXTRAP_MAX_D`(1.5 m)、`ReportReadiness` 的 12 m/3 帧、`OBS_WINDOW`(0.6 s)
都是**按本机延迟定的**。在证实赛场渲染环境之前**放宽它们有风险**
（可能把一个本不存在的约束永久写进系统）。

**建议**：先做 §5 的对照实验得到"渲染路径 → 帧率"的确定关系，再决定
是否需要"为目标环境准备两套参数"。

---

## 7. 本次审计的边界（明确未做）

- ❌ 未修任何代码（`six_radar_connectivity.py:183` 保持原样）
- ❌ 未启动/停止任何仿真；未与在飞轮争抢（实验均为秒级离线微基准）
- ❌ 未放宽/收紧任何门限
- ⚠️ **未验证** D3D12 路径在**城市场景**下的实际成本（只测了 glxgears 简单场景）
- ⚠️ **未验证**"动画人物不可见"在当前环境是否仍然成立（旧结论可能已过期，需复验）
- ⚠️ 未查清城市场景 416 ms 里顶点 / draw call / 阴影各自的占比

---

## 附：复跑命令

```bash
# 1) 后端识别
wsl -d Ubuntu-20.04 -u root -- bash -c 'env LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B | grep -i renderer'
wsl -d Ubuntu-20.04 -u root -- bash -c 'env -u LIBGL_ALWAYS_SOFTWARE glxinfo -B | grep -i renderer'

# 2) 软渲染多线程对照（无收益证据）
wsl -d Ubuntu-20.04 -u root -- bash -c 'timeout 14 env vblank_mode=0 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=1  glxgears -geometry 640x360'
wsl -d Ubuntu-20.04 -u root -- bash -c 'timeout 14 env vblank_mode=0 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=16 glxgears -geometry 640x360'

# 3) GPU 对照
wsl -d Ubuntu-20.04 -u root -- bash -c 'timeout 14 env vblank_mode=0 glxgears -geometry 640x360'

# 4) 上游模型一致性
sha256sum tmp/upstream_typhoon_h480.sdf tmp/typhoon_h480.sdf

# 5) gzserver 实际用的渲染器与 CPU
grep -o 'swrast_dri.so\|libGLX_mesa.so.0' /proc/<gzserver_pid>/maps | sort -u
ps -L -p <gzserver_pid> -o tid,time,pcpu --sort=-pcpu | head
```
