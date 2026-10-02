# 雷达避障模块 · 对接说明（给整合方）

> 版本：2026-09-29（随机图适配 + 三处修复后）
> 交付物：`radar/` 目录全部文件
> 一句话：**给我一个航点文件，我用 2D 雷达反应式避障把它飞完。**

---

## 1 模块边界（先说清楚不做什么）

| 做 | 不做 |
|---|---|
| 从当前位姿出发，按给定航点序列飞行 | **不决定去哪**（航点由你给） |
| 用 2D 激光雷达实时避障、绕行静态障碍 | **不识别目标**（感知是另一套） |
| 到点判定、速度控制、Offboard 起飞/降落 | **不含多机协同**（一进程一机） |
| 世界系 ↔ MAVROS local 系自动换算（含 EKF 原点漂移） | 不做任务规划、不做覆盖搜索策略 |

**关键：它是单机单进程。** 6 架机 = 起 6 个进程，用命令行参数区分。

---

## 2 唯一输入接口：航点文件

格式极简 —— 每行两个数，**世界坐标（米）**，支持 `#` 注释与逗号分隔：

```
# 可选注释行
-6.0 4.0
-14.0 -3.0
-12.0 -39.0
```

规则：
- **必须是世界坐标**（Gazebo 世界系，与 `black_box.txt` / `<state>` 同一套）。
- **不要自己做坐标换算** —— 代码内部会减 EKF 原点增量（原点实测会漂 1.1 m，用实时增量而非固定值）。把 world 坐标直接当 local 坐标发出去会整体偏 1 m 以上。
- 首点通常就是起飞点（会先飞到它再往下走）；给不给都不影响，飞机从当前悬停点出发。
- 到点判定半径 `ARRIVE_R = 1.2 m`。

---

## 3 启动模板

```bash
cd ~/robocup_real
source env_robocup.sh

python3 _radar_test/radar_avoid.py \
    --uav       typhoon_h480_0 \
    --ns        /typhoon_h480_0 \
    --scan-topic /typhoon_h480_0/scan \
    --alt       5.5 \
    --speed     1.5 \
    --wp        /path/to/wp_uav0.txt \
    --verbose
```

6 机就循环起 6 个，把 `_0` 换成 `_1`…`_5`，`--wp` 各给一份。

**也可以不用航点文件**：`--start X Y --goal X Y --black-box <black_box.txt>` 让它自己用 A\* 规划（障碍源优先级 `--black-box` > `--obstacle-txt` > `--world`）。

---

## 4 🔴 三条硬前置（不满足会**静默失效**）

### 4.1 launch 必须用雷达机型

```xml
<arg name="vehicle" value="typhoon_h480"/>
<arg name="sdf"     value="typhoon_h480_lidar"/>   <!-- ← 关键 -->
```

⚠️ **现有所有 launch 都写的是 `sdf=typhoon_h480`（无雷达版）**，包括：
`uav_only.launch`、`robocup_1uav.launch`、`_expm/multi_uav_*.launch`，
以及多机生成器 `_expm/gen_multi_uav_cam.py`（模板里写死）。

⇒ **直接拿现成 launch 起 6 机，雷达根本不存在**，避障会一帧 scan 都收不到。

### 4.2 `<group ns>` 必须与模型名一致

launch 里 `<group ns="typhoon_h480_N">` + `sdf` spawn 出的模型名也是 `typhoon_h480_N`。

🔴 **命名空间写错时订阅会全落空、且不报错** —— 表现是"飞机不动、日志正常"。
（历史坑：曾出现 mavros 在 `/uav_radar_0/`、雷达在 `/typhoon_h480_0/`，
两者不同源 ⇒ `--ns` 给错就 4 个订阅全空。现在两种写法都收：给
`/xxx` 自动补 `/mavros`，给 `/xxx/mavros` 原样用。）

### 4.3 雷达 SDF 副本要在**两处**

```
/data/PX4_Firmware/Tools/sitl_gazebo/models/typhoon_h480_lidar/
~/XTDrone/sitl_config/models/typhoon_h480_lidar/
```

两处内容必须一致（当前 md5 `799c7ca864f36d21de296a7badfc6419`）。
换机器/重装环境时**必须带上**，否则 Gazebo 找不到模型名。

---

## 5 已知静默失败模式（最难查，按这个顺序排查）

| 现象 | 原因 | 查法 |
|---|---|---|
| 飞机不动、日志正常 | ns 写错 ⇒ mavros 订阅全空 | `rostopic list \| grep mavros` 看真实前缀 |
| 收不到 scan、`Valid=0` | launch 的 `sdf` 不是 `_lidar` 版 | `rostopic hz /typhoon_h480_0/scan` |
| 有 scan 但近距恒 1.7~3.8 m | 雷达装太低、扫到 sonar 自检锥体 | 见 §6 自检第 3 项 |
| 速度远低于 `--speed` | PX4 位置环 `MPC_XY_P` 与代码不一致 | 代码里 `MPC_XY_P=0.95`（实测值），改 PX4 参数要同步 |
| 位置整体偏 1 m+ | 拿 world 坐标当 local 坐标用了 | 只喂 world 坐标，别自己换算 |
| 拿不到位姿 | `/gazebo/model_states` 没发布 | 确认起世界走的是 gazebo_ros |

---

## 6 交接自检（一条命令）

栈起来后跑：

```bash
cd ~/robocup_real && source env_robocup.sh
python3 vm_radar_check.py --uav typhoon_h480_0 --secs 8
```

它查 6 项，输出 `RADAR_CHECK_PASS` 才算可用：

| # | 检查 | 期望 |
|---|---|---|
| 1 | scan 频率 | ~250 Hz（实测 248.6） |
| 2 | 有效束数 | 悬停空旷点应很低；看到建筑则几十~几百 |
| 3 | **近距假回波** | **<3.5 m 的束数必须 ≈ 0** |
| 4 | 激光参数 | 512 束 / ±180° / 0.5–20 m（**必须与 XTDrone 自带一致**，规则要求） |
| 5 | mavros ns | 能订阅到 `<ns>/mavros/state` |
| 6 | 调试节点残留 | 无 `diag_scan` / `watch_lidar_rel` / `probe_*` / `dump_scan` |

> ⚠️ 第 6 项是**合规要求**：规则 2.5(11) 规定裁判系统实时监控所有节点的话题订阅，
> 比赛前必须清掉全部诊断/调试节点。

---

## 7 不要做的事

| 别做 | 原因 |
|---|---|
| 改雷达参数（samples/角度/量程） | 规则要求**必须与 XTDrone 自带一致**，改了违规 |
| 把雷达安装位改成"悬空高位" | 规则要求安装角/位**符合实际物理规律**；当前是官方贴装 0.080 m |
| 放宽 `GAP_DEV_MAX`（70°） | 三次放宽全部严重回归，是两级架构的硬约束 |
| 用"腐蚀栅格"消假阳性 | 会把细杆 margin 这类真缺陷一起藏掉 |
| 竞赛时开着调试订阅节点 | 违规（见 §6 第 6 项） |

---

## 8 已验证到什么程度（让人放心用）

| 验证 | 规模 | 结果 |
|---|---|---|
| 离线单测 | 35 项 | **35/35** |
| 闭环仿真 | 5 场景 | 全到达、0 碰撞、净空 0.974 m |
| 压力测试 | 6 组 | 28/28 PASS |
| 两级集成 | 4 场景 | **4/4** |
| 随机图全链路（官方生成器→A\*→雷达控制律） | 6 图 × 6 航线 × 3 种子 = **108 组** | A\* 零穿墙、全到达、**0 碰撞** |
| VM 真飞 | **5 张官方随机图**（try1/2/3/5/6） | 全 PASS，最小净空 1.88 m |
| 雷达传感器自检 | VM 实测 | 248.6 Hz、参数一致、**无自检假回波** |

**巡航高度固定 5.5 m** —— 官方 `score_cal.py` 明文 `z > 6.0` ⇒ `score = 0` 并终止该次尝试。
旧版默认 6.0，实测峰值 6.13 m 会直接归零，已改。

---

## 9 文件清单

| 文件 | 作用 |
|---|---|
| `radar_avoid.py` | 主程序（雷达避障 + 航点飞行） |
| `astar_plan.py` | A\* 全局规划 + 官方障碍源解析 |
| `start_radar_stack.sh` | 起雷达栈（世界 + PX4 + MAVROS） |
| `vm_radar_check.py` | **交接自检**（§6） |
| `preflight_check.py` | 起飞前自检（查空 world / 起飞点被压） |
| `random_map_fly.py` | 随机图全链路验收 |
| `test_*.py` / `sim_*.py` | 四套回归 |
| `fly_rand.sh` / `fly_multi.sh` | VM 真飞模板 |

依赖：Python 3 + `rospy` / `mavros_msgs` / `sensor_msgs`（VM 已装），`numpy` 可选。
`astar_plan.py` 纯标准库，可离线跑。
