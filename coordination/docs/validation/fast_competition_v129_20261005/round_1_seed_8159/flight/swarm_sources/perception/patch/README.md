# patch/ —— 上机前置改动（不打这些，感知节点收不到图）

这个目录放的是**必须对仿真环境做的改动**。它们不属于本仓库，但没做的话
感知节点起来也是空转。**按顺序做，做完逐条验证。**

---

## 1. 给无人机装双目相机 `add_stereo_camera.py`

### 为什么需要

感知节点订阅 `/<uav>/stereo_camera/left/image_raw`。
**PX4 原厂的 typhoon_h480 只有云台相机（cgo3），没有 stereo_camera** ——
不打这个 patch，话题根本不存在，现象是"节点在跑、但一条检测都没有"。

### 一条命令

```bash
# 先看状态（只读，不改任何文件）
python3 add_stereo_camera.py --check

# 打 patch（自动备份为 *.bak_robocup_stereo）
python3 add_stereo_camera.py --px4-root ~/PX4_Firmware

# 需要时可以回滚
python3 add_stereo_camera.py --revert
```

它会改两份文件（都在 `Tools/sitl_gazebo/models/typhoon_h480/`）：

| 文件 | 为什么 |
|---|---|
| `typhoon_h480.sdf` | Gazebo 实际加载的就是这份 |
| `typhoon_h480.sdf.jinja` | PX4 编译时由它生成上面的 `.sdf`；不改的话下次 rebuild 会被覆盖回去 |

### 🔴 这里有个很坑的地方

XTDrone 官方 `stereo_camera` 模型用的是
`<sensor type="multicamera">` + `libgazebo_ros_multicamera.so`。
**本机（gazebo 9 + ROS Noetic）实测：插件能加载、话题也真的 advertise 出来了**

```
$ rostopic list | grep stereo_camera
/typhoon_h480_0/stereo_camera/left/image_raw      <-- 在！
/typhoon_h480_0/stereo_camera/right/image_raw     <-- 在！
```

**但永远收不到图像帧。** 这是最难查的一类故障 —— 话题存在会让人以为已经对了。

所以本脚本**不用** include 那个模型，而是改成**两只独立单目**
`<sensor type="camera">` + `libgazebo_ros_camera.so`
（机上的 cgo3、拍数据集用的相机都是这条路，验证过能出图）。
**光学参数与基线严格照抄官方 stereo_camera，一个数都没改**：

```
752x480, hfov 1.5708 (90deg), fx=fy=376, cx=376, cy=240,
clip 0.1~300, 30Hz, 基线 0.12 m (y = ±0.06), 畸变系数同平台
```

安装位姿是本队自定的（机身前方 0.12m、下沉 0.05m、左右各 0.06m），可改。
话题名沿用官方约定，所以对下游完全透明。

### 验证：必须看**有没有帧**，不能只看话题在不在

```bash
source ~/robocup_real/env_robocup.sh        # 或等价的环境准备
rostopic hz /typhoon_h480_0/stereo_camera/left/image_raw
```

- 输出 ≈ `average rate: 30` → 成功
- 一直 `no new messages` → **装成了 multicamera 版本**，回去看上面的坑

> ⚠️ `rostopic hz` 在 RTF 偏低时也会掉到 6~10 Hz，这正常（Gazebo 实时率就那样）。
> 但**不能是 0**。

### 已知遗留

XTDrone 自带的那份副本（`XTDrone/sitl_config/models/typhoon_h480/typhoon_h480.sdf.jinja`）
里可能还留着旧的 multicamera include。它当前不生效（该目录没有 `.sdf`，
而 Gazebo 加载的是 `.sdf`），但脚本的 `--check` 会把它标成 `BAD_INCLUDE`。
跑一次不带 `--check` 的 patch 就会顺手清理掉。

---

## 2. 资产核对 `../scripts/verify_official_assets.py`

不做修改，只做核对 —— 确认训练数据用的模型/相机/类别映射**就是官方的**：

```bash
python3 ../scripts/verify_official_assets.py --gt ~/robocup_dataset_gt_v3/gt.json
```

它会逐项对照：

| 检查项 | 依据 |
|---|---|
| 6 个 `walk_N.dae` 的实际颜色 | 读 dae 里的 `<effect>` 色值反推，不听信文件名 |
| 模型是否官方 | XTDrone 仓库 `git ls-files sitl_config/models/` |
| 相机内参 | 官方 `stereo_camera/model.sdf` 的 `Fx/Fy/Cx/Cy/hackBaseline` |
| `base.world` 的 skin | `<actor>` 的 `<skin><filename>` |
| 数据集标注 | `gt.json` 的 `actor` 编号 ↔ `color` |

全部 PASS 才说明"训练时的外观 == 比赛时的外观"。
