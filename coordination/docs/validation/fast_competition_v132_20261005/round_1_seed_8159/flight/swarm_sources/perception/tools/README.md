# perception/tools —— 诊断与验收工具

> 这些脚本原先只存在于开发机 `_real_flight/`，未随交付仓库分发。
> 2026-09-24 归位。全部无硬编码路径，用 `ROBOCUP_*` 环境变量或命令行参数。

## 验收器（回答「够不够格」的问题）

| 脚本 | 用途 |
|---|---|
| `val_cm.py` | 在 val 集上跑混淆矩阵，定量确认各类互混（brown/red） |
| `diag_identity.py` | 身份对照诊断：并排打印 actor 真值与各流上报，判定「判错类」还是「本来就有人」 |
| `diag_geom.py` | 几何诊断：坐标解算链路逐段核对 |
| `dual_check.py` | 双红核查：两条 red 流是否指向同一人 |
| `m3_verify.py` | M3 验收器（双口径）：官方判分口径 + 感知质量口径 |
| `audit_brown_red.py` | brown/red 专项审计 |
| `test_red_slots.py` | red 双槽绑定验证（red1/red2 是否各成流） |
| `test_assoc_gate.py` | track 关联门限测试 |
| `calib_selfmask.py` | 自掩膜标定：把 UAV 自身/挂载件从画面里排除 |
| `probe_target_report.py` | target_report 契约探测（逐字段校验） |
| `ht_watch.py` | 观测台监视器：实时回显 hover_teleport 钉住的目标与误差 |
| `hover_teleport.py` | 瞬移悬停观测台：把「飞行」从感知验证里剥离（比赛调试用得上） |
| `fly_to_actor.py` | M2 飞行闭环：起飞→逼近→悬停跟随→误差统计 |
| `snap_frame.py` | 单帧抓取（相机原图落盘） |
| `m5_dump_analyze.py` | M5 快照转储分析 |

## 典型用法

```bash
# 混淆矩阵（需 torch+ultralytics，在部署机跑）
python3 tools/val_cm.py --weights ../weights/best_yolo11n_bino_v1.pt --data ../config/data.yaml

# 观测台：把飞行剥离，专测感知（比赛调试黄金工具）
python3 tools/hover_teleport.py 3 12 5.5 1800   # actor_3, 12m, 5.5m 高, 1800 秒

# red 双槽验证
python3 tools/test_red_slots.py

# M3 双口径验收
python3 tools/m3_verify.py
```

## 注意

- `hover_teleport.py` 会每 100ms 用 `set_model_state` 钉住 UAV ⇒ **不要与飞控（PX4 arm）同时开**，会互相对抗导致姿态震荡。
- `val_cm.py` 需要 torch/ultralytics；其余工具只依赖 rospy/cv_bridge，随镜像走。
