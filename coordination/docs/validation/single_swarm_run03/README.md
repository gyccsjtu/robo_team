# 单机真实仿真证据 run03

2026-10-03，本仓库 SwarmAgent + SwarmManager/TaskAuthority v2，Gazebo/PX4/MAVROS 一架飞机。源执行使用冻结副本，源码SHA见 source_manifest.json。repository_commit 是采样前仓库HEAD；当时未提交修改包含在source_snapshot.zip中，不能只按该Git提交还原运行源码。

- result.json、telemetry.jsonl：估计位姿与两段任务到达。
- truth.jsonl：独立Gazebo自机轨迹，不回灌控制。
- authority_events.jsonl：换任务STOP、停稳确认、原锁保留及退出后释放。
- audit_report.json：独立终点水平误差0.27484m，末速0.06122m/s，最高4.82664m，796样本，最大间隔0.052s；有限原型到达验证true。
- source_snapshot.zip：实际执行的代码与metadata；忽略运行自动生成的__pycache__，原始文件SHA可与manifest逐项核对。

仅单机两段搜索任务的开发用例。读取存储开发地图，场景等价性未验证；没有完整六机搜索、目标视觉、几何通道、三维净空或完整contacts证据。碰撞ABSTAIN、正式比赛false。不能据此宣称六机产品已验收。
