# 六机快速修复：三次实跑结果

2026-10-04，仓库分支codex/radar-coordination-20261003。**最终未达到600秒内6/6目标。**

| 轮次 | 种子 | 实际仿真秒 | 裁判消除 | 建筑/人物/机间接触 | 最高真实高度m |
|---|---|---:|---:|---|---:|
| 1 | 5334 | 90.032 | 1/6 | 0/0/0 | 3.0593 |
| 2 | 2523 | 300.060 | 3/6 | 0/0/0 | 3.5396 |
| 3 | 8159 | 600.124 | 4/6 | 0/0/0 | 3.4917 |

三轮均六机实际解锁移动，原生接触观察覆盖轨迹，执行源码SHA与运行清单一致。
表中时间来自原result；validation_audit的observed_sim_s是首末轨迹采样跨度，略短于运行计时。
第3轮剩actor_3（白色）和actor_4（红色），删除依据为裁判is OK，tracking success不当作删除。
没有建筑接触仅证明本轮轨迹，没有复现旧同一路线，不证明随机图全部适用。

## 执行版本与未实跑候选

第3轮实际使用c77063e控制/视觉v2.12.1；任务schema2、路线schema1、阻塞反馈schema1。
所有飞机使用同一源码快照、共享GPU，局部2.2m配置与WSL内存未改，真实高度单独观察。

后补v1.24b阻止同一飞机重复领取原地失败任务，并对绿白新确认尝试单独计数；444项检查通过。
这两项没有进入第3轮快照，三次预算已用完，未启动第四轮，物理效果仍未知。
白色有持续上报中断及相机失去观测，红色保留static/hits/height/self拒发策略；
详细计数和日志定位见round_3_seed_8159/remaining_blockers.json。CSV行数不是独立原图数。

## 证据与复核

各轮validation_audit.json为只读复核汇总，flight/result.json保留启动器原结果。
archive_manifest.json记录完整本地归档每个文件SHA；gzserver完整大日志留WSL、归档只留最后8KB。
flight/swarm_source_manifest.json记录实际执行文件SHA，不用事后工作区代码冒充已执行版本。

完整本地归档在本README所在目录；原始WSL根为
`/root/robocup_runs/fast_competition_v124_20261004`。
本次Git记录保留汇总、原result、裁判日志、轨迹、原生接触、源码/配置清单及阻塞摘录；
较大的导航窗口、完整命令及源码快照保留在本地归档与WSL，不需要重新跑仿真来复核。

Windows PowerShell下只读复核现有三轮：

```powershell
wsl -d Ubuntu-20.04 -- python3 /mnt/d/a/.robocup/robo_team/coordination/scripts/audit_fast_city.py --root /root/robocup_runs/fast_competition_v124_20261004 --archive /mnt/d/a/.robocup/robo_team/coordination/docs/validation/fast_competition_20261004
```

budget.json三个attempt均ENDED，监听端口11375/11376/19731已关闭。
不要删除预算重复执行这三轮。本轮没有更改原始官方裁判；红色适配遵循用户转述的赛事组口径，
来源及副本SHA见scene_manifest，不能冒称官方已经发布新版。
自身相机位姿仍为开发Gazebo link pose，正式比赛许可、实体机和完整三维通行证明未验收。
