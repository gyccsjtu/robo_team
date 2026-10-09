# 雷达六机接入映射 v1

只冻结接入层命名与端口，不更改 Swarm 任务授权 v2，不构成出生点净空或六机飞行验收。

`scripts/prepare_radar_fleet.py` 从实际 PX4 构建目录的 `px4-rc.mavlink`、`px4-rc.simulator` 和 `rcS` 提取端口及 system ID 映射，保存三个文件的 SHA-256。不读取旧 `fleet.yaml` 的端口。实例使用1..6，保留实例0给既有单机入口。

JSON 的 `schema_version=1`；`uavs` 每项包含逻辑 ID、模型名、MAVROS 命名空间、雷达话题、PX4 实例、system ID、FCU URL、仿真 TCP/UDP 端口和发布者锁 key。模型名与逻辑 ID 均为 `uav_1..uav_6`，MAVROS 为 `/uav_N/mavros`，雷达为 `/uav_N/scan`。后续模型生成和启动必须消费同一份映射。仿真 UDP 的14560基值属于模型适配约定，不能声称从 PX4 init 脚本得到。

当前机器实际提取结果见 `validation/radar_fleet_wiring_20261003.json`：PX4/MAVROS 为14581..14586 / 14541..14546，仿真 TCP 为4561..4566，system ID 为2..7。旧 `config/multi_uav/fleet.yaml` 的3458x/2454x属于另一套补丁环境，不能直接驱动当前构建。

生成命令（输出文件必须不存在）：

```powershell
wsl -d Ubuntu-20.04 -- bash -lc 'python3 /mnt/d/a/.robocup/robo_team/coordination/scripts/prepare_radar_fleet.py --px4-build /root/third_party_official/PX4-Autopilot/build/px4_sitl_default --output /tmp/radar_fleet_wiring.json'
```

不支持的公式、重复端口和未核实的 system ID 会终止生成。接口变更须增加版本并给出迁移说明。此次不覆盖旧机队文件，也不调用旧启动/清理脚本。

下一步生成逐机雷达 SDF，核对传感器、插件话题和 MAVLink 端口，然后核实六个出生点和实际连接。`flight_ready=false`、`spawn_clearance_verified=false` 明确标示这些工作尚未完成，当前 JSON 不允许直接作为飞行放行依据。

验证：4项测试通过，覆盖当前端口、另一套补丁端口、端口冲突拒绝及未核实 system ID 拒绝；另已读取当前 WSL 的真实构建文件生成存档。
