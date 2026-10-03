# 城市联调启动配置 v3

配置语义修订v1.2：城市最大水平速度改为3m/s，manager飞行时间估算与agent共读SWARM_MAX_SPEED。旧1m/s调试限速慢于规则中的2m/s逃跑目标，且旧manager仍按5m/s估时。雷达制动限速、机间约束和授权边界继续生效；3m/s不是官方速度要求，也不意味着实际每段都能达到该速度。启动文件字段不变，完整运行须整套共享新配置。

开发启动器在隔离副本中运行地图生成器，检查原平台六机出生位置与生成的障碍矩形是否相交；失败地图不得放行飞行。v3 启动文件只向 agent 提供出生点附近半径 2m 的初始化区域，不提供城市障碍物或目标位置。完整地图文件仅供场景生成、目标控制和独立诊断。

字段：schema_version=3、purpose=DEVELOPMENT_CITY_STARTUP、run_id、world_path、world_sha256、positions、startup_checks（每机 free_radius_m）。旧空场景 v1、箱体场景 v2 保持兼容。此检查是生成器矩形近似，不是完整三维安全证明，城市联调不得直接标成正式比赛通过。

城市配置同步使用 SWARM_ROUTE_CLEARANCE_M=2.5、SWARM_FLEET_SEPARATION_M=2.5，以适配平台 3m 初始队形。它们是开发参数而非官方规定，manager 与全部 agent 必须共享同一配置；默认旧配置仍为路线 6m、机间 4.5m。路线 JSON schema_version=1 不变，配置语义修订为 v1.1；失联/到期不释放占用，停稳交接规则不变。
