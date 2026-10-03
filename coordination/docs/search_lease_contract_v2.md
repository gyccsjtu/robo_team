# 搜索格租约接口 v2（2026-10-03 冻结）

范围：`swarm_task.LeaseManager` 的纯 Python 生命周期及 manager 接入；`schema_version=2` 为本地租约语义版本，独立于 robocup_navigation 的旧 JSON schema1。ROS SearchAssignment 暂未版本化，不能把本修复当作完整通行预约或目标唯一锁。

旧行为：expire(now) 将超时格置 FREE、清空 owner，manager 删除活跃租约记录并重新拍卖。新行为：expire(now) 返回本次新发现过期的格，但保留 ASSIGNED、owner、lease_until 和 manager 活跃记录；返回值仅表示“需检查失联/超时”，不表示授权释放。重复 expire 不重复报告；同一持有者有新鲜状态时可 renew 延长原租约并清除过期报告标记。renew 不改变持有者。grant 拒绝抢占他机尚在 ASSIGNED 的格。

迁移：所有 expire 调用者必须删除基于返回值清 owner/删除活跃任务/重新派给其他飞机的操作；日志使用“占用保留”，不使用“已释放”。旧返回列表类型保留，语义不兼容，禁止用旧逻辑接新核心。

边界：搜索格不是几何通道。现有覆盖标记、转追踪任务、目标认领及停止确认尚未统一为运行标识/授权代次的锁协议；本版不提供失败机的自动安全交接。旧机停止及冲突区域退出没有证据时不得将本版升级描述为“安全接替已完成”。

兼容增补（2026-10-03）：实际任务授权v2已经接入。搜索转追踪只提交切换意图，不能将旧格提前置FREE；任务STOP、停稳或TTL超时也不能清搜索格占用。只有同轮TaskAuthority的TASK_RELEASED事件（search键，原owner一致，核心已检查停稳与退出）才能将仍ASSIGNED的旧格置FREE。COVERED状态和已改变owner的格不得回退。字段和版本不变，修正旧manager绕过占用保留的直写行为；这不增加未知失败机自动释放能力。
