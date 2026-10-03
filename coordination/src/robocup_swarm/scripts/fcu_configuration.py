"""解锁前的飞控自主飞行参数校验。

来源：队友 `codex/radar-coordination-20261003` 分支的 `fcu_configuration.py`
原样搬入（纯函数、无 ROS 依赖，可直接单测）。

背景（我方 10-03 采信）：
    MAVROS 的参数列表尚未就绪时，`param/set` 会**被拒绝但不报错**。老实现
    `swarm_agent._configure_fcu` 只调 set、不看返回值，于是飞机带着"其实没生效"
    的 NAV_RCL_ACT/COM_RCL_EXCEPT 继续 arm / 进 OFFBOARD，最后在 RC 失联
    failsafe 上被拦下 —— 表象是"解锁失败"，根因却在几百行之前的参数设置处，
    极难排查。这是典型的**静默失效**。

做法：
    pull（拉取全参数表）→ set → get（读回比对），重复 attempts 轮；
    仍不一致就抛异常终止启动。**宁可引起不来，也不要带着未生效参数上天。**
"""


def configure(pull, set_parameter, get_parameter, required, attempts=3):
    for _ in range(attempts):
        if not pull():
            continue
        configured = True
        for name, value in required.items():
            if not set_parameter(name, value) or get_parameter(name) != value:
                configured = False
                break
        if configured:
            return
    raise RuntimeError('FCU_CONFIGURATION_NOT_VERIFIED')
