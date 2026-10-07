"""Verify existing autonomous-flight parameters before any arming attempt."""


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
