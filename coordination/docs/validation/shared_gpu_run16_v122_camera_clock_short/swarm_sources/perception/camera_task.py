"""Read-only view of the executor's versioned task for camera filtering."""
from task_authority import TaskGate


class CameraTask:
    def __init__(self, run_id, logical_uid):
        self.gate = TaskGate(run_id,logical_uid)

    def receive(self, message, now):
        if not isinstance(message,dict):
            return False
        try:
            return self.gate.receive(message,now)
        except (ValueError,TypeError,KeyError):
            return False

    def target(self, now):
        if (not self.gate.can_move(now) or self.gate.task is None
                or self.gate.task['task_type'] != 1):
            return None
        return self.gate.task['target_id']
