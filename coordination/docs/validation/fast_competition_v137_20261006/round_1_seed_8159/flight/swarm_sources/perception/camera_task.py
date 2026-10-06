"""Read-only view of the executor's versioned task for camera filtering."""
from task_authority import TaskGate


class CameraTask:
    def __init__(self, run_id, logical_uid):
        self.gate = TaskGate(run_id,logical_uid)
        self.generation_s = 0.

    def receive(self, message, now):
        if not isinstance(message,dict):
            return False
        try:
            previous = self.gate.generation
            accepted = self.gate.receive(message,now)
            if accepted and previous != self.gate.generation:
                self.generation_s = now
            return accepted
        except (ValueError,TypeError,KeyError):
            return False

    def search_context(self, now, image_s):
        """Bind inference to a search grant; acquiring an image is not observing."""
        if (now < self.gate.last_s or image_s < self.generation_s
                or not self.gate.can_move(now) or self.gate.task is None
                or self.gate.task['task_type'] != 0):
            return None
        return dict(generation=self.gate.generation,
                    cell=[self.gate.task['cell_ix'], self.gate.task['cell_iy']])

    def target(self, now):
        # A camera iteration can begin before a later grant callback. Such an
        # old read is not an executor clock rollback and must not latch STOP.
        if now < self.gate.last_s:
            return None
        if (not self.gate.can_move(now) or self.gate.task is None
                or self.gate.task['task_type'] != 1):
            return None
        return self.gate.task['target_id']
