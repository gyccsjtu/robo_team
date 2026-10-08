"""Adapted robocup_new.py pursuit lifecycle; no ROS, truth or lock release."""


class LegacyPursuit:
    """Distant cues get one bounded approach, not indefinite target ownership."""
    def __init__(self, approach_s=30.):
        self.approach_s = approach_s
        self.key = None
        self.started = None
        self.qualified = False
        self.failed = False

    def observe(self, observation):
        if not self.failed and observation.get('evidence_kind') != 'navigation_candidate':
            self.qualified = True

    def expired(self, generation, target_id, now):
        key = (generation,target_id)
        if key != self.key:
            self.key, self.started, self.qualified, self.failed = key, now, False, False
        if now < self.started or (not self.qualified and now-self.started >= self.approach_s):
            self.failed = True
        return self.failed
