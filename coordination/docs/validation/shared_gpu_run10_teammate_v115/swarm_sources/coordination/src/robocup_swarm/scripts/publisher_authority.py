"""POSIX process lock for a physical model's SwarmAgent setpoint authority."""
import hashlib
from pathlib import Path


class PublisherAuthority:
    def __init__(self, model, directory='/tmp'):
        self.path = Path(directory) / ('robo_team_setpoint_' + hashlib.sha256(model.encode()).hexdigest()[:20] + '.lock')
        self.file = None

    def __enter__(self):
        import fcntl
        handle = self.path.open('a+')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close()
            raise RuntimeError('SETPOINT_AUTHORITY_ALREADY_HELD')
        self.file = handle
        return self

    def __exit__(self, *_):
        self.file.close()
        self.file = None
