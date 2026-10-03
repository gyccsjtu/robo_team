"""Thread-safe evidence recording and strict post-run stream validation."""
import json
from pathlib import Path
import threading


class JsonlEvidence:
    def __init__(self, path):
        self.handle = Path(path).open('w', encoding='utf-8')
        self.lock = threading.Lock()
        self.closed = False

    def write(self, value):
        line = json.dumps(value, allow_nan=False) + '\n'
        with self.lock:
            if self.closed:
                return False
            self.handle.write(line)
        return True

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.handle.close()


def audit_jsonl(paths):
    rows, errors = {}, []
    for path in paths:
        count = 0
        try:
            with Path(path).open(encoding='utf-8') as stream:
                for number, line in enumerate(stream, 1):
                    try:
                        json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                        count += 1
                    except (ValueError, TypeError) as error:
                        errors.append(dict(path=str(path), line=number, error=str(error)))
        except (OSError, UnicodeError) as error:
            errors.append(dict(path=str(path), error=str(error)))
        rows[str(path)] = count
    return dict(valid=not errors, parsed_rows=rows, errors=errors)
