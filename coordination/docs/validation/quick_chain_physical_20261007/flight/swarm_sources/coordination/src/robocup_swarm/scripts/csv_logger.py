#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Small process-local CSV logger shared by the simulation nodes."""

import csv
import os
import threading
import time


_LOG_DIR = os.path.expanduser(os.environ.get("ROBOCUP_LOG_DIR", "~/robocup_logs"))


class CsvLogger(object):
    """Append-only CSV logger with a stable header and immediate flush."""

    def __init__(self, name, fields):
        self.path = os.path.join(_LOG_DIR, "%s.csv" % name)
        self.fields = list(fields)
        self._lock = threading.Lock()
        self._file = None
        self._writer = None
        self._open()

    def _open(self):
        os.makedirs(_LOG_DIR, exist_ok=True)
        exists = os.path.isfile(self.path) and os.path.getsize(self.path) > 0
        expected = self.fields
        if exists:
            with open(self.path, "r", newline="", encoding="utf-8") as old_file:
                old_header = next(csv.reader(old_file), [])
            if old_header != expected:
                legacy = "%s.legacy.%d" % (self.path, int(time.time()))
                os.replace(self.path, legacy)
                exists = False
        self._file = open(self.path, "a", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fields,
                                      extrasaction="ignore")
        if not exists:
            self._writer.writeheader()
            self._file.flush()

    def write(self, **row):
        row.setdefault("wall_time", time.time())
        with self._lock:
            self._writer.writerow(row)
            self._file.flush()

    def close(self):
        with self._lock:
            if self._file is not None:
                self._file.close()
                self._file = None
                self._writer = None


def logger(name, fields):
    return CsvLogger(name, ["wall_time"] + list(fields))
