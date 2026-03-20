"""CSV data logger for the BallButler rover sim.

Logs timestamped simulation data (commands, sensor readings, etc.) to a CSV
file in the logs/ directory.  Designed to be extended in later phases as more
signals become available.
"""

import csv
import time
from pathlib import Path
from typing import Dict, List, Optional

from config import SimConfig, LOGS_DIR


class DataLogger:
    """Buffered CSV logger that writes simulation data to disk."""

    def __init__(self, cfg: SimConfig, extra_fields: Optional[List[str]] = None):
        self.cfg = cfg
        self.enabled = cfg.log_enabled

        # Default fields present in every phase
        self.fields = [
            "sim_time",
            "wall_time",
            "cmd_vx",
            "cmd_vy",
            "cmd_omega",
        ]
        if extra_fields:
            self.fields.extend(extra_fields)

        self._buffer: List[Dict[str, float]] = []
        self._file = None
        self._writer = None
        self._last_flush = 0.0

        if self.enabled:
            LOGS_DIR.mkdir(parents=True, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filepath = LOGS_DIR / f"sim_log_{timestamp}.csv"
            self._file = open(filepath, "w", newline="")
            self._writer = csv.DictWriter(self._file, fieldnames=self.fields,
                                          extrasaction="ignore")
            self._writer.writeheader()
            self._last_flush = time.monotonic()
            print(f"[DataLogger] Logging to {filepath}")

    def log(self, record: Dict[str, float]):
        """Append a record.  Missing fields default to 0."""
        if not self.enabled:
            return
        row = {f: record.get(f, 0.0) for f in self.fields}
        self._writer.writerow(row)

        # Periodic flush so we don't lose data on crash
        now = time.monotonic()
        if now - self._last_flush >= self.cfg.log_flush_interval_s:
            self._file.flush()
            self._last_flush = now

    def close(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
            print("[DataLogger] Log file closed.")
