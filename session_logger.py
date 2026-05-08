import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from robot_config import LoggingConfig


SESSION_LOG_COLUMNS = [
    "seq",
    "host_time_ns",
    "elapsed_ms",
    "source",
    "event_type",
    "robot_timestamp_ms",
    "robot_seq",
    "raw_line",
    "payload_json",
]


class SessionLogger:
    def __init__(
        self,
        enabled: bool,
        directory: str | Path = "logs",
        flush_each_record: bool = True,
        filename: str | None = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.flush_each_record = bool(flush_each_record)
        self.path: Path | None = None
        self._seq = 0
        self._start_monotonic_ns = time.monotonic_ns()
        self._file = None
        self._writer: csv.DictWriter | None = None

        if not self.enabled:
            return

        log_dir = Path(directory)
        log_dir.mkdir(parents=True, exist_ok=True)
        if filename is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"slam_session_{stamp}.csv"

        self.path = log_dir / filename
        self._file = self.path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=SESSION_LOG_COLUMNS)
        self._writer.writeheader()

    @classmethod
    def from_config(cls, config: LoggingConfig) -> "SessionLogger":
        return cls(
            enabled=config.enabled,
            directory=config.directory,
            flush_each_record=config.flush_each_record,
        )

    def log_event(
        self,
        source: str,
        event_type: str,
        payload: dict[str, Any] | None = None,
        robot_timestamp_ms: int | None = None,
        robot_seq: int | None = None,
        raw_line: str = "",
    ) -> None:
        if self._writer is None:
            return

        self._seq += 1
        host_time_ns = time.time_ns()
        elapsed_ms = (time.monotonic_ns() - self._start_monotonic_ns) / 1_000_000
        self._writer.writerow(
            {
                "seq": self._seq,
                "host_time_ns": host_time_ns,
                "elapsed_ms": f"{elapsed_ms:.3f}",
                "source": source,
                "event_type": event_type,
                "robot_timestamp_ms": "" if robot_timestamp_ms is None else robot_timestamp_ms,
                "robot_seq": "" if robot_seq is None else robot_seq,
                "raw_line": raw_line,
                "payload_json": json.dumps(
                    payload or {},
                    sort_keys=True,
                    separators=(",", ":"),
                    default=str,
                ),
            }
        )
        if self.flush_each_record and self._file is not None:
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

    def __enter__(self) -> "SessionLogger":
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
