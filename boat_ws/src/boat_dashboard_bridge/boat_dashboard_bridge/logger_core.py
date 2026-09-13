#!/usr/bin/env python3

"""ROS-independent mission-log lifecycle and CSV writer."""

import csv
import json
import os
import re
import threading
from collections import deque
from datetime import datetime, timezone
from pathlib import Path


LABEL_RE = re.compile(r"[^a-zA-Z0-9_-]+")
MISSION_RE = re.compile(r"^mission_(\d+)_")


def clean_label(value):
    """Return a short filename-safe test label."""
    label = LABEL_RE.sub("_", str(value or "").strip()).strip("_")
    return label[:40]


class MissionLogCore:
    """Own pending/recording/closing state independently of ROS."""

    def __init__(
        self,
        log_directory,
        fieldnames,
        prearm_seconds=5.0,
        disarm_grace_seconds=5.0,
    ):
        self.log_directory = Path(log_directory).expanduser()
        self.fieldnames = list(fieldnames)
        self.prearm_seconds = float(prearm_seconds)
        self.disarm_grace_seconds = float(disarm_grace_seconds)

        self.lock = threading.RLock()
        self.buffer = deque()
        self.events = deque()
        self.last_row = None

        self.state = "IDLE"
        self.label = ""
        self.pending = False
        self.recording = False
        self.mission_id = None
        self.file_path = None
        self.metadata_path = None
        self.row_count = 0
        self.last_end_reason = None
        self.last_error = None

        self._armed = None
        self._start_monotonic = None
        self._disarm_deadline = None
        self._metadata = {}
        self._file = None
        self._writer = None

    def reset(self, label, metadata=None, now_monotonic=0.0, ros_time=""):
        """Close the old run and create a number-free pending session."""
        with self.lock:
            if self.recording:
                self._close("RESET_MISSION", now_monotonic, ros_time)

            self._close_file()
            self.buffer.clear()
            self.events.clear()
            self.last_row = None
            self.pending = True
            self.recording = False
            self.state = "PENDING"
            self.label = clean_label(label)
            self.mission_id = None
            self.file_path = None
            self.metadata_path = None
            self.row_count = 0
            self.last_error = None
            self._start_monotonic = None
            self._disarm_deadline = None
            self._metadata = dict(metadata or {})
            self.events.append(("RESET_MISSION", "pending session created"))
            return self.status()

    def queue_event(self, event_type, event_text=""):
        """Attach an event to the next 20 Hz snapshot row."""
        with self.lock:
            self.events.append((str(event_type), str(event_text)))

    def observe_armed(self, armed, now_monotonic, ros_time=""):
        """Apply arm/disarm transitions to the logging lifecycle."""
        with self.lock:
            armed = bool(armed)
            previous = self._armed
            self._armed = armed

            if previous is armed:
                return

            if previous is None:
                if armed and self.pending and not self.recording:
                    self._start_recording(now_monotonic, ros_time)
                    self.events.append(("ARMED", "mission recording started"))
                return

            if armed:
                if self.recording and self._disarm_deadline is not None:
                    self._disarm_deadline = None
                    self.state = "RECORDING"
                    self.events.append(("REARMED", "disarm close cancelled"))
                elif self.pending and not self.recording:
                    self._start_recording(now_monotonic, ros_time)
                    self.events.append(("ARMED", "mission recording started"))
                else:
                    self.events.append(("ARMED", "vehicle armed"))
            else:
                self.events.append(("DISARMED", "vehicle disarmed"))
                if self.recording:
                    self._disarm_deadline = (
                        float(now_monotonic) + self.disarm_grace_seconds
                    )
                    self.state = "CLOSING"

    def add_snapshot(self, row, now_monotonic, ros_time=""):
        """Buffer or write one cached-value snapshot."""
        with self.lock:
            output = dict(row)
            output["_monotonic"] = float(now_monotonic)
            output.setdefault("ros_time", ros_time)
            self._attach_events(output)
            self.last_row = dict(output)

            if self.pending and not self.recording:
                self.buffer.append(output)
                cutoff = float(now_monotonic) - self.prearm_seconds
                while (
                    self.buffer
                    and self.buffer[0]["_monotonic"] < cutoff
                ):
                    self.buffer.popleft()

            elif self.recording:
                self._write_row(output)

                if (
                    self._disarm_deadline is not None
                    and not self._armed
                    and float(now_monotonic) >= self._disarm_deadline
                ):
                    self._close(
                        "DISARMED_5S",
                        now_monotonic,
                        ros_time,
                    )

            return self.status()

    def close(self, reason, now_monotonic=0.0, ros_time=""):
        """Close an active log; an unarmed pending session is discarded."""
        with self.lock:
            if self.recording:
                self._close(str(reason), now_monotonic, ros_time)
            else:
                self.pending = False
                self.buffer.clear()
                self.events.clear()
                self.state = "IDLE"
            return self.status()

    def update_metadata(self, values):
        """Merge late parameter results and refresh the sidecar file."""
        with self.lock:
            self._metadata.update(dict(values or {}))
            if self.recording:
                self._write_metadata()

    def status(self):
        """Return JSON-serializable logger state."""
        with self.lock:
            return {
                "state": self.state,
                "pending": self.pending,
                "recording": self.recording,
                "mission_id": self.mission_id,
                "label": self.label,
                "file_path": (
                    str(self.file_path) if self.file_path else None
                ),
                "metadata_path": (
                    str(self.metadata_path) if self.metadata_path else None
                ),
                "row_count": self.row_count,
                "buffer_rows": len(self.buffer),
                "last_end_reason": self.last_end_reason,
                "last_error": self.last_error,
            }

    def _start_recording(self, now_monotonic, ros_time):
        cutoff = float(now_monotonic) - self.prearm_seconds
        while (
            self.buffer
            and self.buffer[0]["_monotonic"] < cutoff
        ):
            self.buffer.popleft()

        self.log_directory.mkdir(parents=True, exist_ok=True)
        self.mission_id = self._next_mission_id()
        stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
        suffix = f"_{self.label}" if self.label else ""
        stem = f"mission_{self.mission_id:03d}_{stamp}{suffix}"
        self.file_path = self.log_directory / f"{stem}.csv"
        self.metadata_path = self.log_directory / f"{stem}_metadata.json"

        try:
            self._file = self.file_path.open(
                "x", newline="", encoding="utf-8"
            )
            self._writer = csv.DictWriter(
                self._file,
                fieldnames=self.fieldnames,
                extrasaction="ignore",
            )
            self._writer.writeheader()
            self._start_monotonic = float(now_monotonic)
            self.pending = False
            self.recording = True
            self.state = "RECORDING"
            self.row_count = 0
            self._metadata.update({
                "mission_id": self.mission_id,
                "label": self.label,
                "started_ros_time": ros_time,
                "csv_path": str(self.file_path),
                "prearm_seconds": self.prearm_seconds,
                "disarm_grace_seconds": self.disarm_grace_seconds,
            })
            self._write_metadata()

            while self.buffer:
                self._write_row(self.buffer.popleft())
        except Exception as exc:
            self.last_error = str(exc)
            self.state = "ERROR"
            self.pending = False
            self.recording = False
            self._close_file()
            raise

    def _attach_events(self, row):
        if not self.events:
            row.setdefault("event_type", "")
            row.setdefault("event_text", "")
            return

        events = list(self.events)
        self.events.clear()
        row["event_type"] = "|".join(item[0] for item in events)
        row["event_text"] = "|".join(item[1] for item in events)

    def _write_row(self, row):
        if self._writer is None or self._start_monotonic is None:
            return
        output = dict(row)
        sample_time = float(output.pop("_monotonic", 0.0))
        output["mission_id"] = self.mission_id
        output["elapsed_s"] = sample_time - self._start_monotonic
        output["row_index"] = self.row_count
        self._writer.writerow({
            name: output.get(name, "")
            for name in self.fieldnames
        })
        self.row_count += 1
        # Flush Python's userspace buffer every row so a process fault
        # cannot hide the seconds immediately preceding it. fsync is
        # still reserved for close to avoid a 20 Hz storage penalty.
        self._file.flush()

    def _close(self, reason, now_monotonic, ros_time):
        if self.last_row is not None:
            final_row = dict(self.last_row)
            final_row["_monotonic"] = float(now_monotonic)
            final_row["ros_time"] = ros_time or final_row.get("ros_time", "")
            final_row["event_type"] = "LOG_CLOSED"
            final_row["event_text"] = str(reason)
            final_row["end_reason"] = str(reason)
            self._write_row(final_row)

        self.last_end_reason = str(reason)
        self._metadata.update({
            "ended_ros_time": ros_time,
            "end_reason": self.last_end_reason,
            "row_count": self.row_count,
        })
        self._write_metadata()
        self._close_file()
        self.pending = False
        self.recording = False
        self.state = "SAVED"
        self._disarm_deadline = None

    def _write_metadata(self):
        if self.metadata_path is None:
            return
        temporary = self.metadata_path.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(self._metadata, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self.metadata_path)

    def _close_file(self):
        if self._file is not None:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            finally:
                self._file.close()
        self._file = None
        self._writer = None

    def _next_mission_id(self):
        largest = 0
        for path in self.log_directory.glob("mission_*.csv"):
            match = MISSION_RE.match(path.name)
            if match:
                largest = max(largest, int(match.group(1)))
        return largest + 1
