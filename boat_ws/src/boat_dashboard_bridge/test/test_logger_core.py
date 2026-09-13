import csv
import json
import tempfile
import unittest
from pathlib import Path

from boat_dashboard_bridge.logger_core import MissionLogCore, clean_label


FIELDS = [
    "mission_id", "ros_time", "elapsed_s", "row_index",
    "event_type", "event_text", "end_reason", "value",
]


def make_core(tmp_path):
    return MissionLogCore(
        tmp_path,
        FIELDS,
        prearm_seconds=5.0,
        disarm_grace_seconds=5.0,
    )


def read_rows(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


class MissionLogCoreTest(unittest.TestCase):

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.tmp_path = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def test_pending_session_does_not_allocate_file_or_number(self):
        core = make_core(self.tmp_path)
        status = core.reset("pre calibration!", now_monotonic=1.0)
        core.add_snapshot({"value": 1}, 2.0, "2.0")

        self.assertEqual(status["state"], "PENDING")
        self.assertIsNone(status["mission_id"])
        self.assertFalse(list(self.tmp_path.glob("mission_*.csv")))
        self.assertEqual(clean_label("pre calibration!"), "pre_calibration")

    def test_arm_writes_only_last_five_seconds_of_prearm_data(self):
        core = make_core(self.tmp_path)
        core.reset("precal", now_monotonic=0.0)
        core.observe_armed(False, 0.0)

        for second in range(8):
            core.add_snapshot({"value": second}, float(second), str(second))

        core.observe_armed(True, 8.0, "8.0")
        core.add_snapshot({"value": 8}, 8.0, "8.0")
        status = core.status()
        rows = read_rows(status["file_path"])

        self.assertEqual(status["mission_id"], 1)
        self.assertTrue(status["recording"])
        self.assertEqual([int(row["value"]) for row in rows], [3, 4, 5, 6, 7, 8])
        self.assertEqual(float(rows[0]["elapsed_s"]), -5.0)
        self.assertIn("ARMED", rows[-1]["event_type"])
        core.close("TEST_COMPLETE", 9.0, "9.0")

    def test_disarm_grace_and_rearm_continue_same_file(self):
        core = make_core(self.tmp_path)
        core.reset("water", now_monotonic=0.0)
        core.observe_armed(False, 0.0)
        core.observe_armed(True, 1.0, "1.0")
        core.add_snapshot({"value": 1}, 1.0, "1.0")
        original_path = core.status()["file_path"]

        core.observe_armed(False, 2.0, "2.0")
        core.add_snapshot({"value": 2}, 6.9, "6.9")
        self.assertEqual(core.status()["state"], "CLOSING")

        core.observe_armed(True, 6.95, "6.95")
        core.add_snapshot({"value": 3}, 7.0, "7.0")
        self.assertEqual(core.status()["state"], "RECORDING")
        self.assertEqual(core.status()["file_path"], original_path)

        core.observe_armed(False, 8.0, "8.0")
        core.add_snapshot({"value": 4}, 13.0, "13.0")
        status = core.status()
        self.assertEqual(status["state"], "SAVED")
        self.assertEqual(status["last_end_reason"], "DISARMED_5S")

        with open(status["metadata_path"], encoding="utf-8") as stream:
            metadata = json.load(stream)
        self.assertEqual(metadata["end_reason"], "DISARMED_5S")

    def test_reset_closes_active_file_and_new_pending_uses_next_number(self):
        core = make_core(self.tmp_path)
        core.reset("first", now_monotonic=0.0)
        core.observe_armed(False, 0.0)
        core.observe_armed(True, 1.0, "1.0")
        core.add_snapshot({"value": 1}, 1.0, "1.0")
        first_path = core.status()["file_path"]

        status = core.reset("second", now_monotonic=2.0, ros_time="2.0")
        self.assertEqual(status["state"], "PENDING")
        self.assertIsNone(status["mission_id"])
        self.assertEqual(read_rows(first_path)[-1]["end_reason"], "RESET_MISSION")

        core.observe_armed(False, 3.0, "3.0")
        core.observe_armed(True, 4.0, "4.0")
        core.add_snapshot({"value": 2}, 4.0, "4.0")
        self.assertEqual(core.status()["mission_id"], 2)
        core.close("TEST_COMPLETE", 5.0, "5.0")


if __name__ == "__main__":
    unittest.main()
