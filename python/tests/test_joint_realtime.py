# -*- coding: utf-8 -*-
import json
import csv
import os
import queue
import tempfile
import unittest

import numpy as np

from joint.joint_models import JointAngleState, JointCalibration
from ui.threads import JointAngleThread, abs_u32_delta_ms, signed_u32_delta_ms


def _imu(marker):
    value = np.zeros(9, dtype=float)
    value[0] = marker
    return value


class JointRealtimeSyncTests(unittest.TestCase):
    def _thread(self):
        thread = JointAngleThread(
            "COM_TEST", 921600, "left_knee", queue.Queue(), ".",
            device_map={"P": "L4", "D": "L5"},
            proximal_alias="L4", distal_alias="L5")
        thread._prox_id = "P"
        thread._dist_id = "D"
        thread._device_bufs = {"P": [], "D": []}
        return thread

    def test_pairs_are_consumed_once(self):
        thread = self._thread()
        thread._device_bufs["P"] = [(_imu(1), 100), (_imu(2), 110)]
        thread._device_bufs["D"] = [(_imu(3), 102), (_imu(4), 112)]

        first = thread._get_engine_inputs()
        second = thread._get_engine_inputs()
        third = thread._get_engine_inputs()

        self.assertEqual(first[3], 2)
        self.assertEqual(second[3], 2)
        self.assertEqual(first[2], 0.0)
        self.assertEqual(second[2], 0.010)
        self.assertIsNone(third[0])
        self.assertEqual(thread._sync_matched_count, 2)
        self.assertEqual(thread._device_bufs["P"], [])
        self.assertEqual(thread._device_bufs["D"], [])

    def test_sync_tolerance_boundary(self):
        thread = self._thread()
        thread._device_bufs["P"] = [(_imu(1), 130)]
        thread._device_bufs["D"] = [(_imu(2), 100)]
        self.assertIsNotNone(thread._get_engine_inputs()[0])

        thread = self._thread()
        thread._device_bufs["P"] = [(_imu(1), 131)]
        thread._device_bufs["D"] = [(_imu(2), 100)]
        self.assertIsNone(thread._get_engine_inputs()[0])
        self.assertEqual(thread._sync_drop_dist_count, 1)

    def test_uint32_timestamp_wrap(self):
        self.assertEqual(signed_u32_delta_ms(5, 0xFFFFFFF0), 21)
        self.assertEqual(abs_u32_delta_ms(0xFFFFFFF5, 5), 16)
        thread = self._thread()
        self.assertEqual(thread._unwrap_pair_time(0xFFFFFFF0), 0.0)
        self.assertEqual(thread._unwrap_pair_time(5), 0.021)

    def test_matched_pair_keeps_corresponding_raw_rows(self):
        thread = self._thread()
        raw_p = ["P"] + [str(i) for i in range(1, 16)]
        raw_d = ["D"] + [str(i) for i in range(1, 16)]
        thread._device_bufs["P"] = [(_imu(1), 100, raw_p)]
        thread._device_bufs["D"] = [(_imu(2), 102, raw_d)]

        matched = thread._get_engine_inputs()

        self.assertEqual(matched[4], raw_p)
        self.assertEqual(matched[5], raw_d)


class JointRawDataSaveTests(unittest.TestCase):
    def test_raw_rows_are_saved_by_device(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            calib_dir = os.path.join(temp_dir, "python", "temp")
            os.makedirs(calib_dir)
            thread = JointAngleThread(
                "COM_TEST", 921600, "left_knee", queue.Queue(), calib_dir)
            thread._session_timestamp = "20260715_120000"
            row = [
                "WT/TEST", "2026-07-15 12:00:00.000", "100", "110",
                "0.1", "0.2", "1.0", "1", "2", "3",
                "4", "5", "6", "7", "8", "9",
            ]

            thread._record_raw_row(row)
            thread._close_raw_data()

            expected_dir = os.path.join(
                temp_dir, "joint_rawdate", "joint_angle_20260715_120000")
            expected_file = os.path.join(expected_dir, "WT_TEST.csv")
            self.assertEqual(thread._joint_raw_save_path, expected_dir)
            self.assertTrue(os.path.isfile(expected_file))
            with open(expected_file, newline="", encoding="utf-8") as f:
                saved = list(csv.reader(f))
            self.assertEqual(saved[0][0:4], [
                "device_id", "sensor_timestamp", "sensor_ms", "esp32_rx_ms"])
            self.assertEqual(saved[1], row)
            self.assertEqual(thread._raw_total_rows, 1)


class JointStateRegressionTests(unittest.TestCase):
    def test_first_sample_does_not_include_zero_in_rom(self):
        state = JointAngleState()
        state.update(-133.0, t=0.0)
        self.assertEqual(state.max_flexion_deg, -133.0)
        self.assertEqual(state.min_flexion_deg, -133.0)
        self.assertEqual(state.rom_deg, 0.0)

    def test_reset_removes_precalibration_history(self):
        state = JointAngleState()
        state.update(-133.0, t=0.0)
        state.reset()
        state.update(0.2, t=1.0)
        self.assertEqual(state.history_flexion, [0.2])
        self.assertEqual(state.rom_deg, 0.0)

    def test_calibration_validity_is_json_serializable_bool(self):
        calibration = JointCalibration(
            joint_name="left_knee", proximal_sensor="L4",
            distal_sensor="L5", calibration_mode="standing")
        self.assertIs(type(calibration.is_valid()), bool)
        self.assertEqual(json.loads(json.dumps({"valid": calibration.is_valid()})),
                         {"valid": True})


if __name__ == "__main__":
    unittest.main()
