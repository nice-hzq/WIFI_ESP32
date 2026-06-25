# -*- coding: utf-8 -*-
"""core/math_utils.py 单元测试"""
import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from core.math_utils import (
    safe_float, safe_div, unwrap_deg, angdiff_deg,
    moving_average, moving_avg_1d, lowpass_1d,
    clean_boolean_runs, filter_acceleration,
)

EPS = 1e-10


class TestSafeFloat(unittest.TestCase):
    def test_normal_float(self):
        self.assertEqual(safe_float(3.14), 3.14)

    def test_int(self):
        self.assertEqual(safe_float(42), 42.0)

    def test_string_number(self):
        self.assertEqual(safe_float("2.5"), 2.5)

    def test_none_returns_default(self):
        self.assertEqual(safe_float(None), 0.0)
        self.assertEqual(safe_float(None, default=-1.0), -1.0)

    def test_nan_returns_default(self):
        self.assertEqual(safe_float(float("nan")), 0.0)

    def test_invalid_string(self):
        self.assertEqual(safe_float("abc"), 0.0)
        self.assertEqual(safe_float("abc", default=99.9), 99.9)

    def test_nan_as_float(self):
        self.assertEqual(safe_float(np.nan, default=5.0), 5.0)

    def test_list(self):
        self.assertEqual(safe_float([1, 2]), 0.0)


class TestSafeDiv(unittest.TestCase):
    def test_normal(self):
        self.assertEqual(safe_div(10.0, 2.0), 5.0)

    def test_none_numer(self):
        self.assertEqual(safe_div(None, 2.0), 0.0)

    def test_none_denom(self):
        self.assertEqual(safe_div(10.0, None), 0.0)

    def test_zero_denom(self):
        self.assertEqual(safe_div(10.0, 0.0), 0.0)

    def test_near_zero_denom(self):
        self.assertEqual(safe_div(10.0, 1e-12), 0.0)

    def test_negative_values(self):
        self.assertEqual(safe_div(-6.0, 3.0), -2.0)


class TestUnwrapDeg(unittest.TestCase):
    def test_no_wrap_needed(self):
        a = np.array([10.0, 20.0, 30.0])
        result = unwrap_deg(a)
        np.testing.assert_allclose(result, [10, 20, 30], atol=EPS)

    def test_jump_across_360(self):
        a = np.array([170.0, -170.0, -150.0])
        result = unwrap_deg(a)
        np.testing.assert_allclose(result, [170, 190, 210], atol=1e-6)

    def test_multiple_jumps(self):
        a = np.array([0, 179, -179, 179, -179, 0])
        result = unwrap_deg(a)
        self.assertAlmostEqual(result[-1], 360, delta=1)

    def test_list_input(self):
        a = [170.0, -170.0]
        result = unwrap_deg(a)
        self.assertAlmostEqual(result[1], 190, delta=1)


class TestAngdiffDeg(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(angdiff_deg(10.0, 10.0), 0.0)

    def test_simple_diff(self):
        self.assertAlmostEqual(angdiff_deg(50.0, 20.0), 30.0, places=8)

    def test_across_360(self):
        self.assertAlmostEqual(angdiff_deg(10.0, 350.0), 20.0, places=8)

    def test_negative_result(self):
        self.assertAlmostEqual(angdiff_deg(350.0, 10.0), -20.0, places=8)

    def test_scalar(self):
        result = angdiff_deg(180.0, -180.0)
        self.assertAlmostEqual(abs(result), 0.0, places=8)


class TestMovingAverage(unittest.TestCase):
    def test_1d_smoothing(self):
        x = np.array([1.0, 3.0, 5.0, 7.0, 9.0])
        result = moving_average(x, window_size=3)
        self.assertEqual(len(result), len(x))

    def test_2d_preserves_shape(self):
        x = np.random.randn(100, 3)
        result = moving_average(x, window_size=5)
        self.assertEqual(result.shape, x.shape)

    def test_window_1_returns_copy(self):
        x = np.array([1.0, 2.0, 3.0])
        result = moving_average(x, window_size=1)
        np.testing.assert_allclose(result.flatten(), x, atol=EPS)

    def test_constant_signal(self):
        x = np.ones((20, 2))
        result = moving_average(x, window_size=5)
        np.testing.assert_allclose(result, 1.0, atol=EPS)

    def test_window_larger_than_signal(self):
        x = np.array([1.0, 2.0, 3.0])
        result = moving_average(x, window_size=10)
        self.assertEqual(len(result), 3)

    def test_empty_signal(self):
        x = np.ones((0, 3))
        result = moving_average(x, window_size=5)
        self.assertEqual(len(result), 0)

    def test_reduces_noise(self):
        np.random.seed(42)
        t = np.linspace(0, 1, 200)
        clean = np.sin(2 * np.pi * 3 * t)
        noisy = clean + 0.5 * np.random.randn(200)
        smoothed = moving_average(noisy, window_size=10).flatten()
        self.assertLess(np.std(smoothed), np.std(noisy))


class TestMovingAvg1d(unittest.TestCase):
    def test_basic(self):
        x = np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0])
        result = moving_avg_1d(x, k=3)
        self.assertEqual(len(result), len(x))

    def test_constant_signal(self):
        x = np.ones(50)
        result = moving_avg_1d(x, k=5)
        np.testing.assert_allclose(result[2:-2], 1.0, atol=EPS)

    def test_k_equal_1(self):
        x = np.array([1.0, 2.0, 3.0])
        result = moving_avg_1d(x, k=1)
        np.testing.assert_allclose(result, x, atol=EPS)


class TestLowpass1d(unittest.TestCase):
    def test_no_error(self):
        fs, fc = 50.0, 8.0
        t = np.linspace(0, 2, 200)
        x = np.sin(2 * np.pi * 2 * t) + 0.2 * np.sin(2 * np.pi * 30 * t)
        result = lowpass_1d(x, fs=fs, fc=fc)
        self.assertEqual(len(result), len(x))
        self.assertTrue(np.all(np.isfinite(result)))

    def test_preserves_low_freq(self):
        fs, fc = 50.0, 20.0
        t = np.linspace(0, 1, 100)
        x = np.sin(2 * np.pi * 3 * t)
        result = lowpass_1d(x, fs=fs, fc=fc)
        self.assertGreater(np.corrcoef(x, result)[0, 1], 0.9)

    def test_attenuates_high_freq(self):
        fs, fc = 50.0, 8.0
        t = np.linspace(0, 1, 200)
        x = np.sin(2 * np.pi * 40 * t)
        result = lowpass_1d(x, fs=fs, fc=fc)
        self.assertLess(np.std(result), np.std(x) * 0.5)


class TestCleanBooleanRuns(unittest.TestCase):
    def test_no_cleaning_without_params(self):
        mask = np.array([True, False, True, False, True])
        result = clean_boolean_runs(mask, fs=50)
        np.testing.assert_array_equal(result, mask)

    def test_remove_short_true_runs(self):
        mask = np.array([False, True, False, False, True, False])
        result = clean_boolean_runs(mask, fs=50, min_true_s=0.03)
        self.assertFalse(np.any(result))

    def test_remove_short_false_runs(self):
        mask = np.array([True, False, True, True, False, True])
        result = clean_boolean_runs(mask, fs=50, min_false_s=0.03)
        self.assertTrue(np.all(result))

    def test_keep_long_runs(self):
        mask = np.zeros(100, dtype=bool)
        mask[10:90] = True
        result = clean_boolean_runs(mask, fs=50, min_true_s=0.5)
        self.assertTrue(np.any(result))

    def test_both_thresholds(self):
        mask = np.array([True] * 5 + [False] + [True] * 50 + [False] * 2 + [True] * 5)
        result = clean_boolean_runs(mask, fs=50, min_true_s=0.1, min_false_s=0.05)
        self.assertEqual(len(result), len(mask))


class TestFilterAcceleration(unittest.TestCase):
    def _generate_signal(self, N=500, fs=50.0):
        t = np.linspace(0, N / fs, N)
        x = np.sin(2 * np.pi * 2 * t)
        y = np.cos(2 * np.pi * 3 * t)
        z = np.sin(2 * np.pi * 5 * t) * 0.5
        return np.column_stack([x, y, z])

    def test_basic_lowpass(self):
        acc = self._generate_signal()
        result = filter_acceleration(acc, fs=50.0, lp_hz=10.0)
        self.assertEqual(result.shape, acc.shape)
        self.assertTrue(np.all(np.isfinite(result)))

    def test_bandpass(self):
        acc = self._generate_signal(N=600)
        result = filter_acceleration(acc, fs=50.0, lp_hz=15.0, hp_hz=0.5)
        self.assertEqual(result.shape, acc.shape)

    def test_with_median_filter(self):
        acc = self._generate_signal()
        result = filter_acceleration(acc, fs=50.0, lp_hz=10.0, median_win=5)
        self.assertEqual(result.shape, acc.shape)

    def test_detrend_linear(self):
        acc = self._generate_signal() + np.array([[1.0, 2.0, 3.0]])
        result = filter_acceleration(acc, fs=50.0, lp_hz=10.0, detrend_linear=True)
        self.assertEqual(result.shape, acc.shape)

    def test_zero_mean(self):
        acc = self._generate_signal()
        result = filter_acceleration(acc, fs=50.0, lp_hz=10.0, zero_mean=True)
        self.assertAlmostEqual(np.mean(result, axis=0).max(), 0.0, places=8)

    def test_invalid_shape_raises(self):
        with self.assertRaises(ValueError):
            filter_acceleration(np.array([1.0, 2.0, 3.0]), fs=50.0, lp_hz=10.0)

    def test_invalid_bandpass_raises(self):
        acc = self._generate_signal()
        with self.assertRaises(ValueError):
            filter_acceleration(acc, fs=50.0, lp_hz=10.0, hp_hz=20.0)


if __name__ == "__main__":
    unittest.main()
