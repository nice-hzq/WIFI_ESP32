# -*- coding: utf-8 -*-
"""统一数学/信号处理工具函数"""

import numpy as np
from scipy.signal import detrend, medfilt, filtfilt, butter


def safe_float(val, default=0.0):
    try:
        if val is None:
            return default
        if isinstance(val, float) and np.isnan(val):
            return default
        return float(val)
    except Exception:
        return default


def safe_div(numer, denom, eps=1e-8):
    try:
        if denom is None or abs(denom) < eps:
            return 0.0
        return float(numer / denom)
    except Exception:
        return 0.0


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def angdiff_deg(a, b):
    d = (np.asarray(a) - np.asarray(b) + 180.0) % 360.0 - 180.0
    return d


def moving_average(arr, window_size=5, pad_mode='edge'):
    arr = np.asarray(arr, dtype=float)
    if arr.ndim == 1:
        arr = arr[:, None]
    N, C = arr.shape
    if window_size <= 1 or N == 0:
        return arr.copy()
    k = int(window_size)
    if k > N:
        return np.tile(np.mean(arr, axis=0, keepdims=True), (N, 1))
    pad_left = k // 2
    pad_right = k - 1 - pad_left
    arr_pad = np.pad(arr, ((pad_left, pad_right), (0, 0)), mode=pad_mode)
    kernel = np.ones(k, dtype=float) / k
    out = np.empty_like(arr)
    for c in range(C):
        out[:, c] = np.convolve(arr_pad[:, c], kernel, mode='valid')
    return out


def moving_avg_1d(x, k):
    k = max(1, int(k))
    ker = np.ones(k) / k
    return np.convolve(np.asarray(x), ker, mode='same')


def lowpass_1d(x, fs, fc=8.0, order=2):
    nyq = 0.5 * fs
    wc = min(fc / nyq, 0.99)
    b, a = butter(order, wc, btype='low')
    return filtfilt(b, a, x)


def clean_boolean_runs(mask, fs, min_true_s=None, min_false_s=None):
    m = np.asarray(mask, dtype=bool).copy()
    N = len(m)
    i = 0
    while i < N:
        j = i
        val = m[i]
        while j < N and m[j] == val:
            j += 1
        length = j - i
        if val and min_true_s and length < int(round(min_true_s * fs)):
            m[i:j] = False
        if (not val) and min_false_s and length < int(round(min_false_s * fs)):
            m[i:j] = True
        i = j
    return m


def filter_acceleration(acc_nav_lin, fs=50.0, lp_hz=15.0, hp_hz=None, order=4,
                        median_win=None, detrend_linear=False, zero_mean=False):
    a = np.asarray(acc_nav_lin, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3:
        raise ValueError("acc_nav_lin must be (N,3)")
    x = a.copy()
    if detrend_linear:
        x = detrend(x, axis=0, type='linear')
    if median_win is not None and median_win >= 3 and (median_win % 2 == 1):
        x = np.column_stack([medfilt(x[:, k], kernel_size=median_win) for k in range(3)])
    nyq = 0.5 * fs
    if hp_hz is not None and hp_hz > 0.0:
        low = max(1e-6, hp_hz / nyq)
        high = min(0.999999, lp_hz / nyq)
        if not (0 < low < high < 1):
            raise ValueError("invalid bandpass freqs")
        b, a_ = butter(order, [low, high], btype='band')
    else:
        wc = min(0.999999, lp_hz / nyq)
        if not (0 < wc < 1):
            raise ValueError("invalid lowpass freq")
        b, a_ = butter(order, wc, btype='low')
    x = np.column_stack([filtfilt(b, a_, x[:, k], padtype='odd',
                                   padlen=3 * max(len(b), len(a_))) for k in range(3)])
    if zero_mean:
        x = x - np.mean(x, axis=0, keepdims=True)
    return x
