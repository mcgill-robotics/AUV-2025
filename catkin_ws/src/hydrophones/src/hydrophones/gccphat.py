# -*- coding: utf-8 -*-

"""Through Generalized Cross Correlation with Phase Transform (GCC-PHAT),
solves for time differences of arrival given two signals.
"""

import numpy as np

__author__ = "Anass Al-Wohoush"


def estimate(base, sample, fs, upscale=0.001):
    """Computes time difference of arrival of sample signal with respect to
    base signal using GCC-PHAT.

    Args:
        base: Base reference signal.
        sample: Sample signal.
        fs: Sampling frequency in Hz.
        upscale: Upsampling of GCC-PHAT for interpolation (default: 0.001).

    Returns:
        Time difference of arrival.
    """
    # FFT both signals.
    base_freq = np.fft.fft(base)
    sample_freq = np.fft.fft(sample)

    # With ordinary GCC, given two signals xi and xj: GCC = Xi[Xj]*
    #   where Xi and Xj are the FFT of xi and xj respectively and
    #   [ ]* is the complex conjugate.
    # GCC-PHAT simply normalizes GCC, so GCC-PHAT = GCC / |GCC|.
    gcc = np.multiply(sample_freq, np.conj(base_freq))
    gccphat = np.divide(gcc, np.absolute(gcc))

    # The TDOA is then defined as: TDOA = argmax(IFFT(GCC-PHAT))
    dt_signal = np.fft.ifft(gcc)
    t = np.arange(len(base)) / fs
    max_td = 1e-5
    dt_signal = [0 if time > max_td and time < t[-1] - max_td else s
                for time, s in zip(t, dt_signal)]
    argmax = np.argmax(dt_signal)

    # Interpolate for slightly better accuracy.
    # Only interpolate around the peak because this process is slow.
    # begin, end = (max(0, argmax - 5), min(argmax + 5, len(dt_signal)))
    # s = np.arange(begin, end)
    # u = np.arange(begin, end, upscale)
    # upsampled_dt_signal = upsample(dt_signal[begin:end], s, u)
    # argmax = begin + np.argmax(upsampled_dt_signal) * upscale

    # Convert to seconds.
    dt = float(argmax) / fs

    # If the peak is in the second half of the buffer, the time difference
    # should actually be negative. So compensate for that.
    buffersize = len(base) / fs
    if dt > buffersize / 2:
        dt -= buffersize

    return dt


def upsample(x, s, u):
    """Upsamples signal x, sampled at s to u by interpolating with a sinc
    function.

    Args:
        x: Signal to upsample.
        s: Sampling of original signal.
        u: Sampling of output.

    Returns:
        Upsampled signal.
    """
    period = s[1] - s[0]
    sinc_matrix = (
        np.tile(u, (len(s), 1)) -
        np.tile(s[:, np.newaxis], (1, len(u)))
    )
    y = np.dot(x, np.sinc(sinc_matrix / period))

    return y
