# -*- coding: utf-8 -*-

"""Determines whether a signal has the desired frequency."""

import numpy as np

__author__ = "Anass Al-Wohoush"


def get_frequency_energy(signal, fs, target):
    """Gets normalized energy at the target frequency.

    Note: The DC offset is ignored.

    Args:
        signal: Signal to analyze.
        fs: Sampling frequency in Hz.
        target: Target frequency in Hz.

    Returns:
        Energy at target frequency. Ranges from 0 to 1.
    """
    # FFT signal.
    freq = np.abs(np.fft.rfft(signal))

    # Compute desired bin.
    desired_bin = int(round((target * 2 / fs) * len(freq)))

    # Get signal energy, but ignore DC offset.
    energy = freq[1:] / np.linalg.norm(freq[1:], axis=0)

    return energy[desired_bin - 1]

def get_frequency(signal, fs):
    target_frequencies = [25000, 30000, 35000, 40000]
    max_mag = 0
    frequency = 0

    for f in target_frequencies:
        mag = get_frequency_energy(signal, fs, f)
        if mag > max_mag:
            max_mag = mag
            frequency = f

    return frequency