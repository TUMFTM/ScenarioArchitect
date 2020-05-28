import numpy as np

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 27.04.2019

Documentation:  Function used to normalize psi in the range [-pi, pi]
"""


def normalize_psi(psi_in: np.ndarray) -> np.ndarray:
    """
    Return psi_in defined in the interval [-pi, pi].

    :param psi_in:      provided psi (single value or numpy array)
    :returns psi_out:   normalized psi
    """

    # use modulo operator to remove multiples of 2*pi
    psi_out = np.sign(psi_in) * np.mod(np.abs(psi_in), 2 * np.pi)

    # restrict psi_in to [-pi, pi[
    if type(psi_out) is np.ndarray:
        psi_out[psi_out >= np.pi] -= 2 * np.pi
        psi_out[psi_out < -np.pi] += 2 * np.pi

    else:
        if psi_out >= np.pi:
            psi_out -= 2 * np.pi
        elif psi_out < -np.pi:
            psi_out += 2 * np.pi

    return psi_out
