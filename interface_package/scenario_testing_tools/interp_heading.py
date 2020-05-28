import numpy as np


def interp_heading(heading: np.ndarray,
                   t_series: np.ndarray,
                   t_in: float) -> float:
    """
    Interpolate within a given heading course. (Converts to cos and sin, interpolates and converts back)
    WARNING: Relies on small-angle approximation, i.e. the heading series should be tightly spaced.

    :param heading:             heading course of original data
    :param t_series:            series of time values (matching the heading course)
    :param t_in:                time value for the heading to be generated
    :returns heading_interp:    resulting interpolated heading
    """

    # convert to x, y
    x = np.cos(heading)
    y = np.sin(heading)

    # interpolate in x, y domain
    x1 = np.interp(t_in, t_series, x)
    y1 = np.interp(t_in, t_series, y)

    # convert back to angle
    heading_interp = np.arctan2(y1, x1)

    return heading_interp
