"""Offline trajectory metrics; simulator truth never enters control."""

import numpy as np


def path_length_xy(positions):
    """Total planar path length of an ordered position sequence.

    ``positions`` is any array-like of shape ``(N, >=2)``; only columns ``0:2``
    (x, y) are used, so height is ignored.  Consecutive-sample Euclidean
    distances are summed and returned as ``float``; the value is in the same
    length unit as the input (metres in this repository).  Fewer than two
    samples produce an empty ``np.diff``, whose sum is ``0.0``.
    """
    return float(np.linalg.norm(np.diff(np.asarray(positions)[:, :2], axis=0), axis=1).sum())


def position_error_xy(estimated, reference):
    """Per-sample planar error statistics between two aligned position sequences.

    ``estimated`` and ``reference`` must have the same number of samples (they are
    subtracted element-wise after ``np.asarray``); only x and y (columns ``0:2``)
    participate.  The returned dict is in the input length unit (metres):

    * ``rmse_m`` -- root mean square of the per-sample distances;
    * ``max_m`` -- largest per-sample distance;
    * ``final_m`` -- distance at the last sample.

    For correctly shaped two-dimensional empty arrays, ``max`` on the empty
    error array raises ``ValueError``.
    """
    errors = np.linalg.norm(np.asarray(estimated)[:, :2] - np.asarray(reference)[:, :2], axis=1)
    return {
        "rmse_m": float(np.sqrt(np.mean(errors**2))),
        "max_m": float(errors.max()),
        "final_m": float(errors[-1]),
    }
