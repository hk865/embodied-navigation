"""Offline trajectory metrics; simulator truth never enters control."""
import numpy as np

def path_length_xy(positions):
    return float(np.linalg.norm(np.diff(np.asarray(positions)[:, :2], axis=0), axis=1).sum())

def position_error_xy(estimated, reference):
    errors = np.linalg.norm(np.asarray(estimated)[:, :2] - np.asarray(reference)[:, :2], axis=1)
    return {"rmse_m": float(np.sqrt(np.mean(errors**2))), "max_m": float(errors.max()), "final_m": float(errors[-1])}
