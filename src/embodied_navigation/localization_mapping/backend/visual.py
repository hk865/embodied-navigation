"""Measured LiDAR-backed image patch correspondences; no monocular metric claims.

Pinhole images must already be rectified. Camera coordinates are optical
(x right, y down, z forward). SE3 results map source body to target body.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree
from .pose_graph import invert


@dataclass(frozen=True)
class VisualObservation:
    gray: np.ndarray
    K: np.ndarray
    T_body_camera: np.ndarray
    stamp_ns: int

    def __post_init__(self):
        # These arrays arrive from ROS/files, not trusted internal computation.
        if self.gray.ndim != 2 or not np.isfinite(self.gray).all():
            raise ValueError('visual image must be finite rectified grayscale')
        if self.K.shape != (3, 3) or not np.isfinite(self.K).all() or self.K[0, 0] <= 0 or self.K[1, 1] <= 0:
            raise ValueError('invalid visual camera intrinsics')
        T = self.T_body_camera
        if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]) or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5) or np.linalg.det(T[:3, :3]) < 0:
            raise ValueError('invalid visual camera extrinsics')


@dataclass(frozen=True)
class VisualResult:
    status: str
    T_target_source: np.ndarray | None = None
    matches: int = 0
    inliers: int = 0
    rmse_m: float = float('inf')


def _features(keyframe, limit=400):
    from .keyframes import transform_points
    observation = keyframe.visual
    points = keyframe.points_body()
    camera = transform_points(invert(observation.T_body_camera), points)
    front = camera[:, 2] > .1
    camera, points = camera[front], points[front]
    projected = camera @ observation.K.T
    pixels = np.rint(projected[:, :2] / projected[:, 2:]).astype(int)
    h, w = observation.gray.shape
    valid = (pixels[:, 0] >= 3) & (pixels[:, 0] < w-3) & (pixels[:, 1] >= 3) & (pixels[:, 1] < h-3)
    pixels, points, camera = pixels[valid], points[valid], camera[valid]
    # Retain the nearest measured return per image cell to suppress occlusions
    # and correlated samples; each descriptor remains tied to one actual return.
    order = np.argsort(camera[:, 2], kind='stable')
    _, first = np.unique(pixels[order] // 7, axis=0, return_index=True)
    selected = order[first]
    patches = np.array([observation.gray[y-3:y+4, x-3:x+4].ravel() for x, y in pixels[selected]], dtype=float).reshape(-1, 49)
    patches -= patches.mean(axis=1, keepdims=True)
    norm = np.linalg.norm(patches, axis=1)
    textured = norm > 10.
    selected, patches, norm = selected[textured], patches[textured], norm[textured]
    strongest = np.argsort(-norm, kind='stable')[:limit]
    return points[selected[strongest]], patches[strongest] / norm[strongest, None]


def _rigid(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, b - rotation @ a
    return T


def verify_visual(source, target, *, max_time_delta_s=.05, inlier_m=.12, min_inliers=8):
    """Mutual ratio-test image matches plus metric RANSAC on measured 3D.

    Appearance alone never produces a pose. Missing/time-skewed data,
    untextured scenes, collinear support and inconsistent geometry reject.
    """
    from .keyframes import transform_points
    if source.visual is None or target.visual is None:
        return VisualResult('missing_observation')
    if any(abs(k.visual.stamp_ns-k.stamp_ns) > max_time_delta_s*1e9 for k in (source, target)):
        return VisualResult('time_mismatch')
    a, da = _features(source)
    b, db = _features(target)
    if min(len(a), len(b)) < min_inliers:
        return VisualResult('insufficient_texture')
    distances, indices = cKDTree(db).query(da, k=2)
    reverse = cKDTree(da).query(db, k=1)[1]
    valid = (distances[:, 0] < .7*distances[:, 1]) & (distances[:, 0] < .7) & (reverse[indices[:, 0]] == np.arange(len(a)))
    a, b = a[valid], b[indices[valid, 0]]
    if len(a) < min_inliers:
        return VisualResult('insufficient_matches', matches=len(a))
    rng = np.random.default_rng(0)
    best = np.zeros(len(a), dtype=bool)
    for _ in range(128):
        indices = rng.choice(len(a), 3, replace=False)
        if np.linalg.svd(a[indices]-a[indices].mean(axis=0), compute_uv=False)[1] < .02:
            continue
        T = _rigid(a[indices], b[indices])
        support = np.linalg.norm(transform_points(T, a)-b, axis=1) < inlier_m
        if support.sum() > best.sum():
            best = support
    if best.sum() < min_inliers or best.mean() < .5:
        return VisualResult('geometry_rejected', matches=len(a), inliers=int(best.sum()))
    if min(np.linalg.svd(p[best]-p[best].mean(axis=0), compute_uv=False)[1] for p in (a, b)) < .1:
        return VisualResult('degenerate_geometry', matches=len(a), inliers=int(best.sum()))
    T = _rigid(a[best], b[best])
    residual = np.linalg.norm(transform_points(T, a[best])-b[best], axis=1)
    return VisualResult('verified', T, len(a), int(best.sum()), float(np.sqrt(np.mean(residual**2))))


def decode_image(message):
    """Decode ROS mono8/rgb8/bgr8 including padded rows, without cv_bridge."""
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(message.encoding)
    if channels is None:
        raise ValueError('visual input requires rectified mono8/rgb8/bgr8')
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.step < message.width*channels or len(data) != message.height*message.step:
        raise ValueError('invalid visual image row stride or data length')
    pixels = data.reshape(message.height, message.step)[:, :message.width*channels]
    if channels == 1:
        return pixels.copy()
    rgb = pixels.reshape(message.height, message.width, 3)
    if message.encoding == 'bgr8':
        rgb = rgb[:, :, ::-1]
    return rgb @ np.array([.299, .587, .114])
