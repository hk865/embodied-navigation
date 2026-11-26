"""Layer-2 perception vision tool client, mask-depth 3D grounding, and RGB overlays.

Pure-CPU module (NumPy + Pillow + stdlib only; zero PyTorch/CUDA dependency) so it
works identically inside Python 3.9 Habitat-Sim, Python 3.12 unit tests, and ROS 2
nodes while communicating with an out-of-process local vision model service
(SAM 3, YOLOE-seg, and optional CLIP/SigLIP continuous embeddings).
"""

from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass, field
import http.client
import io
import json
import math
from pathlib import Path
import socket
import time
from typing import Any, Sequence
import urllib.parse
import zlib

import numpy as np


DEFAULT_CONTEXT_PROMPTS: tuple[str, ...] = (
    "chair",
    "couch",
    "bed",
    "table",
    "toilet",
    "tv",
    "plant",
)

MAX_MASK_PIXELS = 4096 * 4096
_MAX_RESPONSE_BYTES = 16 * 1024 * 1024
_READ_CHUNK_BYTES = 65536
_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})

# High-contrast color palette (RGB tuples) for non-target detections; target uses bright lime.
_TARGET_RGB = (34, 235, 110)
_CONTEXT_PALETTE: tuple[tuple[int, int, int], ...] = (
    (255, 159, 28),
    (56, 189, 248),
    (244, 114, 182),
    (250, 204, 21),
    (167, 139, 250),
    (251, 113, 133),
)


@dataclass(frozen=True)
class VisionDetection:
    """Single 2D open-vocabulary detection/segmentation returned by a vision tool service."""

    label: str
    score: float
    box_xyxy: tuple[float, float, float, float]
    mask: np.ndarray | None = None
    backend: str = "vision_tool"
    embedding: tuple[float, ...] | None = None


@dataclass(frozen=True)
class GroundedDetection:
    """Metric 3D object detection grounded from a pixel mask and synchronized depth image."""

    det_id: str
    label: str
    score: float
    box_xyxy: tuple[float, float, float, float]
    local_xyz: tuple[float, float, float]
    extent_xyz_m: tuple[float, float, float]
    surface_distance_m: float
    planar_depth_m: float
    bearing_deg: float
    valid_pixels: int
    range_source: str = "vision_mask_depth_backproj"
    backend: str = "vision_tool"
    mask: np.ndarray | None = None
    embedding: tuple[float, ...] | None = None
    candidate_scores: dict[str, float] = field(default_factory=dict)
    abstain: bool = False

    def to_prompt_dict(self) -> dict[str, Any]:
        """Compact metric summary safe for injecting into VLM ``[TOOL_RESULTS]``."""
        out: dict[str, Any] = {
            "id": self.det_id,
            "label": self.label,
            "score": round(float(self.score), 2),
            "surface_dist_m": round(float(self.surface_distance_m), 2),
            "bearing_deg": round(float(self.bearing_deg), 1),
            "extent_m": [round(float(v), 2) for v in self.extent_xyz_m],
            "box_px": [int(round(v)) for v in self.box_xyxy],
            "range_source": self.range_source,
            "backend": self.backend,
        }
        if len(self.candidate_scores) > 1:
            out["candidate_scores"] = {
                k: round(float(v), 3) for k, v in self.candidate_scores.items()
            }
        if self.abstain:
            out["abstain"] = True
        return out


def encode_mask_b64(mask: np.ndarray) -> dict[str, Any]:
    """Pack a boolean 2D mask ``[H, W]`` into a compact zlib+base64 JSON payload."""
    arr = np.asarray(mask, dtype=bool)
    if arr.ndim != 2 or arr.size == 0:
        raise ValueError("mask must be a non-empty 2D array")
    h, w = int(arr.shape[0]), int(arr.shape[1])
    if h * w > MAX_MASK_PIXELS:
        raise ValueError(f"mask dimensions exceed safety limit: {h}x{w}")
    packed = np.packbits(arr.ravel(), bitorder="big").tobytes()
    compressed = zlib.compress(packed, level=6)
    return {
        "h": h,
        "w": w,
        "data_b64": base64.b64encode(compressed).decode("ascii"),
    }


def decode_mask_b64(payload: dict[str, Any]) -> np.ndarray:
    """Decode a zlib+base64 packed boolean mask with strict size and trailing-data bounds."""
    if not isinstance(payload, dict):
        raise ValueError("mask payload must be a dict")
    try:
        h = int(payload["h"])
        w = int(payload["w"])
        raw_b64 = payload["data_b64"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("missing or invalid mask metadata") from exc
    if not isinstance(raw_b64, str) or h <= 0 or w <= 0 or h * w > MAX_MASK_PIXELS:
        raise ValueError(f"invalid mask shape or encoding: {h}x{w}")
    expected_bytes = (h * w + 7) // 8
    # Upper bound on valid zlib-compressed base64 size (incompressible data + zlib header overhead)
    max_b64_len = ((expected_bytes + 128) * 4) // 3 + 16
    if len(raw_b64) > max_b64_len:
        raise ValueError("encoded mask exceeds declared dimensions")
    try:
        compressed = base64.b64decode(raw_b64.encode("ascii"), validate=True)
        decomp = zlib.decompressobj()
        unpacked_bytes = decomp.decompress(compressed, expected_bytes + 1)
    except (UnicodeEncodeError, binascii.Error, zlib.error, ValueError) as exc:
        raise ValueError("corrupt compressed mask stream") from exc
    if len(unpacked_bytes) != expected_bytes or not decomp.eof or decomp.unused_data:
        raise ValueError("mask byte length mismatch or trailing compressed data")
    bits = np.unpackbits(np.frombuffer(unpacked_bytes, dtype=np.uint8), bitorder="big")
    return bits[: h * w].reshape((h, w)).astype(bool)


def build_vision_prompts(
    target_category: str,
    *,
    context_prompts: Sequence[str] = DEFAULT_CONTEXT_PROMPTS,
    max_prompts: int = 6,
) -> list[str]:
    """Build a clean, deduplicated prompt list with ``target_category`` always first."""
    out: list[str] = []
    seen: set[str] = set()
    limit = max(1, int(max_prompts))
    for raw in (target_category, *context_prompts):
        norm = " ".join(str(raw or "").strip().lower().split())
        if not norm or norm in seen:
            continue
        seen.add(norm)
        out.append(norm)
        if len(out) >= limit:
            break
    return out


def _normalize_embedding_vec(vec: Any) -> tuple[float, ...] | None:
    if not isinstance(vec, (list, tuple)) or not vec:
        return None
    try:
        arr = np.asarray(vec, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if arr.ndim != 1 or not np.all(np.isfinite(arr)):
        return None
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return None
    return tuple(float(v) for v in (arr / norm))


def _resize_mask_nearest(mask: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
    if mask.shape == (target_h, target_w):
        return mask
    from PIL import Image

    img = Image.fromarray(mask.astype(np.uint8) * 255)
    resized = img.resize((int(target_w), int(target_h)), Image.NEAREST)
    return np.asarray(resized) >= 128


def _eroded_box_mask(
    box_xyxy: Sequence[float],
    h: int,
    w: int,
    *,
    inner_fraction: float = 0.65,
) -> np.ndarray:
    """Create a central sub-box mask to limit background bleed when no pixel mask exists."""
    x0, y0, x1, y1 = (float(box_xyxy[0]), float(box_xyxy[1]), float(box_xyxy[2]), float(box_xyxy[3]))
    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    half_w = max(2.0, 0.5 * abs(x1 - x0) * float(inner_fraction))
    half_h = max(2.0, 0.5 * abs(y1 - y0) * float(inner_fraction))
    c0 = max(0, min(w - 1, int(math.floor(cx - half_w))))
    c1 = max(c0 + 1, min(w, int(math.ceil(cx + half_w))))
    r0 = max(0, min(h - 1, int(math.floor(cy - half_h))))
    r1 = max(r0 + 1, min(h, int(math.ceil(cy + half_h))))
    out = np.zeros((h, w), dtype=bool)
    out[r0:r1, c0:c1] = True
    return out


def ground_mask_with_depth(
    mask: np.ndarray | None,
    depth_m: np.ndarray,
    *,
    box_xyxy: Sequence[float] | None = None,
    hfov_deg: float = 90.0,
    camera_height_m: float = 0.88,
    min_range_m: float = 0.25,
    max_range_m: float = 10.0,
    min_valid_pixels: int = 16,
) -> dict[str, Any] | None:
    """Backproject a 2D segmentation mask (or fallback box) into robot-local 3D Z-up coordinates.

    Matches ``DepthOccupancyGrid`` pinhole camera geometry exactly:
    ``x_cam = d`` (+forward), ``y_cam = -(u - cx) * d / fx`` (+left),
    ``z_cam = -(v - cy) * d / fy + camera_height_m`` (+up above floor).

    Isolates the nearest foreground depth cluster anchored around the 25th depth
    percentile so background walls visible through thin objects (even when >50% of
    mask pixels bleed onto the background) never pull the 3D centroid off-object.
    """
    if depth_m is None or depth_m.ndim < 2 or depth_m.size == 0:
        return None
    arr = depth_m[:, :, 0] if depth_m.ndim == 3 else depth_m
    h, w = int(arr.shape[0]), int(arr.shape[1])

    if mask is not None:
        m_bool = np.asarray(mask, dtype=bool)
        if m_bool.ndim != 2 or m_bool.size == 0:
            return None
        m_bool = _resize_mask_nearest(m_bool, h, w)
        range_source = "vision_mask_depth_backproj"
    elif box_xyxy is not None and len(box_xyxy) == 4:
        m_bool = _eroded_box_mask(box_xyxy, h, w)
        range_source = "vision_box_depth_fallback"
    else:
        return None

    valid = m_bool & np.isfinite(arr) & (arr >= float(min_range_m)) & (arr <= float(max_range_m))
    valid_count = int(np.sum(valid))
    if valid_count < int(min_valid_pixels):
        return None

    vs, us = np.nonzero(valid)
    ds = arr[vs, us].astype(np.float64)

    # Nearest supported foreground depth cluster: strictly reject distant background walls
    min_cluster_px = max(8, int(min_valid_pixels) // 2)
    d_anchor = float(np.percentile(ds, 25))
    band_lo = max(float(min_range_m), d_anchor - 0.25)
    band_hi = min(float(max_range_m), d_anchor + max(0.30, 0.18 * d_anchor))
    fg = (ds >= band_lo) & (ds <= band_hi)
    if int(np.sum(fg)) < min_cluster_px:
        # Gently widen once for large sloped objects (e.g. long beds viewed end-on)
        band_hi_wide = min(float(max_range_m), d_anchor + max(0.65, 0.30 * d_anchor))
        fg = (ds >= band_lo) & (ds <= band_hi_wide)
    if int(np.sum(fg)) < min_cluster_px:
        return None

    vs = vs[fg]
    us = us[fg]
    ds = ds[fg]

    fx = (0.5 * w) / math.tan(math.radians(float(hfov_deg)) * 0.5)
    fy = fx
    cx = 0.5 * w
    cy = 0.5 * h

    x_cam = ds
    y_cam = -(us.astype(np.float64) - cx) * ds / fx
    z_cam = -(vs.astype(np.float64) - cy) * ds / fy + float(camera_height_m)

    lx = float(np.median(x_cam))
    ly = float(np.median(y_cam))
    lz = float(np.median(z_cam))
    horiz_dists = np.hypot(x_cam, y_cam)
    surface_dist = float(np.percentile(horiz_dists, 20))
    bearing_deg = math.degrees(math.atan2(ly, max(1e-3, lx)))

    dx = max(0.1, min(3.5, float(np.percentile(x_cam, 90) - np.percentile(x_cam, 10))))
    dy = max(0.1, min(3.5, float(np.percentile(y_cam, 90) - np.percentile(y_cam, 10))))
    dz = max(0.1, min(2.8, float(np.percentile(z_cam, 90) - np.percentile(z_cam, 10))))

    return {
        "local_xyz": (round(lx, 3), round(ly, 3), round(lz, 3)),
        "extent_xyz_m": (round(dx, 2), round(dy, 2), round(dz, 2)),
        "surface_distance_m": round(surface_dist, 3),
        "planar_depth_m": round(float(np.median(ds)), 3),
        "bearing_deg": round(bearing_deg, 1),
        "valid_pixels": int(ds.size),
        "range_source": range_source,
    }


def _box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    ix0 = max(float(a[0]), float(b[0]))
    iy0 = max(float(a[1]), float(b[1]))
    ix1 = min(float(a[2]), float(b[2]))
    iy1 = min(float(a[3]), float(b[3]))
    iw = max(0.0, ix1 - ix0)
    ih = max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, float(a[2]) - float(a[0])) * max(0.0, float(a[3]) - float(a[1]))
    area_b = max(0.0, float(b[2]) - float(b[0])) * max(0.0, float(b[3]) - float(b[1]))
    union = area_a + area_b - inter
    return float(inter / union) if union > 1e-9 else 0.0


def _detection_iou(a: VisionDetection, b: VisionDetection) -> float:
    if (
        a.mask is not None
        and b.mask is not None
        and a.mask.shape == b.mask.shape
    ):
        inter = int(np.sum(a.mask & b.mask))
        if inter == 0:
            return 0.0
        union = int(np.sum(a.mask | b.mask))
        return float(inter / union) if union > 0 else 0.0
    return _box_iou(a.box_xyxy, b.box_xyxy)


def suppress_overlapping_detections(
    detections: Sequence[VisionDetection],
    *,
    same_label_iou: float = 0.50,
    cross_label_iou: float = 0.65,
    cross_label_tie_ratio: float = 0.85,
) -> list[VisionDetection]:
    """Suppress duplicate same-label and weaker cross-label detections on the same object.

    Open-vocabulary segmenters queried per prompt (e.g. SAM 3) often score the same
    physical furniture strongly as its true class (``couch=0.85``) and weakly as a
    neighboring class (``chair=0.38``). Suppressing weaker overlapping cross-label
    masks prevents false-positive 3D memory pollution.
    """
    ordered = sorted(detections, key=lambda d: -float(d.score))
    kept: list[VisionDetection] = []
    for cand in ordered:
        c_lbl = cand.label.strip().lower()
        suppressed = False
        for prev in kept:
            iou = _detection_iou(cand, prev)
            p_lbl = prev.label.strip().lower()
            if c_lbl == p_lbl and iou >= float(same_label_iou):
                suppressed = True
                break
            if (
                c_lbl != p_lbl
                and iou >= float(cross_label_iou)
                and float(cand.score) < float(prev.score) * float(cross_label_tie_ratio)
            ):
                suppressed = True
                break
        if not suppressed:
            kept.append(cand)
    return kept


def cluster_multilabel_detections(
    detections: Sequence[VisionDetection],
    *,
    mask_iou_threshold: float = 0.65,
    abstain_threshold: float = 0.25,
) -> list[tuple[VisionDetection, dict[str, float], bool]]:
    """Cluster overlapping prompts onto physical object regions with per-label scores.

    Preserves multi-hypothesis evidence (e.g. ``{"desk": 0.82, "table": 0.65}``) for
    contrastive disambiguation and flags ``abstain=True`` when the highest-scoring
    prompt falls below ``abstain_threshold``.
    """
    ordered = sorted(detections, key=lambda d: -float(d.score))
    clusters: list[tuple[VisionDetection, dict[str, float]]] = []
    for cand in ordered:
        c_lbl = " ".join(cand.label.strip().lower().split())
        if not c_lbl:
            continue
        sc = float(cand.score)
        matched_cluster: tuple[VisionDetection, dict[str, float]] | None = None
        for rep, scores_map in clusters:
            if _detection_iou(cand, rep) >= float(mask_iou_threshold):
                matched_cluster = (rep, scores_map)
                break
        if matched_cluster is None:
            clusters.append((cand, {c_lbl: round(sc, 3)}))
        else:
            _, scores_map = matched_cluster
            prev_sc = scores_map.get(c_lbl)
            if prev_sc is None or sc > prev_sc:
                scores_map[c_lbl] = round(sc, 3)

    out: list[tuple[VisionDetection, dict[str, float], bool]] = []
    thr = float(abstain_threshold)
    for rep, scores_map in clusters:
        sorted_scores = dict(sorted(scores_map.items(), key=lambda kv: -kv[1]))
        abstain = float(rep.score) < thr
        out.append((rep, sorted_scores, abstain))
    return out


def ground_detections_with_depth(
    detections: Sequence[VisionDetection],
    depth_m: np.ndarray,
    *,
    hfov_deg: float = 90.0,
    camera_height_m: float = 0.88,
    min_range_m: float = 0.25,
    max_range_m: float = 10.0,
    min_valid_pixels: int = 16,
    apply_nms: bool = True,
    multilabel_cluster: bool = False,
    abstain_threshold: float = 0.25,
) -> list[GroundedDetection]:
    """Ground a sequence of 2D vision detections into metric 3D ``GroundedDetection`` items."""
    if multilabel_cluster:
        clustered = cluster_multilabel_detections(
            detections, abstain_threshold=abstain_threshold
        )
    else:
        dets_use = (
            suppress_overlapping_detections(detections) if apply_nms else list(detections)
        )
        clustered = [
            (d, {d.label.strip().lower(): round(float(d.score), 3)}, False)
            for d in dets_use
        ]

    out: list[GroundedDetection] = []
    for det, cand_scores, abstain in clustered:
        g = ground_mask_with_depth(
            det.mask,
            depth_m,
            box_xyxy=det.box_xyxy,
            hfov_deg=hfov_deg,
            camera_height_m=camera_height_m,
            min_range_m=min_range_m,
            max_range_m=max_range_m,
            min_valid_pixels=min_valid_pixels,
        )
        if g is None:
            continue
        out.append(
            GroundedDetection(
                det_id=f"V{len(out) + 1}",
                label=det.label,
                score=float(det.score),
                box_xyxy=det.box_xyxy,
                local_xyz=g["local_xyz"],
                extent_xyz_m=g["extent_xyz_m"],
                surface_distance_m=g["surface_distance_m"],
                planar_depth_m=g["planar_depth_m"],
                bearing_deg=g["bearing_deg"],
                valid_pixels=g["valid_pixels"],
                range_source=g["range_source"],
                backend=det.backend,
                mask=det.mask,
                embedding=det.embedding,
                candidate_scores=cand_scores,
                abstain=abstain,
            )
        )
    return out


def render_visual_enhancement_overlay(
    rgb: np.ndarray,
    detections: Sequence[GroundedDetection],
    out_path: str | Path,
    *,
    target_label: str | None = None,
    mask_alpha: float = 0.28,
) -> Path:
    """Render semi-transparent segmentation masks, boxes, and metric labels onto a copy of RGB.

    Never modifies the input array or raw RGB archive; writes to ``out_path``.
    """
    from PIL import Image, ImageDraw

    arr = np.asarray(rgb)
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    canvas = arr.astype(np.float32).copy()
    h, w = int(canvas.shape[0]), int(canvas.shape[1])
    norm_target = " ".join(str(target_label or "").strip().lower().split())
    alpha = max(0.05, min(0.65, float(mask_alpha)))

    colors: list[tuple[int, int, int]] = []
    for idx, det in enumerate(detections):
        norm_lbl = " ".join(det.label.strip().lower().split())
        is_target = bool(norm_target and (norm_lbl == norm_target or norm_target in norm_lbl))
        color = _TARGET_RGB if is_target else _CONTEXT_PALETTE[idx % len(_CONTEXT_PALETTE)]
        colors.append(color)
        if det.mask is not None:
            m = _resize_mask_nearest(np.asarray(det.mask, dtype=bool), h, w)
            if np.any(m):
                col_vec = np.asarray(color, dtype=np.float32)
                canvas[m] = (1.0 - alpha) * canvas[m] + alpha * col_vec

    img = Image.fromarray(np.clip(canvas, 0, 255).astype(np.uint8), mode="RGB")
    draw = ImageDraw.Draw(img)

    for det, color in zip(detections, colors):
        x0 = max(0, min(w - 1, int(round(det.box_xyxy[0]))))
        y0 = max(0, min(h - 1, int(round(det.box_xyxy[1]))))
        x1 = max(x0 + 1, min(w - 1, int(round(det.box_xyxy[2]))))
        y1 = max(y0 + 1, min(h - 1, int(round(det.box_xyxy[3]))))
        draw.rectangle([x0, y0, x1, y1], outline=color, width=2)
        tag = f"[{det.det_id}] {det.label} {det.surface_distance_m:.1f}m ({det.score:.2f})"
        pill_w = min(w - x0, len(tag) * 6 + 6)
        pill_h = 14
        ty = max(0, y0 - pill_h) if y0 >= pill_h else min(h - pill_h, y0 + 2)
        draw.rectangle([x0, ty, min(w - 1, x0 + pill_w), min(h - 1, ty + pill_h)], fill=color)
        draw.text((x0 + 3, ty + 2), tag, fill=(12, 16, 20))

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    return out


def _encode_rgb_to_png_b64(rgb: np.ndarray | str | Path) -> str:
    from PIL import Image

    if isinstance(rgb, (str, Path)):
        data = Path(rgb).read_bytes()
        return base64.b64encode(data).decode("ascii")
    arr = np.asarray(rgb)
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    buf = io.BytesIO()
    Image.fromarray(arr.astype(np.uint8), mode="RGB").save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


class VisionToolClient:
    """Pure-stdlib HTTP client for the local vision model tool service (SAM 3 / YOLOE / CLIP).

    Enforces loopback-only endpoints by default (SSRF guard), total request deadlines,
    bounded response sizes, and fail-open behavior on transient errors so navigation
    never crashes if the optional vision service drops.
    """

    def __init__(
        self,
        base_url: str,
        *,
        timeout_s: float = 8.0,
        allow_remote: bool = False,
        max_response_bytes: int = _MAX_RESPONSE_BYTES,
    ) -> None:
        parsed = urllib.parse.urlparse(str(base_url).strip())
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise ValueError(f"invalid vision tool endpoint URL: {base_url!r}")
        if not allow_remote and parsed.hostname.lower() not in _LOOPBACK_HOSTS:
            raise ValueError(
                f"refusing non-loopback vision endpoint {parsed.hostname!r} without allow_remote=True"
            )
        self.base_url = str(base_url).rstrip("/")
        self._parsed = parsed
        self.timeout_s = max(0.5, float(timeout_s))
        self.max_response_bytes = max(4096, int(max_response_bytes))
        self.last_error: str | None = None
        self.backend_name: str = "vision_tool"
        self.embed_model: str = ""

    def _read_bounded_response(self, resp: http.client.HTTPResponse, conn: Any, deadline: float) -> bytes:
        chunks: list[bytes] = []
        total = 0
        while True:
            rem = deadline - time.monotonic()
            if rem <= 0.0:
                raise TimeoutError("vision_tool_deadline_exceeded")
            if conn.sock is not None:
                conn.sock.settimeout(rem)
            chunk = resp.read(_READ_CHUNK_BYTES)
            if not chunk:
                break
            total += len(chunk)
            if total > self.max_response_bytes:
                raise RuntimeError("vision_response_too_large")
            chunks.append(chunk)
        return b"".join(chunks)

    def _request_json(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        req_path = f"{self._parsed.path.rstrip('/')}{path}"
        conn_cls = (
            http.client.HTTPSConnection
            if self._parsed.scheme == "https"
            else http.client.HTTPConnection
        )
        deadline = time.monotonic() + self.timeout_s
        conn = conn_cls(self._parsed.hostname, self._parsed.port, timeout=self.timeout_s)
        body = (
            json.dumps(payload, allow_nan=False).encode("utf-8")
            if payload is not None
            else None
        )
        headers = {"Accept": "application/json"}
        if body is not None:
            headers["Content-Type"] = "application/json"
        try:
            rem = deadline - time.monotonic()
            if rem <= 0.0:
                raise TimeoutError("vision_tool_deadline_exceeded")
            conn.timeout = rem
            conn.request(method, req_path, body=body, headers=headers)
            if conn.sock is not None:
                rem = deadline - time.monotonic()
                if rem <= 0.0:
                    raise TimeoutError("vision_tool_deadline_exceeded")
                conn.sock.settimeout(rem)
            resp = conn.getresponse()
            raw_bytes = self._read_bounded_response(resp, conn, deadline)
            if resp.status >= 400:
                raise RuntimeError(f"vision_http_{resp.status}")
            parsed = json.loads(raw_bytes.decode("utf-8", errors="replace"))
            if not isinstance(parsed, dict):
                raise RuntimeError("vision_response_not_object")
            self.last_error = None
            return parsed
        finally:
            conn.close()

    def health(self) -> dict[str, Any]:
        """Query ``GET /health`` and cache advertised backend/embedding capabilities."""
        try:
            res = self._request_json("GET", "/health")
            self.backend_name = str(res.get("backend") or self.backend_name)
            self.embed_model = str(res.get("embed_model") or "")
            return res
        except (
            OSError,
            TimeoutError,
            socket.timeout,
            http.client.HTTPException,
            RuntimeError,
            ValueError,
        ) as exc:
            self.last_error = str(exc)
            return {"ok": False, "error": str(exc)}

    def segment_text(
        self,
        rgb: np.ndarray | str | Path,
        prompts: Sequence[str],
        *,
        score_threshold: float = 0.25,
        max_detections: int = 12,
        include_masks: bool = True,
        include_crop_embeddings: bool = False,
    ) -> list[VisionDetection]:
        """Run open-vocabulary text-prompted segmentation on an RGB image."""
        clean_prompts = [
            " ".join(str(p).strip().lower().split())
            for p in prompts
            if str(p or "").strip()
        ]
        if not clean_prompts:
            return []
        try:
            payload = {
                "image_b64": _encode_rgb_to_png_b64(rgb),
                "prompts": clean_prompts[:16],
                "score_threshold": float(score_threshold),
                "max_detections": max(1, min(32, int(max_detections))),
                "include_masks": bool(include_masks),
                "include_crop_embeddings": bool(include_crop_embeddings),
            }
            resp = self._request_json("POST", "/segment_text", payload)
        except (
            OSError,
            TimeoutError,
            socket.timeout,
            http.client.HTTPException,
            RuntimeError,
            ValueError,
        ) as exc:
            self.last_error = str(exc)
            return []

        backend = str(resp.get("backend") or self.backend_name)
        raw_dets = resp.get("detections")
        if not isinstance(raw_dets, list):
            return []

        out: list[VisionDetection] = []
        for item in raw_dets[: max(1, int(max_detections))]:
            if not isinstance(item, dict):
                continue
            label = str(item.get("label", "")).strip()
            box = item.get("box_xyxy")
            if not label or not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            try:
                score = float(item.get("score", 0.0))
                b_tuple = tuple(float(v) for v in box)
                if not math.isfinite(score) or not all(math.isfinite(v) for v in b_tuple):
                    continue
            except (TypeError, ValueError):
                continue
            mask_arr: np.ndarray | None = None
            if "mask" in item and item["mask"] is not None:
                try:
                    mask_arr = decode_mask_b64(item["mask"])
                except ValueError:
                    # Skip detections with corrupt mask payloads instead of silently downgrading to box fallback
                    continue
            emb = _normalize_embedding_vec(item.get("embedding"))
            out.append(
                VisionDetection(
                    label=label,
                    score=score,
                    box_xyxy=b_tuple,  # type: ignore[arg-type]
                    mask=mask_arr,
                    backend=backend,
                    embedding=emb,
                )
            )
        return out

    def embed_image(self, rgb: np.ndarray | str | Path) -> tuple[float, ...] | None:
        """Encode an RGB image into an L2-normalized continuous vision-language embedding."""
        try:
            resp = self._request_json(
                "POST", "/embed_image", {"image_b64": _encode_rgb_to_png_b64(rgb)}
            )
        except (
            OSError,
            TimeoutError,
            socket.timeout,
            http.client.HTTPException,
            RuntimeError,
            ValueError,
        ) as exc:
            self.last_error = str(exc)
            return None
        return _normalize_embedding_vec(resp.get("embedding"))

    def embed_text(self, text: str) -> tuple[float, ...] | None:
        """Encode a text query into an L2-normalized continuous vision-language embedding."""
        clean = " ".join(str(text or "").strip().split())
        if not clean:
            return None
        try:
            resp = self._request_json("POST", "/embed_text", {"text": clean})
        except (
            OSError,
            TimeoutError,
            socket.timeout,
            http.client.HTTPException,
            RuntimeError,
            ValueError,
        ) as exc:
            self.last_error = str(exc)
            return None
        return _normalize_embedding_vec(resp.get("embedding"))
