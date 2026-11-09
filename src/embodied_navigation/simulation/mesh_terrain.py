"""Experiment-only exact vertical rays against the imported Building mesh.

Used solely by the existing privileged locomotion observation/safety reader.
It is not a localization reference and supplies no navigation pose.
"""

from embodied_navigation.project import project_root as _project_root
from collections import defaultdict
import os
from embodied_navigation.project import data_root
from pathlib import Path
import numpy as np


class MeshSurface:
    def __init__(self, name, uri):
        root = _project_root()
        assets = (
            data_root() / "runtime_assets/assets"
            if os.environ.get("PROJECT_DATA_ROOT")
            else root / "assets"
        )
        mesh = assets / "building/Building.dae"
        candidate = Path(uri[7:]) if uri.startswith("file://") else None
        if candidate is None or not candidate.is_file() or not candidate.samefile(mesh):
            raise ValueError("Building mesh must belong to this project")
        self.name = name
        self.cell = 0.5
        self.bins = defaultdict(list)
        tri = np.load(assets / "building/building_triangles.npz")["triangles"]
        a = tri[:, 0]
        u = tri[:, 1] - a
        v = tri[:, 2] - a
        det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
        use = np.abs(det) > 1e-10
        self.a = a[use]
        self.u = u[use]
        self.v = v[use]
        self.det = det[use]
        tri = tri[use]
        low = np.floor(tri[:, :, :2].min(1) / self.cell).astype(int)
        high = np.floor(tri[:, :, :2].max(1) / self.cell).astype(int)
        for i, (lo, hi) in enumerate(zip(low, high)):
            for x in range(lo[0], hi[0] + 1):
                for y in range(lo[1], hi[1] + 1):
                    self.bins[(x, y)].append(i)
        self.bins = {key: np.asarray(ix, dtype=int) for key, ix in self.bins.items()}

    def intersect(self, starts):
        """Vectorized ground-clearance raycast for one batch of probe points.

        Triangles are pre-indexed into 0.5 m XY bins and each point tests only
        its own cell. Per point the highest hit at or below the probe height is
        kept and the result is the clearance ``p.z - hit_z`` clamped to >= 0;
        a point with no triangle hit keeps ``+inf``.
        """
        out = np.full(len(starts), np.inf)
        for k, p in enumerate(starts):
            ix = self.bins.get(tuple(np.floor(p[:2] / self.cell).astype(int)))
            if ix is None:
                continue
            q = p[:2] - self.a[ix, :2]
            u = self.u[ix]
            v = self.v[ix]
            d = self.det[ix]
            s = (q[:, 0] * v[:, 1] - q[:, 1] * v[:, 0]) / d
            t = (u[:, 0] * q[:, 1] - u[:, 1] * q[:, 0]) / d
            z = self.a[ix, 2] + s * u[:, 2] + t * v[:, 2]
            valid = (s >= -1e-9) & (t >= -1e-9) & (s + t <= 1 + 1e-9) & (z <= p[2] + 1e-9)
            if valid.any():
                out[k] = max(0.0, float(p[2] - z[valid].max()))
        return out
