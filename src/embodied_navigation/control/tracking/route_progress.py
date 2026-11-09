"""Monotone progress on connected 3D segments, including folded/closed routes.

Progress is a one-way (monotone) arc length in metres along a single connected
3D route polyline. Every update is derived from the measured body position plus
the static polyline. Search is restricted to a connected forward prefix rather
than choosing a later global-nearest match at repeated XYZ or XY coordinates.
"""

import numpy as np


class ConnectedRouteProgress:
    """Witness-based arc-length progress along one connected 3D polyline.

    ``points`` is the ordered route polyline, shape (N, 3), metres.
    ``maximum_distance`` is the gate for accepting a projection, metres.
    ``forward_arc`` is the local forward search window, metres. The corner-witness
    phase may advance progress before the projection phase uses that new value.

    Public state after construction: ``points``, ``delta``, ``length``, ``arc``
    (cumulative arc length per vertex, metres), ``progress`` (monotone arc
    position, metres) and ``segment`` (index of the current segment). Callers
    such as the planned-step filter read ``arc``/``segment``/``progress`` to
    index the connected local route prefix.
    """

    def __init__(self, points, maximum_distance=0.65, forward_arc=1.3):
        """Validate the polyline and precompute segment geometry.

        Raises ``ValueError`` for a non-finite input, a shape other than (N, 3)
        with N >= 2, or a segment shorter than 1e-6 m; the caller must coalesce
        duplicate route points first.
        """
        self.points = np.asarray(points, dtype=float)
        # Fail closed before any geometry is used: a route must be a finite,
        # at least two-vertex 3D polyline.
        if (
            self.points.ndim != 2
            or self.points.shape[1] != 3
            or len(self.points) < 2
            or not np.isfinite(self.points).all()
        ):
            raise ValueError("finite 3D route required")
        self.delta = np.diff(self.points, axis=0)
        self.length = np.linalg.norm(self.delta, axis=1)
        # A zero-length segment would divide by zero in the per-segment
        # projection below.
        if np.any(self.length < 1e-6):
            raise ValueError("coalesce duplicate route points first")
        self.arc = np.r_[0.0, np.cumsum(self.length)]
        self.distance = float(maximum_distance)
        self.forward_arc = float(forward_arc)
        self.progress = 0.0
        self.segment = 0

    def update(self, position):
        """Advance monotone progress for one measured 3D body position.

        Returns the updated ``self.progress`` in metres. The caller owns the
        measurement rate. Corner witnesses are bounded from the starting progress;
        projection then searches within ``forward_arc`` of the updated progress.
        This is not a single ``forward_arc`` bound on the whole update.
        """
        position = np.asarray(position, dtype=float)
        # A smoothed SCAN turn may pass within the existing .5m route fence
        # without visiting the exact polyline vertex. Witness only consecutive
        # nearby vertices, in 3D and within the same bounded connected arc.
        limit = self.progress + self.forward_arc
        while (
            # The witness loop stops before the final vertex: on a folded or
            # closed route that vertex may coincide with the start, so it is
            # excluded from intermediate progress witnesses.
            self.segment + 1 < len(self.points) - 1
            and self.arc[self.segment + 1] <= limit
            and np.linalg.norm(position - self.points[self.segment + 1]) <= 0.5
            # The normalized dot product (cosine of the corner angle) must
            # be below .95.
            and np.dot(self.delta[self.segment], self.delta[self.segment + 1])
            < 0.95 * self.length[self.segment] * self.length[self.segment + 1]
        ):
            self.progress = max(self.progress, float(self.arc[self.segment + 1]))
            self.segment += 1
        best_distance = float("inf")
        best_progress = self.progress
        # Search a connected local prefix only. A later revisit of the same XYZ
        # or the same XY on another floor is never a global-nearest shortcut.
        for i in range(self.segment, len(self.length)):
            if self.arc[i] > self.progress + self.forward_arc:
                break
            # Clamped projection of the measured position onto segment i,
            # u in [0, 1] along the segment.
            u = float(
                np.clip(
                    np.dot(position - self.points[i], self.delta[i]) / self.length[i] ** 2,
                    0.0,
                    1.0,
                )
            )
            # Clamp the candidate to the forward window measured from the
            # progress reached by the preceding corner-witness phase.
            progress = min(
                self.arc[i] + u * self.length[i],
                self.progress + self.forward_arc,
            )
            projected = self.points[i] + (progress - self.arc[i]) / self.length[i] * self.delta[i]
            distance = float(np.linalg.norm(position - projected))
            if distance < best_distance:
                best_distance = distance
                best_progress = progress
            # Stop this local search at the first worsening beyond .05 m.
            # Arc ordering does not make projection distance globally unimodal.
            elif distance > best_distance + 0.05:
                break
        # Accept only a projection inside the route fence; progress is monotone
        # (max) and the segment index is re-derived from it.
        if best_distance <= self.distance:
            self.progress = max(self.progress, best_progress)
            self.segment = min(
                len(self.length) - 1,
                int(np.searchsorted(self.arc, self.progress, side="right") - 1),
            )
        return self.progress

    @property
    def remaining(self):
        """Route length still ahead in metres (total arc minus progress)."""
        return float(self.arc[-1] - self.progress)
