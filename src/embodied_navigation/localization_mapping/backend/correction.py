"""Temporal admission of map->odom corrections ``C = T_world_odom``.

Candidates are compared where they matter: at the current body pose. Two
corrections ``C1, C2`` differ by ``pose_difference(C1 @ T_odom_body,
C2 @ T_odom_body)``, never by the translation of ``C1 - C2``, whose error
grows with the body's distance from the odom origin (audit B-5 / A-1).

A correction is adopted once ``confirmations`` mutually agreeing candidates
arrive within ``window_s``; the latest of them is adopted. The first adoption
has no jump check (relocalization initialisation). Afterwards a candidate
that differs from the adopted C by more than the jump limits is rejected,
appended to ``rejections`` and never clears the adopted C.
"""

from __future__ import annotations

import numpy as np

from .pose_graph import pose_difference


class CorrectionAdmission:
    def __init__(
        self,
        *,
        confirmations: int = 2,
        window_s: float = 10.0,
        agree_translation_m: float = 0.15,
        agree_rotation_rad: float = 0.05,
        max_jump_translation_m: float = 1.0,
        max_jump_rotation_rad: float = 0.35,
    ):
        self.confirmations = confirmations
        self.window_ns = int(window_s * 1e9)
        self.agree = (agree_translation_m, agree_rotation_rad)
        self.max_jump = (max_jump_translation_m, max_jump_rotation_rad)
        self.pending: list[tuple[int, np.ndarray]] = []
        self.rejections: list[dict] = []
        self._current: np.ndarray | None = None

    @property
    def current(self) -> np.ndarray | None:
        return self._current

    def propose(self, T_world_odom: np.ndarray, T_odom_body: np.ndarray, stamp_ns: int) -> np.ndarray | None:
        """Return the newly adopted C, or None when nothing was adopted."""
        if self._current is not None:
            translation, rotation = pose_difference(self._current @ T_odom_body, T_world_odom @ T_odom_body)
            if translation > self.max_jump[0] or rotation > self.max_jump[1]:
                self.rejections.append(
                    {"stamp_ns": stamp_ns, "translation_m": translation, "rotation_rad": rotation}
                )
                return None
        self.pending = [(stamp, C) for stamp, C in self.pending if stamp_ns - stamp <= self.window_ns]
        agreeing = sum(self._agrees(C, T_world_odom, T_odom_body) for _, C in self.pending)
        if agreeing + 1 < self.confirmations:
            self.pending.append((stamp_ns, T_world_odom))
            return None
        self.pending = []
        self._current = np.array(T_world_odom, dtype=np.float64)
        return self._current

    def _agrees(self, a: np.ndarray, b: np.ndarray, T_odom_body: np.ndarray) -> bool:
        translation, rotation = pose_difference(a @ T_odom_body, b @ T_odom_body)
        return translation <= self.agree[0] and rotation <= self.agree[1]
