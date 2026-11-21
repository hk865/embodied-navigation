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

from .pose_graph import pose_difference, rotation_angle


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
        self._pending_entries: list[tuple[int, np.ndarray, np.ndarray, bool]] = []
        self._anchored_seed: tuple[int, np.ndarray, np.ndarray] | None = None
        self.rejections: list[dict] = []
        self._current: np.ndarray | None = None

    @property
    def pending(self) -> list[tuple[int, np.ndarray]]:
        return [(stamp, C) for stamp, C, _, _ in self._pending_entries]

    @property
    def current(self) -> np.ndarray | None:
        return self._current

    def _within_jump_gate(
        self,
        T_world_odom: np.ndarray,
        T_odom_body: np.ndarray,
        stamp_ns: int,
        *,
        reference_C: np.ndarray | None = None,
    ) -> bool:
        ref = self._current if self._current is not None else reference_C
        if ref is None:
            return True
        translation, rotation = pose_difference(ref @ T_odom_body, T_world_odom @ T_odom_body)
        if translation > self.max_jump[0] or rotation > self.max_jump[1]:
            self.rejections.append(
                {"stamp_ns": stamp_ns, "translation_m": translation, "rotation_rad": rotation}
            )
            return False
        return True

    def propose(
        self,
        T_world_odom: np.ndarray,
        T_odom_body: np.ndarray,
        stamp_ns: int,
        *,
        reference_C: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """Return the newly adopted C, or None when nothing was adopted."""
        self._anchored_seed = None
        if not self._within_jump_gate(T_world_odom, T_odom_body, stamp_ns, reference_C=reference_C):
            return None
        self._pending_entries = [
            entry for entry in self._pending_entries if stamp_ns - entry[0] <= self.window_ns
        ]
        agreeing = sum(self._agrees(C, T_world_odom, T_odom_body) for _, C, _, _ in self._pending_entries)
        if agreeing + 1 < self.confirmations:
            world_pos = (T_world_odom @ T_odom_body)[:3, 3]
            self._pending_entries.append((stamp_ns, T_world_odom, world_pos, False))
            return None
        self._pending_entries = []
        self._current = np.array(T_world_odom, dtype=np.float64)
        return self._current

    def seed_anchored(self, T_world_odom: np.ndarray, T_odom_body: np.ndarray, stamp_ns: int) -> None:
        """Mark a just-closed pending loop observation as anchored for ZUPT follow-up."""
        world_pos = (T_world_odom @ T_odom_body)[:3, 3]
        for idx in range(len(self._pending_entries) - 1, -1, -1):
            if self._pending_entries[idx][0] == stamp_ns:
                self._pending_entries[idx] = (stamp_ns, T_world_odom, world_pos, True)
                return
        self._pending_entries.append((stamp_ns, T_world_odom, world_pos, True))

    def propose_anchored(
        self,
        T_world_odom: np.ndarray,
        T_odom_body: np.ndarray,
        stamp_ns: int,
        *,
        reference_C: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """Confirm continuous stationary ZUPT corrections that hold the same world anchor position.

        Enforces the exact same jump gate, ``window_s``, ``confirmations``, and
        rotation agreement as ``propose``, while allowing stationary and moving
        candidates within ``window_s`` to mutually confirm consistent corrections.
        """
        if not self._within_jump_gate(T_world_odom, T_odom_body, stamp_ns, reference_C=reference_C):
            return None
        world_pos = (T_world_odom @ T_odom_body)[:3, 3]
        R_corr = T_world_odom[:3, :3]
        self._pending_entries = [
            entry for entry in self._pending_entries if stamp_ns - entry[0] <= self.window_ns
        ]
        if self._anchored_seed is not None and stamp_ns - self._anchored_seed[0] > self.window_ns:
            self._anchored_seed = None
        agreeing = 0
        if self._anchored_seed is not None:
            _, seed_C, seed_pos = self._anchored_seed
            if self._anchored_agrees(seed_pos, seed_C[:3, :3], world_pos, R_corr):
                agreeing += 1
        for _, entry_C, entry_pos, is_anchored in self._pending_entries:
            if is_anchored:
                if self._anchored_agrees(entry_pos, entry_C[:3, :3], world_pos, R_corr):
                    agreeing += 1
            elif self._agrees(entry_C, T_world_odom, T_odom_body):
                agreeing += 1
        if agreeing + 1 < self.confirmations:
            self._pending_entries.append((stamp_ns, T_world_odom, world_pos, True))
            return None
        self._pending_entries = []
        self._anchored_seed = (stamp_ns, np.array(T_world_odom, dtype=np.float64), world_pos)
        self._current = np.array(T_world_odom, dtype=np.float64)
        return self._current

    def _anchored_agrees(
        self, pos_a: np.ndarray, R_a: np.ndarray, pos_b: np.ndarray, R_b: np.ndarray
    ) -> bool:
        if float(np.linalg.norm(pos_a - pos_b)) > self.agree[0]:
            return False
        return rotation_angle(R_a, R_b) <= self.agree[1]

    def _agrees(self, a: np.ndarray, b: np.ndarray, T_odom_body: np.ndarray) -> bool:
        translation, rotation = pose_difference(a @ T_odom_body, b @ T_odom_body)
        return translation <= self.agree[0] and rotation <= self.agree[1]
