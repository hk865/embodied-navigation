"""Pluggable BeforeFinishHook task verifiers (ObjectNav, EQA, mapping reports)."""

from __future__ import annotations

import math
from typing import Any, Callable

from embodied_navigation.agent.runtime import FinishCheckResult, ToolCall
from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
    _label_strict_match,
)


class ObjectNavFinishVerifier:
    """Sensor-only BeforeFinishHook enforcing current-frame proximity and mask consensus."""

    def __init__(
        self,
        *,
        daemon: Any,
        target_category: str,
        stop_radius_m: float,
        on_attempt_cb: Callable[[FinishCheckResult], None] | None = None,
    ) -> None:
        self.daemon = daemon
        self.target_category = str(target_category).strip()
        self.stop_radius_m = float(stop_radius_m)
        self.on_attempt_cb = on_attempt_cb
        self.first_stop_attempted = False
        self.first_stop_success = False

    def __call__(self, call: ToolCall) -> FinishCheckResult:
        claimed = call.arguments.get("target_label")
        if (
            isinstance(claimed, str)
            and claimed.strip()
            and not _label_strict_match(self.target_category, claimed)
        ):
            res = FinishCheckResult(
                allowed=False,
                reason="target_label_mismatch",
                metadata={"claimed": claimed, "expected": self.target_category},
            )
            if self.on_attempt_cb:
                self.on_attempt_cb(res)
            return res

        robot_mem, _, _ = self.daemon.agent_pose()
        kf_id = self.daemon.current_kf_id

        # Confusable-class & low-score stop guard on current frame detections (prevents
        # bedroom nightstands/dressers/consoles from slipping past a 'table' stop gate).
        _CONFUSABLE_FOR_TARGET: dict[str, frozenset[str]] = {
            "table": frozenset(
                {
                    "nightstand",
                    "bedside table",
                    "dresser",
                    "chest of drawers",
                    "cabinet",
                    "vanity",
                    "counter",
                    "kitchen counter",
                    "shelf",
                    "bookshelf",
                    "console",
                    "stair railing",
                    "railing",
                }
            ),
            "chair": frozenset({"bed", "couch", "sofa", "table", "cabinet", "toilet"}),
            "couch": frozenset({"bed", "chair", "table", "ottoman", "bench"}),
            "bed": frozenset({"couch", "sofa", "table", "dresser"}),
        }
        _MIN_TARGET_SCORE: dict[str, float] = {"table": 0.45}

        if self.daemon.vision_has_seg and self.daemon.current_detections:
            conf_set = _CONFUSABLE_FOR_TARGET.get(self.target_category, frozenset())
            min_req = _MIN_TARGET_SCORE.get(self.target_category, 0.35)
            near_matches = [
                g
                for g in self.daemon.current_detections
                if _label_strict_match(self.target_category, g.label)
                and float(g.surface_distance_m) <= (self.stop_radius_m + 0.6)
            ]
            if near_matches:
                best_det = max(near_matches, key=lambda d: float(d.score))
                conf_winner: tuple[str, float] | None = None
                for clbl, csc in (best_det.candidate_scores or {}).items():
                    cl_norm = str(clbl).strip().lower()
                    if (
                        any(cf in cl_norm for cf in conf_set)
                        and float(csc) >= float(best_det.score) - 0.08
                    ):
                        if conf_winner is None or float(csc) > conf_winner[1]:
                            conf_winner = (cl_norm, float(csc))
                # Also check spatially overlapping stronger confusable detection on the same frame
                if conf_winner is None and conf_set:
                    for other in self.daemon.current_detections:
                        ol_norm = str(other.label).strip().lower()
                        if not any(cf in ol_norm for cf in conf_set):
                            continue
                        dx = float(other.local_xyz[0]) - float(best_det.local_xyz[0])
                        dy = float(other.local_xyz[1]) - float(best_det.local_xyz[1])
                        if math.hypot(dx, dy) <= 0.65 and float(other.score) >= float(best_det.score) - 0.05:
                            conf_winner = (ol_norm, float(other.score))
                            break
                if best_det.abstain or conf_winner is not None:
                    cname = conf_winner[0] if conf_winner is not None else "ambiguous_confusor"
                    res = FinishCheckResult(
                        allowed=False,
                        reason="confusable_class_override",
                        metadata={
                            "target_score": round(float(best_det.score), 3),
                            "confusable_label": cname,
                            "hint": (
                                f"STOP rejected: this object looks more like '{cname}' than a genuine "
                                f"'{self.target_category}'. Refute it with update_belief and keep searching."
                            ),
                        },
                    )
                    if self.on_attempt_cb:
                        self.on_attempt_cb(res)
                    return res
                if float(best_det.score) < min_req and not (
                    self.daemon.latest_disambiguated_ok
                    and self.daemon.latest_disambiguated_kf == kf_id
                ):
                    res = FinishCheckResult(
                        allowed=False,
                        reason="low_confidence_target_detection",
                        metadata={
                            "target_score": round(float(best_det.score), 3),
                            "min_required": min_req,
                            "hint": (
                                f"STOP rejected: '{self.target_category}' score={best_det.score:.2f} < {min_req:.2f}. "
                                f"Run inspect_vision(prompts=['{self.target_category}', ...], mode='disambiguate') "
                                f"with confusors to verify before stopping."
                            ),
                        },
                    )
                    if self.on_attempt_cb:
                        self.on_attempt_cb(res)
                    return res

        # When an exact pixel-mask backend (SAM 3) is active, require vision-mask consensus
        # on the current keyframe so ungrounded VLM guesses or stair-rail hallucinations fail.
        excluded: tuple[str, ...] = (
            ("opencode_vlm", "opencode_vlm_unverified", "vlm_depth_grounded", "agent_relabeled")
            if (self.daemon.vision_has_seg and self.daemon.vision_backend == "sam3")
            else ("opencode_vlm", "opencode_vlm_unverified", "agent_relabeled")
        )
        ver = self.daemon.memory.verify_stop_claim(
            robot_mem,
            target_label=self.target_category,
            stop_radius_m=self.stop_radius_m,
            require_current_keyframe_id=kf_id,
            excluded_producers=excluded,
        )
        if not ver.allowed and self.daemon.vision_has_seg and self.daemon.vision_backend == "sam3":
            hint = (
                f"STOP rejected ({ver.reason}, dist={ver.distance_m}): no verified SAM 3 pixel mask "
                f"consensus for '{self.target_category}' on current keyframe '{kf_id}' within "
                f"{self.stop_radius_m:.1f} m. Run inspect_vision or move closer with direct LOS."
            )
        elif not ver.allowed:
            hint = f"STOP rejected ({ver.reason}, dist={ver.distance_m}). Move closer or inspect."
        else:
            hint = "Verified target proximity within stop_radius."
        res = FinishCheckResult(
            allowed=ver.allowed,
            reason=ver.reason,
            metadata={**ver.to_dict(), "hint": hint},
        )
        if self.on_attempt_cb:
            self.on_attempt_cb(res)
        return res


class ReportFinishVerifier:
    """Task-agnostic BeforeFinishHook for EQA / mapping-report delivery tasks."""

    def __init__(self, daemon: Any) -> None:
        self.daemon = daemon

    def __call__(self, call: ToolCall) -> FinishCheckResult:
        summary = str(call.arguments.get("summary") or "").strip()
        if not summary:
            return FinishCheckResult(allowed=False, reason="missing_summary")
        ev_ids = call.arguments.get("evidence_ids") or []
        valid_ev = []
        if isinstance(ev_ids, (list, tuple)):
            for eid in ev_ids:
                s = str(eid).strip()
                if s in self.daemon.memory.graph.keyframe_poses or s in self.daemon.memory.graph.objects:
                    valid_ev.append(s)
        return FinishCheckResult(
            allowed=True,
            reason="report_verified",
            metadata={"summary": summary, "validated_evidence_ids": valid_ev},
        )
