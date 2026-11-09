"""Online RGB person filtering and exact-estimator-pose map experiments.

No USD stage, renderer labels, native robot pose or evaluator enters this class.
Unlabelled frontend points are temporary LIO candidates, never certified static.
Persistent geometry requires repeated observations with exact estimated poses.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    DYNAMIC,
    LayeredMap,
    Observation,
    PointMask,
)
from embodied_navigation.perception.learned_person_mapping import (
    learned_person_evidence,
)
from embodied_navigation.perception.person_geometry_tracking import (
    PersonGeometryTracker,
)
from embodied_navigation.perception.person_worker import (
    PersonTriggerPolicy,
    PersonWorkerClient,
)
from embodied_navigation.perception.temporal_geometry import TemporalGeometryCache
from embodied_navigation.simulation.dynanav.slam_route import pose_transform


def inside_boxes(points, boxes):
    selected = np.zeros(len(points), dtype=bool)
    for box in boxes:
        low, high = np.asarray(box["bounds_map"], dtype=float)
        selected |= np.all((points >= low) & (points <= high), axis=1)
    return selected


class LearnedMapping:
    """Synchronous real inference; physics pauses while the local worker runs.

    Filtering is immediate in the sensor frame. Geometry and object histories
    wait for an exact post-VIO header match; no last-pose or GT substitution.
    A causal last estimated pose is permitted only for conservative quarantine
    of temporary frontend candidates, with its age recorded explicitly.
    """

    def __init__(
        self, output, profile, *, client=None, trigger=None, tracker=None, geometry=None
    ):
        self.output, self.profile = Path(output), profile
        self.output.mkdir()
        (self.output / "keyframes").mkdir()
        self.client = client or PersonWorkerClient(
            profile["person_worker"],
            expected_model_sha256=profile["person_model_sha256"],
        )
        self.client.wait_ready(timeout_s=30)
        self.trigger = trigger or PersonTriggerPolicy(
            max_interval_s=profile["person_max_interval_s"],
            min_interval_s=profile["person_min_interval_s"],
            view_change_threshold=0.06,
        )
        self.tracker = tracker or PersonGeometryTracker(
            epoch=1,
            frame_id="go2_imu",
            calibration_id="rgbd:v1",
            map_frame_id="dynanav_lio_map",
        )
        self.geometry = geometry or TemporalGeometryCache()
        self.store = LayeredMap(
            map_id="online-person", frame_id="dynanav_lio_map", max_observations=2000
        )
        self.pending = {}
        self.pose_cache = {}
        self.completed = []
        self.last_keyframe_ns = None
        self.last_source_ns = 0
        self.last_integrated_ns = -1
        self.consecutive_model_errors = 0
        self.frames = self.model_calls = self.model_errors = self.matched = (
            self.keyframes
        ) = 0
        self.unmatched = self.geometry_admissions = 0
        self.last_inference_position = None
        self.last_frame = {}
        self.stream_points = self.stream_unknown_background = (
            self.stream_kept_points
        ) = 0

    def journal(self, name, value):
        with (self.output / name).open("a") as handle:
            handle.write(json.dumps(value, allow_nan=False) + "\n")

    def process(self, *, packet, source_ns, rgb, depth, latest_odom=None):
        self.frames += 1
        self.last_source_ns = source_ns
        latest_source_valid = (
            latest_odom is not None
            and latest_odom.get("pose_source") == "livo_post_vio"
            and latest_odom.get("frame_id") == "dynanav_lio_map"
            and latest_odom.get("child_frame_id") == "go2_imu"
        )
        motion = None
        if latest_source_valid and self.last_inference_position is not None:
            motion = float(
                np.linalg.norm(
                    np.asarray(latest_odom["position"]) - self.last_inference_position
                )
            )
        decision = self.trigger.decide(
            rgb,
            source_ns=source_ns,
            estimated_motion_m=motion,
            motion_source="livo_post_vio" if motion is not None else None,
        )
        prediction = None
        failure = None
        if decision["trigger"]:
            try:
                prediction = self.client.predict(
                    rgb,
                    source_ns=source_ns,
                    frame_id="go2_camera_optical",
                    reason=decision["reason"],
                    timeout_s=3,
                )
                if prediction is None:
                    failure = "worker_returned_no_exact_result"
                else:
                    self.trigger.mark_inferred(
                        rgb, source_ns=source_ns, result=prediction
                    )
                    self.model_calls += 1
                    if latest_source_valid:
                        self.last_inference_position = np.array(
                            latest_odom["position"], dtype=float
                        )
            except (RuntimeError, TimeoutError, ValueError, OSError) as error:
                failure = repr(error)
                prediction = None
            if failure is not None:
                self.model_errors += 1
                self.consecutive_model_errors += 1
            else:
                self.consecutive_model_errors = 0
        states = np.zeros(len(packet.points), dtype=np.uint8)
        if prediction is not None:
            states, _, _, _ = learned_person_evidence(
                prediction.masks,
                prediction.scores,
                packet.pixel_flat_indices,
                source_ns,
                score_threshold=0.1,
            )
        # Excluding a low-score candidate is conservative frontend filtering;
        # it does not assert a confirmed semantic identity or static background.
        keep = states != DYNAMIC
        quarantine = self.tracker.current_quarantine(source_ns)
        pose_age = None
        quarantine_pose_valid = not quarantine
        quarantined = np.zeros(len(states), dtype=bool)
        if latest_source_valid:
            pose_ns = latest_odom.get("source_ns", round(latest_odom["time"] * 1e9))
            pose_age = source_ns - pose_ns
            if 0 <= pose_age <= 250_020_000:
                transform = pose_transform(
                    latest_odom["position"], latest_odom["orientation_wxyz"]
                )
                world = packet.points @ transform[:3, :3].T + transform[:3, 3]
                quarantined = inside_boxes(world, quarantine)
                keep &= ~quarantined
                quarantine_pose_valid = True
        withheld_reason = None
        if failure is not None:
            keep[:] = False
            withheld_reason = "model_failure"
        elif quarantine and not quarantine_pose_valid:
            keep[:] = False
            withheld_reason = "active_person_quarantine_without_fresh_estimated_pose"
        receipt = {
            "sequence": self.frames,
            "source_ns": source_ns,
            "fresh_inference": prediction is not None,
            "trigger": decision,
            "model_error": failure,
            "model_calls": self.model_calls,
            "filter_candidate_points": int(np.sum(states == DYNAMIC)),
            "quarantined_candidate_points": int(quarantined.sum()),
            "frontend_input_points": len(states),
            "frontend_kept_points": int(keep.sum()),
            "frontend_withheld_reason": withheld_reason,
            "unknown_background_points": int(np.sum(states == 0)),
            "background_certified_static": False,
            "frontend_topic": "/dynanav/lidar_perception_filtered",
            "frontend_scope": "temporary non-person/non-quarantined LIO candidates, not certified static",
            "quarantine_pose_age_ns": pose_age,
            "quarantine_pose_valid": quarantine_pose_valid,
            "physics_paused_for_synchronous_inference": bool(decision["trigger"]),
            "model_metadata": prediction.metadata if prediction is not None else None,
            "semantic_oracle_input": False,
            "native_pose_input": False,
        }
        self.stream_points += len(states)
        self.stream_unknown_background += int(np.sum(states == 0))
        self.stream_kept_points += int(keep.sum())
        self.last_frame = receipt
        self.journal("frames.jsonl", receipt)
        if self.consecutive_model_errors >= 3:
            raise RuntimeError(
                "Three consecutive online model failures; bounded experiment aborted"
            )
        self.pending[source_ns] = {
            "sequence": self.frames,
            "points": packet.points.copy(),
            "pixels": packet.pixel_flat_indices.copy(),
            "rgb": rgb.copy(),
            "depth": depth.copy(),
            "prediction": prediction,
            "candidate_states": states,
            "inference_error": failure,
        }
        while len(self.pending) > 64:
            old = min(self.pending)
            self.pending.pop(old)
            self.unmatched += 1
            self.journal(
                "unmatched.jsonl",
                {"source_ns": old, "reason": "bounded_pending_without_exact_pose"},
            )
        if source_ns in self.pose_cache:
            self._integrate(source_ns, self.pose_cache.pop(source_ns))
        return states, keep, receipt, quarantined

    def offer_pose(self, odom):
        source_ns = odom["source_ns"]
        if (
            odom.get("pose_source") != "livo_post_vio"
            or odom.get("frame_id") != "dynanav_lio_map"
            or odom.get("child_frame_id") != "go2_imu"
        ):
            self.journal(
                "rejected_poses.jsonl",
                {"source_ns": source_ns, "reason": "source_or_frame_mismatch"},
            )
            return
        try:
            transform = pose_transform(odom["position"], odom["orientation_wxyz"])
        except ValueError as error:
            self.journal(
                "rejected_poses.jsonl",
                {
                    "source_ns": source_ns,
                    "reason": "invalid_measured_pose",
                    "error": str(error),
                },
            )
            return
        if source_ns in self.pending:
            self._integrate(source_ns, transform)
        else:
            self.pose_cache[source_ns] = transform
            while len(self.pose_cache) > 64:
                self.pose_cache.pop(min(self.pose_cache))

    def _integrate(self, source_ns, transform):
        if source_ns <= self.last_integrated_ns:
            if self.pending.pop(source_ns, None) is not None:
                self.unmatched += 1
            self.journal(
                "unmatched.jsonl",
                {
                    "source_ns": source_ns,
                    "reason": "nonmonotonic_exact_pose_match",
                    "last_integrated_ns": self.last_integrated_ns,
                },
            )
            return
        frame = self.pending.pop(source_ns)
        prediction = frame["prediction"]
        track = self.tracker.update(
            source_ns,
            frame["points"],
            frame["pixels"],
            masks=prediction.masks if prediction is not None else None,
            scores=prediction.scores if prediction is not None else None,
            mask_source_ns=source_ns if prediction is not None else None,
            produced_ns=source_ns,
            now_ns=source_ns,
            T_map_sensor=transform,
            pose_source_ns=source_ns,
            pose_source="slam",
            image_shape=frame["rgb"].shape[:2],
            temporal_support=prediction.temporal_support
            if prediction is not None
            else None,
            detector_provenance=prediction.metadata if prediction is not None else None,
        )
        boxes = np.asarray(
            [volume["bounds_map"] for volume in track.quarantine_volumes], dtype=float
        ).reshape(-1, 2, 3)
        excluded = (frame["candidate_states"] == DYNAMIC) | (track.states == DYNAMIC)
        if frame["inference_error"] is not None:
            excluded[:] = True
        admitted, geometry_receipt = self.geometry.update(
            frame["points"],
            transform,
            source_ns,
            excluded_indices=excluded,
            person_quarantine=boxes,
            pose_source_ns=source_ns,
            pose_source="slam",
        )
        self.last_integrated_ns = source_ns
        self.matched += 1
        self.geometry_admissions += len(admitted)
        dynamic = np.flatnonzero(track.states == DYNAMIC)
        if len(dynamic):
            obs = Observation(
                epoch=1,
                sequence=frame["sequence"],
                source_ns=source_ns,
                pose_source_ns=source_ns,
                points_sensor=frame["points"][dynamic],
                T_map_sensor=transform,
                anchor_id=f"measured:{source_ns}",
                frame_id="go2_imu",
                calibration_id="rgbd:v1",
                pose_source="slam",
            )
            mask = PointMask(
                epoch=1,
                sequence=frame["sequence"],
                source_ns=source_ns,
                produced_ns=source_ns,
                frame_id="go2_imu",
                calibration_id="rgbd:v1",
                states=track.states[dynamic],
                instance_ids=tuple(track.instance_ids[i] for i in dynamic),
                labels=track.labels,
                provenance=track.provenance,
            )
            self.store.integrate(obs, mask, now_ns=source_ns)
        counts = {
            "source_ns": source_ns,
            "exact_pose_match": True,
            "geometry": geometry_receipt,
            "tracking": track.receipt,
            "dynamic_points": len(dynamic),
            "geometry_admitted_points": len(admitted),
        }
        self.journal("integrated.jsonl", counts)
        self.completed.append(
            {
                "source_ns": source_ns,
                "static_indices": admitted.copy(),
                "transform": transform.copy(),
                "geometry": geometry_receipt,
                "fresh_inference": prediction is not None,
                "tracked_ids": tuple(track.instance_ids),
                "tracked_states": track.states.copy(),
            }
        )
        if (
            self.last_keyframe_ns is None
            or source_ns - self.last_keyframe_ns
            >= self.profile["map_keyframe_interval_ns"]
        ):
            states = track.states.copy()
            states[admitted] = 1
            np.savez_compressed(
                self.output / "keyframes" / f"rgbd_{frame['sequence']:06d}.npz",
                rgb=frame["rgb"],
                depth=frame["depth"],
                points_sensor=frame["points"],
                pixel_flat_indices=frame["pixels"],
                states=states,
                instance_paths=np.asarray(track.instance_ids, dtype=str),
                T_map_sensor=transform,
                source_ns=np.int64(source_ns),
                labels_json=np.asarray(json.dumps(track.labels)),
                mask_provenance_json=np.asarray(json.dumps(track.provenance)),
                semantic_oracle=np.bool_(False),
                pose_oracle=np.bool_(False),
            )
            self.last_keyframe_ns = source_ns
            self.keyframes += 1

    def take_completed(self):
        completed, self.completed = self.completed, []
        return completed

    def finish(self):
        points = self.geometry.confirmed_voxel_points()
        np.savez_compressed(
            self.output / "static_map.npz",
            points=points,
            map_frame="dynanav_lio_map",
            pose_source="slam",
            mask_source="learned_person_plus_repeated_geometry",
            complete_full_scene=False,
            semantic_oracle=False,
            no_GT_pose_used=True,
        )
        records = self.geometry.confirmed_records()
        (self.output / "static_support_ledger.json").write_text(json.dumps(records))
        scene = self.store.snapshot(now_ns=self.last_source_ns, include_points=False)
        scene["geometry_candidate_map"] = {
            "path": str(self.output / "static_map.npz"),
            "points": len(points),
            "scope": "repeated measured-geometry candidates; not a proof of permanent static ownership",
        }
        scene["provenance_summary"].update(
            model_execution="local synchronous online worker",
            exact_live_post_vio_pose=self.matched > 0,
            oracle_labels_used_by_algorithm=False,
            realtime_deadline_certified=False,
            taught_navigation_reference_upstream_oracle_free=False,
            geometry_scope="repeated measured geometry candidates, not permanent static ownership",
        )
        scene["online_perception"] = {
            "model_rgb_only": True,
            "oracle_labels_used_by_algorithm": False,
            "pose_pairing": "exact live post_vio_odometry headers",
            "model_execution": "local synchronous online worker",
            "realtime_deadline_certified": False,
            "taught_navigation_reference_upstream_oracle_free": False,
        }
        scene["mask_coverage_complete"] = False
        scene["semantic_label_coverage_complete"] = False
        scene["observation_point_counts_scope"] = (
            "dynamic-only object store; whole RGB-D stream separately counted"
        )
        scene["provenance_summary"].update(
            actual_accepted_model_results=self.model_calls,
            exact_pose_matched_frames=self.matched,
            rgbd_stream_frames=self.frames,
            stream_point_count=self.stream_points,
            semantic_unknown_background_points=self.stream_unknown_background,
            temporary_frontend_kept_points=self.stream_kept_points,
            pending_unmatched_frames=len(self.pending),
            evicted_unmatched_frames=self.unmatched,
            geometry_candidate_points=len(points),
            full_stream_coverage_complete=False,
            unscheduled_frames_do_not_claim_new_semantics=True,
            capacity_scope="dynamic object store and independently bounded geometry cache; not full office coverage",
        )
        (self.output / "scene_map.json").write_text(json.dumps(scene, indent=2))
        from embodied_navigation.localization_mapping.semantic_mapping import (
            query as scene_query,
        )

        query = scene_query.select(scene, include_expired=True)
        (self.output / "person_query.json").write_text(json.dumps(query, indent=2))
        (self.output / "person_context.txt").write_text(scene_query.context_text(query))
        receipt = {
            "frames": self.frames,
            "model_calls": self.model_calls,
            "model_errors": self.model_errors,
            "exact_pose_matches": self.matched,
            "unmatched_evicted": self.unmatched,
            "pending_at_finish": len(self.pending),
            "keyframes": self.keyframes,
            "geometry_admissions": self.geometry_admissions,
            "geometry_candidate_points": len(points),
            "pose_source": "live_livo_post_vio_exact",
            "mask_source": "online_rgb_person_and_bounded_history",
            "learned_perception": True,
            "oracle_labels_used_by_algorithm": False,
            "GT_pose_used_by_mapping": False,
            "frontend_topic": "/dynanav/lidar_perception_filtered",
            "frontend_background_static_claimed": False,
            "semantic_frequency": {
                "max_interval_s": self.profile["person_max_interval_s"],
                "min_interval_s": self.profile["person_min_interval_s"],
            },
            "physics_paused_for_model_inference": True,
            "realtime_deadline_certified": False,
            "dynamic_objects": len(self.store.objects),
            "full_scene_coverage_verified": False,
            "taught_reference_pose_upstream": "previous oracle-filtered run; not end-to-end oracle-free navigation",
        }
        (self.output / "receipt.json").write_text(json.dumps(receipt, indent=2))
        return receipt
