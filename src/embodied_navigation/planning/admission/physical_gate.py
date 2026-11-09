"""R85 binding of unchanged physical geometry proofs to real path admission.

This adds no planner/control mathematics. Teacher callers do not construct it.
The existing physical certifier and original execution/source gates stay separate.
"""

from embodied_navigation.common.config import loads as _project_loads
import copy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import BSpline

import embodied_navigation.planning.admission.static_geometry as geometry

INITIALIZATION_METADATA_MAX_BYTES = 8 * 1024 * 1024


def digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def file_value(path, limit=2_000_000):
    path = Path(path).resolve()
    if path.stat().st_size > limit:
        raise ValueError("r85_physical_binding_file_size_budget")
    raw = path.read_bytes()
    if len(raw) > limit:
        raise ValueError("r85_physical_binding_file_size_budget")
    return _project_loads(raw), dict(
        path=str(path), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest()
    )


def finite_observed_prediction(proof, payload, context):
    """Identity-valid finite UNKNOWN geometry score; never a formal certificate."""
    try:
        if (
            not isinstance(proof, dict)
            or proof.get("schema") != "r85_continuous_scan_physical_certificate/v1"
            or proof.get("allowed") is not False
            or proof.get("status") != "UNKNOWN_REJECTED"
            or proof.get("reason")
            not in ("continuous_outer_bound_not_certified", "patch_or_wall_budget_exhausted")
            or proof.get("coordinate_frame") != "camera_init"
            or payload.get("frame_id") != "camera_init"
            or payload.get("order") != 3
            or proof.get("T_navigation_world") != context["T_navigation_world"]
            or proof.get("budgets") != dict(max_patches=4096, max_depth=18, wall_budget_s=2.0)
        ):
            return False
        points = np.asarray(payload["pos_pts"], dtype=float)
        knots = np.asarray(payload["knots"], dtype=float)
        if (
            points.ndim != 2
            or points.shape[1] != 3
            or not 4 <= len(points) <= 512
            or not np.isfinite(points).all()
            or knots.shape != (len(points) + 4,)
            or not np.isfinite(knots).all()
            or np.any(np.diff(knots) < 0)
            or knots[-4] <= knots[3]
        ):
            return False
        json.dumps(proof, allow_nan=False)
        return True
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return False


class R85PhysicalPathGate:
    def __init__(self, run):
        self.run = Path(run).resolve()
        self.static_sources = None
        self._bound_route_key = None
        self._bound_route_context = None
        self.route_bindings = 0

    def _context_file_value(self, path, limit=2_000_000):
        # Called only at the actual route-definition/reference binding boundary.
        return file_value(path, limit)

    def _program_key(self, node, binding):
        runtime = node.automatic_reference
        key = getattr(node.trajectory_association, "route_key", None)
        if runtime is None or key is None:
            raise ValueError("r85_route_definition_not_bound")
        return (
            id(runtime),
            runtime.file_sha,
            tuple(runtime.scope),
            node.request_id,
            key,
            binding.get("reference_id"),
            binding.get("planner_epoch"),
            binding.get("planner_process_id"),
            binding.get("localization_generation"),
        )

    def bind_route_definition(self, node, binding):
        """Once per actual version boundary; ordinary program-owned state."""
        key = self._program_key(node, binding)
        if key != self._bound_route_key:
            context = self._bind_reference_context(node, binding)
            self._bound_route_context = context
            self._bound_route_key = key
            self.route_bindings += 1
        return self._bound_route_context

    def context(self, node, binding):
        # No file I/O, content scan, parse, full-route copy/hash on this path.
        key = self._program_key(node, binding)
        if key != self._bound_route_key:
            return self.bind_route_definition(node, binding)
        return self._bound_route_context

    def _bind_reference_context(self, node, binding):
        runtime = node.automatic_reference
        if runtime is None or not isinstance(binding, dict):
            raise ValueError("r85_physical_requires_bound_automatic_reference")
        doc = runtime.doc
        ref_source = dict(path=str(runtime.path.resolve()))
        registration = dict(
            run_id=self.run.name,
            reference_id=doc.get("reference_id"),
            epoch=doc.get("epoch"),
            localization_generation=doc.get("localization_generation"),
        )
        scope = (
            binding.get("planner_epoch"),
            binding.get("planner_process_id"),
            binding.get("localization_generation"),
        )
        if (
            not isinstance(doc, dict)
            or binding.get("reference_id") != doc.get("reference_id")
            or doc.get("run_id") != self.run.name
            or doc.get("frame_id") != "camera_init"
            or doc.get("navigation_ground_truth_used") is not False
            or tuple(runtime.scope) != scope
            or (doc.get("epoch"), doc.get("localization_generation")) != (scope[0], scope[2])
            or doc.get("reference_id")
            != str(node.request_id) + ":epoch:" + str(scope[0]) + ":generation:" + str(scope[2])
        ):
            raise ValueError("physical_reference_request_or_coordinate_scope_mismatch")
        # Current map/geometry is an algorithm input, not an experimental
        # equivalence certificate or a previously approved task.
        metadata, metadata_source = self._context_file_value(geometry.METADATA)
        limits, limits_source = self._context_file_value(geometry.CONFIG)
        actual, actual_source = self._context_file_value(self.run / "known_scene_reference.json")
        expected_T = doc.get("T_navigation_world")
        from embodied_navigation.localization_mapping.known_scene_matcher import checked_transform

        checked_transform(expected_T)
        if not isinstance(actual.get("boxes"), list) or not actual["boxes"]:
            raise ValueError("Current scene map is empty")
        sources = dict(
            metadata=metadata_source, limits=limits_source, actual_reference=actual_source
        )
        definition = node.trajectory_association.route_definition
        route_key = node.trajectory_association.route_key
        sent = runtime.reference["sent_reference"]
        if (
            not isinstance(definition, dict)
            or (definition.get("route_id"), definition.get("route_version")) != route_key
            or definition.get("run_id") != self.run.name
            or (
                (
                    definition.get("reference_epoch"),
                    definition.get("planner_process_id"),
                    definition.get("localization_generation"),
                )
                != scope
            )
            or (definition.get("reference_stamp") != sent["stamp"])
            or ({k: definition.get("reference_content", {}).get(k) for k in sent} != sent)
        ):
            raise ValueError("r85_physical_route_definition_binding_mismatch")
        self.static_sources = sources
        return dict(
            route_id=route_key[0],
            route_version=route_key[1],
            sent_reference_sha256=digest(sent),
            run_id=self.run.name,
            request_id=node.request_id,
            reference_id=doc["reference_id"],
            planner_epoch=scope[0],
            planner_process_id=scope[1],
            localization_generation=scope[2],
            registration_sha256=digest(registration),
            static_sources=sources,
            static_sources_sha256=digest(sources),
            T_navigation_world=expected_T,
        )

    def candidate_identity(
        self, node, value, metadata, context, *, require_current_association=False
    ):
        association = node.trajectory_association
        key = (metadata.get("route_id"), metadata.get("route_version"))
        if (
            metadata.get("schema") != 3
            or metadata.get("run_id") != self.run.name
            or metadata.get("trajectory_id") != value["traj_id"]
            or metadata.get("trajectory_start_time") != value["start_time"]
            or key != association.route_key
            or key != (context["route_id"], context["route_version"])
            or metadata.get("reference_epoch") != context["planner_epoch"]
            or metadata.get("planner_process_id") != context["planner_process_id"]
            or metadata.get("localization_generation") != context["localization_generation"]
            or metadata.get("reference_stamp") != association.reference_stamp
        ):
            raise ValueError("r85_physical_actual_payload_metadata_mismatch")
        if require_current_association and (
            association.accepted != value or association.accepted_metadata != metadata
        ):
            raise ValueError("r85_physical_not_current_schema3_association_payload")
        return dict(
            route_id=key[0],
            route_version=key[1],
            raw_payload_sha256=digest(value),
            metadata_sha256=digest(metadata),
            sent_reference_sha256=context["sent_reference_sha256"],
            body_output_frame_id="camera_init",
            scan_transport_offset_m=0.4,
            body_curve_coordinates_changed=False,
            source_binding=context,
        )

    def prepare(self, node, value, metadata, binding):
        """Freeze only static-certifier inputs while the owner holds its group.

        Live pose/source freshness and path admission remain the owner's gates.
        The original pose stamp is evidence, never rewritten at completion.
        """
        context = self.context(node, binding)
        identity = self.candidate_identity(
            node, value, metadata, context, require_current_association=True
        )
        pose_stamp = int(node.pose_stamp)
        physical_payload = copy.deepcopy(
            dict(
                value,
                frame_id="camera_init",
                run_id=self.run.name,
                request_id=node.request_id,
                epoch=context["planner_epoch"],
                localization_generation=context["localization_generation"],
                source_stamp_ns=pose_stamp,
            )
        )
        return dict(
            context=copy.deepcopy(context),
            identity=copy.deepcopy(identity),
            source_pose_stamp_ns=pose_stamp,
            physical_payload=physical_payload,
        )

    def complete(self, node, value, metadata, binding, prepared, proof):
        """Validate a worker result in the owner group; grant static proof only."""
        try:
            context = self.context(node, binding)
            if context != prepared["context"]:
                raise ValueError("r85_physical_sources_changed_during_compute")
            identity = self.candidate_identity(
                node, value, metadata, context, require_current_association=True
            )
            if identity != prepared["identity"]:
                raise ValueError("r85_physical_candidate_changed_during_compute")
            pose_stamp = prepared["source_pose_stamp_ns"]
            if type(pose_stamp) is not int:
                raise ValueError("r85_physical_original_pose_stamp_invalid")
            expected = dict(
                value,
                frame_id="camera_init",
                run_id=self.run.name,
                request_id=node.request_id,
                epoch=context["planner_epoch"],
                localization_generation=context["localization_generation"],
                source_stamp_ns=pose_stamp,
            )
            if expected != prepared["physical_payload"]:
                raise ValueError("r85_physical_prepared_payload_changed")
            allowed = physical_allowed(proof, expected, context)
            # UNKNOWN can be observed by the existing diagnostic profile, so
            # bind even a negative worker prediction before that policy sees it.
            if (
                isinstance(proof, dict)
                and proof.get("status") == "UNKNOWN_REJECTED"
                and not finite_observed_prediction(proof, expected, context)
            ):
                raise ValueError("r85_physical_worker_unknown_identity_mismatch")
            return dict(
                schema="r85_actual_scan_physical_execution_gate/v1",
                allowed=allowed,
                reason="actual_curve_physical_space_certified"
                if allowed
                else "actual_curve_physical_space_not_certified",
                **identity,
                source_pose_stamp_ns=pose_stamp,
                physical_payload=prepared["physical_payload"],
                physical_certificate=proof,
                navigation_PASS=False,
            )
        except (
            OSError,
            KeyError,
            ValueError,
            TypeError,
            AttributeError,
            IndexError,
            OverflowError,
        ) as error:
            return dict(
                schema="r85_actual_scan_physical_execution_gate/v1",
                allowed=False,
                reason="r85_physical_binding_rejected",
                error=type(error).__name__ + ":" + str(error),
            )

    def certify(self, node, value, metadata, binding):
        try:
            context = self.context(node, binding)
            identity = self.candidate_identity(
                node, value, metadata, context, require_current_association=True
            )
            pose_stamp = int(node.pose_stamp)
            physical_payload = dict(
                value,
                frame_id="camera_init",
                run_id=self.run.name,
                request_id=node.request_id,
                epoch=context["planner_epoch"],
                localization_generation=context["localization_generation"],
                source_stamp_ns=pose_stamp,
            )
            proof = geometry.certify_physical_curve(physical_payload)
            allowed = physical_allowed(proof, physical_payload, context)
            # Geometry computation must not grant a proof across changed inputs.
            if self.context(node, binding) != context:
                raise ValueError("r85_physical_sources_changed_during_compute")
            return dict(
                schema="r85_actual_scan_physical_execution_gate/v1",
                allowed=allowed,
                reason="actual_curve_physical_space_certified"
                if allowed
                else "actual_curve_physical_space_not_certified",
                **identity,
                source_pose_stamp_ns=pose_stamp,
                physical_payload=physical_payload,
                physical_certificate=proof,
                navigation_PASS=False,
            )
        except (
            OSError,
            KeyError,
            ValueError,
            TypeError,
            AttributeError,
            IndexError,
            OverflowError,
        ) as error:
            return dict(
                schema="r85_actual_scan_physical_execution_gate/v1",
                allowed=False,
                reason="r85_physical_binding_rejected",
                error=type(error).__name__ + ":" + str(error),
            )

    def fresh(self, node, binding, generation):
        """Check original source clocks after work; never update source timestamps."""
        try:
            context = self.context(node, binding)
            now = time.monotonic()
            clock = node.get_clock().now().nanoseconds

            def wall(value):
                return (
                    isinstance(value, (int, float))
                    and math.isfinite(value)
                    and 0 <= now - value < 0.3
                )

            def header(value):
                return type(value) is int and 0 <= clock - value < 300_000_000

            ready = node.known_scene_scan_epoch
            conditions = dict(
                current_planner_epoch_ready=isinstance(ready, dict)
                and ready.get("schema") == "known_scene_SCAN_epoch_ready/v1"
                and ready.get("run_id") == self.run.name
                and ready.get("ready") is True
                and ready.get("generation") == generation
                and ready.get("localization_generation") == context["localization_generation"]
                and ready.get("planner_epoch") == context["planner_epoch"]
                and ready.get("planner_identity", {}).get("pid") == context["planner_process_id"],
                same_generation=node.known_scene_inputs["generation"]
                == generation
                == context["localization_generation"],
                generation_ready=node._known_generation_ready(),
                pose_present=node.pose is not None,
                pose_wall_fresh=wall(node.pose_updated),
                pose_header_fresh=header(node.pose_stamp),
                cloud_wall_fresh=wall(node.cloud_updated),
                cloud_header_fresh=header(node.cloud_stamp),
                raw_imu_fresh=node.raw_imu.fresh(now, clock / 1e9),
                bridge_wall_fresh=wall(node.bridge_updated),
                bridge_ready=node.bridge_safety.get("state") == "ready",
                state_running=node.state == "running",
                obstacle_not_held=not node.obstacle_hold,
                tilt_not_held=not node.tilt_hold,
            )
            return dict(
                schema="r85_post_compute_original_source_gate/v1",
                allowed=all(conditions.values()),
                conditions=conditions,
                source_binding=context,
                checked_monotonic_wall=now,
                control_stamp_ns=clock,
                source_pose_stamp_ns=node.pose_stamp,
                source_cloud_stamp_ns=node.cloud_stamp,
                source_timestamps_refreshed=False,
            )
        except (
            OSError,
            KeyError,
            ValueError,
            TypeError,
            AttributeError,
            IndexError,
            OverflowError,
        ) as error:
            return dict(
                schema="r85_post_compute_original_source_gate/v1",
                allowed=False,
                reason="r85_post_compute_source_binding_rejected",
                error=type(error).__name__ + ":" + str(error),
                source_timestamps_refreshed=False,
            )

    def retain(self, node, active, route_proof, binding):
        try:
            context = self.context(node, binding)
            old = active["proof"]["r85_physical"]
            value, metadata = active["payload"], active["metadata"]
            identity = self.candidate_identity(node, value, metadata, context)
            if (
                old.get("schema") != "r85_actual_scan_physical_execution_gate/v1"
                or old.get("allowed") is not True
                or any(old.get(k) != v for k, v in identity.items())
                or old.get("physical_payload")
                != dict(
                    value,
                    frame_id="camera_init",
                    run_id=self.run.name,
                    request_id=node.request_id,
                    epoch=context["planner_epoch"],
                    localization_generation=context["localization_generation"],
                    source_stamp_ns=old.get("source_pose_stamp_ns"),
                )
            ):
                raise ValueError("r85_old_path_missing_same_payload_physical_proof")
            old_physical_passed = physical_allowed(
                old["physical_certificate"], old["physical_payload"], context
            )
            old_prediction_observed = False
            if not old_physical_passed:
                from embodied_navigation.common.observation import api as _observer_api

                diagnostic_enabled = _observer_api("diagnostic_observation").enabled
                diagnostic_record = _observer_api("diagnostic_observation").record
                if (
                    diagnostic_enabled(node.profile)
                    and old.get("diagnostic_observe_only") is True
                    and old.get("diagnostic_original_allowed") is False
                    and old.get("diagnostic_original_reason")
                    == "actual_curve_physical_space_not_certified"
                    and finite_observed_prediction(
                        old["physical_certificate"], old["physical_payload"], context
                    )
                    and self.fresh(node, binding, context["localization_generation"]).get("allowed")
                    is True
                ):
                    old_prediction_observed = diagnostic_record(
                        node.run,
                        node.profile,
                        "retained_SCAN_old_physical_prediction",
                        "original_old_curve_prediction_not_formally_certified",
                        dict(
                            payload_sha256=old["raw_payload_sha256"],
                            original_certificate=old["physical_certificate"],
                            old_source_and_payload_identity_unchanged=True,
                            formal_physical_permission_passed=False,
                        ),
                    )
                if not old_prediction_observed:
                    raise ValueError("r85_old_path_missing_same_payload_physical_proof")
            parameter = route_proof["remaining_begin_parameter_s"]
            from embodied_navigation.common.observation import api as _observer_api

            diagnostic_enabled = _observer_api("diagnostic_observation").enabled
            if diagnostic_enabled(node.profile):
                if (
                    type(parameter) not in (int, float)
                    or not math.isfinite(parameter)
                    or not value["knots"][3] <= parameter <= value["knots"][-4]
                ):
                    raise ValueError("r85_retained_parameter_outside_original_curve_domain")
            position = np.asarray(node.pose, float).copy()
            target = BSpline(value["knots"], value["pos_pts"], 3)(parameter)
            # Identical construction to original remaining_certificate connector.
            connector = dict(
                order=3,
                pos_pts=[
                    (position + (target - position) * u).tolist() for u in (0, 1 / 3, 2 / 3, 1)
                ],
                knots=[0, 0, 0, 0, 1, 1, 1, 1],
                frame_id="camera_init",
                run_id=self.run.name,
                request_id=node.request_id,
                epoch=context["planner_epoch"],
                localization_generation=context["localization_generation"],
                source_stamp_ns=int(node.pose_stamp),
            )
            proof = geometry.certify_physical_curve(connector)
            allowed = physical_allowed(proof, connector, context)
            if self.context(node, binding) != context:
                raise ValueError("r85_retained_physical_sources_changed_during_compute")
            fresh = self.fresh(node, binding, context["localization_generation"])
            connector_physical_passed = allowed
            connector_prediction_observed = False
            if not allowed and fresh["allowed"]:
                from embodied_navigation.common.observation import api as _observer_api

                diagnostic_enabled = _observer_api("diagnostic_observation").enabled
                diagnostic_record = _observer_api("diagnostic_observation").record
                if diagnostic_enabled(node.profile) and finite_observed_prediction(
                    proof, connector, context
                ):
                    connector_prediction_observed = diagnostic_record(
                        node.run,
                        node.profile,
                        "retained_SCAN_actual_body_connector_prediction",
                        "original_connector_prediction_not_formally_certified",
                        dict(
                            original_certificate=proof,
                            actual_body_stamp_ns=int(node.pose_stamp),
                            source_binding=context,
                            actual_source_fresh=True,
                            formal_physical_permission_passed=False,
                        ),
                    )
                    allowed = connector_prediction_observed
            allowed = allowed and fresh["allowed"]
            diagnostic_continued = bool(old_prediction_observed or connector_prediction_observed)
            result = dict(
                schema="r85_retained_curve_and_actual_connector_physical_gate/v1",
                allowed=allowed,
                reason=(
                    "diagnostic_retained_curve_prediction_continued"
                    if allowed and diagnostic_continued
                    else "old_curve_and_actual_body_connector_physical_certified"
                    if allowed
                    else "actual_body_connector_physical_not_certified"
                ),
                whole_curve_proof_payload_sha256=old["raw_payload_sha256"],
                source_binding=context,
                remaining_parameter_s=parameter,
                actual_body_xyz=position.tolist(),
                actual_body_stamp_ns=int(node.pose_stamp),
                connector_payload=connector,
                connector_physical_certificate=proof,
                post_compute_source_gate=fresh,
            )
            if diagnostic_continued:
                result.update(
                    diagnostic_execution_continued=True,
                    original_old_curve_physical_permission_passed=old_physical_passed,
                    original_connector_physical_permission_passed=connector_physical_passed,
                    navigation_PASS=False,
                )
            return result
        except (
            OSError,
            KeyError,
            ValueError,
            TypeError,
            AttributeError,
            IndexError,
            OverflowError,
        ) as error:
            return dict(
                schema="r85_retained_curve_and_actual_connector_physical_gate/v1",
                allowed=False,
                reason="r85_retained_physical_binding_rejected",
                error=type(error).__name__ + ":" + str(error),
            )


def file_value_world(path):
    """SDF is bound as bytes, never parsed as JSON or used for robot pose."""
    path = Path(path).resolve()
    if path.stat().st_size > 2_000_000:
        raise ValueError("r85_actual_world_file_size_budget")
    raw = path.read_bytes()
    if len(raw) > 2_000_000:
        raise ValueError("r85_actual_world_file_size_budget")
    return None, dict(path=str(path), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())


def physical_allowed(proof, payload, context):
    return (
        isinstance(proof, dict)
        and proof.get("schema") == "r85_continuous_scan_physical_certificate/v1"
        and proof.get("allowed") is True
        and proof.get("status") == "CERTIFIED_STATIC_SPACE_ONLY"
        and proof.get("T_navigation_world") == context["T_navigation_world"]
        and proof.get("budgets") == dict(max_patches=4096, max_depth=18, wall_budget_s=2.0)
    )
