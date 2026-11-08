#!/usr/bin/env python3
"""Typed ground/body static certificates; no ROS, model or physics imports.

This leaf preserves the original validator/config/receipts. All returned
allowed flags mean static geometry only, never navigation acceptance or
control authority. Observation-domain predicates are diagnostic sidecars.
"""
from __future__ import annotations
from embodied_navigation.common.config import loads as _project_loads

import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

import numpy as np

import embodied_navigation.common.paths as _package_paths
STAGE = _package_paths.GEOMETRY_STAGE
WORK = STAGE.parent
LEGACY = _package_paths.GEOMETRY_HELPER
LEGACY_SHA = hashlib.sha256(LEGACY.read_bytes()).hexdigest()
CONFIG = _package_paths.GEOMETRY_CONFIG
CONFIG_SHA = hashlib.sha256(CONFIG.read_bytes()).hexdigest()
METADATA = _package_paths.GEOMETRY_METADATA
BODY_HEIGHT = .32
HERE = Path(__file__).resolve().parent


def source(path):
    p = Path(path).resolve()
    raw = p.read_bytes()
    return {"path": str(p), "bytes": len(raw)}


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _helpers():
    spec = importlib.util.spec_from_file_location("r85_frozen_static_helpers", LEGACY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Geometry is loaded normally once per process/configuration, not checked
# against a historical file identity. Restart after changing map/configuration.
from functools import lru_cache
@lru_cache(maxsize=4)
def _context(metadata=METADATA, config=CONFIG):
    h=_helpers();d=h.limits(Path(config))
    boxes,uppers,tops,transform,bindings=h.geometry(Path(metadata))
    return h,d,boxes,uppers,tops,transform,bindings


def _array(value, name, minimum=2):
    a = np.asarray(value, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != 3 or not minimum <= len(a) <= 20000 or not np.isfinite(a).all():
        raise ValueError(name + " must be finite Nx3")
    return a


def deterministic_business_chain_prefix(source_doc):
    """Exact 2m prefix of the complete prepared typed source; no route search.

    Arc length is the ordered float64 Euclidean 3D body-polyline length.
    Retain every source vertex with cumulative arc <= 2m, including zero-length
    repeats. If the cut is inside an edge, append its linear interpolation using
    the same alpha for body and ground. No arbitrary start/length/simplification.
    The result is a business-chain prefix only, never an F3 navigation PASS.
    """
    if (source_doc.get("schema") != "r85_prepared_typed_global_reference/v1"
            or source_doc.get("reference_scope", "full_route") != "full_route"
            or source_doc.get("physical_body_ground_offset_m") != BODY_HEIGHT):
        raise ValueError("Prefix requires a complete prepared typed source")
    body = _array(source_doc["body_points"], "source body")
    ground = _array(source_doc["ground_points"], "source ground")
    if body.shape != ground.shape or not np.allclose(body-ground, [0, 0, BODY_HEIGHT], rtol=0, atol=1e-12):
        raise ValueError("Source body/ground relationship changed")
    lengths = np.linalg.norm(np.diff(body, axis=0), axis=1)
    if not np.isfinite(lengths).all():
        raise ValueError("Nonfinite source segment length")
    target, travelled = 2.0, 0.0
    prefix_body, prefix_ground = [body[0].copy()], [ground[0].copy()]
    retained, terminal_edge, alpha, interpolated = 1, None, None, False
    for edge, length_value in enumerate(lengths):
        length = float(length_value)
        if travelled == target and length > 0:
            break
        next_arc = travelled+length
        if next_arc <= target:
            prefix_body.append(body[edge+1].copy())
            prefix_ground.append(ground[edge+1].copy())
            retained += 1
            travelled = next_arc
            terminal_edge, alpha = edge, 1.0
            continue
        alpha = (target-travelled)/length
        if not 0 < alpha < 1:
            raise ValueError("Invalid deterministic prefix cut")
        prefix_body.append(body[edge]+alpha*(body[edge+1]-body[edge]))
        prefix_ground.append(ground[edge]+alpha*(ground[edge+1]-ground[edge]))
        travelled, terminal_edge, interpolated = target, edge, True
        break
    if travelled != target or len(prefix_body) < 2:
        raise ValueError("Complete prepared route is shorter than 2m")
    return {"reference_scope": "business_chain_prefix", "prefix_length_m": target,
        "body_points": np.asarray(prefix_body).tolist(), "ground_points": np.asarray(prefix_ground).tolist(),
        "prefix_endpoint_body_xyz": prefix_body[-1].tolist(),
        "prefix_evidence": {"method": "ordered_float64_3D_body_arclength_from_source_start",
            "source_point_count": len(body), "retained_source_vertex_count": retained,
            "terminal_source_edge_index": terminal_edge, "terminal_segment_fraction": alpha,
            "interpolated_terminal": interpolated, "arbitrary_subpath_supported": False,
            "F3_navigation_PASS": False}}


def _nav_to_world(a, transform):
    # transform is nav <- world, hence inverse is required.
    return (np.asarray(a, float) - transform[:3, 3]) @ transform[:3, :3]


def _world_to_nav(a, transform):
    return np.asarray(a, float) @ transform[:3, :3].T + transform[:3, 3]



def _support_under_body(body, h, tops, d):
    """Choose actual finite support below measured body within the fixed band.

    This labels ground geometry, not a projection of measured body Z onto an
    infinite plane. Overhead decks are excluded by below-body and height-band
    conditions; the original task body point remains separate and unchanged.
    """
    eligible = []
    for face in tops:
        if face["slope_rad"] > d["max_support_slope_rad"]:
            continue
        lo, hi, z, _ = h.face_intervals(body[None, :2], body[None, :2], face)
        if lo[0] > hi[0] or z[0] > body[2]:
            continue
        residual = body[2] - z[0] - BODY_HEIGHT
        if abs(residual) <= d["support_height_tolerance_m"]:
            eligible.append((float(z[0]), face, float(residual)))
    if not eligible:
        raise ValueError("No genuine finite support face under task body within .32/.10 contract")
    z, face, residual = max(eligible, key=lambda item: item[0])
    return np.r_[body[:2], z], {"model": face["model"], "face": "z+", "ground_world_z_m": z,
        "true_plane_slope_rad": face["slope_rad"], "measured_body_height_residual_m": residual,
        "eligible_finite_faces": [{"model": f["model"], "world_z_m": zz, "residual_m": rr} for zz, f, rr in eligible],
        "measured_body_point_changed": False, "plane_extrapolation_used": False}


def _physical_segment(g0, g1, b0, b1, context, *, grade=True):
    h, d, boxes, _, tops, _, _ = context
    center, _ = h.center_support(g0, g1, tops, d)
    live0, live1 = b0.copy(), b1.copy()
    live0[2] -= BODY_HEIGHT
    live1[2] -= BODY_HEIGHT
    live, _ = h.center_support(live0, live1, tops, d)
    radius = d["body_radius_m"] + d["body_offset_m"] + d["position_uncertainty_m"]
    support = h.filled_support(g0, g1, tops, radius, d)
    clear, collision_models, _, _ = h.collision_sweep(b0, b1, boxes, d)
    horizontal = float(np.linalg.norm(g1[:2] - g0[:2]))
    dz = abs(float(g1[2] - g0[2]))
    angle = math.atan2(dz, horizontal) if dz or horizontal else 0.
    failures = {"ground_center_not_supported": not center, "live032_center_not_supported": not live,
        "global_chassis_support_not_certified": not support, "chassis_static_clearance_not_certified": not clear,
        "typed_ground_grade_exceeds_limit": bool(grade and angle > d["max_support_slope_rad"])}
    return not any(failures.values()), failures, collision_models, angle


def check_typed_polyline(ground_nav, body_nav, *, metadata=METADATA, config=CONFIG, observation_diagnostics=True):
    context = _context(metadata, config)
    h, d, boxes, uppers, tops, transform, bindings = context
    ground_nav, body_nav = _array(ground_nav, "ground"), _array(body_nav, "body")
    if ground_nav.shape != body_nav.shape or not np.allclose(body_nav-ground_nav, [0, 0, BODY_HEIGHT], rtol=0, atol=1e-12):
        raise ValueError("Typed body must equal genuine ground +.32; no measured-body-as-ground connector")
    ground, body = _nav_to_world(ground_nav, transform), _nav_to_world(body_nav, transform)
    layers = {k: [f for f in uppers if f["model"] in names] for k, names in h.LAYERS.items()}
    layer_boxes = {k: [b for b in boxes if b["model"] in names] for k, names in h.LAYERS.items()}
    counts, examples, diagnostics = Counter(), [], {k: Counter() for k in h.LAYERS}
    checked, max_spacing, diagnostic_examples = 0, 0., []
    for original, (g0, g1, b0, b1) in enumerate(zip(ground[:-1], ground[1:], body[:-1], body[1:])):
        length = max(float(np.linalg.norm(g1-g0)), float(np.linalg.norm(b1-b0)))
        if length == 0:
            continue
        pieces = max(1, math.ceil(length/d["sample_spacing_m"]))
        if checked+pieces > 200000:
            raise ValueError("Bounded sample budget exceeded")
        yaw = math.atan2(float(g1[1]-g0[1]), float(g1[0]-g0[0]))
        for j in range(pieces):
            t0, t1 = j/pieces, (j+1)/pieces
            ga, gb, ba, bb = g0+t0*(g1-g0), g0+t1*(g1-g0), b0+t0*(b1-b0), b0+t1*(b1-b0)
            ok, failed, hit, slope = _physical_segment(ga, gb, ba, bb, context)
            counts.update(k for k, bad in failed.items() if bad)
            checked += 1
            max_spacing = max(max_spacing, float(np.linalg.norm(bb-ba)))
            if not ok and len(examples) < 30:
                examples.append({"edge": original, "fraction": [t0, t1], "ground_world_xyz": ga.tolist(),
                    "body_world_xyz": ba.tolist(), "failures": failed, "collision_models": hit, "ground_grade_rad": slope})
            if observation_diagnostics:
                details = {}
                for layer in layers:
                    provider_ok, diag = h.provider_segment(ga, gb, ba, bb, yaw, layers[layer], d)
                    layer_support = h.filled_support(ga, gb, [f for f in tops if f["model"] in h.LAYERS[layer]], .51, d)
                    robust, robust_diag = h.full_yaw_domain(ba, bb, layers[layer], layer_boxes[layer], d)
                    diagnostics[layer].update({"coverage187_failure": int(not diag["continuous_187_ray_coverage"]),
                        "center_provider_band_failure": int(not diag["center_highest_hit_matches_support_band"]),
                        "provider_upper_band_failure": int(diag["potential_highest_hit_above_body_band"]),
                        "provider_local_chassis_support_failure": int(not layer_support), "actor_full_yaw_outer_domain_failure": int(not robust)})
                    details[layer] = {"provider_segment": diag, "provider_local_support": layer_support,
                                      "actor_full_yaw": robust_diag, "would_pass_old_nominal_predicate": bool(provider_ok and layer_support)}
                if not any(item["would_pass_old_nominal_predicate"] for item in details.values()) and len(diagnostic_examples) < 10:
                    diagnostic_examples.append({"edge": original, "fraction": [t0, t1], "body_world_xyz": ba.tolist(), "details": details})
    if checked == 0 or max_spacing > d["sample_spacing_m"]+d["numeric_epsilon_m"]:
        raise ValueError("No checked segments or subdivision spacing contract violated")
    return {"schema": "r85_typed_polyline_physical_certificate/v1", "allowed": not bool(counts),
        "scope": "Continuous finite-face interval/whole-body outer geometry on this typed polyline; observation predicates do not gate",
        "ready_for_control": False, "navigation_PASS": False, "sources": bindings, "limits": d,
        "physical_body_ground_offset_m": BODY_HEIGHT, "coordinate_frame": "camera_init", "T_navigation_world": transform.tolist(),
        "checked_subsegments": checked, "maximum_spacing_m": max_spacing, "physical_failure_counts": dict(counts), "first_examples": examples,
        "actual_support_plane_slope_max_rad": max(f["slope_rad"] for f in tops),
        "observation_source_diagnostics": {"hard_gate": False, "counts": {k: dict(x) for k, x in diagnostics.items()},
            "examples": diagnostic_examples, "not_actor_behavior_or_training_proof": True},
        "runtime_finite_time_freshness_dynamic_obstacle_and_ack": "NOT_EVALUATED", "source_bindings_unchanged": True}


def prepare(candidate, old_receipt, output_dir):
    candidate, old_receipt, output_dir = Path(candidate).resolve(), Path(old_receipt).resolve(), Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    names = ["typed_r85_global_reference.json", "physical_route_admission.json", "typed_ground_xyz.npy", "typed_body_xyz.npy"]
    if any((output_dir/n).exists() for n in names):
        raise FileExistsError("New typed outputs never overwrite any previous evidence")
    context = _context()
    h, d, _, _, tops, transform, bindings = context
    raw = _project_loads(candidate.read_text())
    old = _project_loads(old_receipt.read_text())
    if raw.get("coordinate_frame") != "camera_init" or old["inputs"]["route"] != source(candidate):
        raise ValueError("Candidate/frame/old-evidence identity mismatch")
    grid_ground = _array(raw["ground_xyz"], "raw PCT ground")
    original_body = _array(raw["body_xyz"], "original PCT body")
    if not np.allclose(original_body-grid_ground, [0, 0, .3], rtol=0, atol=1e-8):
        raise ValueError("Original PCT .30 conversion identity changed")
    task = raw["source_task"]
    task_start, task_goal = np.asarray(task["start_body_xyz"], float), np.asarray(task["goal_body_xyz"], float)
    if task_start.shape != (3,) or task_goal.shape != (3,) or not np.isfinite(np.r_[task_start, task_goal]).all():
        raise ValueError("Task endpoints must remain finite and exact")
    endpoints, endpoint_checks = [], []
    for role, measurement in (("start", task_start), ("goal", task_goal)):
        body_world = _nav_to_world(measurement, transform)
        support_world, evidence = _support_under_body(body_world, h, tops, d)
        planned_body_world = support_world+np.array([0, 0, BODY_HEIGHT])
        # Height residual is checked as body measurement evidence, not encoded
        # into a fictitious terrain slope. No physical height band is relaxed.
        ok, failures, hits, _ = _physical_segment(support_world, support_world, body_world, planned_body_world, context, grade=False)
        if not ok:
            raise ValueError(role + " measured/planned body residual sweep not physically certified: " + str(failures))
        endpoints.append(_world_to_nav(support_world, transform))
        endpoint_checks.append({"role": role, "actual_task_body_nav_xyz": measurement.tolist(),
            "actual_task_body_world_xyz": body_world.tolist(), "actual_finite_support": evidence,
            "ground_nav_xyz": endpoints[-1].tolist(), "planned_body_nav_xyz": _world_to_nav(planned_body_world, transform).tolist(),
            "measurement_residual_sweep_allowed": ok, "collision_models": hits,
            "task_body_and_goal_tolerances_changed": False, "height_residual_not_used_as_ground_grade": True})
    ground = np.vstack((endpoints[0], grid_ground, endpoints[1]))
    body = ground+np.array([0, 0, BODY_HEIGHT])
    certificate = check_typed_polyline(ground, body)
    physical_sources = {"geometry_metadata_file": bindings["metadata"]["path"], "geometry_metadata_sha256": bindings["metadata"]["sha256"],
        "known_scene_reference_file": bindings["reference"]["path"], "known_scene_reference_sha256": bindings["reference"]["sha256"],
        "world_file": bindings["world"]["path"], "world_sha256": bindings["world"]["sha256"],
        "limits_file": bindings["limits"]["path"], "limits_sha256": bindings["limits"]["sha256"],
        "frozen_geometry_helper_file": str(LEGACY), "frozen_geometry_helper_sha256": LEGACY_SHA}
    doc = {"schema": "r85_prepared_typed_global_reference/v1", "coordinate_frame": "camera_init", "frame_id": "camera_init",
        "physical_static_allowed": certificate["allowed"], "ready_for_control": False, "navigation_PASS": False,
        "ground_points": ground.tolist(), "body_points": body.tolist(), "ground_xyz": ground.tolist(), "body_xyz": body.tolist(),
        "baseground_semantics": "Finite support terrain coordinates, not measured body endpoints minus a fixed offset",
        "physical_body_ground_offset_m": BODY_HEIGHT, "T_navigation_world": transform.tolist(), "physical_sources": physical_sources,
        "source_task": task, "task_start_body_xyz": task_start.tolist(), "task_goal_body_xyz": task_goal.tolist(),
        "goal_region_binding": {"status": "PENDING_ROOT_ORIGINAL_TASK_REGION_BINDING", "coordinates_changed": False,
                                "tolerances_changed": False, "source_goal_identity": task["goal_source"]},
        "endpoint_support_connectors": endpoint_checks,
        "representation_correction": {"original_grid_body_offset_m": .3, "new_physical_body_offset_m": BODY_HEIGHT,
            "original_grid_ground_points_changed": False, "measured_task_body_points_used_as_ground": False,
            "new_ground_connectors": "Actual finite support at original endpoint XY to unmodified PCT grid ground endpoints",
            "measurement_body_z_residuals_retained_separately": True},
        "original_raw_connector_evidence": {"receipt": source(old_receipt), "unchanged_status": old["status"],
            "original_failure_count": old["failure_counts"].get("route_grade_exceeds_limit", 0),
            "original_grade_threshold_rad": old["limits"]["max_support_slope_rad"],
            "original_first_failure": old["first_failure_per_reason"].get("route_grade_exceeds_limit"),
            "original_endpoint_connectors": raw["endpoint_connectors_requiring_validation"], "original_evidence_modified": False},
        "source_route_file": str(candidate), "source_route_sha256": source(candidate)["sha256"],
        "physical_certificate_file": str(output_dir/"physical_route_admission.json"),
        "pending_runtime_bindings": ["run_id", "reference_id", "planner_epoch", "localization_generation", "actual_task_goal_region",
                                     "fresh_actual_SLAM_start_connector", "actual_SCAN_curve_certificate", "original_finite_time_and_terrain_ACK_gates"],
        "no_control_authority": True}
    inputs = {"candidate": source(candidate), "old_receipt": source(old_receipt), **bindings}
    for filename, values in (("typed_ground_xyz.npy", ground), ("typed_body_xyz.npy", body)):
        with (output_dir/filename).open("xb") as stream:
            np.save(stream, values, allow_pickle=False)
    with (output_dir/names[0]).open("x") as stream:
        json.dump(doc, stream, indent=2, allow_nan=False); stream.write("\n")
    certificate.update(created_at_utc=datetime.now(timezone.utc).isoformat(), typed_reference=source(output_dir/names[0]),
                       typing_evidence=endpoint_checks, original_raw_grade_failures_preserved=doc["original_raw_connector_evidence"],
                       inputs=inputs, no_physics_ros_model_or_planner_started=True)
    with (output_dir/names[1]).open("x") as stream:
        json.dump(certificate, stream, indent=2, allow_nan=False); stream.write("\n")
    return {"typed_reference": source(output_dir/names[0]), "physical_certificate": source(output_dir/names[1]),
            "physical_static_allowed": certificate["allowed"], "ready_for_control": False}


def _reference(document):
    if isinstance(document,(str,Path)):
        path=Path(document).resolve();return _project_loads(path.read_text()),dict(path=str(path))
    return copy.deepcopy(document),{}


def _connector_support_grade(g0, g1, support_model, context):
    """A direction grade only when one finite face supports the whole footprint.

    This does not move typed ground/body points or certify any other physical
    condition. Otherwise the caller keeps the original typed-chord predicate.
    """
    h, d, _, _, tops, _, _ = context
    proof = dict(schema="r85_actual_connector_terrain_grade/v1", applied=False,
                 fallback="original_typed_ground_chord_grade", support_model=support_model)
    faces = [f for f in tops if f["model"] == support_model]
    if len(faces) != 1:
        return dict(proof, reason="same_finite_support_face_not_unique")
    face = faces[0]
    radius = d["body_radius_m"] + d["body_offset_m"] + d["position_uncertainty_m"]
    lo, hi, z0, z1 = h.face_intervals(g0[None, :2], g1[None, :2], face)
    full_interval = bool(h.covers(lo, hi, d["numeric_epsilon_m"])[0])
    full_footprint = bool(h.filled_support(g0, g1, [face], radius, d))
    if not full_interval or not full_footprint:
        return dict(proof, reason="same_finite_face_does_not_cover_complete_connector_footprint",
                    finite_face_full_interval=full_interval, same_face_full_footprint=full_footprint)
    if not np.isfinite([z0[0], z1[0]]).all():
        return dict(proof, reason="nonfinite_support_plane_height")
    horizontal = float(np.linalg.norm(g1[:2] - g0[:2]))
    height_delta = abs(float(z1[0] - z0[0]))
    angle = math.atan2(height_delta, horizontal) if horizontal or height_delta else 0.
    return dict(proof, applied=True, reason="same_finite_face_and_complete_footprint_proven",
                finite_face_full_interval=True, same_face_full_footprint=True,
                face_interval=[float(lo[0]), float(hi[0])], footprint_radius_m=radius,
                support_face_slope_rad=float(face["slope_rad"]),
                support_plane_endpoint_world_z_m=[float(z0[0]), float(z1[0])],
                typed_ground_endpoint_residual_m=[float(g0[2]-z0[0]), float(g1[2]-z1[0])],
                terrain_direction_grade_rad=angle, original_grade_limit_rad=d["max_support_slope_rad"],
                coordinate_or_tolerance_changed=False)


def certify_start_connector(reference_doc, actual_pose, pose_stamp_ns, localization_generation, planner_epoch=None,
                            *, resume_from_actual_route_projection=False):
    receipt = {"schema": "r85_actual_start_connector/v1", "allowed": False, "ready_for_control": False,
        "navigation_PASS": False, "source_pose_stamp_ns": pose_stamp_ns, "localization_generation": localization_generation,
        "planner_epoch": planner_epoch, "actual_start_body_xyz": list(actual_pose), "physical_space_only": True,
        "requires_original_external_pose_freshness_and_time_gate": True}
    try:
        if type(resume_from_actual_route_projection) is not bool:
            raise ValueError("Resume projection selection must be an explicit boolean")
        doc, binding = _reference(reference_doc)
        receipt.update(run_id=doc["run_id"], reference_id=doc["reference_id"], reference_file=binding.get("path"))
        if not isinstance(pose_stamp_ns, int) or isinstance(pose_stamp_ns, bool) or pose_stamp_ns <= 0:
            raise ValueError("Actual pose stamp must be a positive integer")
        if doc.get("frame_id") != "camera_init" or doc.get("physical_body_ground_offset_m") != BODY_HEIGHT:
            raise ValueError("Reference physical frame/body typing mismatch")
        if localization_generation != doc["localization_generation"] or (planner_epoch is not None and planner_epoch != doc["epoch"]):
            raise ValueError("Actual/reference generation or epoch mismatch")
        if (not isinstance(localization_generation, int) or isinstance(localization_generation, bool)
                or localization_generation < 0 or not isinstance(doc["epoch"], int)
                or isinstance(doc["epoch"], bool) or doc["epoch"] < 0):
            raise ValueError("Generation/epoch must have explicit valid identity types")
        receipt["planner_epoch"] = doc["epoch"]
        physical = doc["physical_sources"]
        context = _context(physical["geometry_metadata_file"], physical["limits_file"])
        h, d, _, _, tops, transform, bindings = context
        if not np.array_equal(np.asarray(doc["T_navigation_world"], float), transform):
            raise ValueError("Reference nav<-world transform mismatch")
        points = _array(doc["body_points"], "reference body")
        grounds = _array(doc["ground_points"], "reference ground")
        if points.shape != grounds.shape or not np.allclose(points-grounds, [0, 0, BODY_HEIGHT], rtol=0, atol=1e-12):
            raise ValueError("Reference body/ground relationship changed")
        receipt.update(reference_scope='current_route',reference_endpoint_body_xyz=points[-1].tolist())
        actual = np.asarray(actual_pose, float)
        if actual.shape != (3,) or not np.isfinite(actual).all():
            raise ValueError("Actual body pose must be finite xyz")
        connector_body, connector_ground = points[0], grounds[0]
        if resume_from_actual_route_projection:
            from embodied_navigation.planning.reference.route_suffix import select_route_suffix
            selected = select_route_suffix(points.tolist(), grounds.tolist(), actual.tolist())
            connector_body, connector_ground = selected['body_points'][0], selected['ground_points'][0]
            receipt['route_resume'] = selected['receipt']
        actual_world, first_world = _nav_to_world(actual, transform), _nav_to_world(connector_body, transform)
        actual_ground, evidence = _support_under_body(actual_world, h, tops, d)
        target_ground = _nav_to_world(connector_ground, transform)
        terrain_ok, failures, hits, grade = _physical_segment(actual_ground, target_ground,
            actual_ground+np.array([0, 0, BODY_HEIGHT]), first_world, context)
        # Actual body connector itself must also be clear/supported; measured
        # residual is not a fictitious terrain derivative/grade measurement.
        body_ok, body_failures, body_hits, _ = _physical_segment(actual_ground, target_ground, actual_world, first_world, context, grade=False)
        original_failures = dict(failures)
        grade_proof = dict(schema="r85_actual_connector_terrain_grade/v1", applied=False,
            fallback="original_typed_ground_chord_grade", reason="original_other_physical_condition_failed")
        if body_ok and not any(bad for key, bad in failures.items() if key != "typed_ground_grade_exceeds_limit"):
            grade_proof = _connector_support_grade(actual_ground, target_ground, evidence["model"], context)
            if grade_proof["applied"]:
                failures = {k: v for k, v in failures.items() if k != "typed_ground_grade_exceeds_limit"}
                failures["terrain_direction_grade_exceeds_limit"] = (
                    grade_proof["terrain_direction_grade_rad"] > d["max_support_slope_rad"])
                terrain_ok = not any(failures.values())
        receipt.update(allowed=bool(terrain_ok and body_ok), reason="typed_ground_and_actual_body_connector_certified" if terrain_ok and body_ok else "physical_connector_not_certified",
            target_first_body_xyz=list(connector_body), actual_finite_support=evidence,
            actual_start_ground_world_xyz=actual_ground.tolist(), target_ground_world_xyz=target_ground.tolist(),
            physical_failure_conditions=failures, actual_body_failure_conditions=body_failures,
            collision_models=sorted(set(hits+body_hits)), typed_ground_grade_rad=grade,
            original_physical_failure_conditions=original_failures,
            connector_grade_evaluation=grade_proof,
            sources=bindings, reference_binding=binding, existing_doc_allowed_flag_used=False,
            body_measurement_residual_not_encoded_as_ground_grade=True)
    except Exception as error:
        receipt.update(allowed=False, reason="connector_contract_rejected", error=type(error).__name__+": "+str(error))
    return receipt


def _split(control):
    a = (control[:-1]+control[1:])/2
    b = (a[:-1]+a[1:])/2
    c = (b[0]+b[1])/2
    return np.array([control[0], a[0], b[0], c]), np.array([c, b[1], a[2], control[3]])


def _curve_patch(control, context, duration):
    h, d, boxes, _, tops, _, _ = context
    ground = control-np.array([0, 0, BODY_HEIGHT])
    radius = .51
    qlo, qhi = ground[:, :2].min(0)-radius, ground[:, :2].max(0)+radius
    rectangles = []
    same_face = None
    for face in tops:
        if face["slope_rad"] > d["max_support_slope_rad"]:
            continue
        plane_z = face["center"][2]-(ground[:, :2]-face["center"][:2])@face["n"][:2]/face["n"][2]
        # Affine residual extrema at Bezier controls bound all parameter values.
        if np.max(np.abs(plane_z-ground[:, 2])) > d["support_height_tolerance_m"]-d["numeric_epsilon_m"]:
            continue
        a, b = np.maximum(qlo, face["xy_lo"]), np.minimum(qhi, face["xy_hi"])
        if np.all(a <= b):
            rectangles.append((a, b))
        # The whole Bezier XY hull plus the original body footprint must lie
        # on this ONE finite face. The affine control residual bound above
        # already covers every curve parameter, not sampled center points.
        if same_face is None and np.all(qlo >= face["xy_lo"]) and np.all(qhi <= face["xy_hi"]):
            same_face = face
    support = h.union_rect_covers(qlo, qhi, rectangles, 0.)
    lo, hi = control.min(0)-np.array([.51, .51, .20]), control.max(0)+np.array([.51, .51, .20])
    collision_models = [b["model"] for b in boxes if not h.aabb_obb_disjoint(lo, hi, b, d["numeric_epsilon_m"])]
    # Normalized-u derivative suffices for a direction/grade cone and avoids
    # overflow/underflow from extreme physical-time knot intervals.
    derivative = 3*np.diff(ground, axis=0)
    heading = derivative[:, :2].sum(0)
    if np.linalg.norm(heading) == 0:
        heading = derivative[int(np.argmax(np.linalg.norm(derivative[:, :2], axis=1))), :2]
    if np.linalg.norm(heading) == 0:
        grade = bool(np.all(derivative[:, 2] == 0))
    else:
        heading = heading/np.linalg.norm(heading)
        parallel = derivative[:, :2]@heading
        # A convex derivative cone includes exact stationary zero controls.
        exact_zero = np.all(derivative == 0, axis=1)
        if np.all(derivative[:, 2] == 0):
            grade = True  # Structurally flat, never an approximate-zero shortcut.
        else:
            margin = d["numeric_epsilon_m"]*(1+float(np.abs(derivative).max()))
            residual = math.tan(d["max_support_slope_rad"])*parallel-np.abs(derivative[:, 2])
            grade = bool(np.all(exact_zero | ((parallel >= margin) & (residual >= margin))))
    terrain_grade = grade
    grade_evaluation = None
    vertical_only = np.all(control[:, :2] == control[0, :2]) and np.ptp(control[:, 2]) > 0
    if not grade and not vertical_only and support and not collision_models and same_face is not None:
        # Body Z residual/convergence is not the derivative of the terrain.
        # A finite plane's full slope bounds EVERY tangent direction; keep the
        # original limit and all original body-height/collision hull checks.
        terrain_grade = bool(same_face["slope_rad"] <= d["max_support_slope_rad"])
        grade_evaluation = {"schema": "same_finite_face_curve_terrain_grade/v1", "applied": terrain_grade,
            "model": same_face["model"], "finite_face_full_patch_and_footprint": True,
            "footprint_radius_m": radius, "original_body_residual_bound_preserved": True,
            "terrain_max_direction_grade_rad": float(same_face["slope_rad"]),
            "original_grade_limit_rad": d["max_support_slope_rad"],
            "original_typed_ground_grade_cone_certified": grade}
    return bool(support and not collision_models and terrain_grade), {"whole_body_support_certified": bool(support),
        "chassis_clearance_certified": not collision_models, "collision_outer_bound_models": collision_models,
        "typed_ground_grade_cone_certified": grade, "terrain_grade_certified": terrain_grade,
        "terrain_grade_evaluation": grade_evaluation}


def certify_physical_curve(payload, *, geometry_metadata=METADATA, config=CONFIG, max_patches=4096, max_depth=18, wall_budget_s=2.):
    """Static continuous certificate for an exact cubic SCAN body-nav payload.

    Supports order/pos_pts/knots. All actual IDs/time bindings are preserved as
    evidence; this API supplies space checks and never a runtime control PASS.
    """
    started = time.monotonic()
    result = {"schema": "r85_continuous_scan_physical_certificate/v1", "allowed": False, "ready_for_control": False,
        "navigation_PASS": False, "physical_space_only": True, "status": "CONTRACT_REJECTED",
        "method": "Exact cubic spans to Bezier; control-hull finite-support union/SAT/derivative-cone; bounded de Casteljau",
        "runtime_finite_time_freshness_dynamic_and_ack": "EXTERNAL_ORIGINAL_GATES_REQUIRED"}
    try:
        context = _context(geometry_metadata, config)
        _, _, _, _, _, transform, bindings = context
        body = _array(payload["pos_pts"], "curve control points", minimum=4)
        knots = np.asarray(payload["knots"], float)
        if payload.get("frame_id", payload.get("coordinate_frame")) != "camera_init" or payload["order"] != 3:
            raise ValueError("Only actual cubic body-nav camera_init curves are supported")
        if knots.shape != (len(body)+4,) or not np.isfinite(knots).all() or np.any(np.diff(knots) < 0):
            raise ValueError("Invalid/nonfinite cubic knot vector")
        begin, end = float(knots[3]), float(knots[-4])
        if end <= begin or max_patches < 1 or max_depth < 0 or not math.isfinite(wall_budget_s) or wall_budget_s <= 0:
            raise ValueError("Empty curve or invalid bounded compute budget")
        interior = knots[(knots > begin) & (knots < end)]
        if len(interior) and np.unique(interior, return_counts=True)[1].max() > 3:
            raise ValueError("Discontinuous cubic interior knot multiplicity")
        from scipy.interpolate import PPoly
        world = _nav_to_world(body, transform)
        pp = [PPoly.from_spline((knots, world[:, axis], 3)) for axis in range(3)]
        stack = []
        for i, (a, b) in enumerate(zip(pp[0].x[:-1], pp[0].x[1:])):
            if b <= a or a < begin or b > end:
                continue
            dt = float(b-a)
            control = np.empty((4, 3))
            for axis in range(3):
                aa, bb, cc, dd = pp[axis].c[:, i]
                control[:, axis] = [dd, dd+cc*dt/3, dd+2*cc*dt/3+bb*dt*dt/3, dd+cc*dt+bb*dt*dt+aa*dt*dt*dt]
            if not np.isfinite(control).all():
                raise ValueError("Nonfinite cubic-to-Bezier conversion")
            if np.all(control[:, :2] == control[0, :2]) and np.ptp(control[:, 2]) > 0:
                raise ValueError("Nonconstant vertical-only typed ground curve violates grade contract")
            stack.append((control, float(a), float(b), 0))
        if not stack:
            raise ValueError("No positive-length cubic spans")
        processed, certified, deepest = 0, 0, 0
        terrain_grade_reclassified_patches, last_terrain_grade_evaluation = 0, None
        while stack:
            if processed >= max_patches or time.monotonic()-started > wall_budget_s:
                result.update(status="UNKNOWN_REJECTED", reason="patch_or_wall_budget_exhausted", pending_intervals=len(stack)); break
            control, a, b, depth = stack.pop()
            processed += 1; deepest = max(deepest, depth)
            ok, checks = _curve_patch(control, context, b-a)
            if checks["terrain_grade_evaluation"] is not None and checks["terrain_grade_evaluation"]["applied"]:
                terrain_grade_reclassified_patches += 1
                last_terrain_grade_evaluation = dict(checks["terrain_grade_evaluation"], parameter_interval=[a, b])
            if ok:
                certified += 1; continue
            if depth >= max_depth:
                result.update(status="UNKNOWN_REJECTED", reason="continuous_outer_bound_not_certified", unproved_interval=[a, b], checks=checks); break
            left, right = _split(control)
            mid = (a+b)/2
            stack.extend(((right, mid, b, depth+1), (left, a, mid, depth+1)))
        else:
            result.update(status="CERTIFIED_STATIC_SPACE_ONLY", reason="all_continuous_spans_certified")
        result.update(sources=bindings, coordinate_frame="camera_init",
            T_navigation_world=transform.tolist(), curve_parameter_interval=[begin, end], physical_body_ground_offset_m=BODY_HEIGHT,
            processed_patches=processed, certified_patches=certified, deepest_subdivision=deepest,
            terrain_grade_semantics="same_finite_face_max_direction_bound_else_original_typed_cone/v1",
            terrain_grade_reclassified_patches=terrain_grade_reclassified_patches,
            last_terrain_grade_evaluation=last_terrain_grade_evaluation,
            identity_fields={k: payload.get(k) for k in ("run_id", "request_id", "traj_id", "epoch", "localization_generation", "source_stamp_ns", "start_time")},
            budgets={"max_patches": max_patches, "max_depth": max_depth, "wall_budget_s": wall_budget_s},
            arithmetic="float64 engineering bounds; not formal exact-arithmetic proof",
            limitations=["No live time/freshness/dynamic/actual terrain-ACK proof", "No foot contact/load/gait proof",
                         "AABB/rectangle/cone refusal is not an actual collision/support-loss witness", "Zero derivative does not certify a heading or dynamic followability"])
        result["allowed"] = result["status"] == "CERTIFIED_STATIC_SPACE_ONLY"
    except Exception as error:
        result.update(allowed=False, status="CONTRACT_REJECTED", error=type(error).__name__+": "+str(error), reason="invalid_curve_or_bound_sources")
    result["elapsed_wall_ms"] = (time.monotonic()-started)*1000
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    prepare_parser = sub.add_parser("prepare")
    prepare_parser.add_argument("--candidate", type=Path, default=STAGE/"run_f1_to_f3_02/route_candidate.json")
    prepare_parser.add_argument("--old-receipt", type=Path, default=STAGE/"run_f1_to_f3_02/second_support_validation_01.json")
    prepare_parser.add_argument("--output-dir", type=Path, required=True)
    curve_parser = sub.add_parser("curve")
    curve_parser.add_argument("--curve-json", type=Path, required=True)
    curve_parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.operation == "prepare":
        result = prepare(args.candidate, args.old_receipt, args.output_dir)
    else:
        if args.output.exists():
            raise FileExistsError("New continuous receipts never overwrite old evidence")
        result = certify_physical_curve(_project_loads(args.curve_json.read_text()))
        with args.output.open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
