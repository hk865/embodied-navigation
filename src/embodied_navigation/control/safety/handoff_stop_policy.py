"""Restrict old-path driving at an incompatible nonfixed tail; no motion grant."""
import math

def tail_stop_request(proof,handoff,spatial_config,pose_ns,cloud_ns,*,capture_active=False,failure_active=False):
    old=(handoff or {}).get('old_reference',{})
    try:
        remaining=float(old['remaining_m']);arc=float(spatial_config['arc_length_m'])
        finite=math.isfinite(remaining) and math.isfinite(arc)
    except (KeyError,TypeError,ValueError,OverflowError):remaining=arc=None;finite=False
    conditions=dict(original_geometry=proof.get('original_geometry_allowed')is True,
        original_ordered_leg=proof.get('ordered_current_leg',{}).get('allowed')is True,
        original_direction=proof.get('direction_coherence',{}).get('allowed')is True,
        spatial_reference_enabled=spatial_config.get('enabled')is True,
        original_rejection=proof.get('allowed')is False and proof.get('reason')=='handoff_not_ready',
        required=handoff is not None and handoff.get('required')is True,
        incompatible=handoff is not None and handoff.get('heading_compatible')is False,
        replacement_context=handoff is not None and handoff.get('eligibility',{}).get('replacement_allowed')is True,
        nonfixed_endpoint=old.get('endpoint_is_fixed_goal')is False,
        original_short_tail=finite and 0<remaining<=arc,
        unchanged_pose=handoff is not None and type(pose_ns)is int and pose_ns>0 and handoff.get('source_pose_stamp_ns')==pose_ns,
        unchanged_cloud=handoff is not None and type(cloud_ns)is int and cloud_ns>0 and handoff.get('source_cloud_stamp_ns')==cloud_ns,
        no_capture=capture_active is False,no_failure=failure_active is False)
    return dict(schema='nonfixed_tail_original_stop_request/v1',required=all(conditions.values()),
        conditions=conditions,remaining_m=remaining,original_preview_arc_m=arc,
        source_pose_stamp_ns=pose_ns,source_cloud_stamp_ns=cloud_ns,
        path_id=old.get('path_id'),actual_source_times_refreshed=False,
        stop_request_only=True,path_replacement_authorized=False,motion_authorized=False,
        Teacher_transition_bypassed=False)
