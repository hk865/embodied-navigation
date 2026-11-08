"""Prepare a private original cascade path binding before callback promotion.

This owns no command, receipt, source gate or control algorithm. Capture and
startup without the same original goal keep their original lazy binding.
"""
import copy
import hashlib
import numpy as np

def staged_cascade_binding(node, receipt):
    cascade=node.cascade
    key=(receipt['request_id'],receipt['waypoint_index'])
    if cascade is None or cascade.capture_stamp is not None or node.cascade_key!=key:
        return None
    if key!=(node.request_id,node.waypoint_index) or receipt['trajectory_id']!=node.active_trajectory_id:
        raise ValueError('admitted_path_execution_identity_changed')
    stamp=int(node.active_reference_stamp[0])*1_000_000_000+int(node.active_reference_stamp[1])
    if receipt['stamp_ns']!=stamp:
        raise ValueError('admitted_path_reference_identity_changed')
    points=np.asarray(receipt['points_xyz'],dtype='<f8')
    if points.ndim!=2 or points.shape[1]!=3 or not 2<=len(points)<=2000 or not np.isfinite(points).all():
        raise ValueError('Path points must be finite Nx3 within capacity')
    prepared=copy.deepcopy(cascade)
    prepared.set_path(receipt['points_xyz'],receipt['path_id'],receipt['stamp_ns'],
        receipt['received_wall_ns'],receipt['frame_id'],position=node.pose)
    if prepared.path_id!=receipt['path_id'] or prepared.path_record['points_xyz']!=receipt['points_xyz']:
        raise ValueError('admitted_path_original_binding_incomplete')
    row=dict(**prepared.path_record,path_sha256=prepared.path_sha,
        trajectory_id=receipt['trajectory_id'],array_file=receipt['array_file'],
        source_sample_indices=receipt['source_sample_indices'],source_samples_count=receipt['source_samples_count'],
        source_samples_float64_sha256=receipt['source_samples_float64_sha256'],
        points_float64_sha256=receipt['points_float64_sha256'],navigation_ground_truth_used=False)
    return prepared,row
