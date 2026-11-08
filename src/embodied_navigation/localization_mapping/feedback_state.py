"""Canonical state constructed after the corrected SLAM body commit is accepted.

This is not a truth-to-SLAM converter. Callers must already own the verified
SLAM commit; simulation truth has a different state/clock contract.
"""

def accepted_slam_feedback(*, stamp_ns, received_wall_ns, position_world_xyz,
                           quaternion_wxyz, origin_velocity_body, callback_ros_clock_ns):
    return dict(stamp_ns=stamp_ns,received_wall_ns=received_wall_ns,frame_id="camera_init",
                position_world_xyz=list(position_world_xyz),quaternion_wxyz=list(quaternion_wxyz),
                origin_velocity_body=list(origin_velocity_body),callback_ros_clock_ns=callback_ros_clock_ns,
                navigation_ground_truth_used=False,pose_producer="fast_livo2_async_backend")
