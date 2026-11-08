"""Additional output restriction before the unchanged physical guard.
No source, controller state, path or stop predicate is changed here.
"""
import math
import numpy as np

def constrain_exactzero_launch(previous,desired,original,mode,rotation_xy):
    p,d,o=(np.asarray(v,dtype=float) for v in (previous,desired,original))
    r=np.asarray(rotation_xy,dtype=float)
    if any(v.shape!=(3,) or not np.isfinite(v).all() for v in (p,d,o)) or r.shape!=(2,2) or not np.isfinite(r).all():
        raise ValueError('launch constraint requires finite exact current vectors')
    if not isinstance(mode,str) or not mode:raise ValueError('launch mode invalid')
    out=o.copy();alpha=None;applied=mode=='drive' and bool(np.all(p[:2]==0.))
    reason='outside_exactzero_drive_launch'
    if applied:
        if d[0]<=0.:
            alpha=0.;out[:2]=0.;reason='no_positive_forward_launch_translation'
        else:
            if any(o[i]*d[i]<0. for i in (0,1)):
                raise ValueError('original launch output has opposing direction')
            ratios=[abs(o[i])/abs(d[i]) for i in (0,1) if d[i]!=0.]
            alpha=min([1.,*ratios])
            if not math.isfinite(alpha) or not 0.<=alpha<=1.:raise ValueError('launch scale invalid')
            out[:2]=alpha*d[:2]
            for _ in range(4):
                if np.all(np.abs(out[:2])<=np.abs(o[:2])):break
                alpha=math.nextafter(alpha,0.);out[:2]=alpha*d[:2]
            if not np.all(np.abs(out[:2])<=np.abs(o[:2])):raise ValueError('launch rounding would enlarge original output')
            reason='uniform_direction_preserving_body_cap'
            # This is the existing world-direction update boundary. Do not
            # publish a tiny translation using another guard direction.
            if np.linalg.norm(r@out[:2])<=1e-9:
                alpha=0.;out[:2]=0.;reason='below_original_world_direction_update_boundary'
    receipt=dict(schema='exactzero_launch_output_constraint/v1',applied=applied,
        reason=reason,alpha=alpha,previous_actual_command=p.tolist(),
        desired_body_command=d.tolist(),original_after_slew=o.tolist(),
        constrained_body_command=out.tolist(),yaw_changed=False,
        source_times_refreshed=False,new_motion_authorization=False,
        original_native_guard_still_required=True)
    return out,receipt
