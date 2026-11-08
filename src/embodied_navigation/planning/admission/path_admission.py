"""Bounded continuous cubic-curve admission against the frozen SLAM route tube.

This proves route-fence containment only. Occupancy, source freshness, heading,
arrival, Teacher and parking protections remain independent runtime gates.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import BSpline, PPoly

MAX_CONTROL_POINTS = 512
MAX_ROUTE_POINTS = 4096
MAX_PATCHES = 2048
MAX_DEPTH = 24
MAX_WALL_S = .020
FP_REL_MARGIN = 1e-8


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def replayable(value):
    """Keep explicit nonfinite tokens in rejected JSON without invalid JSON."""
    if isinstance(value, dict):return {str(k):replayable(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):return [replayable(v) for v in value]
    if isinstance(value, (float,np.floating)) and not math.isfinite(value):
        return {'nonfinite_float': 'NaN' if math.isnan(value) else '+Infinity' if value>0 else '-Infinity'}
    if isinstance(value,np.generic):return value.item()
    return value


def payload(msg):
    return dict(order=int(msg.order),traj_id=int(msg.traj_id),
        start_time=[int(msg.start_time.sec),int(msg.start_time.nanosec)],
        pos_pts=[[float(p.x),float(p.y),float(p.z)] for p in msg.pos_pts],
        knots=list(map(float,msg.knots)))


def candidate(msg, metadata, reference_stamp, goal):
    value=payload(msg)
    if (not isinstance(metadata,dict) or metadata.get('schema')!=1
            or metadata.get('reference_stamp')!=list(reference_stamp or [])
            or metadata.get('trajectory')!=value):
        raise ValueError('candidate_reference_or_payload_mismatch')
    expected=np.asarray(goal,dtype=float);measured=np.asarray(metadata.get('body_goal'),dtype=float)
    if measured.shape!=(3,) or not np.isfinite(measured).all() or not np.allclose(measured,expected,rtol=0,atol=1e-8):
        raise ValueError('candidate_goal_mismatch')
    return value


class FrozenFence:
    def __init__(self, run, profile):
        self.run=Path(run).resolve();self.profile=profile;self.anchor_sha=None;self.registration_sha=None
    def load(self):
        path=self.run/'navigation_anchor.json'
        if path.stat().st_size>2_000_000:raise ValueError('anchor_size_budget')
        raw=path.read_bytes()
        if len(raw)>2_000_000:raise ValueError('anchor_size_budget')
        sha=hashlib.sha256(raw).hexdigest()
        if self.anchor_sha is not None and sha!=self.anchor_sha:raise ValueError('frozen_anchor_changed')
        anchor=json.loads(raw)
        if (anchor.get('schema')!=1 or anchor.get('run_dir')!=str(self.run)
                or anchor.get('frame_id')!='camera_init' or anchor.get('source')!='/demo/slam/body_odom'
                or anchor.get('frozen_once') is not True or anchor.get('ground_truth_navigation_used') is not False
                or anchor.get('original_region_count')!=46
                or anchor.get('registered_route_reference')!='original46 centers with single actualSLAM/IMU yaw and initial measured origin'):
            raise ValueError('invalid_actual_SLAM_frozen_anchor')
        if anchor.get('scene_axis_registration_file')!='navigation_scene_axis_registration.json':
            raise ValueError('unbound_registration_file')
        regpath=self.run/'navigation_scene_axis_registration.json'
        if regpath.stat().st_size>2_000_000:raise ValueError('registration_size_budget')
        regraw=regpath.read_bytes()
        if len(regraw)>2_000_000:raise ValueError('registration_size_budget')
        regsha=hashlib.sha256(regraw).hexdigest();registration=json.loads(regraw)
        if (regsha!=anchor.get('scene_axis_registration_file_sha256')
                or self.registration_sha is not None and regsha!=self.registration_sha
                or registration!=anchor.get('scene_axis_registration')
                or registration.get('schema')!='teacher_scene_axis_registration/v1'
                or registration.get('status')!='frozen' or registration.get('ground_truth_navigation_used') is not False):
            raise ValueError('invalid_frozen_actual_sensor_registration')
        initial=anchor.get('initial_actual_slam_source') or {}
        origin=np.asarray(anchor.get('origin'),dtype=float);yaw=float(anchor['yaw'])
        if (origin.shape!=(3,) or not np.isfinite(origin).all() or not math.isfinite(yaw)
                or initial.get('source_topic')!='/demo/slam/body_odom'
                or initial.get('frame_id')!='camera_init' or initial.get('child_frame_id')!='demo_slam_body'
                or initial.get('navigation_ground_truth_used') is not False
                or initial.get('stamp_ns')!=anchor.get('pose_stamp_ns')
                or initial.get('position')!=anchor.get('origin')
                or yaw!=registration['heading_receipt']['yaw_camera_init_from_world']):
            raise ValueError('invalid_initial_actual_SLAM_pose')
        c,s=math.cos(yaw),math.sin(yaw)
        scenario=self.profile['original_scenario']
        world_prior=[point for name in ('exploration','return_origin','navigation_f1_f3') for point in scenario[name]]
        if len(world_prior)!=46:raise ValueError('original46_centers_required')
        expected=[origin.tolist()]+[[origin[0]+c*p[0]-s*p[1],origin[1]+s*p[0]+c*p[1],origin[2]+p[2]] for p in world_prior]
        points=anchor.get('registered_route_camera_init_xyz')
        if points!=expected or digest(points)!=anchor.get('registered_route_points_sha256'):
            raise ValueError('registered_original46_route_hash_or_geometry_mismatch')
        self.anchor_sha=sha;self.registration_sha=regsha
        config=self.profile['registered_route_fence']
        return points,config,{'anchor_sha256':sha,'registration_sha256':regsha,
            'route_points_sha256':digest(points),'frame_id':'camera_init','navigation_ground_truth_used':False}


def split_bezier(control):
    a=(control[:-1]+control[1:])*.5;b=(a[:-1]+a[1:])*.5;c=(b[0]+b[1])*.5
    return np.asarray([control[0],a[0],b[0],c]),np.asarray([c,b[1],a[2],control[3]])


@np.errstate(over='raise', invalid='raise', divide='raise')
def certify_curve(value, points, config, *, begin_override=None, max_patches=MAX_PATCHES,
                  max_depth=MAX_DEPTH, wall_budget_s=MAX_WALL_S, max_route_points=256):
    """Prove every parameter interval; inconclusive or out-of-budget rejects."""
    started=time.monotonic();deadline=started+wall_budget_s
    result={'schema':'continuous_cubic_route_fence/v1','allowed':False,'reason':None,
        'method':'Bezier convex hull ball; all possible nearest3D segments D_i<=D_min+2r; component residuals1-Lipschitz; deCasteljau subdivision',
        'patch_budget':max_patches,'depth_budget':max_depth,'wall_budget_s':wall_budget_s,
        'route_point_budget':max_route_points,
        'floating_point_relative_margin':FP_REL_MARGIN,'processed_patches':0,'certified_patches':0,
        'maximum_depth':0,'navigation_ground_truth_used':False,'occupancy_or_terrain_support_certified':False}
    def finish(reason, allowed=False, **extra):
        return dict(result,reason=reason,allowed=allowed,elapsed_wall_ms=(time.monotonic()-started)*1000,**extra)
    try:
        if (not math.isfinite(wall_budget_s) or wall_budget_s<=0 or max_patches<1 or max_depth<0
                or type(max_route_points) is not int or not 2<=max_route_points<=MAX_ROUTE_POINTS
                or len(value['pos_pts'])>MAX_CONTROL_POINTS or len(value['knots'])>MAX_CONTROL_POINTS+4):
            return finish('input_or_compute_budget_invalid')
        coefficients=np.asarray(value['pos_pts'],dtype=float);knots=np.asarray(value['knots'],dtype=float)
        k=value['order'];route=np.asarray(points,dtype=float)
        result['actual_route_point_count']=len(route) if route.ndim else None
        horizontal=float(config['horizontal_radius_m']);vertical=float(config['height_error_m'])
        if (k!=3 or coefficients.ndim!=2 or coefficients.shape[1]!=3 or not 4<=len(coefficients)<=MAX_CONTROL_POINTS
                or knots.shape!=(len(coefficients)+4,) or not np.isfinite(coefficients).all()
                or not np.isfinite(knots).all() or np.any(np.diff(knots)<0)
                or route.ndim!=2 or route.shape[1]!=3 or not 2<=len(route)<=max_route_points or not np.isfinite(route).all()
                or not math.isfinite(horizontal+vertical) or min(horizontal,vertical)<=0):
            return finish('invalid_or_nonfinite_curve_or_fence')
        begin,end=float(knots[3]),float(knots[-4])
        if end<=begin:return finish('empty_curve')
        if begin_override is not None:
            if not math.isfinite(begin_override) or not begin<=begin_override<=end:return finish('invalid_remaining_interval')
            begin=float(begin_override)
        # Interior multiplicity4 permits an actual jump, not a continuous route.
        interior=knots[(knots>knots[3]) & (knots<end)]
        if len(interior) and max(np.unique(interior,return_counts=True)[1])>3:return finish('discontinuous_cubic')
        a=route[:-1];delta=route[1:]-a;length2=np.sum(delta*delta,axis=1)
        if not np.isfinite(delta).all() or not np.isfinite(length2).all():return finish('nonfinite_segment_geometry')
        valid=length2>=1e-12
        # Deduplicate geometrically identical directed segments, preserving first index/ties.
        _,indices=np.unique(np.hstack((a[valid],delta[valid])),axis=0,return_index=True)
        indices=np.flatnonzero(valid)[np.sort(indices)];a=a[indices];delta=delta[indices];length2=length2[indices]
        if not len(a):return finish('degenerate_route')
        scale=1.+max(float(np.max(np.abs(coefficients))),float(np.max(np.abs(route))))
        margin=FP_REL_MARGIN*scale
        if not math.isfinite(margin):return finish('nonfinite_numeric_margin')
        result.update(nondegenerate_unique_segments=len(a),numeric_margin_m=margin,
                      curve_parameter_interval_s=[begin,end])
        def distances(point):
            u=np.clip(np.sum((point-a)*delta,axis=1)/length2,0.,1.)
            residual=point-(a+u[:,None]*delta)
            if not np.isfinite(u).all() or not np.isfinite(residual).all():raise FloatingPointError('nonfinite_projection')
            values=(np.linalg.norm(residual,axis=1),np.linalg.norm(residual[:,:2],axis=1),np.abs(residual[:,2]))
            if not all(np.isfinite(item).all() for item in values):raise FloatingPointError('nonfinite_distance')
            return values
        polynomials=[PPoly.from_spline((knots,coefficients[:,axis],3)) for axis in range(3)]
        if time.monotonic()>=deadline:return finish('wall_budget_exhausted')
        if not all(np.isfinite(poly.c).all() for poly in polynomials):return finish('nonfinite_power_coefficients')
        patches=[]
        for i,(left,right) in enumerate(zip(knots[:-1],knots[1:])):
            if time.monotonic()>=deadline:return finish('wall_budget_exhausted')
            left=max(float(left),begin);right=min(float(right),end)
            if right<=left:continue
            h=right-left;shift=left-float(knots[i])
            power=np.asarray([poly.c[:,i] for poly in polynomials]).T
            aa,bb,cc,dd=power
            # Shift the power origin, then convert the exact cubic span to Bezier.
            dd=((aa*shift+bb)*shift+cc)*shift+dd
            cc=(3*aa*shift+2*bb)*shift+cc;bb=bb+3*aa*shift
            control=np.asarray([dd,dd+cc*h/3,dd+2*cc*h/3+bb*h*h/3,
                dd+cc*h+bb*h*h+aa*h*h*h])
            if not np.isfinite(control).all():return finish('nonfinite_extraction')
            patches.append((control,left,right,0))
        if not patches:
            # Remaining endpoint: a degenerate point patch still must satisfy fence.
            point=BSpline(knots,coefficients,3)(end)
            if not np.isfinite(point).all():return finish('nonfinite_endpoint')
            patches=[(np.tile(point,(4,1)),end,end,0)]
        while patches:
            if result['processed_patches']>=max_patches:return finish('patch_budget_exhausted')
            if time.monotonic()>=deadline:return finish('wall_budget_exhausted')
            control,left,right,depth=patches.pop();result['processed_patches']+=1
            result['maximum_depth']=max(result['maximum_depth'],depth)
            center=(control[0]+3*control[1]+3*control[2]+control[3])/8
            radius=float(np.max(np.linalg.norm(control-center,axis=1)))+margin
            if not np.isfinite(center).all() or not math.isfinite(radius):return finish('nonfinite_patch_bounds')
            distance,h,z=distances(center)
            possible=distance<=float(distance.min())+2*radius+margin
            if not np.any(possible):return finish('empty_possible_nearest_segments')
            if np.all(h[possible]+radius+margin<horizontal) and np.all(z[possible]+radius+margin<vertical):
                result['certified_patches']+=1;continue
            # Center is an actual curve midpoint, so it can establish a rejection witness.
            nearest=int(np.argmin(distance))
            if h[nearest]>horizontal+margin or z[nearest]>vertical+margin:
                return finish('outside_fence',witness={'parameter_s':(left+right)/2,
                    'camera_init_xyz':center.tolist(),'original_segment_index':int(indices[nearest]),
                    'horizontal_error_m':float(h[nearest]),'height_error_m':float(z[nearest])})
            if depth>=max_depth:return finish('depth_budget_exhausted')
            l,r=split_bezier(control);mid=(left+right)*.5
            if not np.isfinite(l).all() or not np.isfinite(r).all() or not math.isfinite(mid):return finish('nonfinite_subdivision')
            patches.extend(((r,mid,right,depth+1),(l,left,mid,depth+1)))
        return finish('continuous_fence_certified',True)
    except (KeyError,ValueError,TypeError,OverflowError,IndexError,FloatingPointError) as error:
        return finish('invalid_curve:'+type(error).__name__+':'+str(error))


def remaining_certificate(active, samples, pose, points, config, *, max_route_points=256,business_observer=None):
    """Recheck old same-goal remainder and actual-body connector, never time progress."""
    values=np.asarray(samples);position=np.asarray(pose,dtype=float)
    if (values.ndim!=2 or values.shape[1]!=3 or not 2<=len(values)<=2000 or not np.isfinite(values).all()
            or position.shape!=(3,) or not np.isfinite(position).all()):
        return {'allowed':False,'reason':'missing_measured_remaining_projection'}
    distances=np.linalg.norm(values[:,:2]-position[:2],axis=1)
    if business_observer is not None and not np.isfinite(distances).all():
        return {'allowed':False,'reason':'nonfinite_measured_remaining_projection'}
    nearest=int(np.argmin(distances))
    value=active['payload'];knots=value['knots'];begin,end=knots[3],knots[-4]
    parameter=begin+(end-begin)*nearest/max(1,len(values)-1)
    proof=certify_curve(value,points,config,begin_override=parameter,max_route_points=max_route_points)
    remaining_observed=False
    business_reasons={'outside_fence','depth_budget_exhausted','patch_budget_exhausted','wall_budget_exhausted'}
    if not proof['allowed']:
        if business_observer is None or proof.get('reason')not in business_reasons:return proof
        json.dumps(proof,allow_nan=False)
        if business_observer(proof)is not True:return proof
        remaining_observed=True
    target=BSpline(value['knots'],value['pos_pts'],3)(parameter)
    if business_observer is not None and not np.isfinite(target).all():
        return {'allowed':False,'reason':'nonfinite_actual_body_connector_target'}
    connector=dict(order=3,pos_pts=[(position+(target-position)*u).tolist() for u in (0,1/3,2/3,1)],knots=[0,0,0,0,1,1,1,1])
    bridge=certify_curve(connector,points,config,max_route_points=max_route_points)
    connector_observed=False
    if (not bridge['allowed'] and business_observer is not None
            and bridge.get('reason')in business_reasons):
        json.dumps(bridge,allow_nan=False)
        connector_observed=business_observer(bridge)is True
    allowed=bridge['allowed'] or connector_observed
    result={'allowed':allowed,'reason':'old_remaining_and_body_connector_certified' if allowed else 'old_body_connector_not_certified',
        'remaining':proof,'connector':bridge,'measured_nearest_sample_index':nearest,
        'remaining_begin_parameter_s':parameter,'occupancy_or_support_certificate':False}
    if remaining_observed or connector_observed:
        result.update(reason='diagnostic_retained_route_prediction_continued'if allowed else'old_body_connector_not_certified',
            diagnostic_execution_continued=bool(allowed),original_remaining_prediction_passed=proof['allowed'],
            original_connector_prediction_passed=bridge['allowed'],navigation_PASS=False)
    return result
