"""Ordinary terminal ownership after original SLAM/IMU/ACK gates, never T0.

The owner uses only fresh measured navigation pose/motion. It neither changes
route success nor starts the formal parking window, and never retries a failed
zero hold. Native physical motion remains independently checked by acceptance.
"""
import math
import json
from embodied_navigation.control.parking.source_hold import PREQUALIFICATION_SOURCE_HOLD_ENABLED, UNSPECIFIED_ZERO_EVENT, eligible as prequalification_source_hold_eligible


DIAGNOSTIC_BUSINESS_FAILURES = frozenset((
    'parking_fixed_target_lost',
    'capture_fixed_goal_out_of_range',
    'ordinary_actual_zero_ACK_timeout',
    'ordinary_precise_capture_deceleration_timeout',
    'ordinary_zero_hold_actual_motion_or_drift_exceeded',
))


def diagnostic_business_failure(reason):
    """Exact business reasons only; source/ACK/interrupt faults are never scores."""
    return type(reason) is str and reason in DIAGNOSTIC_BUSINESS_FAILURES


def _diagnostic_enabled(diagnostic):
    if not isinstance(diagnostic, dict) or diagnostic.get('run') is None:
        return False
    profile = diagnostic.get('profile')
    if not isinstance(profile, dict):
        return False
    value = profile.get('diagnostic_observe_only')
    return (isinstance(value, dict) and type(value.get('enabled')) is bool
            and value['enabled'] is True)
import numpy as np

class OrdinaryParking:
    def __init__(self):
        self.prequalification_source_hold_enabled=PREQUALIFICATION_SOURCE_HOLD_ENABLED
        self.prequalification_source_hold_count=0;self.prequalification_source_hold_first=None;self.prequalification_source_hold_last=None
        self.enabled=False;self.stage=None;self.dwell=None;self.failure=None
        self.start_clock=None;self.deadline=None;self.ready_stamp=None;self.ready_clock=None
        self.handoff_clock=None;self.origin=None;self.origin_yaw=None
        self.first_zero_publication_clock=None;self.first_actor_zero_ack=None
        self.preT0_attempts_disqualified=0;self.preT0_first_disqualification=None;self.preT0_last_disqualification=None
        self.deceleration_trigger=None
        self.diagnostic_business_failure_reason=None
        self.diagnostic_business_run=None
        self.diagnostic_business_observation_count=0
        self.diagnostic_last_business_rejection=None
        self.pose10hz=False;self.capture_grid=None;self.pose_grid=None
        self.capture_goal=None;self.capture_heading=None
        self.pose10hz_last_source=None

    def configure_pose10hz(self,value):
        from embodied_navigation.control.parking.parking_pose10hz import configuration, CaptureGrid
        self.pose10hz=configuration(value)
        if self.pose10hz:self.capture_grid=CaptureGrid()

    def start_pose10hz_window(self,proof):
        if not self.pose10hz:return
        from embodied_navigation.control.parking.parking_pose10hz import PoseGrid
        if self.pose_grid is not None:raise ValueError('Never reset the first 10Hz parking anchor')
        origin=proof['qualification_origin']
        self.pose_grid=PoseGrid(proof['qualification_clock_ns'],origin)
        self.origin=np.asarray(origin['position'],float).copy()
        self.origin_yaw=self.pose_grid.base_yaw

    def observe_pose10hz_source(self,row):
        """Every accepted commit feeds one persistent grid; no positive handoff."""
        if not self.pose10hz or not self.enabled:return
        from embodied_navigation.control.parking.parking_pose10hz import pose
        stamp,p,yaw=pose(row)
        if self.pose10hz_last_source is not None:
            old_stamp,old_p,old_yaw=pose(self.pose10hz_last_source)
            if stamp<old_stamp:raise ValueError('10Hz committed SLAM clock went backwards')
            if stamp==old_stamp:
                if (p,yaw)!=(old_p,old_yaw):raise ValueError('10Hz repeated committed header changed pose')
                return
        if self.pose_grid is not None:self.pose_grid.observe(row)
        else:self.capture_grid.observe(row,self.capture_goal,self.capture_heading)
        self.pose10hz_last_source=row

    def invalidate_pose10hz_capture(self):
        if self.pose10hz and self.pose_grid is None:
            from embodied_navigation.control.parking.parking_pose10hz import CaptureGrid
            self.capture_grid=CaptureGrid();self.pose10hz_last_source=None

    def update_pose10hz(self,stamp,clock,position,yaw,goal_distance,yaw_error,sensor_pose):
        if sensor_pose is None:raise ValueError('10Hz parking needs its exact original SLAM pose record')
        self.observe_pose10hz_source(sensor_pose)
        if self.pose_grid is not None:
            if self.pose_grid.missing:
                self.failure='parking_pose10hz_data_unavailable';self.stage='failed'
            elif self.pose_grid.motion_failure:
                self.failure='parking_pose10hz_fixed_anchor_exceeded';self.stage='failed'
            else:self.stage='zero_hold'
            return self.stage
        if clock>self.deadline and not(getattr(self,'daily_navigation',False)and self.ready_stamp is not None):
            self.failure='ordinary_actual_zero_ACK_timeout' if self.ready_stamp is not None else 'ordinary_precise_capture_deceleration_timeout'
            self.stage='failed';return self.stage
        if self.capture_goal is None:raise ValueError('10Hz capture requires the fixed goal')
        ready=self.capture_grid.ready
        self.dwell=self.capture_grid.since
        if self.ready_stamp is not None:
            self.stage='zero_hold';return self.stage
        if ready:
            selected=self.capture_grid.last
            self.ready_stamp=selected['source_stamp_ns'];self.ready_clock=clock;self.handoff_clock=clock
            self.origin=np.asarray(selected['position'],float).copy();self.origin_yaw=selected['yaw_rad']
            self.stage='handoff_zero'
        else:
            # This controls deceleration only; positive qualification comes from
            # the 10Hz geometric capture above, never instantaneous motion.
            band=.020 if self.stage=='controlled_deceleration' else .015
            self.stage='controlled_deceleration' if goal_distance<=band and abs(yaw_error)<=.035 else 'capture'
        return self.stage

    def begin(self,clock,timeout_ns):
        if self.enabled:return False
        if type(clock)is not int or type(timeout_ns)is not int or timeout_ns<=0:
            raise ValueError('Original integer parking clock/deadline required')
        self.enabled=True;self.stage='capture';self.start_clock=clock;self.deadline=clock+timeout_ns
        return True

    def diagnostic_failure_is_observed(self,diagnostic):
        return (diagnostic_business_failure(self.failure)
                and self.failure == self.diagnostic_business_failure_reason
                and _diagnostic_enabled(diagnostic)
                and str(diagnostic['run']) == self.diagnostic_business_run)

    def observe_business_rejection(self,reason,stamp,clock,evidence,diagnostic,*,
                                   terminal_zero_event=UNSPECIFIED_ZERO_EVENT):
        """Record before execution permission; never erase formal failure facts."""
        if not _diagnostic_enabled(diagnostic):return False
        if not diagnostic_business_failure(reason):return False
        if self.failure and not self.diagnostic_failure_is_observed(diagnostic):
            return False
        if (type(stamp)is not int or stamp<0 or type(clock)is not int or clock<stamp):
            raise ValueError('Original causal integer parking observation clocks required')
        # The shared ledger deduplicates before JSON encoding. Validate this
        # observation even when the preceding same-reason row is <1s old.
        json.dumps(evidence,sort_keys=True,allow_nan=False)
        if prequalification_source_hold_eligible(self,reason,terminal_zero_event):
            # Returning False invokes each caller's original strict _protect /
            # guard path. No failure, source, deadline, dwell or T0 is edited here.
            self.prequalification_source_hold_count+=1
            self.prequalification_source_hold_last=dict(reason=reason,original_stage=self.stage,
                source_pose_stamp_ns=stamp,control_stamp_ns=clock,
                goal_distance_xy_m=evidence.get('goal_distance_xy_m'),
                goal_z_error_m=evidence.get('goal_z_error_m'),goal_yaw_error_rad=evidence.get('goal_yaw_error_rad'),
                original_goal_z_tolerance_m=evidence.get('original_goal_z_tolerance_m'),
                queued_pose_checked=evidence.get('queued_pose_checked',False),
                action='original_strict_protection_required',positive_qualification_granted=False)
            if self.prequalification_source_hold_first is None:
                self.prequalification_source_hold_first=dict(self.prequalification_source_hold_last)
            return False
        from embodied_navigation.common.observation import api as _observer_api; record = _observer_api('diagnostic_observation').record
        recorded=record(diagnostic['run'],diagnostic['profile'],'ordinary_terminal_parking_business',reason,
            dict(original_stage=self.stage,original_failure=reason,
                 source_pose_stamp_ns=stamp,control_stamp_ns=clock,
                 original_first_failure=self.failure,measurements=evidence,
                 formal_parking_pass=False,navigation_PASS=False))
        if recorded is not True:
            raise RuntimeError('Business parking observation was not recorded')
        self.failure=self.failure or reason
        self.diagnostic_business_failure_reason=self.failure
        self.diagnostic_business_run=str(diagnostic['run'])
        self.diagnostic_business_observation_count+=1
        self.diagnostic_last_business_rejection=dict(reason=reason,source_pose_stamp_ns=stamp,
            control_stamp_ns=clock,formal_parking_pass=False)
        return True

    def recapture_before_first_qualification(self,record):
        """Retire an unqualified zero attempt; the original capture clock survives."""
        if (self.first_actor_zero_ack is not None or self.failure is not None
                or self.ready_stamp is None or self.stage not in ('handoff_zero','zero_hold')):
            return False
        self.preT0_attempts_disqualified=min(self.preT0_attempts_disqualified+1,2**63-1)
        if self.preT0_first_disqualification is None:self.preT0_first_disqualification=record
        self.preT0_last_disqualification=record
        self.ready_stamp=self.ready_clock=self.handoff_clock=None
        self.first_zero_publication_clock=None;self.origin=self.origin_yaw=None
        self.dwell=None;self.stage='capture';self.deceleration_trigger=None
        self.invalidate_pose10hz_capture()
        return True

    def guard(self,reason):
        if not self.enabled:return
        if self.ready_stamp is not None:
            # A new actual protection supersedes observed business execution
            # permission. Its first business fact remains in the ledger/marker.
            if (diagnostic_business_failure(self.failure)
                    and self.failure == self.diagnostic_business_failure_reason):
                self.failure='ordinary_zero_hold_interrupted: '+str(reason)
            else:
                self.failure=self.failure or 'ordinary_zero_hold_interrupted: '+str(reason)
            self.stage='failed'
        else:
            self.stage='capture';self.dwell=None;self.invalidate_pose10hz_capture()

    def update(self,stamp,clock,position,yaw,velocity,yawdot,body_wz,goal_distance,yaw_error,
               *,diagnostic=None,sensor_pose=None):
        if not self.enabled:return None
        if self.failure and not self.diagnostic_failure_is_observed(diagnostic):return 'failed'
        if self.pose10hz:
            return self.update_pose10hz(stamp,clock,position,yaw,goal_distance,yaw_error,sensor_pose)
        if _diagnostic_enabled(diagnostic):
            # A score-only exception never admits malformed measured state.
            p,v=np.asarray(position,dtype=float),np.asarray(velocity,dtype=float)
            scalars=np.asarray([yaw,yawdot,body_wz,goal_distance,yaw_error],dtype=float)
            if (type(stamp)is not int or stamp<0 or type(clock)is not int or clock<stamp
                    or p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(p).all()
                    or not np.isfinite(v).all() or not np.isfinite(scalars).all()
                    or goal_distance<0):
                raise ValueError('Finite causal actual parking measurements required')
        if self.ready_stamp is not None:
            if clock>self.deadline and self.first_actor_zero_ack is None:
                if not self.observe_business_rejection('ordinary_actual_zero_ACK_timeout',stamp,clock,
                        dict(deadline_clock_ns=self.deadline,first_actor_zero_ack_missing=True),diagnostic):
                    self.failure='ordinary_actual_zero_ACK_timeout';self.stage='failed';return self.stage
            xy=float(np.linalg.norm(np.asarray(position)[:2]-self.origin[:2]))
            dyaw=abs(math.atan2(math.sin(yaw-self.origin_yaw),math.cos(yaw-self.origin_yaw)))
            if (xy>.05 or dyaw>.1 or np.linalg.norm(np.asarray(velocity)[:2])>.08 or
                    abs(yawdot)>.1 or abs(body_wz)>.1):
                if self.observe_business_rejection('ordinary_zero_hold_actual_motion_or_drift_exceeded',stamp,clock,
                        dict(xy_drift_m=xy,yaw_drift_rad=dyaw,velocity_body=np.asarray(velocity).tolist(),
                             euler_yawrate_radps=yawdot,body_wz_radps=body_wz),diagnostic):
                    self.stage='zero_hold'
                else:
                    self.failure='ordinary_zero_hold_actual_motion_or_drift_exceeded'
                    self.stage='failed'
            else:self.stage='zero_hold'
            return self.stage
        if clock>self.deadline:
            if not self.observe_business_rejection('ordinary_precise_capture_deceleration_timeout',stamp,clock,
                    dict(deadline_clock_ns=self.deadline,goal_distance_xy_m=goal_distance,
                         yaw_error_rad=yaw_error,velocity_body=np.asarray(velocity).tolist(),
                         euler_yawrate_radps=yawdot),diagnostic):
                self.failure='ordinary_precise_capture_deceleration_timeout';self.stage='failed';return self.stage
        close=goal_distance<=.025 and abs(yaw_error)<=.035
        low=np.linalg.norm(np.asarray(velocity)[:2])<.03 and abs(yawdot)<.06
        # Trigger hysteresis is inside the unchanged strict25mm qualification.
        was_decelerating=self.stage=='controlled_deceleration'
        band=.020 if was_decelerating else .015
        braking_band=goal_distance<=band and abs(yaw_error)<=.035
        self.stage='controlled_deceleration' if braking_band else 'capture'
        self.deceleration_trigger=dict(entry_xy_m=.015,exit_xy_m=.020,
            was_decelerating=was_decelerating,current_xy_limit_m=band,
            actual_goal_distance_m=float(goal_distance),inside=bool(braking_band),
            qualification_xy_m=.025,qualification_changed=False)
        self.dwell=(stamp if self.dwell is None else self.dwell) if close and low else None
        if self.dwell is not None and stamp-self.dwell>=600_000_000:
            self.ready_stamp=stamp;self.ready_clock=clock;self.handoff_clock=clock
            self.origin=np.asarray(position,float).copy();self.origin_yaw=float(yaw)
            self.stage='handoff_zero'
        return self.stage

    def evidence(self):
        result=dict(schema='ordinary_terminal_parking_stages/v1',enabled=self.enabled,stage=self.stage,
            capture_begin_clock_ns=self.start_clock,capture_deadline_clock_ns=self.deadline,
            precise_dwell_start_pose_stamp_ns=self.dwell,ready_pose_stamp_ns=self.ready_stamp,
            ready_clock_ns=self.ready_clock,handoff_clock_ns=self.handoff_clock,
            first_zero_publication_clock_ns=self.first_zero_publication_clock,
            first_actor_zero_ack=self.first_actor_zero_ack,failure=self.failure,
            controlled_deceleration_trigger=self.deceleration_trigger,
            formal_T0_set_here=False,navigation_ground_truth_used=False,
            diagnostic_business_failure_reason=self.diagnostic_business_failure_reason,
            diagnostic_business_run=self.diagnostic_business_run,
            diagnostic_business_observation_count=self.diagnostic_business_observation_count,
            diagnostic_last_business_rejection=self.diagnostic_last_business_rejection,
            formal_parking_acceptance_invalid=self.diagnostic_business_failure_reason is not None)
        if self.preT0_attempts_disqualified:
            result['preT0_attempts_disqualified']=self.preT0_attempts_disqualified
            result['preT0_first_disqualification']=self.preT0_first_disqualification
            result['preT0_last_disqualification']=self.preT0_last_disqualification
        if self.prequalification_source_hold_enabled:
            result['r106_prequalification_source_hold']=dict(
                prequalification_source_hold_enabled=self.prequalification_source_hold_enabled,
                count=self.prequalification_source_hold_count,
                first=None if self.prequalification_source_hold_first is None else dict(self.prequalification_source_hold_first),
                last=None if self.prequalification_source_hold_last is None else dict(self.prequalification_source_hold_last))
        if self.pose10hz:
            result['parking_rule']='ordinary_pose10hz_zero_ack_fixed5/v1'
            if self.pose_grid is not None:
                summary=self.pose_grid.evidence();summary.pop('samples',None)
                result['pose10hz_window']=summary
            result['high_frequency_motion_used_for_decision']=False
        return result
