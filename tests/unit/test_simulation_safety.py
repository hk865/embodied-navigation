"""NumPy-only contract and exact legacy-expression safety regression fixtures."""
from dataclasses import replace
import math
import unittest
import numpy as np
from embodied_navigation.common.state_contract import (
    GAZEBO_NATIVE64_SOURCE, GAZEBO_SIMULATION_CLOCK, StateContractError,
    StateOrigin, StatePurpose, validate_state_provenance,
)
from embodied_navigation.simulation.safety_state import from_gazebo_native64
from embodied_navigation.control.safety.simulation_gate import SimulationSafetyGate


def legacy_guard(state, first, ground, prior_fault, fall_started, policy_finite):
    """Independent verbatim numerical guard from the original native loop."""
    t=float(state[0]-first);w,x,y,z=state[4:8]
    angles=[math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(np.clip(2*(w*y-z*x),-1,1)),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
    contacts=state[50:55].tolist();fault=prior_fault
    if not policy_finite:fault=fault or 'nonfinite_policy'
    if state[56]>0:fault=fault or f'actuator_fault_{int(state[56])}'
    fall=bool(t>1.5 and (max(abs(angles[0]),abs(angles[1]))>.8 or state[3]-ground<.15))
    body=bool(t>1.5 and contacts[0]>0)
    if fall:fault=fault or 'navigation_physical_fall_or_low_clearance'
    persistent=bool(t>2. and (max(abs(angles[0]),abs(angles[1]))>1.1 or state[3]-ground<.10))
    if not persistent:fall_started=None
    elif fall_started is None:fall_started=t
    end=bool(fall_started is not None and t-fall_started>=4.)
    return dict(fault=fault,fall=fall,body=body,persistent=persistent,fall_started=fall_started,
                end=end,stop=bool(end or (fault is not None and t>=2)))


def native(t, *, tilt=0., clearance=.32, body_contact=0., actuator=0., first=10.):
    state=np.zeros(64);state[0]=first+t;state[3]=clearance
    state[4:8]=[math.cos(tilt/2),math.sin(tilt/2),0.,0.]
    state[50]=body_contact;state[56]=actuator
    return state


def observed(state, first=10., ground=0., run_id='fixture'):
    return from_gazebo_native64(state,first_world_time_s=first,ground_height_m=ground,run_id=run_id)


class SimulationSafety(unittest.TestCase):
    def assert_matches(self, decision, reference):
        self.assertEqual(decision.fault,reference['fault'])
        self.assertEqual(decision.observed_fall_or_low,reference['fall'])
        self.assertEqual(decision.observed_body_contact,reference['body'])
        self.assertEqual(decision.persistent_fall,reference['persistent'])
        self.assertEqual(decision.fall_started_s,reference['fall_started'])
        self.assertEqual(decision.persistent_fall_observation_end,reference['end'])
        self.assertEqual(decision.stop,reference['stop'])
        self.assertTrue(decision.state_contract_valid)

    def test_randomized_finite_native64_legacy_equivalence(self):
        rng=np.random.default_rng(1126)
        # Independent frames exercise fault priority before any latch masks it.
        for _ in range(3000):
            state=rng.normal(size=64);state[0]=10+rng.uniform(-1,10)
            state[4:8]/=np.linalg.norm(state[4:8]);state[56]=rng.choice([-1,0,0,0,1,6])
            ground=float(rng.uniform(-1,1));finite=bool(rng.integers(0,2))
            prior='ordinary_zero_observation_contract_rejected' if rng.integers(0,7)==0 else None
            gate=SimulationSafetyGate(run_id='fixture')
            ref=legacy_guard(state,10.,ground,prior,None,finite)
            self.assert_matches(gate.evaluate(observed(state,ground=ground),policy_finite=finite,prior_fault=prior),ref)
        # Sequential stateful replay verifies latch and timer reset behavior.
        for _ in range(40):
            gate=SimulationSafetyGate(run_id='fixture');fault=None;started=None
            for t in np.arange(0.,12.,.02):
                state=native(float(t),tilt=float(rng.uniform(-1.5,1.5)),clearance=float(rng.uniform(.05,.4)),body_contact=float(rng.integers(0,2)))
                ref=legacy_guard(state,10.,0.,fault,started,True)
                self.assert_matches(gate.evaluate(observed(state),prior_fault=fault),ref)
                fault=ref['fault'];started=ref['fall_started']

    def test_grace_threshold_boundaries_and_priority(self):
        for t in [0.,1.5,np.nextafter(1.5,np.inf),1.50001,2.,2.00001,6.00001]:
            for tilt in [.8,np.nextafter(.8,np.inf),.80001,1.1,1.10001]:
                for clearance in [.15,np.nextafter(.15,0),.14999,.10,np.nextafter(.10,0),.09999]:
                    state=native(t,tilt=tilt,clearance=clearance,body_contact=1)
                    ref=legacy_guard(state,10.,0.,None,None,True)
                    self.assert_matches(SimulationSafetyGate(run_id='fixture').evaluate(observed(state)),ref)
        cases=[('existing',False,3.,'existing'),(None,False,3.,'nonfinite_policy'),(None,True,3.,'actuator_fault_3'),(None,True,0.,'navigation_physical_fall_or_low_clearance')]
        for prior,finite,code,expected in cases:
            gate=SimulationSafetyGate(run_id='fixture')
            result=gate.evaluate(observed(native(1.6,tilt=1.2,actuator=code)),policy_finite=finite,prior_fault=prior)
            self.assertEqual(result.fault,expected);self.assertFalse(result.stop)
            result=gate.evaluate(observed(native(2.)))
            self.assertEqual(result.fault,expected);self.assertTrue(result.stop)

    def test_body_contact_is_diagnostic_not_a_new_stop(self):
        result=SimulationSafetyGate(run_id='fixture').evaluate(observed(native(3.,body_contact=1)))
        self.assertTrue(result.observed_body_contact);self.assertFalse(result.observed_fall_or_low)
        self.assertIsNone(result.fault);self.assertFalse(result.stop)

    def test_persistent_fall_four_seconds_and_reset(self):
        gate=SimulationSafetyGate(run_id='fixture');started=None;fault=None
        for t,tilt in [(2.,1.2),(2.25,1.2),(6.249,1.2),(6.25,1.2),(6.3,0.),(6.4,1.2),(10.4,1.2)]:
            state=native(t,tilt=tilt);ref=legacy_guard(state,10.,0.,fault,started,True)
            result=gate.evaluate(observed(state));self.assert_matches(result,ref)
            started=ref['fall_started'];fault=ref['fault']
        self.assertTrue(result.persistent_fall_observation_end)

    def test_simulation_truth_never_admitted_for_navigation(self):
        p=observed(native(0.)).provenance
        kwargs=dict(expected_source=GAZEBO_NATIVE64_SOURCE,expected_clock_domain=GAZEBO_SIMULATION_CLOCK,expected_run_id='fixture')
        validate_state_provenance(p,purpose=StatePurpose.EVALUATION,**kwargs)
        with self.assertRaisesRegex(StateContractError,'navigation_requires_slam_estimate'):
            validate_state_provenance(p,purpose=StatePurpose.NAVIGATION,**kwargs)
        with self.assertRaisesRegex(StateContractError,'cannot_be_relabelled'):
            validate_state_provenance(replace(p,origin=StateOrigin.SLAM_ESTIMATE),purpose=StatePurpose.NAVIGATION,**kwargs)

    def test_missing_mismatched_source_clock_run_timestamp_fail_closed(self):
        state=observed(native(.2))
        changes=[dict(source=''),dict(source='other.simulator'),dict(clock_domain=''),dict(clock_domain='monotonic_wall'),dict(run_id='another_run'),dict(run_id=''),dict(sample_stamp_ns=None),dict(frame_stamp_ns=True),dict(sample_stamp_ns=state.provenance.sample_stamp_ns+1),dict(frame_stamp_ns=state.provenance.frame_stamp_ns+1),dict(origin=StateOrigin.REAL_SENSOR)]
        for change in changes:
            gate=SimulationSafetyGate(run_id='fixture')
            result=gate.evaluate(replace(state,provenance=replace(state.provenance,**change)))
            self.assertTrue(result.stop,change);self.assertFalse(result.state_contract_valid,change)
            self.assertEqual(result.fault,'simulation_safety_state_contract_rejected')
        self.assertTrue(SimulationSafetyGate(run_id='fixture').evaluate(None).stop)

    def test_factory_copies_true_clock_and_rejects_nonfinite(self):
        raw=native(.2);state=observed(raw)
        self.assertEqual(state.provenance.sample_stamp_ns,round((raw[0]-.005)*1e9))
        self.assertEqual(state.provenance.frame_stamp_ns,round(raw[0]*1e9))
        raw[50]=123.;self.assertEqual(state.contacts[0],0.)
        for raw,ground in [(raw[:63],0.),(np.full(64,np.nan),0.),(native(0.),float('nan'))]:
            with self.assertRaises(StateContractError):observed(raw,ground=ground)

    def test_earlier_policy_fault_survives_terrain_or_contract_failure(self):
        gate=SimulationSafetyGate(run_id='fixture')
        self.assertEqual(gate.latch_runtime_faults(policy_finite=False,actuator_fault_code=4.),'nonfinite_policy')
        result=gate.reject_state('terrain_contract_failure')
        self.assertEqual(result.fault,'nonfinite_policy');self.assertTrue(result.stop)
        self.assertFalse(result.state_contract_valid)


if __name__=='__main__':unittest.main()
