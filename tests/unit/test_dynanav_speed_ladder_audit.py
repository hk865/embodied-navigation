"""Speed-rung evidence tests use constructed files, no SDK/ROS or guard edits."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPT=Path(__file__).resolve().parents[2]/'scripts/audit_dynanav_speed_ladder.py'
spec=importlib.util.spec_from_file_location('speed_ladder_audit',SCRIPT)
audit=importlib.util.module_from_spec(spec);sys.modules[spec.name]=audit;spec.loader.exec_module(audit)


class SpeedLadderTests(unittest.TestCase):
    def setUp(self):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        self.root=Path(directory.name);self.capture=self.root/'run/capture';self.capture.mkdir(parents=True)
        self.experiment={'speed_mps':1.,'route':'basement-to-ground','control_mode':'measured_slam_route',
            'waypoints_xyz':[[0,0,0]],'physical_speed_guard_mps':1.5,
            'joint_speed_readback_tolerance_radps':.001,'imu_publish_hz':200}
        self.result={'experiment':self.experiment,'physics_dt':.005,'policy':{'policy_hz':50},
            'ground_truth_navigation':False,'SLAM_control_enabled':True,'stair_speed_cap_enabled':False,
            'end_reason':'navigation_terminal_and_parked',
            'route_tracker':{'arrived':True,'stage':2,'events':[{'event':'turn_completed','stage':0},{'event':'turn_completed','stage':1}]}}
        self.commands=[{'measured_time':.015+i*.02,'actual_command':[speed,0.,0.]}
                       for i,speed in enumerate([0.,1.,.5,1.,0.,0.])]
        self.truth=[{'time':round(.02+i*.005,6),'velocity':[.4,0.,.3],'position':[0.,0.,0.]}
                    for i in range(23)]
        self.controls=[{'current_sim_s':row['measured_time'],'stage':min(i//2,2)} for i,row in enumerate(self.commands)]
        self.save()

    def save(self):
        for name,value in [('experiment.json',self.experiment),('result.json',self.result)]:
            (self.capture/name).write_text(json.dumps(value))
        for name,rows in [('policy_frames.jsonl',self.commands),('truth_scoring_stream.jsonl',self.truth),
                          ('slam_route_control.jsonl',self.controls)]:
            (self.capture/name).write_text(''.join(json.dumps(row)+'\n' for row in rows))

    def test_actual_speed_and_exact_command_selection_not_target_as_measurement(self):
        result=audit.audit_run(self.capture)
        cruise=result['cruise_actual_forward_command_selection']
        self.assertEqual(result['target_command_mps'],1.)
        self.assertEqual(cruise['selected_command_records'],2)
        self.assertAlmostEqual(cruise['nonzero_command_duration_s'],.04)
        self.assertAlmostEqual(cruise['measured_duration_s'],.04)
        self.assertAlmostEqual(cruise['xy']['average_mps'],.4)
        self.assertAlmostEqual(cruise['three_dimensional']['average_mps'],.5)
        self.assertNotEqual(cruise['xy']['average_mps'],result['target_command_mps'])
        whole=result['entire_native_journal']
        self.assertAlmostEqual(whole['xy']['average_mps'],.4)
        self.assertAlmostEqual(whole['three_dimensional']['average_mps'],.5)
        self.assertAlmostEqual(whole['measured_duration_s'],.115)
        self.assertIn('startup',whole['selection'])
        self.assertEqual(result['stream_validation']['command_control_exact_timestamp_pairs'],6)
        self.assertEqual(result['observed_stages_from_control_journal'],[0,1,2])
        self.assertTrue(result['reported_full_three_stage_measured_closedloop_completion'])
        # Source-reported completion does not substitute a physical parking test.
        self.assertAlmostEqual(result['zero_actual_command_tail']['native_motion']['three_dimensional']['average_mps'],.5)
        self.assertIn('descriptive',result['zero_actual_command_tail']['scope'])

    def test_GT_first_flight_cannot_become_full_SLAM_rung(self):
        self.result['ground_truth_navigation']=True;self.result['SLAM_control_enabled']=False
        self.experiment['route']='first-flight';self.experiment['control_mode']='native_pose_diagnostic'
        self.controls=[];self.result['route_tracker']['stage']=0;self.result['route_tracker']['events']=[]
        self.save();result=audit.audit_run(self.capture)
        self.assertFalse(result['reported_full_three_stage_measured_closedloop_completion'])
        self.assertEqual(result['comparison_signature_without_target_speed']['route'],'first-flight')
        self.assertTrue(result['comparison_signature_without_target_speed']['ground_truth_navigation'])

    def test_failure_evidence_retains_small_readback_and_missing_exit_code(self):
        self.result.update(error='Joint speed exceeds Go2HV motor envelope',end_reason=None,
                           actuator_failure_state={'physics_s':9.64,'qd_isaac':[-30.000137329], 'dof_names':['FR_calf']})
        self.result['route_tracker']['arrived']=False;self.save()
        result=audit.audit_run(self.capture);failure=result['failure_evidence']
        self.assertEqual(failure['category'],'recorded_joint_speed_guard')
        self.assertIsNone(failure['exit_codes'])
        self.assertAlmostEqual(failure['actuator_failure']['maximum_absolute_joint_speed_radps'],30.000137329)
        self.assertFalse(result['reported_full_three_stage_measured_closedloop_completion'])

    def test_missing_target_is_not_inferred_and_conflicting_target_rejected(self):
        self.assertEqual(audit.target_speed({},{}),(None,{'status':'missing','sources':[]}))
        with self.assertRaisesRegex(ValueError,'conflicting'):
            audit.target_speed({'experiment':{'speed_mps':.6}},{'speed_mps':.3})
        for target in [float('nan'),float('inf'),True,-1]:
            with self.subTest(target=target),self.assertRaises(ValueError):
                audit.target_speed({}, {'speed_mps':target})

    def test_invalid_stream_does_not_claim_full_completion_or_extrapolate(self):
        self.truth=self.truth[:8];self.commands.append({'measured_time':.15,'actual_command':[float('nan'),0,0]})
        self.save();result=audit.audit_run(self.capture)
        self.assertFalse(result['reported_full_three_stage_measured_closedloop_completion'])
        self.assertEqual(result['stream_validation']['invalid_ordered_commands'],1)
        cruise=result['cruise_actual_forward_command_selection']
        self.assertLess(cruise['measured_duration_s'],cruise['nonzero_command_duration_s'])
        self.assertLess(cruise['duration_coverage_fraction'],1)

    def test_comparison_differences_and_exclusive_output(self):
        left=audit.audit_run(self.capture);right={**left,'comparison_signature_without_target_speed':
            {**left['comparison_signature_without_target_speed'],'ground_truth_navigation':True}}
        diff=audit.compare_signatures([left,right])
        self.assertIn('ground_truth_navigation',diff['different_recorded_configuration_fields'])
        output=self.root/'newreport'
        self.assertEqual(audit.main(['--run',str(self.capture),'--output',str(output)]),0)
        report=(output/'speed-ladder-evidence.json').read_bytes()
        self.assertEqual(audit.main(['--run',str(self.capture),'--output',str(output)]),2)
        self.assertEqual(report,(output/'speed-ladder-evidence.json').read_bytes())


if __name__=='__main__':unittest.main()
