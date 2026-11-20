"""Recover invalid static-exploration frames without weakening accepted bindings."""
from pathlib import Path
import tempfile
import json
from unittest.mock import patch
import unittest
import numpy as np
from test_dynanav_camera_binding import SdkCallbackTests, rig, ready


class NativeCameraRecoveryTests(unittest.TestCase):
    sdk = SdkCallbackTests.sdk
    native = SdkCallbackTests.native
    def test_native_global_reference_anomaly_drops_then_original_next_frame_is_accepted(self):
        r=rig();r.recover_invalid_native_frames=True
        _,c,evaluated,guards,parsed=self.sdk(r);r.attach_rtx_writer()
        with tempfile.TemporaryDirectory() as out:
            r.quarantine_output=Path(out)
            r.reference_ann.value={'referenceTimeNumerator':1,'referenceTimeDenominator':10}
            with patch('time.perf_counter',return_value=42.):c['native']['on_event'](self.native())
            self.assertIsNone(r.rtx_error);self.assertFalse(r._binding_stopped)
            self.assertFalse(r.rtx_cache);self.assertEqual(r.native_frame_drop_count,1)
            self.assertEqual([g.resets for g in guards],[0,0,0])
            diag=json.loads((Path(out)/'camera_frame_validation.jsonl').read_text())
            self.assertEqual(diag['reference_time'],[2,10]);self.assertEqual(diag['global_reference']['referenceTimeNumerator'],1)
            self.assertFalse(diag['valid']);self.assertFalse(diag['simulator_termination_requested'])
            parsed[1]=3;ready(r,3,30)
            with patch('time.perf_counter',return_value=99.):c['native']['on_event'](self.native(30,3))
            c['update']['on_event']({})
            self.assertEqual(len(r.rtx_cache),1)
            f=r.rtx_cache[0];self.assertEqual(f['time_s'],.3);self.assertEqual(f['reference_time'],[3,10]);self.assertEqual(f['arrival_wall_s'],99.)
            self.assertFalse(f['time_evidence']['body_now_substituted']);self.assertEqual(len(evaluated),2)

    def test_recovery_keeps_binding_checks_and_missing_pixels_out_of_queue(self):
        for fault in ('rp_time','pointer','missing_rgb'):
            with self.subTest(fault=fault):
                r=rig();r.recover_invalid_native_frames=True
                _,c,_,guards,parsed=self.sdk(r);r.attach_rtx_writer();ready(r,2,20)
                c['native']['on_event'](self.native())
                if fault=='rp_time':ready(r,1,20)
                if fault=='pointer':r.test_ptr.value=99
                if fault=='missing_rgb':r.rgb_ann.value=None
                c['update']['on_event']({})
                self.assertFalse(r.rtx_cache);self.assertIsNone(r.rtx_error)
                self.assertFalse(r._pending_native_frames);self.assertEqual(r.native_frame_drop_count,1)
                self.assertEqual([g.resets for g in guards],[0,0,0])
                parsed[1]=3;ready(r,3,30);r.rgb_ann.value=np.zeros((512,512,4),np.uint8)
                c['native']['on_event'](self.native(30,3));c['update']['on_event']({})
                self.assertEqual([f['time_s'] for f in r.rtx_cache],[.3])
                self.assertEqual(r.rtx_cache[0]['time_evidence']['binding_before'],r.rtx_cache[0]['time_evidence']['binding_after'])

    def test_nonexploration_global_reference_failure_keeps_original_strict_behavior(self):
        r=rig();_,c,_,guards,_=self.sdk(r);r.attach_rtx_writer()
        r.reference_ann.value={'referenceTimeNumerator':1,'referenceTimeDenominator':10}
        c['native']['on_event'](self.native())
        self.assertIsNotNone(r.rtx_error);self.assertIn('global reference',str(r.rtx_error))
        self.assertTrue(r._binding_stopped);self.assertFalse(r.rtx_cache)
        self.assertEqual([g.resets for g in guards],[1,1,1])

