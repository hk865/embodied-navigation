"""Execute both native loops with real CPU observation code and fake transport.

No simulation, ROS, hardware, weights, or real socket is used. Torch is optional;
source-hook checks and the numerical safety tests also run without Torch.
"""
import ast
from dataclasses import replace
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from embodied_navigation.project import project_root

LOOPS=(('student','robot/unitree/policy/native/worker.py',247),
       ('robogauge','simulation/gazebo/robogauge/native_loop.py',45))


class NativeSafetySourceHooks(unittest.TestCase):
    def test_both_backends_use_the_shared_gate_and_factory(self):
        for name,relative,_ in LOOPS:
            tree=ast.parse((project_root()/'src/embodied_navigation'/relative).read_text())
            imports={node.module for node in ast.walk(tree) if isinstance(node,ast.ImportFrom)}
            self.assertIn('embodied_navigation.control.safety.simulation_gate',imports,name)
            self.assertIn('embodied_navigation.simulation.safety_state',imports,name)
            calls={node.func.id for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Name)}
            methods={node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)}
            self.assertIn('SimulationSafetyGate',calls,name)
            self.assertIn('from_gazebo_native64',calls,name)
            self.assertTrue({'latch_runtime_faults','evaluate','reject_state'}<=methods,name)


@unittest.skipUnless(importlib.util.find_spec('torch'),'Optional CPU Torch environment is not installed')
class NativeSafetyLoopIntegration(unittest.TestCase):
    def execute(self, backend, *, fall=False, reject_clock=False, terrain_failure=False):
        import torch
        from embodied_navigation.robot.unitree.policy.native import worker as student
        from embodied_navigation.robot.unitree.policy.native.observation import build_observation
        from embodied_navigation.robot.unitree.robogauge.contract import observation
        from embodied_navigation.simulation.gazebo.robogauge import native_loop as robogauge
        from embodied_navigation.control.safety.simulation_gate import SimulationSafetyGate
        from embodied_navigation.simulation.safety_state import from_gazebo_native64
        loop=student if backend=='student' else robogauge
        self.assertIs(loop.SimulationSafetyGate,SimulationSafetyGate)
        self.assertIs(loop.from_gazebo_native64,from_gazebo_native64)
        dimension=247 if backend=='student' else 45
        class Peer:
            def __init__(self):self.sent=[]
            def settimeout(self,*args):pass
            def sendall(self,raw):self.sent.append(np.frombuffer(raw,dtype='<f8').copy())
            def __enter__(self):return self
            def __exit__(self,*args):pass
        peer=Peer()
        class Server:
            def bind(self,*args):pass
            def listen(self,*args):pass
            def settimeout(self,*args):pass
            def accept(self):return peer,None
            def close(self):pass
        times=[0.,1.5,1.6,1.99,2.] if fall else [i*.02 for i in range(7)]
        states=[]
        for t in times:
            state=np.zeros(64);state[0]=10.+t;state[3]=.32;state[4]=1.
            if fall and t>=1.5:state[4:8]=[np.cos(.5),np.sin(.5),0.,0.]
            state[14:26]=loop.QDEFAULT;state[51:55]=1.
            if terrain_failure:state[56]=4.
            states.append(state)
        iterator=iter(states);seen=[];frames=[];gate_calls=[];latch_calls=[];gates=[]
        action=np.linspace(-.2,.2,12,dtype=np.float32)
        def actor(obs):
            seen.append(obs.numpy().copy())
            return torch.full((1,12),float('nan')) if terrain_failure else torch.from_numpy(action.reshape(1,12))
        def receive(conn,n):
            self.assertEqual(n,512);return next(iterator).astype('<f8').tobytes()
        class RecordedGate(SimulationSafetyGate):
            def latch_runtime_faults(self,**kwargs):
                latch_calls.append(kwargs.copy());return super().latch_runtime_faults(**kwargs)
            def evaluate(self,state,**kwargs):
                gate_calls.append(state);return super().evaluate(state,**kwargs)
        def make_gate(**kwargs):
            gate=RecordedGate(**kwargs);gates.append(gate);return gate
        def factory(*args,**kwargs):
            state=from_gazebo_native64(*args,**kwargs)
            if reject_clock:state=replace(state,provenance=replace(state.provenance,clock_domain='monotonic_wall'))
            return state
        def obs_builder(*args,**kwargs):
            if terrain_failure:return np.zeros(dimension,dtype=np.float32),{}
            return (build_observation if backend=='student' else observation)(*args,**kwargs)
        class BadTerrain:
            def height(self,*args,**kwargs):raise RuntimeError('scripted safety terrain failure')
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,NAVIGATION_RECORD_RAW='0'):
            run=Path(tmp)
            (run/'world.sdf').write_text('<sdf version="1.9"><world name="test"><model name="ground"><static>true</static><link name="ground"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model></world></sdf>')
            argv=['fixture','--run',str(run),'--socket',str(run/'never_opened.sock')]
            common=dict(actor_loader=lambda:actor,observation_builder=obs_builder,
                command_source=lambda *args:(np.array([.15,0,0]),'tracking'),receive=receive,
                publish=loop.atomic_json,finalize=lambda row:frames.append(row) or row,
                prepare=loop.prepare_velocity_input,runtime_duration=20. if fall else .119)
            with patch.object(loop.socket,'socket',return_value=Server()),patch.object(loop,'SimulationSafetyGate',side_effect=make_gate),patch.object(loop,'from_gazebo_native64',side_effect=factory),patch.object(sys,'argv',argv):
                if terrain_failure:
                    with patch.object(loop.TerrainHeightMap,'from_sdf',return_value=BadTerrain()):loop.main(**common)
                else:loop.main(**common)
            result=json.loads((run/'worker_result.json').read_text())
        self.assertEqual(len(gates),1)
        self.assertEqual(len(gate_calls),len(frames))
        self.assertTrue(all(obs.shape==(1,dimension) for obs in seen))
        for frame in frames:
            source=frame['simulation_safety']['source']
            self.assertEqual(source['origin'],'simulation_truth')
            self.assertEqual(frame['simulation_safety']['purpose'],'simulation_safety')
        return peer.sent,frames,result,latch_calls,gate_calls

    def test_both_loops_preserve_targets_and_call_gate_every_frame(self):
        outputs=[]
        for backend,_,_ in LOOPS:
            sent,frames,result,_,calls=self.execute(backend)
            self.assertIsNone(result['fault']);self.assertEqual(result['samples'],7)
            self.assertEqual(len(calls),7)
            self.assertTrue(all(frame['simulation_safety']['state_contract_valid'] for frame in frames))
            self.assertTrue(all(frame['simulation_safety']['source']['clock_domain']=='gazebo_simulation' for frame in frames))
            self.assertEqual(sent[-1][12],0);self.assertEqual(sent[-1][13],1)
            outputs.append(np.asarray(sent))
        np.testing.assert_array_equal(*outputs)

    def test_both_loops_use_shared_fault_grace_and_stop(self):
        for backend,_,_ in LOOPS:
            sent,frames,result,_,calls=self.execute(backend,fall=True)
            self.assertEqual(len(calls),5)
            self.assertEqual(result['fault'],'navigation_physical_fall_or_low_clearance')
            self.assertEqual([int(row[12]) for row in sent],[0,0,1,1,1])
            self.assertEqual([int(row[13]) for row in sent],[0,0,0,0,1])
            self.assertTrue(frames[-1]['simulation_safety']['stop'])

    def test_both_loops_reject_wrong_time_domain_before_motion(self):
        for backend,_,_ in LOOPS:
            sent,frames,result,_,calls=self.execute(backend,reject_clock=True)
            self.assertEqual(len(calls),1);self.assertEqual(len(sent),1)
            self.assertEqual(result['fault'],'simulation_safety_state_contract_rejected')
            self.assertEqual(tuple(sent[0][12:14]),(1.,1.))
            self.assertEqual(frames[0]['simulation_safety']['state_contract_error'],'mismatched_clock_domain')

    def test_both_loops_latch_policy_fault_before_terrain_exception(self):
        for backend,_,_ in LOOPS:
            sent,frames,result,latches,calls=self.execute(backend,terrain_failure=True)
            self.assertFalse(sent);self.assertFalse(frames);self.assertFalse(calls)
            self.assertEqual(result['fault'],'nonfinite_policy')
            self.assertEqual(len(latches),1)
            self.assertFalse(latches[0]['policy_finite'])
            self.assertEqual(latches[0]['actuator_fault_code'],4.)


if __name__=='__main__':unittest.main()
