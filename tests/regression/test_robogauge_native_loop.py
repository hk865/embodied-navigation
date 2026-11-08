"""Optional CPU-only request64/response16 fixture with a fake socket peer."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np


@unittest.skipUnless(importlib.util.find_spec('torch'),'Optional CPU Torch environment is not installed')
class RoboGaugeNativeLoop(unittest.TestCase):
    def test_native_transport_bootstrap_targets_and_termination(self):
        import torch
        from embodied_navigation.simulation.gazebo.robogauge import native_loop as loop
        from embodied_navigation.robot.unitree.robogauge.contract import QDEFAULT,observation
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
        states=[]
        for i in range(7):
            state=np.zeros(64);state[0]=1+i*.02;state[3]=.32;state[4]=1
            state[14:26]=QDEFAULT;state[51:55]=1;states.append(state)
        iterator=iter(states);observations=[];frames=[]
        action=np.linspace(-.2,.2,12,dtype=np.float32)
        def actor(obs):observations.append(obs.numpy().copy());return torch.from_numpy(action.reshape(1,12))
        def receive(conn,n):self.assertEqual(n,512);return next(iterator).astype('<f8').tobytes()
        def request(*args):return np.array([.15,0,0]),'slam_route_tracking'
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,NAVIGATION_RECORD_RAW='0'):
            run=Path(tmp)
            (run/'world.sdf').write_text('<sdf version="1.9"><world name="test"><model name="ground"><static>true</static><link name="ground"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model></world></sdf>')
            with patch.object(loop.socket,'socket',return_value=Server()):
                loop.main(actor_loader=lambda:actor,observation_builder=observation,command_source=request,
                          receive=receive,publish=loop.atomic_json,finalize=lambda row:frames.append(row) or row,
                          prepare=loop.prepare_velocity_input,runtime_duration=.119,
                          argv=['--run',str(run),'--socket',str(run/'never_opened.sock')])
            result=json.loads((run/'worker_result.json').read_text())
            self.assertIsNone(result['fault']);self.assertEqual(result['samples'],7)
        self.assertEqual(len(peer.sent),7)
        for row in peer.sent[:5]:np.testing.assert_array_equal(row[:12],QDEFAULT)
        np.testing.assert_allclose(peer.sent[-1][:12],QDEFAULT+.25*action,rtol=0,atol=1e-8)
        self.assertEqual(peer.sent[-1][13],1);self.assertEqual(peer.sent[-1][12],0)
        self.assertTrue(all(x.shape==(1,45) for x in observations))
        self.assertAlmostEqual(frames[-1]['state_physics_world_time'],states[-1][0]-.005)
        self.assertAlmostEqual(observations[0][0,6],.006,places=7)


if __name__=='__main__':unittest.main()
