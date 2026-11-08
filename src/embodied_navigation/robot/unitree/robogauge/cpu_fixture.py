"""Optional real-weight CPU replay; never launches ROS, Gazebo, Isaac or hardware."""
import argparse
import json
import time
import numpy as np
from .runtime import Runtime
from .contract import MODEL_SHA256


def run_fixture(model_path=None):
    import torch
    torch.set_num_threads(1)
    runtime=Runtime(model_path)
    reference=torch.jit.load(str(runtime.path),map_location='cpu').eval()
    reference.reset();runtime.reset()
    rng=np.random.default_rng(8);maximum=0.;times=[]
    with torch.inference_mode():
        for _ in range(30):
            obs=rng.normal(0,.1,45).astype(np.float32);obs[3:6]=[0,0,-1]
            started=time.perf_counter();actual=runtime.infer(obs)
            times.append((time.perf_counter()-started)*1000)
            expected=reference(torch.from_numpy(obs).reshape(1,45)).clamp(-100,100).numpy()[0]
            maximum=max(maximum,float(np.max(abs(actual[runtime.to_official]-expected))))
    if maximum != 0 or runtime.calls != 30:raise AssertionError('Sequential actor replay differs')
    state=np.zeros(64);state[4]=1;state[14:26]=runtime.qdefault
    obs=runtime.observation(state,np.zeros(3),np.zeros(12))
    runtime.reset();a=runtime.infer(obs);runtime.reset();b=runtime.infer(obs)
    if not np.array_equal(a,b):raise AssertionError('Reset is not reproducible')
    runtime.reset()
    return dict(passed=True,model_sha256=MODEL_SHA256,torch_version=torch.__version__,
                device='cpu',threads=1,sequence_frames=30,sequential_action_max_error=maximum,
                reset_reproducible=True,reset_after_fixture=runtime.calls==0,
                latency_ms_p50=float(np.median(times)),latency_ms_p95=float(np.percentile(times,95)),
                neutral_first_native_action=a.tolist(),no_simulator_or_hardware_started=True,
                scope='CPU actor and history replay only; not a navigation or physics pass')


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model');ap.add_argument('--output')
    args=ap.parse_args(argv);result=run_fixture(args.model)
    text=json.dumps(result,indent=2)+'\n'
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(text)
    print(text,end='')


if __name__=='__main__':main()
