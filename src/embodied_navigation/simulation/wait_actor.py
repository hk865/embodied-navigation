"""A real socket/actor readiness barrier, never a fixed launch delay."""
from pathlib import Path
import argparse,os,time
def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--socket',type=Path,required=True)
    a=p.parse_args();deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        ready=a.run/'worker_ready'
        if ready.is_file() and a.socket.is_socket():
            pid=int(ready.read_text());os.kill(pid,0);return 0
        time.sleep(.05)
    raise TimeoutError('Actor did not load, bind and listen within 30 seconds')
if __name__=='__main__':raise SystemExit(main())
