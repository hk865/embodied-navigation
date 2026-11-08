"""Shared launch mechanics; module files own their component lists."""
from pathlib import Path
import json,os
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
ROS=['--ros-args','-p','use_sim_time:=true']

def python(module,*arguments,ros=True,executable='/usr/bin/python3'):
    return [str(executable),'-B','-m',module,*map(str,arguments),*(ROS if ros else [])]

def owned(role,command,run,*,environment=None):
    process=ExecuteProcess(cmd=command,name=role,output='screen',additional_env=environment or {},
                           sigterm_timeout='8',sigkill_timeout='5')
    def stopped(event,context):
        with (Path(run)/'component_exit.jsonl').open('a') as out:
            out.write(json.dumps(dict(role=role,pid=event.pid,returncode=event.returncode))+'\n')
        return [EmitEvent(event=Shutdown(reason=role+' exited: '+str(event.returncode)))]
    return [RegisterEventHandler(OnProcessExit(target_action=process,on_exit=stopped)),process]
