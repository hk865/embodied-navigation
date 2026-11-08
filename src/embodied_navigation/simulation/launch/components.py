import os
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from embodied_navigation.common.config import path
from embodied_navigation.launch.launching import python, owned
def components(run):
    socket=os.environ['TEACHER_SOCKET']
    actions=owned('actor',python('embodied_navigation.robot.unitree.policy.worker','--run',run,'--socket',socket,ros=False,executable=path('actor_python')),run,environment={'CUDA_VISIBLE_DEVICES':''})
    actions+=owned('sensor_bridge',['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','--ros-args','-p','config_file:='+str(run/'sensor_bridge.yaml')],run)
    barrier=ExecuteProcess(cmd=python('embodied_navigation.simulation.wait_actor','--run',run,'--socket',socket,ros=False),name='actor_ready',output='screen')
    def ready(event,context):
        if event.returncode!=0:return [EmitEvent(event=Shutdown(reason='Actor readiness failed'))]
        return owned('gazebo',['gz','sim','-s','-r','--headless-rendering',str(run/'world.sdf'),'--seed','42'],run)
    actions += [RegisterEventHandler(OnProcessExit(target_action=barrier,on_exit=ready)),barrier]
    return actions
