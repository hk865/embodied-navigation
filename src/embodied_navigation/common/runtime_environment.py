"""Resolve freshly built project install and the public ROS base."""
from pathlib import Path
import os,subprocess
from embodied_navigation.common.config import ROOT, path

def environment(ros_base='/opt/ros/jazzy/setup.bash',**unused):
    prefix=path('ros_prefix');setup=prefix/'local_setup.bash'
    if not setup.is_file():raise FileNotFoundError('Build SDK, SLAM and SCAN first: '+str(setup))
    child=dict(os.environ)
    for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','GZ_SIM_RESOURCE_PATH','GAZEBO_MODEL_PATH'):
        child.pop(key,None)
    result=subprocess.run(['bash','-c','set -e; source "$1" >/dev/null; source "$2" >/dev/null; env -0',
        'navigation-environment',ros_base,str(setup)],env=child,check=True,capture_output=True)
    value=dict(x.decode().split('=',1) for x in result.stdout.split(b'\0') if b'=' in x)
    value['PYTHONPATH']=str(ROOT/'src')+':'+value.get('PYTHONPATH','')
    value['LD_LIBRARY_PATH']=str(ROOT/'install/livox/lib')+':'+value.get('LD_LIBRARY_PATH','')
    # ROS Jazzy's transport/msg vendors set GZ_CONFIG_PATH without the separately
    # installed Harmonic simulator. Keep its official command configs visible.
    value['GZ_CONFIG_PATH']='/usr/share/gz:'+value.get('GZ_CONFIG_PATH','')
    value['PATH']='/usr/bin:'+value.get('PATH','')
    return value
