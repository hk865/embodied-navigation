"""File-only dependency check: useful before ROS, PyTorch or native builds exist."""
from embodied_navigation.common.config import ROOT, path
def check():
    files={
        'Student':path('student_model'),'Actor Python':path('actor_python'),
        'Go2 URDF':ROOT/'assets/world/go2_measured.urdf',
        'Go2 SDF':ROOT/'assets/world/go2_converted.sdf',
        'world':ROOT/'assets/world/three_floors.sdf',
        'route':ROOT/'assets/geometry/route.json',
        'L1 mesh':ROOT/'assets/meshes/L1_4D_Lidar.dae',
        'actuator':path('actuator'),
        'writer helper':ROOT/'install/communication/lib/writer_qos.so',
        'patched DDS':ROOT/'install/dds/lib/libfastrtps.so.2.14.6',
        'ROS install':path('ros_prefix')/'local_setup.bash',
        'SLAM':path('ros_prefix')/'lib/fast_livo2_ros/fastlivo_mapping',
        'SCAN':path('ros_prefix')/'lib/scan_planner/scan_planner_node',
    }
    result={name:dict(path=str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),present=p.is_file()) for name,p in files.items()}
    return dict(ready=all(v['present'] for v in result.values()),dependencies=result,simulation_started=False)
