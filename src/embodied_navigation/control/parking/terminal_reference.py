"""Honest fixed-goal terminal reference: never a SCAN path or trajectory."""
import hashlib,json,math
KIND='terminal_fixed_goal/v1'
FIELDS=('reference_kind','reference_sha256','terminal_reference','coordinate_frame_sha256','generation')

def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def validate_reference(record, expected_sha=None):
    if not isinstance(record,dict) or record.get('schema')!=KIND:
        raise ValueError('No typed terminal fixed-goal reference')
    if record.get('path_geometry_present')is not False or record.get('navigation_ground_truth_used')is not False:
        raise ValueError('Terminal reference cannot impersonate checked SCAN geometry')
    for k in ('generation','source_pose_stamp_ns','source_cloud_stamp_ns','paired_imu_stamp_ns',
              'created_control_clock_ns','created_wall_ns','source_pose_received_wall_ns',
              'source_cloud_received_wall_ns','source_imu_received_wall_ns','waypoint_index','deadline_clock_ns'):
        if type(record.get(k))is not int or record[k]<0:raise ValueError('Invalid terminal original scalar: '+k)
    pose=record['source_pose_stamp_ns'];clock=record['created_control_clock_ns'];wall=record['created_wall_ns']
    if (pose!=record['source_cloud_stamp_ns'] or not 0<=pose-record['paired_imu_stamp_ns']<=20_000_000
            or not 0<=clock-pose<300_000_000 or clock>=record['deadline_clock_ns']
            or not all(0<=wall-record[k]<300_000_000 for k in
                ('source_pose_received_wall_ns','source_cloud_received_wall_ns','source_imu_received_wall_ns'))):
        raise ValueError('Terminal creation sources not originally causal/fresh')
    arrival=record['region_receipt']
    if (arrival.get('request_id')!=record['request_id'] or arrival.get('waypoint_index')!=record['waypoint_index']
            or arrival.get('goals_definition_sha256')!=record['goals_definition_sha256']
            or arrival.get('region_inside')is not True or arrival.get('control_region_inside')is not True
            or arrival.get('protected')is not False or arrival.get('reason')!='arrived'
            or type(arrival.get('stamp_ns'))is not int or arrival['stamp_ns']>pose):
        raise ValueError('Terminal owner lacks actual same-task final region receipt')
    goal=record['fixed_goal'];xyz=goal['position_world_xyz'];yaw=goal['heading_rad']
    if len(xyz)!=3 or not all(math.isfinite(v) for v in (*xyz,yaw)) or digest(goal)!=record['fixed_goal_sha256']:
        raise ValueError('Terminal immutable fixed goal changed')
    if digest(record['coordinate_frame'])!=record['coordinate_frame_sha256'] or record['coordinate_frame'][0]!=record['generation']:
        raise ValueError('Terminal coordinate identity changed')
    value=digest(record)
    if expected_sha is not None and value!=expected_sha:raise ValueError('Terminal reference hash changed')
    return value

def validate_fields(data,run_id=None):
    if not any(k in data for k in FIELDS[:3]):
        if data.get('schema')=='actual_SLAM_terminal_fixed_goal_command_source/v1':
            raise ValueError('Terminal command source missing typed reference')
        return None
    if data.get('reference_kind')!=KIND:raise ValueError('Unknown terminal reference kind')
    if data.get('schema')=='actual_SLAM_SCAN_cascade_command_source/v1':
        raise ValueError('Terminal reference cannot use a SCAN command source schema')
    record=data.get('terminal_reference');value=validate_reference(record,data.get('reference_sha256'))
    if (any(data.get(k)is not None for k in ('path_sha256','path_id','trajectory_id','trajectory_archive_file'))
        or data.get('path_geometry_present')is True or data.get('reference_sha256')!=value
        or data.get('generation')!=record['generation']
        or data.get('coordinate_frame_sha256')!=record['coordinate_frame_sha256']
        or run_id is not None and record['run_id']!=run_id
        or data.get('request_id',record['request_id'])!=record['request_id']
        or data.get('goal_sha256',data.get('fixed_goal_sha256'))!=record['fixed_goal_sha256']):
        raise ValueError('Terminal lineage identity differs')
    return record

def fields(record):
    return dict(reference_kind=KIND,reference_sha256=validate_reference(record),terminal_reference=record,
        coordinate_frame_sha256=record['coordinate_frame_sha256'],generation=record['generation'])
