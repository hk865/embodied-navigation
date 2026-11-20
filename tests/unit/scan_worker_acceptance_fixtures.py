"""Controlled ROS/process transport for executing the real measured SCAN worker."""
from concurrent.futures import Future
import copy
from contextlib import ExitStack
import json
import os
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import yaml

from embodied_navigation.simulation.dynanav import pct_scan_worker as worker
from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker


def stamp(seconds):
    ns = round(seconds * 1e9)
    return NS(sec=ns // 10**9, nanosec=ns % 10**9)


def header(seconds=0., frame='camera_init'):
    return NS(stamp=stamp(seconds), frame_id=frame)


def point(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


class Odometry:
    def __init__(self):
        self.header = header()
        self.child_frame_id = 'body'
        self.pose = NS(pose=NS(position=point(), orientation=NS(x=0., y=0., z=0., w=1.)))
        self.twist = NS(twist=NS(linear=point(), angular=point()))


class PoseStamped:
    def __init__(self):
        self.header = header()
        self.pose = NS(position=point(), orientation=NS(x=0., y=0., z=0., w=1.))


class RosPath:
    def __init__(self):
        self.header = header()
        self.poses = []


class String:
    def __init__(self, data=''):
        self.data = data


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return 99


class Node:
    def __init__(self, name):
        self.publishers = {}
        self.callbacks = {}
        self.timer = None
        self.destroyed = False

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, kind, topic, callback, qos):
        self.callbacks[topic] = callback
        return object()

    def create_timer(self, interval, callback):
        self.timer = callback
        return object()

    def get_logger(self):
        return NS(warning=lambda *a, **k: None, info=lambda *a, **k: None, error=lambda *a, **k: None)

    def destroy_node(self):
        self.destroyed = True

    def publisher(self, suffix):
        return next(value for name, value in self.publishers.items() if name.endswith('/' + suffix))

    def receive(self, suffix, message):
        callbacks = [value for name, value in self.callbacks.items() if name.endswith('/' + suffix)]
        if len(callbacks) != 1:
            raise AssertionError('Expected one actual subscription for ' + suffix)
        callbacks[0](message)


class ImmediatePool:
    def __init__(self, **kwargs):
        pass

    def submit(self, function, *args, **kwargs):
        future = Future()
        try:
            future.set_result(function(*args, **kwargs))
        except BaseException as error:
            future.set_exception(error)
        return future

    def shutdown(self, **kwargs):
        pass


class Process:
    def __init__(self, command):
        self.pid = os.getpid()
        self.returncode = None
        self.command = command
        self.terminated = False
        self.params = yaml.safe_load(Path(command[command.index('--params-file') + 1]).read_text())['/**']['ros__parameters']

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = 0

    def wait(self, **kwargs):
        return self.returncode

    def kill(self):
        self.returncode = -9


def ros_modules(scenario):
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'nav_msgs', 'nav_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
        'sensor_msgs_py', 'sensor_msgs_py.point_cloud2', 'scan_planner_msgs',
        'scan_planner_msgs.msg', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy'].init = lambda: None
    modules['rclpy'].shutdown = lambda: None
    modules['rclpy'].spin = scenario
    modules['rclpy.node'].Node = Node
    modules['rclpy.qos'].qos_profile_sensor_data = object()
    modules['rclpy.qos'].QoSProfile = lambda **kwargs: NS(**kwargs)
    for enum in ('ReliabilityPolicy', 'QoSReliabilityPolicy', 'DurabilityPolicy', 'QoSDurabilityPolicy', 'HistoryPolicy'):
        setattr(modules['rclpy.qos'], enum, NS(RELIABLE=1, BEST_EFFORT=2, TRANSIENT_LOCAL=1, VOLATILE=2, KEEP_LAST=1))
    modules['nav_msgs.msg'].Odometry = Odometry
    modules['nav_msgs.msg'].Path = RosPath
    modules['geometry_msgs.msg'].PoseStamped = PoseStamped
    modules['sensor_msgs.msg'].PointCloud2 = type('PointCloud2', (), {})
    modules['sensor_msgs_py.point_cloud2'].read_points = lambda cloud, **kw: cloud.points
    modules['scan_planner_msgs.msg'].Bspline = type('Bspline', (), {})
    modules['std_msgs.msg'].String = String
    return modules


class WorkerRig:
    def __init__(self, root):
        self.root = Path(root)
        self.run = self.root / 'generation'
        self.mailbox = self.run / 'interactive'
        self.prefix = self.root / 'build' / 'install'
        binary = self.prefix / 'lib/scan_planner/scan_planner_node'
        binary.parent.mkdir(parents=True)
        binary.touch()
        (self.prefix.parent / 'source_receipt.json').write_text(json.dumps({'source_sha256': 'acceptance'}))
        self.wall = 100.
        self.processes = []
        self.route = [[0., 0., .4], [1., 0., .4], [4., 0., .4]]
        self.adapter = PctScanWaypointTracker(dict(slam_pose_source='livo_post_vio',
            speed_mps=.5, motion_end_s=None, interactive_mailbox=str(self.mailbox)))
        self.adapter._handle_request(dict(version=1, run_id='generation', seq=1,
            action='goto', pose_source='livo_post_vio', goal_xyz_m=[4., 0., .4]))
        self.tick_adapter(10.)
        self.node = None
        self.last_odom = None

    def tick_adapter(self, t=10.6):
        return self.adapter.tick(current_sim_s=t, odom_stamp_s=t, position=[0., 0., .4],
            quat_wxyz=[1., 0., 0., 0.], estimated_velocity=[0., 0., 0.], measured_yaw_rate=0.,
            ready=True, now_wall=t, arrival_wall=t, frame_id='camera_init',
            child_frame_id='body', pose_source='livo_post_vio')

    def run_scenario(self, scenario):
        rig = self
        class Planner:
            def __init__(self, *args, **kwargs):
                pass
            def plan(self, start, goal):
                return dict(found=True, body_path_xyz=copy.deepcopy(rig.route), provenance={})
        def popen(command, **kwargs):
            process = Process(command)
            self.processes.append(process)
            return process
        def drive(node):
            self.node = node
            scenario(self)
        args = NS(run_dir=str(self.run), mailbox='interactive', scan_prefix=str(self.prefix),
            speed=.5, body_support_offset=.4, pct_start_blind_radius=1.5,
            pct_binary=None, reference_response_timeout_s=10.)
        with ExitStack() as stack:
            stack.enter_context(patch.dict('sys.modules', ros_modules(drive)))
            stack.enter_context(patch.object(worker, 'scan_source_digest', return_value='acceptance'))
            stack.enter_context(patch.object(worker, 'ThreadPoolExecutor', ImmediatePool))
            stack.enter_context(patch.object(worker, 'subprocess',
                NS(**{**vars(worker.subprocess), 'Popen': popen})))
            stack.enter_context(patch.object(worker.time, 'monotonic', side_effect=lambda: self.wall))
            stack.enter_context(patch.object(worker.signal, 'signal'))
            stack.enter_context(patch('embodied_navigation.planning.pct.measured_tomogram.MeasuredPctPlanner', Planner))
            worker.run(args)

    def observe(self, source=10.5, position=(0., 0., .4)):
        message = Odometry()
        message.header = header(source)
        message.pose.pose.position = point(*position)
        self.last_odom = message
        self.node.receive('post_vio_odometry', message)
        lio = copy.deepcopy(message)
        self.node.receive('aft_mapped_to_init', lio)
        cloud = NS(header=header(source), fields=[NS(name=n) for n in ('x','y','z','curvature')])
        cloud.points = np.array([(0., 0., 0., 0.), (1., 1., 0., 40.)], dtype=[(n, 'f4') for n in ('x','y','z','curvature')])
        self.node.receive('cloud_registered_full', cloud)
        return message

    def poll(self, delta=.05):
        self.wall += delta
        self.node.timer()

    def prepare(self):
        for i in range(6):
            self.observe(10. + .1 * i)
        self.poll()
        self.poll()
        self.observe(10.6, (.00029, 0., .4))
        self.poll()

    def path_messages(self):
        return self.node.publisher('initial_path').messages

    def output(self):
        path = self.mailbox / 'planner_path.json'
        return json.loads(path.read_text()) if path.exists() else {}

    def ack(self, **changes):
        process = self.processes[-1]
        body = self.node.publisher('body_pose').messages[-1]
        ns = body.header.stamp.sec * 10**9 + body.header.stamp.nanosec
        fields = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
        data = dict(schema='r85_scan_child_odometry_consumed/v1', run_id='generation',
            planner_epoch=1, localization_generation=0, planner_pid=process.pid,
            planner_ppid=os.getppid(), planner_start_ticks=int(fields[19]),
            startup_nonce=process.params.get('fsm.child_odometry_ack_nonce', 'a' * 32),
            source_stamp_ns=ns, source_frame_id='camera_init', have_odom=True,
            ack_sequence=100, publisher_gid='ab' * 16)
        data.update(changes)
        self.node.receive('planning/odometry_consumption_ack', String(json.dumps(data)))
        return data

    def reference_stamp(self):
        h = self.path_messages()[-1].header.stamp
        return [h.sec, h.nanosec]

    def definition(self, **changes):
        data = dict(schema='scan_route_definition/v1', run_id='generation', reference_epoch=1,
            localization_generation=0, planner_process_id=self.processes[-1].pid,
            committed_reference_revision=1, reference_stamp=self.reference_stamp(),
            body_goal=[4., 0., .4], reference_original_duration=8.,
            reference_content={'frame_id': 'camera_init'})
        data.update(changes)
        self.node.receive('planning/route_definition', String(json.dumps(data)))

    def event(self, event='reference_rejected', **changes):
        data = dict(schema='scan_reference_status/v1', event=event, run_id='generation',
            reference_epoch=1, planner_process_id=self.processes[-1].pid,
            frame_id='camera_init', reference_stamp=self.reference_stamp(),
            last_consumed_odometry_stamp_ns=10_600_000_000, reason='acceptance_rejection')
        data.update(changes)
        self.node.receive('planning/reference_status', String(json.dumps(data)))

    def motion(self, trajectory=1, kind='motion', **changes):
        spline = NS(traj_id=trajectory, start_time=stamp(11. + trajectory * .1), order=3,
            knots=[-3., -2., -1., 0., 1., 2., 3., 4.],
            pos_pts=[point(0., 0., .4), point(.2, 0., .4), point(.4, 0., .4), point(.6, 0., .4)])
        info = dict(trajectory_id=spline.traj_id,
            trajectory_start_time=[spline.start_time.sec, spline.start_time.nanosec],
            reference_stamp=self.reference_stamp(), run_id='generation', reference_epoch=1,
            localization_generation=0, planner_process_id=self.processes[-1].pid,
            committed_reference_revision=1, candidate_kind=kind)
        info.update(changes)
        self.node.receive('planning/bspline', spline)
        self.node.receive('planning/trajectory_metadata', String(json.dumps(info)))
