#!/usr/bin/env python3
"""Accumulate only this run's observed camera-coloured SLAM points.

The binary browser format is XYZ little-endian float32, R/G/B uint8, one pad
byte (16 bytes/point). Immutable revision files plus an atomic metadata swap
keep HTTP readers from mixing partial point buffers with a new point count.
"""
import argparse
import json
import math
import os
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from embodied_navigation.localization_mapping.geometry import cloud_records

BROWSER_DTYPE = np.dtype([('xyz', '<f4', (3,)), ('rgb', 'u1', (3,)), ('pad', 'u1')])


def atomic_write(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('wb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class VoxelArchive:
    """Bounded sparse accumulation, retaining measured RGB, never synthetic colour."""
    def __init__(self, voxel_size=0.08, max_points=8000000):
        self.voxel_size = voxel_size
        self.max_points = max_points
        self.voxels = {}
        self.observations = 0
        self.rejected_capacity = 0

    def add(self, xyz, rgb):
        keys = np.floor(xyz / self.voxel_size).astype(np.int64)
        # Each incoming voxel contributes one real observation; later scans
        # update its geometry/colour without fabricating unseen map points.
        _, indices = np.unique(keys, axis=0, return_index=True)
        for i in indices:
            key = tuple(keys[i])
            if key in self.voxels or len(self.voxels) < self.max_points:
                self.voxels[key] = (xyz[i].copy(), rgb[i].copy())
            else:
                self.rejected_capacity += 1
        self.observations += len(xyz)

    def records(self):
        result = np.zeros(len(self.voxels), dtype=BROWSER_DTYPE)
        if self.voxels:
            points = list(self.voxels.values())
            result['xyz'] = np.array([p[0] for p in points])
            result['rgb'] = np.array([p[1] for p in points])
        return result


def pcd_bytes(records):
    data = np.empty(len(records), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    for i, name in enumerate(('x', 'y', 'z')):
        data[name] = records['xyz'][:, i]
    color = records['rgb'].astype(np.uint32)
    data['rgb'] = (color[:, 0] << 16) | (color[:, 1] << 8) | color[:, 2]
    header = ('# Camera RGB observed during this run; frame camera_init\n'
              'VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n'
              f'WIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(data)}\nDATA binary\n')
    return header.encode('ascii') + data.tobytes()


class MapArchive(Node):
    def __init__(self, run_dir, voxel_size):
        super().__init__('demo_slam_map_archive')
        self.directory = Path(run_dir).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        # Existing runs must never be silently replaced or merged.
        if (self.directory / 'map_metadata.json').exists():
            raise RuntimeError('This run already has map metadata; choose a new DEMO_RUN_DIR')
        self.archive = VoxelArchive(voxel_size)
        self.counts = Counter()
        self.received = {}
        self.stamps = {}
        self.revision = 0
        self.binary_filename = None
        self.dirty = False
        self.bounds = None
        self.last_error = None
        self.save_state = {'requested': False, 'complete': False, 'filename': None}
        self.started_at = time.time()
        self.status_pub = self.create_publisher(String, '/demo/slam/map_status', 10)
        self.create_subscription(PointCloud2, '/cloud_registered', self.cloud, qos_profile_sensor_data)
        for name, topic, message_type in [
                ('lidar', '/livox/lidar', PointCloud2),
                ('imu', '/livox/imu', Imu),
                ('camera', '/camera/image_color', Image),
                ('odom', '/demo/slam/body_odom', Odometry),
                ('full_cloud', '/cloud_registered_full', PointCloud2)]:
            self.create_subscription(message_type, topic, lambda msg, n=name: self.observe(n, msg), qos_profile_sensor_data)
        self.create_service(Trigger, '/demo/slam/save_map', self.save)
        self.create_timer(2.0, self.snapshot)
        self.snapshot()

    def observe(self, name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not math.isfinite(stamp) or (name in self.stamps and stamp <= self.stamps[name]):
            self.counts[f'rejected_{name}_timestamp'] += 1
            return False
        self.counts[name] += 1
        self.received[name] = time.monotonic()
        self.stamps[name] = stamp
        return True

    def cloud(self, msg):
        if msg.header.frame_id != 'camera_init':
            self.counts['rejected_frame'] += 1
            self.last_error = f'RGB cloud has unexpected frame {msg.header.frame_id}'
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if 'colored_cloud' in self.stamps and stamp <= self.stamps['colored_cloud']:
            self.counts['rejected_clock_reset'] += 1
            self.last_error = 'Colored cloud timestamp did not increase; reject replay or clock reset'
            return
        try:
            xyz, rgb = cloud_records(msg)
        except ValueError as exc:
            self.counts['rejected_format'] += 1
            self.last_error = str(exc)
            return
        if not len(xyz):
            self.counts['empty_colored_cloud'] += 1
            return
        self.archive.add(xyz, rgb)
        self.observe('colored_cloud', msg)
        self.dirty = True
        self.last_error = None

    def metadata(self):
        now = time.monotonic()
        ages = {name: now - self.received[name] if name in self.received else None
                for name in ['lidar', 'imu', 'camera', 'odom', 'full_cloud', 'colored_cloud']}
        return {
            'run_id': self.directory.name, 'frame_id': 'camera_init',
            'revision': self.revision, 'binary_filename': self.binary_filename,
            'point_count': len(self.archive.voxels), 'rgb_points': len(self.archive.voxels),
            'bounds': self.bounds, 'voxel_size': self.archive.voxel_size,
            'format': {'point_step': 16, 'xyz': 'little-endian float32 at offsets 0,4,8',
                       'rgb': 'uint8 RGB at offsets 12,13,14', 'padding_offset': 15},
            'source_topic': '/cloud_registered',
            'color_source': 'FAST-LIVO2 projection of /camera/image_color simulated RGB images',
            'geometry_source': 'this run LiDAR/IMU/visual SLAM observations',
            'ground_truth_used': False, 'reference_map_loaded': False,
            'observed_rgb_samples': self.archive.observations,
            'capacity_rejections': self.archive.rejected_capacity,
            'capacity_points': self.archive.max_points,
            'started_at': self.started_at, 'updated_at': time.time(),
            'counts': dict(self.counts), 'ages': ages, 'stamps': dict(self.stamps),
            'slam_healthy': all(ages[name] is not None and ages[name] < 2
                                for name in ('odom', 'lidar', 'imu', 'full_cloud')),
            'camera_healthy': ages['camera'] is not None and ages['camera'] < 2,
            'message_age': ages['colored_cloud'], 'save': self.save_state,
            'error': self.last_error,
        }

    def snapshot(self):
        if self.dirty:
            records = self.archive.records()
            self.revision += 1
            self.binary_filename = f'colored_map.{self.revision:06d}.bin'
            atomic_write(self.directory / self.binary_filename, records.tobytes())
            self.bounds = {'min': records['xyz'].min(axis=0).tolist(),
                           'max': records['xyz'].max(axis=0).tolist()}
            self.dirty = False
        metadata = self.metadata()
        payload = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode('utf-8')
        atomic_write(self.directory / 'map_metadata.json', payload)
        if self.context.ok():
            self.status_pub.publish(String(data=payload.decode('utf-8')))
        # Keep at least 3 immutable revisions for HTTP readers in flight.
        old = sorted(self.directory.glob('colored_map.*.bin'))[:-3]
        for path in old:
            path.unlink(missing_ok=True)

    def save(self, request, response):
        self.save_state = {'requested': True, 'complete': False, 'filename': None}
        if not self.archive.voxels:
            response.success = False
            response.message = 'No measured camera RGB map yet'
            self.snapshot()
            return response
        if self.archive.rejected_capacity:
            response.success = False
            response.message = 'Map capacity exceeded; refusing to mark an incomplete map saved'
            self.last_error = response.message
            self.snapshot()
            return response
        health = self.metadata()
        if not health['slam_healthy'] or not health['camera_healthy']\
                or health['message_age'] is None or health['message_age'] >= 2 or health['error']:
            response.success = False
            response.message = 'SLAM, sensor or colored-cloud evidence is stale/invalid; refusing map save'
            self.snapshot()
            return response
        filename = 'colored_map.pcd'
        atomic_write(self.directory / filename, pcd_bytes(self.archive.records()))
        self.save_state = {'requested': True, 'complete': True, 'filename': filename,
                           'point_count': len(self.archive.voxels), 'saved_at': time.time(),
                           'last_observation_stamp': self.stamps.get('colored_cloud'),
                           'healthy_sensor_evidence': {'slam_healthy': health['slam_healthy'],
                               'camera_healthy': health['camera_healthy'], 'ages': health['ages'],
                               'stamps': health['stamps'], 'error': health['error']}}
        self.snapshot()
        response.success = True
        response.message = str(self.directory / filename)
        return response


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', default=os.environ.get('DEMO_RUN_DIR'))
    parser.add_argument('--voxel-size', type=float, default=.08)
    args, ros_args = parser.parse_known_args()
    if not args.run_dir:
        parser.error('--run-dir or DEMO_RUN_DIR is required; never reuse an old map')
    if not .02 <= args.voxel_size <= .5:
        parser.error('voxel-size must be between .02 and .5 metres')
    rclpy.init(args=ros_args)
    node = MapArchive(args.run_dir, args.voxel_size)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _ros.RCLError:
        # SIGINT can invalidate the context between executor wait-set calls.
        # Errors while the context is live must still fail this required node.
        if rclpy.ok():raise
    finally:
        node.snapshot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
