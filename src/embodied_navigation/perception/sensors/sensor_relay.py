#!/usr/bin/env python3
"""Original IMU/RGB passthrough with process-local writer selection.

No application queue, payload copy, timestamp repair or sensor retirement.
Timing stays in bounded memory and is saved once on exit, not per message.
"""
import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


TOPICS = {'imu': '/demo/teacher/slam/imu', 'image': '/demo/teacher/slam/image'}
NS = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'


def configure_transport(run, mode):
    """Keep original transports; change only these two publication modes."""
    original = Path(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']).resolve()
    raw = original.read_bytes()
    ET.register_namespace('', NS)
    tree = ET.fromstring(raw)
    profiles = tree.find('{%s}profiles' % NS)
    if profiles is None:
        raise ValueError('Missing original transport profiles')
    for topic in TOPICS.values():
        if any(p.get('profile_name') == topic for p in profiles):
            raise ValueError('Unexpected pre-existing relay writer profile')
        writer = ET.SubElement(profiles, '{%s}publisher' % NS, profile_name=topic)
        qos = ET.SubElement(writer, '{%s}qos' % NS)
        publish = ET.SubElement(qos, '{%s}publishMode' % NS)
        ET.SubElement(publish, '{%s}kind' % NS).text = {
            'sync': 'SYNCHRONOUS', 'async': 'ASYNCHRONOUS'}[mode]
    target = run / 'sensor_relay_transport.xml'
    with target.open('xb') as stream:
        stream.write(ET.tostring(tree, encoding='utf-8', xml_declaration=True))
    # AUTO selects the topic XML mode, retaining rmw's original memory/data
    # sharing defaults. QoS-from-XML=1 would override those too; do not use it.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(target)
    os.environ['RMW_FASTRTPS_PUBLICATION_MODE'] = 'AUTO'
    os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '0'
    return dict(mode=mode, baseline=str(original),
        baseline_sha256=hashlib.sha256(raw).hexdigest(), xml=str(target),
        xml_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        reliability_history_depth_unchanged=True, application_queue_added=False)


class PublishTiming:
    def __init__(self):
        self.rows = {n: dict(entered=0, returned=0, errors=0,
            wall_total_ns=0, cpu_total_ns=0, wall_max_ns=0,
            first_source_ns=None, last_source_ns=None) for n in TOPICS}
        self.first = {n: [] for n in TOPICS}
        self.last = {n: deque(maxlen=4) for n in TOPICS}
        self.slowest = {n: None for n in TOPICS}
        self.inflight = None

    def publish(self, name, msg, publisher, stamp):
        row = self.rows[name]
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        row['entered'] += 1
        if row['first_source_ns'] is None:
            row['first_source_ns'] = stamp
        row['last_source_ns'] = stamp
        self.inflight = (name, stamp, wall, cpu)
        returned = False
        try:
            publisher.publish(msg)
            row['returned'] += 1
            returned = True
        except BaseException:
            row['errors'] += 1
            raise
        finally:
            end_wall, end_cpu = time.monotonic_ns(), time.thread_time_ns()
            elapsed = end_wall - wall
            row['wall_total_ns'] += elapsed
            row['cpu_total_ns'] += end_cpu - cpu
            sample = (stamp, wall, end_wall, cpu, end_cpu, returned)
            if elapsed >= row['wall_max_ns']:
                row['wall_max_ns'] = elapsed
                self.slowest[name] = sample
            if len(self.first[name]) < 4:
                self.first[name].append(sample)
            self.last[name].append(sample)
            self.inflight = None

    def snapshot(self):
        return dict(rows=self.rows, first=self.first,
            last={n: list(v) for n, v in self.last.items()}, slowest=self.slowest,
            inflight=self.inflight,
            sample_fields=['source_ns', 'publish_wrapper_enter_monotonic_ns',
                'publish_return_monotonic_ns', 'enter_thread_cpu_ns',
                'return_thread_cpu_ns', 'returned'],
            endpoint='relay publish wrapper enter / publish return; not callback entry or DDS take/ACK',
            bounded_samples_per_stream=9, hot_path_IO=False)


def make_relay(Node, Image, Imu, String, sensor_qos):
    class Relay(Node):
        def __init__(self):
            super().__init__('teacher_slam_actual_sensor_relay')
            self.counts = {'imu': 0, 'image': 0}
            self.rejected = {'imu': 0, 'image': 0}
            self.stamps, self.walls, self.pubs = {}, {}, {}
            self.timing = PublishTiming()
            for name, typ, src in [('imu', Imu, '/livox/imu'),
                                   ('image', Image, '/demo/camera')]:
                self.pubs[name] = self.create_publisher(typ, TOPICS[name], 10)
                self.create_subscription(typ, src,
                    lambda m, n=name: self.receive(n, m), sensor_qos)
            self.status = self.create_publisher(String, '/demo/teacher/slam/relay_status', 10)
            self.create_timer(1., self.report)

        def receive(self, name, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= self.stamps.get(name, -1):
                self.rejected[name] += 1
                return
            self.timing.publish(name, msg, self.pubs[name], stamp)
            # These retain the original publication-return semantics.
            self.stamps[name] = stamp
            self.walls[name] = time.monotonic()
            self.counts[name] += 1

        def report(self):
            self.status.publish(String(data=json.dumps(dict(
                source='actual Teacher sensor messages', counts=self.counts,
                rejected=self.rejected, stamp_ns=self.stamps,
                wall_age={n: time.monotonic() - v for n, v in self.walls.items()},
                field_payload_and_stamp_unchanged=True, overview_used=False,
                ground_truth_used=False, publish_timing=self.timing.rows,
                publish_inflight=self.timing.inflight))))
    return Relay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--mode', choices=['sync', 'async'], required=True)
    args, ros = parser.parse_known_args()
    run = args.run_dir.resolve()
    transport = configure_transport(run, args.mode)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import String
    rclpy.init(args=ros)
    node = make_relay(Node, Image, Imu, String, qos_profile_sensor_data)()
    writer_qos = {}
    try:
        # Reuse the existing read-only ABI helper once at startup. This is
        # evidence, not a new control/sensor gate or per-message validation.
        try:
            from embodied_navigation.perception.sensors.async_writers import read_writer
            writer_qos = {n: read_writer(p) for n, p in
                dict(node.pubs, status=node.status).items()}
        except Exception as error:
            writer_qos = dict(read_error=type(error).__name__ + ':' + str(error))
        try:
            with (run / 'sensor_relay_loaded.json').open('x') as stream:
                json.dump(dict(pid=os.getpid(), source=str(Path(__file__).resolve()),
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    transport=transport, actual_writers=writer_qos,
                    ground_truth_used=False), stream, allow_nan=False)
        except OSError as error:
            print('Relay startup evidence unavailable: ' + str(error), file=sys.stderr)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # One bounded exit snapshot. Missing exit evidence stays unknown.
        try:
            with (run / 'sensor_relay_exit.json').open('x') as stream:
                json.dump(dict(pid=os.getpid(), counts=node.counts,
                    rejected=node.rejected, stamp_ns=node.stamps,
                    publish_timing=node.timing.snapshot(), actual_writers=writer_qos,
                    transport=transport), stream, allow_nan=False)
        except OSError as error:
            print('Relay exit evidence unavailable: ' + str(error), file=sys.stderr)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
