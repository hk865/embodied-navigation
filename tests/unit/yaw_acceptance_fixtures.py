"""Drive real measured-pose adapters and their temporary mailbox, without ROS."""
import json
import math
from pathlib import Path

from embodied_navigation.simulation.dynanav.pct_scan_waypoints import (
    PctScanWaypointTracker, atomic_json,
)


class YawRig:
    def __init__(self, directory):
        self.mailbox = Path(directory) / "run" / "mailbox"
        self.adapter = PctScanWaypointTracker(dict(
            slam_pose_source="livo_post_vio", speed_mps=.5, motion_end_s=None,
            interactive_mailbox=str(self.mailbox)))
        self.last_inputs = None
        self._sample_id = 0

    def request(self, seq=1, **changes):
        payload = dict(version=1, run_id="run", seq=seq, action="yaw",
                       pose_source="livo_post_vio", created_wall_s=10., ttl_s=30.)
        payload.update(changes)
        self.adapter._handle_request(payload, now_wall=10.)
        return payload

    def tick(self, stamp=10., yaw=0., **changes):
        inputs = dict(current_sim_s=stamp, odom_stamp_s=stamp, position=[0., 0., 0.],
            quat_wxyz=[math.cos(yaw/2), 0., 0., math.sin(yaw/2)],
            estimated_velocity=None, measured_yaw_rate=0., ready=True,
            now_wall=stamp, arrival_wall=stamp, frame_id="camera_init", child_frame_id="body",
            pose_source="livo_post_vio", imu_source_stamp_s=stamp, imu_arrival_wall_s=stamp,
            imu_frame="go2_imu", imu_valid=True)
        inputs.update(changes)
        self.last_inputs = inputs
        return self.adapter.tick(**inputs)

    def receipts(self):
        if not self.adapter.receipts_path.exists():
            return []
        return [json.loads(line) for line in self.adapter.receipts_path.read_text().splitlines()]

    def events(self, event=None):
        return [row for row in self.receipts() if event is None or row["event"] == event]

    def native_path(self, revision=1):
        request = json.loads((self.mailbox / "planner_request.json").read_text())
        atomic_json(self.mailbox / "planner_path.json", dict(request, revision=revision,
            status="tracking", source_stamp_s=10., points_xyz_m=[[0., 0., 0.], [4., 0., 0.]],
            provenance={"local_planner": "native_SCAN"}))

    def query(self):
        return json.loads((self.mailbox / "planner_yaw_query.json").read_text())

    def feedback(self, *, source_changes=None, **changes):
        query, inputs = self.query(), self.last_inputs
        w, x, y, z = inputs["quat_wxyz"]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        stamp_ns = round(inputs["odom_stamp_s"] * 1e9)
        quaternion = [x, y, z, w]
        source = dict(body_stamp_ns=stamp_ns, body_frame_id=inputs["frame_id"],
            body_child_frame_id=inputs["child_frame_id"], body_quaternion_xyzw=quaternion,
            cloud_stamp_ns=stamp_ns, cloud_frame_id=inputs["frame_id"],
            sensor_pose_stamp_ns=stamp_ns, sensor_frame_id=inputs["frame_id"],
            sensor_child_frame_id=inputs["child_frame_id"], sensor_pose_xyz_m=inputs["position"],
            sensor_quaternion_xyzw=quaternion, fusion_ray_position_xyz_m=inputs["position"],
            fusion_ray_quaternion_xyzw=quaternion, fusion_sim_s=inputs["odom_stamp_s"],
            map_revision=1)
        source.update(source_changes or {})
        target = query["target_yaw_rad"]
        self._sample_id += 1
        payload = dict(version=1, run_id=query["run_id"], seq=query["seq"], sample_id=self._sample_id,
            query_id=query["query_id"], status="clear", reason="native_rotation_clear",
            target_yaw_rad=target, body_stamp_ns=stamp_ns, position_xyz_m=inputs["position"],
            yaw_rad=yaw, checked_arc=dict(start_yaw_rad=yaw, target_yaw_rad=target,
                delta_yaw_rad=math.atan2(math.sin(target-yaw), math.cos(target-yaw)), queries=3),
            first_blocking_yaw_rad=None, occupancy=0, blocking_part=None, source=source,
            native_identity=dict(pid=731, startup_nonce="acceptance-native-child"),
            received_wall_s=inputs["now_wall"])
        payload.update(changes)
        self.write_feedback(payload)
        return payload

    def write_feedback(self, payload):
        atomic_json(self.mailbox / "planner_yaw_space.json", payload)
