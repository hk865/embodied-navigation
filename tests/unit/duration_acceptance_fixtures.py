"""CPU-only measured mailbox setup shared by duration acceptance checks."""
import json
from pathlib import Path

from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker


class MailboxRig:
    def __init__(self, directory, tracker_class, deadline=None):
        self.box = Path(directory) / 'generation' / 'interactive'
        self.adapter = tracker_class(dict(
            slam_pose_source='livo_post_vio', speed_mps=.15,
            motion_end_s=deadline, duration_s=None,
            interactive_mailbox=str(self.box)))
        self.planned = tracker_class is PctScanWaypointTracker

    def request(self, seq=1, action='goto'):
        self.adapter._handle_request(dict(
            version=1, run_id='generation', seq=seq, action=action,
            pose_source='livo_post_vio', goal_xyz_m=[4., 0., 0.]))

    def tick(self, sim=2000., **changes):
        values = dict(current_sim_s=sim, odom_stamp_s=sim,
            position=[0., 0., 0.], quat_wxyz=[1., 0., 0., 0.],
            estimated_velocity=None, measured_yaw_rate=0., ready=True,
            now_wall=sim, arrival_wall=sim, frame_id='camera_init',
            child_frame_id='body', pose_source='livo_post_vio')
        values.update(changes)
        return self.adapter.tick(**values)

    def start(self, sim=2000.):
        self.request()
        self.tick(sim)
        if self.planned:
            request = json.loads((self.box / 'planner_request.json').read_text())
            path = dict(request, revision=1, status='tracking',
                points_xyz_m=[[0., 0., 0.], [4., 0., 0.]],
                provenance={'local_planner': 'native_SCAN'})
            (self.box / 'planner_path.json').write_text(json.dumps(path))
        return self.tick(sim + .1)

    def receipts(self):
        return [json.loads(x) for x in self.adapter.receipts_path.read_text().splitlines()]
