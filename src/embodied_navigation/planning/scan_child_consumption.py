"""Scoped evidence that the owned SCAN child consumed actual source odometry.

No position, velocity, commands, source timestamp refresh or navigation permit
is produced. Readiness still requires the original exact outside source slots.
"""
import copy
import json
import math
from collections import OrderedDict
from pathlib import Path

TOPIC = '/demo/navigation/scan_child_odometry_consumed'
SCHEMA = 'r85_scan_child_odometry_consumed/v1'
FLAG = 'require_child_odometry_consumption_ack'
GID_BYTES = 16  # Source-bound installed Jazzy rmw/types.h RMW_GID_STORAGE_SIZE.


def enabled(profile):
    config=profile.get('r85_auto_reference_contract',{})
    if not isinstance(config,dict):raise ValueError('Planner options must be an object')
    value=config.get(FLAG,False)
    if type(value)is not bool:raise ValueError('Planner ACK option must be boolean')
    return value


def gid_hex(value):
    if isinstance(value, str):
        result = value
    else:
        try: result = bytes(value).hex()
        except (TypeError, ValueError): raise ValueError('Invalid publisher GID')
    if (len(result) != 2*GID_BYTES or any(c not in '0123456789abcdef' for c in result)
            or result == '0'*(2*GID_BYTES)):
        raise ValueError('Invalid publisher GID')
    return result


def verify_owned_child(saved, current, binary):
    if not isinstance(saved, dict) or not isinstance(current, dict):
        raise ValueError('Owned SCAN child identity unavailable')
    for key in ('pid', 'ppid', 'pgid', 'start_ticks'):
        if type(saved.get(key)) is not int or saved[key] <= 0 or current.get(key) != saved[key]:
            raise ValueError('Owned SCAN child identity differs: '+key)
    if Path(saved.get('exe', '')).resolve() != Path(binary).resolve() or current.get('exe') != saved['exe']:
        raise ValueError('Owned SCAN child executable differs')


def verify_startup_command(saved, current, binary, supervisor_pid, nonce, command):
    verify_owned_child(saved, current, binary)
    if type(supervisor_pid) is not int or saved['ppid'] != supervisor_pid:
        raise ValueError('SCAN child parent differs from this supervisor')
    if not isinstance(nonce, str) or len(nonce) != 32 or any(c not in '0123456789abcdef' for c in nonce):
        raise ValueError('Invalid fresh child startup nonce')
    if (command.count('fsm.child_odometry_consumption_ack:=true') != 1
            or command.count('fsm.child_odometry_ack_nonce:='+nonce) != 1):
        raise ValueError('Actual owned child argv lacks its exact ACK nonce/flag')


class ChildConsumption:
    def __init__(self, run_id, epoch, generation, saved, nonce, restart_clock_ns):
        self.run_id, self.epoch, self.generation = run_id, epoch, generation
        self.saved, self.nonce, self.restart_clock_ns = copy.deepcopy(saved), nonce, restart_clock_ns
        self.rows = OrderedDict()

    def observe(self, text, clock_ns, wall):
        if not isinstance(text, str) or len(text.encode('utf-8')) > 4096:
            raise ValueError('Child odometry ACK exceeds finite record bound')
        d = json.loads(text)
        if not isinstance(d, dict): raise ValueError('Child odometry ACK must be an object')
        expected = dict(schema=SCHEMA, run_id=self.run_id, planner_epoch=self.epoch,
            localization_generation=self.generation, planner_pid=self.saved['pid'],
            planner_ppid=self.saved['ppid'], planner_start_ticks=self.saved['start_ticks'],
            startup_nonce=self.nonce, source_frame_id='camera_init', have_odom=True)
        for key, value in expected.items():
            if type(d.get(key)) is not type(value) or d[key] != value:
                raise ValueError('Child odometry ACK scope differs: '+key)
        ns, sequence = d.get('source_stamp_ns'), d.get('ack_sequence')
        if (type(ns) is not int or type(sequence) is not int or sequence <= 0 or ns <= 0
                or ns < self.restart_clock_ns or type(clock_ns) is not int
                or not 0 <= clock_ns-ns < 300_000_000
                or type(wall) not in (int, float) or not math.isfinite(wall)):
            raise ValueError('Child odometry ACK source header/sequence is invalid or stale')
        # Installed Jazzy MessageInfo has no per-message GID. This is the GID
        # emitted by the exact source-bound child's get_gid(), subsequently
        # matched to the unique actual graph endpoint and owned process scope.
        gid = gid_hex(d.get('publisher_gid'))
        if self.rows and sequence <= next(reversed(self.rows.values()))['data']['ack_sequence']:
            raise ValueError('Child odometry ACK sequence did not advance')
        self.rows[ns] = dict(data=d, received_wall=wall, publisher_gid=gid)
        self.rows.move_to_end(ns)
        while len(self.rows) > 16: self.rows.popitem(last=False)

    def evidence(self, confirmed, slots, endpoints, current, binary, clock_ns, wall):
        verify_owned_child(self.saved, current, binary)
        if len(endpoints) != 1:
            return dict(ready=False, reason='child_ACK_publisher_not_unique', matched_source_stamps=[])
        endpoint = endpoints[0]
        if endpoint.node_name != 'scan_planner_node' or endpoint.node_namespace != '/':
            return dict(ready=False, reason='child_ACK_graph_node_differs', matched_source_stamps=[])
        graph_gid = gid_hex(endpoint.endpoint_gid)
        matched = []
        for ns in confirmed:
            row, slot = self.rows.get(ns), slots.get(ns, {})
            if row is None or row['publisher_gid'] != graph_gid: continue
            if not (0 <= wall-row['received_wall'] < .3 and 0 <= clock_ns-ns < 300_000_000): continue
            if not all(role in slot and 0 <= wall-slot[role][1] < .3 for role in ('metadata', 'odom', 'cloud')): continue
            metadata = slot['metadata'][0]
            if (metadata.get('source_ns') != ns or metadata.get('generation') != self.generation
                    or metadata.get('correction_transition') is not False): continue
            matched.append(ns)
        proof = dict(ready=len(matched) >= 2, reason='owned_child_consumed_original_fresh_source_slots'
            if len(matched) >= 2 else 'waiting_owned_child_consumption_of_two_original_source_slots',
            matched_source_stamps=matched, publisher_gid=graph_gid,
            planner_identity=copy.deepcopy(self.saved), startup_nonce=self.nonce,
            source_timestamp_refreshed=False, original_two_source_and_300ms_gates_retained=True,
            message_info_publisher_gid_available=False, transport_level_authentication=False,
            attribution_scope='private_domain_owned_processes_nonmalicious_publishers')
        if matched: proof['latest_actual_child_ACK'] = copy.deepcopy(self.rows[matched[-1]]['data'])
        return proof
