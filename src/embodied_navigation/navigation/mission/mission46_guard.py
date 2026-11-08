"""Independent, timestamp preserving controller gate for mission coordination."""
import hashlib
import json
from pathlib import Path

TRANSITIONS = {'exploration:11': ('lower12', 'upper23'),
               'return_origin:5': ('upper23', 'lower12'),
               'navigation_f1_f3:7': ('lower12', 'upper23')}


def mission_hold(run, request_id, arrivals, goal_hash, now_ns, wall_s):
    run = Path(run).resolve()
    try:
        control = json.loads((run / 'mission46_control.json').read_text())
        if (control.get('run_id') != run.name or type(control.get('sim_ns')) is not int
                or not 0 <= now_ns-control['sim_ns'] <= 300000000
                or not 0 <= wall_s-control['monotonic_wall'] <= .3
                or control.get('navigation_ground_truth_used') is not False
                or type(control.get('hold')) is not bool):
            return True, 'mission coordination expired or malformed'
        if control['hold']:
            return True, str(control.get('reason', 'mission transition hold'))
        if control.get('current_request') != request_id:
            return True, 'mission request identity is not released'
        connectors = [r for r in arrivals if r.get('goal_id') in TRANSITIONS]
        if connectors:
            arrival = connectors[-1]
            ack = json.loads((run / 'mission46_terrain_ack.json').read_text())
            before, after = TRANSITIONS[arrival['goal_id']]
            evidence = Path(ack['source_evidence_file']).resolve()
            if (not evidence.is_relative_to(run) or not evidence.is_file()
                    or hashlib.sha256(evidence.read_bytes()).hexdigest() != ack['source_evidence_sha256']
                    or ack.get('run_id') != run.name or ack.get('request_id') != request_id
                    or ack.get('goal_id') != arrival['goal_id'] or ack.get('arrival_receipt') != arrival
                    or ack.get('goals_definition_sha256') != goal_hash
                    or ack.get('from_layer') != before or ack.get('to_layer') != after
                    or ack.get('passed') is not True or ack.get('navigation_ground_truth_used') is not False
                    or type(ack.get('effective_sim_ns')) is not int
                    or not arrival['stamp_ns'] <= ack['effective_sim_ns'] <= now_ns):
                return True, 'connector awaits verified causal Actor provider acknowledgment'
        return False, 'fresh mission release'
    except (OSError, KeyError, ValueError, TypeError):
        return True, 'mission input or connector acknowledgment not ready'
