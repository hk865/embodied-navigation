#!/usr/bin/env python3
"""Parameterised native DynaNav SOP; sends no motion unless launch/goto/cancel/stop is invoked.

Use --help and references/native-interface.md. All run/domain/coordinate inputs
are explicit. This is an operator/evidence wrapper around the existing client,
not a planner, a new waypoint gate, or a direct velocity controller.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def utc():
    return datetime.now(timezone.utc).isoformat()


def load(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def rows(path):
    try:
        lines = Path(path).read_text().splitlines()
    except OSError:
        return []
    result = []
    for line in lines:
        try:
            value = json.loads(line)
            if isinstance(value, dict):
                result.append(value)
        except ValueError:
            pass
    return result


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temp.replace(path)


def emit(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def journal(args, event, **data):
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    row = dict(recorded_utc=utc(), generation_id=args.run_dir.name,
               source_run_dir=str(args.run_dir), event=event, **data)
    with (args.evidence_dir / 'sop_journal.jsonl').open('a') as handle:
        handle.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
    return row


def capture(args):
    return args.run_dir / 'capture'


def mailbox(args):
    return capture(args) / 'interactive'


def client(args, *arguments):
    return [sys.executable, str(args.repo / 'scripts/explore_waypoint_client.py'),
            *arguments, '--run-dir', str(capture(args))]


def inside(path, root):
    path = Path(path).resolve()
    if not path.is_relative_to(Path(root).resolve()):
        raise ValueError(f'current-run source path is outside {root}: {path}')
    return path


def observe(args):
    result = subprocess.run(client(args, 'observe', '--receipts', '40'),
                            capture_output=True, text=True, cwd=args.repo, timeout=30)
    if result.returncode:
        raise RuntimeError(f'observe failed ({result.returncode}): {result.stderr or result.stdout}')
    summary = json.loads(result.stdout)
    raw_path = mailbox(args) / 'observation.json'
    raw_value = load(raw_path)
    raw_status = ('missing' if not raw_path.exists() else
                  'unreadable_or_invalid_object' if not isinstance(raw_value, dict) else
                  'empty_object' if not raw_value else 'available')
    raw = raw_value if isinstance(raw_value, dict) else {}
    position = raw.get('pose', {}).get('position_xyz_m')
    goal = raw.get('waypoints', {}).get('active_goal_xyz_m')
    summary.update(generation_id=args.run_dir.name, failure=raw.get('failure'),
                   raw_observation_source_status=raw_status,
                   raw_observation_available=raw_status == 'available',
                   terminal_reason=raw.get('terminal_reason'),
                   vertical_error_m=abs(position[2] - goal[2]) if position and goal else None,
                   observation_file_wall_age_s=time.time() - raw_path.stat().st_mtime if raw_path.exists() else None)
    return summary, raw


def request_outcome(args, seq, action, acknowledge_only=False):
    """Close only the requested sequence; distinguish goal and run failures."""
    expected_run = capture(args).name  # Current native mailbox contract: usually 'capture'.
    receipts = rows(mailbox(args) / 'receipts.jsonl')
    matching = [r for r in receipts if r.get('run_id') == expected_run and
                type(r.get('seq')) is int and r['seq'] == seq]
    accepted_event = 'waypoint_accepted' if action == 'goto' else action + '_accepted'
    accepted = next((r for r in matching if r.get('event') == accepted_event), None)
    for row in reversed(matching):
        event = row.get('event', '')
        if event.endswith('_rejected') or event in ('waypoint_failed', 'waypoint_dropped',
                'waypoint_expired', 'action_expired', 'action_cancelled'):
            return dict(state='failed', seq=seq, accepted_receipt=accepted, closing_receipt=row)
        success = ('waypoint_reached' if action == 'goto' else action + '_accepted')
        if event == success:
            return dict(state='reached' if action == 'goto' else 'acknowledged', seq=seq,
                        accepted_receipt=accepted, closing_receipt=row)
    if accepted and acknowledge_only:
        return dict(state='accepted', seq=seq, accepted_receipt=accepted)
    if accepted and action == 'goto':
        later_control = next((r for r in receipts if r.get('run_id') == expected_run
                              and type(r.get('seq')) is int and r['seq'] > seq
                              and r.get('event') in ('cancel_accepted', 'stop_accepted')), None)
        if later_control:
            return dict(state='cancelled_by_later_control_ack', seq=seq,
                        accepted_receipt=accepted, closing_control_receipt=later_control)
    raw = load(mailbox(args) / 'observation.json', {}) or {}
    failure = raw.get('failure') or raw.get('waypoints', {}).get('failure')
    terminal = raw.get('terminal_reason') or raw.get('waypoints', {}).get('terminal_reason')
    global_terminal = next((r for r in reversed(receipts)
                            if r.get('run_id') == expected_run and r.get('event') == 'terminal'), None)
    # A retained goal failure must not close a newer sequence or declare the
    # simulator dead. Run-level termination needs explicit terminal evidence.
    terminal_phase = (raw.get('phase') == 'terminal' or
                      raw.get('status') == 'terminal' or
                      raw.get('waypoints', {}).get('phase') == 'terminal')
    if terminal or global_terminal or (failure and terminal_phase):
        return dict(state='run_terminal_without_this_action_closing_receipt', seq=seq,
                    accepted_receipt=accepted, failure=failure, terminal_reason=terminal,
                    global_terminal_receipt=global_terminal)
    return dict(state='accepted_pending' if accepted else 'submission_pending', seq=seq,
                accepted_receipt=accepted, active_seq=raw.get('waypoints', {}).get('active_seq'),
                unscoped_failure=failure,
                last_waypoint_failure=raw.get('waypoints', {}).get('last_failure'))


def wait_existing(args, seq, action, timeout_s, acknowledge_only=False):
    end = time.monotonic() + timeout_s
    while True:
        outcome = request_outcome(args, seq, action, acknowledge_only)
        if outcome['state'] not in ('submission_pending', 'accepted_pending'):
            return outcome
        if time.monotonic() >= end:
            return dict(outcome, state='timeout_uncertain', last_observed_state=outcome['state'],
                        timeout_s=timeout_s, instruction='Observe or wait on this same seq; do not resend blindly.')
        time.sleep(min(.1, max(0, end-time.monotonic())))


def status(args):
    summary, _ = observe(args)
    journal(args, 'status', observation=summary)
    emit(summary)
    return 0


def send(args):
    before = load(mailbox(args) / 'request.json', {})
    prior_observation = load(mailbox(args) / 'observation.json', {}) or {}
    active_pose_source = (
        prior_observation.get('pose_source')
        or (prior_observation.get('pose') or {}).get('source')
        or 'livo_post_vio'
    )
    command = [args.command]
    if args.command == 'goto':
        command += ['--pose-source', active_pose_source, '--x', str(args.x),
                    '--y', str(args.y), '--z', str(args.z)]
    try:
        result = subprocess.run(client(args, *command), cwd=args.repo,
                                capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError) as exc:
        after = load(mailbox(args) / 'request.json', {})
        outcome = dict(state='submission_uncertain', error=str(exc), before_request=before,
                       after_request=after, instruction='Inspect request, observation and receipts before any retry.')
        journal(args, 'request_uncertain', action=args.command, outcome=outcome)
        emit(outcome)
        return 2
    payloads = []
    for line in result.stdout.splitlines():
        try:
            value = json.loads(line)
            if isinstance(value, dict):
                payloads.append(value)
        except ValueError:
            pass
    submitted = next((v for v in payloads if v.get('action') == args.command
                      and type(v.get('seq')) is int), None)
    if not submitted:
        after = load(mailbox(args) / 'request.json', {})
        uncertain = after != before
        outcome = dict(state='submission_uncertain' if uncertain else 'local_submission_failed',
                       client_exit_code=result.returncode, stdout=result.stdout, stderr=result.stderr,
                       after_request=after, instruction='Do not resend automatically.')
        journal(args, 'request_unconfirmed', action=args.command, outcome=outcome)
        emit(outcome)
        return 2 if uncertain else 1
    journal(args, 'request_submitted', request=submitted, client_exit_code=result.returncode,
            decision_note=args.note, evidence_tag=args.evidence_tag,
            pre_submission_waypoints=prior_observation.get('waypoints'))
    # Client exit0 is submission only. Poll existing receipts; never submit again here.
    outcome = wait_existing(args, submitted['seq'], args.command, args.timeout,
                            acknowledge_only=args.ack_only)
    journal(args, 'request_observed', action=args.command, outcome=outcome)
    emit(dict(generation_id=args.run_dir.name, action=args.command, **outcome))
    return 0 if outcome['state'] in ('reached', 'acknowledged', 'accepted') else 2 if outcome['state'] == 'timeout_uncertain' else 1


def wait_command(args):
    outcome = wait_existing(args, args.seq, args.action, args.timeout, args.ack_only)
    journal(args, 'existing_request_observed', action=args.action, outcome=outcome)
    emit(outcome)
    return 0 if outcome['state'] in ('reached', 'acknowledged', 'accepted') else 2 if outcome['state'] == 'timeout_uncertain' else 1


def _load_nav_trace_xyz(run_dir, expected_frame=None, max_points=400):
    """Load subsampled visited XYZ positions from the current generation's control log."""
    import numpy as np

    log_path = run_dir / 'capture/interactive_waypoint_control.jsonl'
    if not log_path.is_file():
        return None
    pts = []
    try:
        with log_path.open('r', encoding='utf-8') as fh:
            for line in fh:
                if 'aligned_position_xyz_m' not in line:
                    continue
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if row.get('no_gt_pose_used') is False:
                    continue
                if expected_frame and row.get('frame_id') and row.get('frame_id') != expected_frame:
                    continue
                pos = row.get('aligned_position_xyz_m')
                if isinstance(pos, list) and len(pos) == 3 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in pos):
                    if not pts or math.hypot(pos[0] - pts[-1][0], pos[1] - pts[-1][1]) >= 0.15 or abs(pos[2] - pts[-1][2]) >= 0.10:
                        pts.append(pos)
    except OSError:
        return None
    if not pts:
        return None
    if len(pts) > max_points:
        step = max(1, len(pts) // max_points)
        pts = pts[::step] + [pts[-1]]
    return np.asarray(pts, dtype=np.float64)


def plot_levels(metadata, observation, destination, run_dir, repo=None):
    import numpy as np
    if repo is not None and str(Path(repo) / 'src') not in sys.path:
        sys.path.insert(0, str(Path(repo) / 'src'))
    from embodied_navigation.planning.exploration.visual_map_overlay import render_annotated_2d_maps

    pose = observation.get('pose', {})
    position, yaw = pose.get('position_xyz_m'), pose.get('yaw_rad')
    active = (metadata.get('last_frame') or {}).get('level_index')
    backend_status = load(run_dir / 'backend_status.json', {}) or {}
    trace_xyz = _load_nav_trace_xyz(run_dir, expected_frame=pose.get('frame'))
    products, notes = [], []
    for number, (key, level) in enumerate(metadata.get('levels', {}).items()):
        export = level.get('grid_export') or {}
        if not export.get('npz'):
            notes.append(f'height chart {key}: grid export missing')
            continue
        source = inside(export['npz'], run_dir)
        local = destination / f'height_chart_{number:03d}.npz'
        manifest_path = inside(export.get('manifest', source.with_suffix('.json')), run_dir)
        for attempt in range(3):
            layer_manifest = load(manifest_path, {}) or {}
            grid_data = source.read_bytes()
            recorded_hash = layer_manifest.get('npz_sha256')
            if not recorded_hash or hashlib.sha256(grid_data).hexdigest() == recorded_hash:
                break
        else:
            notes.append(f'height chart {key}: grid/manifest changed during three reads; no mixed overlay made')
            continue
        local.write_bytes(grid_data)
        if manifest_path.exists():
            write(local.with_suffix('.json'), layer_manifest)
        layer_index = layer_manifest.get('level_index')
        is_active = active is not None and layer_index is not None and active == layer_index
        chart_state = 'active' if is_active else 'retained' if active is not None and layer_index is not None else 'active status unknown'
        with np.load(local, allow_pickle=False) as data:
            resolution = float(export['resolution_m'])
            ix, iy, codes = data['ix'], data['iy'], data['code']
        pose_frame = pose.get('frame')
        grid_frame = layer_manifest.get('odometry_map_frame')
        transform_se3 = None
        matching_frame = bool(pose_frame and pose_frame == grid_frame)
        if not matching_frame and pose_frame and grid_frame:
            if (
                pose_frame == backend_status.get('world_frame', 'map')
                and grid_frame == backend_status.get('odom_frame')
                and isinstance(backend_status.get('T_world_odom'), list)
            ):
                mat = np.asarray(backend_status['T_world_odom'], dtype=np.float64)
                if mat.shape == (4, 4) and np.isfinite(mat).all():
                    transform_se3 = mat
                    matching_frame = True
        level_z_m = float(level.get('level_z_m', 0.0) or 0.0)
        image = destination / f'height_chart_{number:03d}.png'
        local_image = (destination / f'height_chart_{number:03d}_local.png') if is_active else None
        overlay = render_annotated_2d_maps(
            ix,
            iy,
            codes,
            resolution,
            robot_xyz=position if (position is not None and matching_frame) else None,
            yaw=yaw if (yaw is not None and matching_frame) else None,
            level_z_m=level_z_m,
            global_path=image,
            local_path=local_image,
            title=f'Height chart {key} / {chart_state}',
            trace_xyz=trace_xyz if matching_frame else None,
            transform_se3=transform_se3,
        )
        if not matching_frame:
            notes.append(f'height chart {key}: estimated robot marker omitted because matching map-frame provenance is unavailable')
        products.append(dict(
            level_key=key,
            level_index=layer_index,
            active=is_active,
            image=overlay['image'],
            local_image=overlay['local_image'],
            turning_circle=overlay['turning_circle'],
            candidates=overlay['candidates'],
            grid=str(local),
            metadata=level,
        ))
    return products, notes


def snapshot(args):
    if Path(args.tag).name != args.tag or args.tag in ('.', '..'):
        raise ValueError('tag must be one directory name')
    destination = args.evidence_dir / args.tag
    destination.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('MPLCONFIGDIR', str(args.evidence_dir / '.matplotlib'))
    summary, raw = observe(args)
    write(destination / 'observation.json', summary)
    write(destination / 'raw_native_observation.json', raw)
    notes, products = [], {}
    raw_available = summary.get('raw_observation_available', bool(raw))
    if not raw_available:
        notes.append(f'Current-run raw native observation is {summary.get("raw_observation_source_status", "unavailable")}; '
                     'raw_native_observation.json is an empty placeholder. Estimated-pose 3D overlay is omitted; '
                     'reread the current observation source. No pose or source time is invented.')
    frames = summary.get('sim_frames') or {}
    if frames.get('latest'):
        from PIL import Image
        import io
        for attempt in range(3):
            try:
                source = inside(frames['latest'], capture(args))
                image_data = source.read_bytes()
                Image.open(io.BytesIO(image_data)).verify()
                (destination / 'rgb.png').write_bytes(image_data)
                products['rgb'] = str(destination / 'rgb.png')
                source_rows = rows(inside(frames['index'], capture(args))) if frames.get('index') else []
                matching = [row for row in source_rows if row.get('path') == str(source)]
                write(destination / 'rgb_source.json', matching[-1] if matching else
                      dict(source_path=str(source), source_stamp_unavailable=True))
                break
            except (OSError, ValueError, SyntaxError) as exc:
                if attempt == 2:
                    notes.append(f'Front RGB unavailable after three reads: {exc}')
                else:
                    time.sleep(.05)
    else:
        notes.append('Front RGB is not available; no synthetic or overview image substituted.')
    metadata = load(capture(args) / 'operator_map/latest.json')
    if metadata:
        write(destination / 'map.json', metadata)
        try:
            products['height_charts'], level_notes = plot_levels(metadata, summary, destination, args.run_dir, repo=args.repo)
            notes += level_notes
        except (OSError, ValueError, KeyError) as exc:
            notes.append(f'Per-height-chart view unavailable: {exc}; reread current map sources.')
    else:
        notes.append('Current-run operator map is not available.')
    backend_live = load(args.run_dir / 'backend_status.json')
    if isinstance(backend_live, dict):
        products['backend_status'] = {
            k: backend_live.get(k)
            for k in ('adopted', 'loops', 'keyframes', 'last_error', 'world_frame', 'odom_frame', 'stats')
        }
    if args.cloud_dir:
        cloud_dir = args.cloud_dir.resolve()
        cloud_summary = load(cloud_dir / 'summary.json', {}) or {}
        if cloud_summary.get('run') != args.run_dir.name:
            notes.append('3D overlay refused: recorder generation does not match explicit current run.')
        else:
            if not (cloud_dir / 'latest_paired_cloud.npz').exists():
                notes.append('Same-generation latest single-frame cloud latest_paired_cloud.npz is missing; '
                             'accumulated cloud views do not replace this raw paired-frame evidence.')
            for name in ('measured_partial_cloud.npz', 'latest_paired_cloud.npz', 'summary.json'):
                if (cloud_dir / name).exists():
                    shutil.copy2(cloud_dir / name, destination / name)
            if not (destination / 'measured_partial_cloud.npz').exists():
                notes.append('Same-generation accumulated cloud measured_partial_cloud.npz is missing; '
                             'no accumulated 3D views are made. Any available latest paired frame is retained separately.')
            elif not raw_available:
                write(destination / 'render_execution.json',
                      dict(status='skipped_raw_observation_unavailable', source_status=summary.get('raw_observation_source_status'),
                           reason='No current raw estimated pose is available for the 3D robot-pose overlay; cloud evidence is retained.'))
            else:
                renderer = Path(__file__).with_name('render_live_3d_views.py')
                command = [sys.executable, str(renderer), '--cloud', str(destination / 'measured_partial_cloud.npz'),
                           '--observation', str(destination / 'raw_native_observation.json'),
                           '--output-dir', str(destination), '--prefix', 'scene',
                           '--expected-run-id', args.run_dir.name, '--observation-run-id', args.run_dir.name,
                           '--roi-span', str(args.roi_span), '--z-span', str(args.z_span)]
                result = subprocess.run(command, capture_output=True, text=True, timeout=120)
                write(destination / 'render_execution.json', dict(command=command, exit_code=result.returncode,
                      stdout=result.stdout, stderr=result.stderr))
                if result.returncode:
                    notes.append('3D rendering failed; inspect render_execution.json. No overlay invented.')
                else:
                    products['three_dimensional_views'] = json.loads(result.stdout)
    else:
        notes.append('No cloud directory supplied: actual 3D evidence is missing, not inferred from 2D.')
    record = dict(generation_id=args.run_dir.name, captured_utc=utc(), tag=args.tag,
                  products=products, notes=notes,
                  semantics='Measured partial current-run evidence only; footprint and point count are not room coverage.')
    write(destination / 'snapshot_manifest.json', record)
    journal(args, 'snapshot', manifest=str(destination / 'snapshot_manifest.json'), notes=notes)
    emit(record)
    return 0


def note(args):
    record = journal(args, 'coverage_gap' if args.kind == 'gap' else 'decision_note',
                     evidence_tag=args.evidence_tag, text=args.text, state=args.state,
                     limitation='Human assessment from current sensors; not automatic room completeness or waypoint admission.')
    emit(record)
    return 0


def verify_exit(args):
    exit_file = args.run_dir / 'capture_exit.txt'
    alive = []
    pids = args.run_dir / 'owned_pids.txt'
    for token in pids.read_text().split() if pids.exists() else []:
        try:
            pid = int(token)
            os.kill(pid, 0)
            alive.append(pid)
        except ProcessLookupError:
            pass
        except PermissionError:
            alive.append(pid)
    lifecycle = (args.run_dir / 'lifecycle.log').read_text() if (args.run_dir / 'lifecycle.log').exists() else ''
    # Read only the labelled after-run finish fields, after capture has exited.
    # Never feed native pose/velocity or the full result back into target choice.
    final = load(capture(args) / 'result.json', {}) if exit_file.exists() else {}
    final = final or {}
    finish = {key: final.get(key) for key in ('outcome', 'error', 'end_reason',
              'navigation_terminal_request', 'body_contact_steps') if key in final}
    result = dict(capture_exit_recorded=exit_file.exists(),
                  capture_exit_code=exit_file.read_text().strip() if exit_file.exists() else None,
                  cleanup_recorded=any(line.startswith('cleanup ') for line in lifecycle.splitlines()),
                  listed_pids_currently_alive=alive,
                  offline_finish=finish,
                  native_parking_confirmed=final.get('end_reason') == 'navigation_terminal_and_parked',
                  limitation='PID liveness alone cannot distinguish reuse. Native after-run finish/contact fields are diagnostics, not navigation inputs; body contact/falls require explicit reporting.')
    journal(args, 'exit_check', **result); emit(result)
    closed = result['capture_exit_recorded'] and result['cleanup_recorded'] and not alive
    if not closed:
        return 2
    failure = final.get('error') or (final.get('navigation_terminal_request') or {}).get('failure')
    return 0 if result['capture_exit_code'] == '0' and result['native_parking_confirmed'] and not failure else 1


def launch(args):
    if args.run_dir.exists():
        raise ValueError('launch requires a new run directory; no checkpoint/resume is implemented')
    if not 8 <= args.duration_sim_s <= 1800 or not math.isfinite(args.duration_sim_s):
        raise ValueError('duration must be finite and within 8..1800 simulated seconds')
    if not 0 < args.speed_mps <= 1 or not math.isfinite(args.speed_mps):
        raise ValueError('native requested speed ceiling must be within (0,1] m/s')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain_id),
                       SPEED_MPS=str(args.speed_mps), DURATION_SIM_S=str(args.duration_sim_s),
                       PYTHONDONTWRITEBYTECODE='1')
    journal(args, 'launch_requested', ros_domain_id=args.ros_domain_id,
            requested_speed_ceiling_mps=args.speed_mps, duration_sim_s=args.duration_sim_s,
            note='Foreground original native entry; no preselected goals, no resume/checkpoint claim.')
    result = subprocess.run(['bash', str(args.repo / 'scripts/run_dynanav_native_exploration.sh'),
                             str(args.run_dir)], cwd=args.repo, env=environment)
    journal(args, 'launch_returned', exit_code=result.returncode)
    return result.returncode


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, required=True)
    ap.add_argument('--run-dir', type=Path, required=True, help='Explicit generation root; capture is appended once')
    ap.add_argument('--evidence-dir', type=Path, required=True, help='Separate writable evidence/journal directory')
    commands = ap.add_subparsers(dest='command', required=True)
    start = commands.add_parser('launch', help='Run original launcher in foreground; requires explicit authorization to start')
    start.add_argument('--ros-domain-id', type=int, required=True)
    start.add_argument('--duration-sim-s', type=float, default=300)
    start.add_argument('--speed-mps', type=float, default=1.0)
    start.set_defaults(handler=launch)
    commands.add_parser('status').set_defaults(handler=status)
    shot = commands.add_parser('snapshot', help='Read-only RGB + per-height-chart 2D + measured same-generation 3D')
    shot.add_argument('--tag', required=True)
    shot.add_argument('--cloud-dir', type=Path)
    shot.add_argument('--roi-span', type=float, default=20)
    shot.add_argument('--z-span', type=float, default=12)
    shot.set_defaults(handler=snapshot)
    for action in ('goto', 'cancel', 'stop'):
        child = commands.add_parser(action)
        if action == 'goto':
            for axis in ('x', 'y', 'z'):
                child.add_argument('--' + axis, type=float, required=True)
        child.add_argument('--timeout', type=float, default=20)
        child.add_argument('--ack-only', action='store_true', help='For goto: stop waiting at native acceptance, not arrival')
        child.add_argument('--note', default='', help='Optional decision explanation; not a waypoint gate')
        child.add_argument('--evidence-tag', default='')
        child.set_defaults(handler=send)
    waiting = commands.add_parser('wait', help='Observe one existing seq without resubmitting')
    waiting.add_argument('--seq', type=int, required=True)
    waiting.add_argument('--action', choices=('goto', 'cancel', 'stop'), required=True)
    waiting.add_argument('--timeout', type=float, default=20)
    waiting.add_argument('--ack-only', action='store_true')
    waiting.set_defaults(handler=wait_command)
    notes = commands.add_parser('note', help='Record sensor-based decision or remaining coverage gap')
    notes.add_argument('--kind', choices=('gap', 'decision'), default='gap')
    notes.add_argument('--text', required=True)
    notes.add_argument('--state', choices=('open', 'observed', 'revisited', 'unresolved'), default='open')
    notes.add_argument('--evidence-tag', default='')
    notes.set_defaults(handler=note)
    commands.add_parser('verify-exit').set_defaults(handler=verify_exit)
    return ap


def main():
    args = parser().parse_args()
    args.repo, args.run_dir, args.evidence_dir = (p.expanduser().resolve() for p in
                                               (args.repo, args.run_dir, args.evidence_dir))
    if not (args.repo / 'scripts/explore_waypoint_client.py').is_file():
        raise ValueError('--repo does not contain the native client')
    if args.run_dir.name == 'capture':
        raise ValueError('--run-dir must be the generation root, not its capture directory')
    if hasattr(args, 'timeout') and (not math.isfinite(args.timeout) or args.timeout <= 0):
        raise ValueError('timeout must be finite and positive')
    if getattr(args, 'seq', 1) <= 0:
        raise ValueError('seq must be positive')
    return args.handler(args)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(json.dumps(dict(error=str(exc), instruction='No automatic resend was performed.'), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(3)
