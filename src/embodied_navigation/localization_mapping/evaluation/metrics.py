"""离线完整SE(3)评价；真值绝不回流至运行时控制。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from embodied_navigation.localization_mapping.backend.pose_graph import invert

BASE_TO_IMU = np.array([0.020153064280748367, 0., -0.005108955316245556])


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def pose(position, quaternion_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    T[:3, 3] = position
    return T


def interpolate(stamps, transforms, queries):
    """线性位置与SLERP旋转；范围外返回NaN，禁止外推。时间单位由调用者统一。"""
    stamps = np.asarray(stamps, dtype=float)
    queries = np.asarray(queries, dtype=float)
    transforms = np.asarray(transforms, dtype=float)
    if len(stamps) < 2 or np.any(np.diff(stamps) <= 0):
        raise ValueError('pose input requires at least two strictly increasing timestamps')
    valid = (queries >= stamps[0]) & (queries <= stamps[-1])
    output = np.full((len(queries), 4, 4), np.nan)
    q = queries[valid]
    output[valid] = np.eye(4)
    for axis in range(3):
        output[valid, axis, 3] = np.interp(q, stamps, transforms[:, axis, 3])
    if len(q):
        output[valid, :3, :3] = Slerp(stamps, Rotation.from_matrix(transforms[:, :3, :3]))(q).as_matrix()
    return output, valid


def initial_alignment(estimated, truth):
    """初始共同稳定窗口姿态均值与平移均值；不拟合尺度或全程轨迹。"""
    estimated, truth = np.asarray(estimated), np.asarray(truth)
    A = np.eye(4)
    A[:3, :3] = Rotation.from_matrix(truth[:, :3, :3] @ estimated[:, :3, :3].transpose(0, 2, 1)).mean().as_matrix()
    A[:3, 3] = np.mean(truth[:, :3, 3] - estimated[:, :3, 3] @ A[:3, :3].T, axis=0)
    return A


def error_values(E):
    return {'translation_m': float(np.linalg.norm(E[:3, 3])),
            'rotation_rad': float(Rotation.from_matrix(E[:3, :3]).magnitude())}


def absolute_error(estimate, truth, alignment):
    aligned = alignment @ estimate
    return {**error_values(invert(truth) @ aligned),
            'world_xyz_error_m': (aligned[:3, 3] - truth[:3, 3]).tolist(),
            'raw_rpy_xyz_rad': Rotation.from_matrix(estimate[:3, :3]).as_euler('xyz').tolist(),
            'aligned_rpy_xyz_rad': Rotation.from_matrix(aligned[:3, :3]).as_euler('xyz').tolist(),
            'truth_rpy_xyz_rad': Rotation.from_matrix(truth[:3, :3]).as_euler('xyz').tolist()}


def relative_error(xi, xj, gi, gj):
    return error_values(invert(invert(gi) @ gj) @ (invert(xi) @ xj))


def optimization_changes(before, after):
    previous = {tuple(key) if isinstance(key, list) else key: value
                for key, value in zip(before['keys'], before['poses'])}
    rows = []
    for key, value in zip(after['keys'], after['poses']):
        lookup = tuple(key) if isinstance(key, list) else key
        if lookup not in previous:
            continue
        old, new = np.array(previous[lookup]), np.array(value)
        D = new @ invert(old)
        rows.append({'key': key, 'T_after_inverse_before': D.tolist(),
                     'world_displacement_xyz_m': (new[:3, 3] - old[:3, 3]).tolist(),
                     'before_xyz_m': old[:3, 3].tolist(), 'after_xyz_m': new[:3, 3].tolist(),
                     'before_rpy_xyz_rad': Rotation.from_matrix(old[:3, :3]).as_euler('xyz').tolist(),
                     'after_rpy_xyz_rad': Rotation.from_matrix(new[:3, :3]).as_euler('xyz').tolist(),
                     **error_values(D)})
    return {'nodes': rows, 'edges_before': before['edges'], 'edges_after': after['edges']}


def summarize(rows, end_ns=None):
    def rmse(values):
        return float(np.sqrt(np.mean(np.square(values)))) if values else None
    output = {'samples': len(rows), 'ate_translation_rmse_m': rmse([r['translation_m'] for r in rows]),
              'ape_rotation_rmse_rad': rmse([r['rotation_rad'] for r in rows])}
    for delta in (10, 100):
        errors = [r[f'rpe_{delta}s'] for r in rows if r[f'rpe_{delta}s'] is not None
                  and (end_ns is None or r['stamp_ns'] + delta * 10**9 <= end_ns)]
        output[f'rpe_{delta}s'] = {'samples': len(errors), 'rte_translation_rmse_m': rmse([r['translation_m'] for r in errors]),
                                  'rotation_rmse_rad': rmse([r['rotation_rad'] for r in errors])}
    return output


def evaluate_stream(stamps, estimates, gt_stamps, truth, alignment):
    paired, valid = interpolate(gt_stamps, truth, stamps)
    rows = []
    relative = {}
    for delta in (10, 100):
        future_x, vx = interpolate(stamps, estimates, np.asarray(stamps) + delta)
        future_g, vg = interpolate(gt_stamps, truth, np.asarray(stamps) + delta)
        relative[delta] = (future_x, future_g, vx & vg)
    for i in np.flatnonzero(valid):
        row = {'stamp_ns': int(round(stamps[i] * 1e9)), **absolute_error(estimates[i], paired[i], alignment)}
        for delta, (future_x, future_g, mask) in relative.items():
            row[f'rpe_{delta}s'] = relative_error(estimates[i], future_x[i], paired[i], future_g[i]) if mask[i] else None
        rows.append(row)
    return rows


def evaluate_run(run_dir, stable_start_ns=None):
    run = Path(run_dir)
    # 文件是外部输入：重复源时间仅保留最后一次发布，避免SLERP时间轴重复。
    def ordered(rows, stamp, transform):
        pairs = {stamp(r): transform(r) for r in rows}
        times = sorted(pairs)
        return np.asarray(times), np.asarray([pairs[t] for t in times])
    gt_stamps, truth = ordered(read_jsonl(run / 'capture/truth_scoring_stream.jsonl'), lambda r: r['time'],
        lambda r: pose(r['position'], np.roll(r['quat_wxyz'], -1)) @ pose(BASE_TO_IMU, [0, 0, 0, 1]))
    streams = {}
    for name in ('frontend_post_vio', 'frontend_lio', 'backend_online'):
        streams[name] = ordered(read_jsonl(run / f'trajectories/{name}.jsonl'), lambda r: r['stamp_ns'] / 1e9,
                                 lambda r: np.asarray(r['T_parent_child']))
    common_start = float(max(gt_stamps[0], *(s[0][0] for s in streams.values())))
    start = common_start if stable_start_ns is None else stable_start_ns / 1e9
    if start < common_start:
        raise ValueError('stable alignment window precedes common recorded data')
    times, estimates = streams['frontend_post_vio']
    selection = (times >= start) & (times <= start + 5)
    paired, valid = interpolate(gt_stamps, truth, times[selection])
    if selection.sum() < 2 or not valid.all() or times[selection][-1] - times[selection][0] < 4.9:
        raise ValueError('initial common stable five-second alignment window is incomplete')
    A = initial_alignment(estimates[selection], paired)
    map_path = run / 'map/map.json'
    if map_path.exists():
        keys = json.loads(map_path.read_text())['keyframes']
        for name, field in (('final_keyframes', 'T_world_body'), ('raw_keyframes', 'T_odom_body')):
            streams[name] = ordered(keys, lambda r: r['stamp_ns'] / 1e9, lambda r: np.asarray(r[field]))
    stage_path = run / 'stages/events.jsonl'
    stages = read_jsonl(stage_path) if stage_path.exists() else []
    results = {'method': {'alignment': 'fixed initial common stable 5s, orientation mean; no full-run fit, no Sim3',
                         'alignment_source': 'frontend_post_vio; shared by all streams', 'alignment_start_ns': int(start * 1e9),
                         'stability': 'caller must verify stable window; no automatic stability claim',
                         'A': A.tolist(), 'truth_point': 'go2_imu', 'base_to_imu_xyz_m': BASE_TO_IMU.tolist(),
                         'ape': 'inverse(G) A X', 'rpe': 'inverse(inverse(Gi) Gj) (inverse(Xi) Xj)',
                         'rte': 'RPE translation norm in metres; no distance normalization',
                         'stage_rpe': 'both relative endpoints must lie within the stage; bucket RPE is grouped by start time',
                         'interpolation': 'linear position, SLERP rotation, no extrapolation; final graph sampled only at keyframes'}, 'streams': {}}
    for name, (stamps, transforms) in streams.items():
        if len(stamps) < 2:
            results['streams'][name] = {'samples': len(stamps), 'status': 'insufficient samples'}
            continue
        rows = evaluate_stream(stamps, transforms, gt_stamps, truth, A)
        buckets = {}
        for seconds in (10, 100):
            indices = sorted({int((r['stamp_ns'] / 1e9 - start) // seconds) for r in rows})
            buckets[str(seconds)] = [{'start_s': start + k * seconds, 'end_s': start + (k + 1) * seconds,
                 **summarize([r for r in rows if start + k * seconds <= r['stamp_ns'] / 1e9 < start + (k + 1) * seconds])} for k in indices]
        stage_summaries = []
        starts = {}
        for event in stages:
            if event['event'] == 'start':
                starts[event['stage']] = event['stamp_ns']
            elif event['event'] == 'end' and event['stage'] in starts:
                begin = starts.pop(event['stage'])
                stage_summaries.append({'stage': event['stage'], 'start_ns': begin, 'end_ns': event['stamp_ns'],
                    **summarize([r for r in rows if begin <= r['stamp_ns'] <= event['stamp_ns']], event['stamp_ns'])})
        results['streams'][name] = {'summary': summarize(rows), 'buckets_seconds': buckets, 'stages': stage_summaries, 'rows': rows}
    trace_path = run / 'backend_trace/events.jsonl'
    changes = []
    before = None
    for event in read_jsonl(trace_path) if trace_path.exists() else []:
        if event['event'] == 'graph_before':
            before = event
        elif event['event'] == 'graph_after' and before is not None:
            changes.append({'before_seq': before['seq'], 'after_seq': event['seq'], **optimization_changes(before['graph'], event['graph'])})
            before = None
    results['optimization_changes'] = changes
    output = run / 'evaluation'
    output.mkdir(exist_ok=False)
    (output / 'metrics.json').write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--stable-start-ns', type=int, help='已人工确认的初始稳定窗口起点；默认共同记录起点')
    args = parser.parse_args()
    evaluate_run(args.run_dir, args.stable_start_ns)


if __name__ == '__main__':
    main()
