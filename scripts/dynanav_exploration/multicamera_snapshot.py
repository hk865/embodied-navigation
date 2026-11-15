#!/usr/bin/env python3
"""Read-only DSH multi-view snapshot and shared measured-LIO projection.

No goto, SLAM mutation, angular point partition, scene truth or pose fusion.
The output consists of independent camera images, never a stitched panorama.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src'))
from embodied_navigation.simulation.dynanav.multicamera import nearest_pose, project_shared_map


def resolve_inside(root, name):
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Archive path escapes this generation: ' + name)
    return path


def select_frames(records, camera_ids, target_ns=None, max_skew_ns=30_000_000):
    by_camera = {name: [r for r in records if r['camera_id'] == name] for name in camera_ids}
    if any(not rows for rows in by_camera.values()):
        raise ValueError('Missing camera archive: ' + ', '.join(k for k, v in by_camera.items() if not v))
    target = min(max(r['stamp_ns'] for r in rows) for rows in by_camera.values()) if target_ns is None else int(target_ns)
    selected = [min(rows, key=lambda r: abs(r['stamp_ns'] - target)) for rows in by_camera.values()]
    stamps = [r['stamp_ns'] for r in selected]
    if max(abs(s - target) for s in stamps) > max_skew_ns or max(stamps) - min(stamps) > max_skew_ns:
        raise ValueError('No selected-camera set inside explicit source-time skew tolerance; never retimestamping pixels')
    return selected


def mosaic(images, labels):
    from PIL import Image, ImageDraw
    columns = 2 if len(images) == 4 else min(3, len(images))
    rows = (len(images) + columns - 1) // columns
    output = Image.new('RGB', (columns * 512, rows * 552), '#15181c')
    draw = ImageDraw.Draw(output)
    for index, (im, label) in enumerate(zip(images, labels)):
        x, y = (index % columns) * 512, (index // columns) * 552
        if im.size != (512, 512):
            raise ValueError('Unexpected camera image dimensions')
        output.paste(im, (x, y + 40))
        draw.text((x + 10, y + 10), label, fill='white')
    return output


def build_snapshot(run_dir, output, *, target_ns=None, images_only=False, map_npz=None,
                   max_camera_skew_ns=30_000_000, max_pose_delta_ns=30_000_000,
                   depth_tolerance_m=.15, video=False):
    from PIL import Image, ImageDraw
    run_dir = run_dir.resolve()
    archive = run_dir / 'multicamera'
    manifest = json.loads((archive / 'calibration.json').read_text())
    if manifest['generation'] != str(run_dir):
        raise ValueError('Camera manifest belongs to another generation')
    records = [json.loads(line) for line in (archive / 'index.jsonl').read_text().splitlines() if line]
    if any(r['generation'] != str(run_dir) for r in records):
        raise ValueError('Camera frame from another generation')
    calibrations = manifest['cameras']
    ids = [c['camera_id'] for c in calibrations]
    selected = select_frames(records, ids, target_ns, max_camera_skew_ns)
    if output.exists() or output.resolve() == run_dir or run_dir.is_relative_to(output.resolve()):
        raise ValueError('Use a new output directory, not a parent of this run')
    output.mkdir(parents=True)
    images = [Image.open(resolve_inside(archive, r['rgb_path'])).convert('RGB') for r in selected]
    labels = [f"{c['camera_id']} | yaw {np.rad2deg(c['yaw_base_rad']):.0f} deg | {r['source_time_s']:.6f}s" for c, r in zip(calibrations, selected)]
    mosaic(images, labels).save(output / 'view_grid.png')
    if len(images) == 6:
        mosaic(images, labels).save(output / 'six_views.png')
    report = dict(schema='dynanav-multicamera-snapshot/v1', generation=str(run_dir),
                  source_frames=selected, camera_source_skew_ns=max(r['stamp_ns'] for r in selected)-min(r['stamp_ns'] for r in selected),
                  images=f'{len(images)} independent real simulation camera images, not panorama stitching', camera_count=len(images),
                  ground_truth_used=False, estimator_or_control_changed=False,
                  independent_vio_implemented=False, pose_fusion_implemented=False,
                  missing_view_does_not_mean_free=True, map_projection=None)
    if not images_only:
        map_path = (map_npz or run_dir / 'interactive/planner_measured_map.npz').resolve()
        if not map_path.is_relative_to(run_dir):
            raise ValueError('Shared map must be inside this capture generation')
        with np.load(map_path, allow_pickle=False) as data:
            provenance = json.loads(str(data['provenance_json']))
            if provenance['run_id'] != run_dir.name or provenance['map_frame'] != 'dynanav_lio_map' or provenance['child_frame'] != 'go2_imu' or provenance['pairing'] != 'exact_integer_nanoseconds' or provenance['transform'] != 'none_already_world':
                raise ValueError('Map must be this generation exact-paired measured LIO map/go2_imu data')
            xyz = data['points_xyz_m'].copy()
            point_stamps = data['point_source_stamp_ns'].copy()
            stamps, poses = data['lio_pose_stamp_ns'].copy(), data['lio_pose_xyz_xyzw'].copy()
        if point_stamps.shape != (len(xyz),) or point_stamps.dtype.kind not in 'iu':
            raise ValueError('Missing integer per-point LIO source provenance')
        # Same common causal map for every camera, not a different azimuth slice.
        causal = point_stamps <= min(r['stamp_ns'] for r in selected)
        common_indices = np.flatnonzero(causal)
        common = xyz[causal]
        overlays, results, arrays = [], [], {}
        for image, frame, cal in zip(images, selected, calibrations):
            pose, timing = nearest_pose(stamps, poses, frame['stamp_ns'], maximum_delta_ns=max_pose_delta_ns)
            with np.load(resolve_inside(archive, frame['depth_path']), allow_pickle=False) as data:
                depth = data['axial_depth_m'].copy()
            result = project_shared_map(common, pose, cal, depth, depth_tolerance_m=depth_tolerance_m)
            visible = common_indices[result.pop('point_indices')]
            pixels = result.pop('pixels_uv')
            depths = result.pop('depths_m')
            arrays[cal['camera_id'] + '_input_point_indices'] = visible
            arrays[cal['camera_id'] + '_pixels_uv'] = pixels
            overlay = image.copy()
            draw = ImageDraw.Draw(overlay)
            for (u, v), d in zip(pixels, depths):
                ratio = min(float(d) / 20., 1.)
                color = (int(255 * ratio), 255, int(255 * (1. - ratio)))
                draw.ellipse((u-1, v-1, u+1, v+1), fill=color)
            overlay.save(output / (cal['camera_id'] + '_lio_overlay.png'))
            overlays.append(overlay)
            results.append(dict(camera_id=cal['camera_id'], timing=timing, original_input_indices_preserved=True, **result))
        np.savez_compressed(output / 'visible_input_indices.npz', **arrays)
        mosaic(overlays, labels).save(output / 'view_grid_lio_overlay.png')
        if len(images) == 6:
            mosaic(overlays, labels).save(output / 'six_views_lio_overlay.png')
        report['map_projection'] = dict(source=str(map_path), provenance=provenance,
                                        total_input_points=len(xyz), shared_causal_input_points=len(common),
                                        future_point_exclusions=int((~causal).sum()),
                                        common_source_cutoff_ns=min(r['stamp_ns'] for r in selected),
                                        artificial_angular_partition=False, camera_overlap_allowed=True,
                                        cameras=results)
    if video:
        video_frames = output / 'video_frames'
        video_frames.mkdir()
        targets = sorted({r['stamp_ns'] for r in records if r['camera_id'] == ids[0]})
        video_records = []
        for target in targets:
            try:
                group = select_frames(records, ids, target, max_camera_skew_ns)
            except ValueError:
                continue
            frames = [Image.open(resolve_inside(archive, r['rgb_path'])).convert('RGB') for r in group]
            caption = [f"{c['camera_id']} yaw {np.rad2deg(c['yaw_base_rad']):.0f} deg {r['source_time_s']:.6f}s" for c, r in zip(calibrations, group)]
            name = f'{len(video_records):06d}.png'
            mosaic(frames, caption).save(video_frames / name)
            video_records.append(dict(frame=len(video_records), source_stamps_ns=[r['stamp_ns'] for r in group]))
        (output / 'video_source_frames.json').write_text(json.dumps(video_records, indent=2))
        if video_records:
            subprocess.run(['ffmpeg', '-v', 'error', '-framerate', '1', '-i', str(video_frames / '%06d.png'),
                            '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p', str(output / 'view_grid.mp4')], check=True)
            if len(images) == 6:
                import shutil
                shutil.copyfile(output / 'view_grid.mp4', output / 'six_views.mp4')
            report['video'] = dict(path='view_grid.mp4', frames=len(video_records), playback_fps=1.,
                                   timing='1 Hz archive, timestamps labelled; missing source sets are skipped, not synthetic pixels')
    (output / 'snapshot.json').write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True, help='This generation capture directory')
    parser.add_argument('--output', type=Path, required=True, help='New evidence directory')
    parser.add_argument('--source-time', type=float)
    parser.add_argument('--images-only', action='store_true')
    parser.add_argument('--map-npz', type=Path)
    parser.add_argument('--max-camera-skew-ms', type=float, default=30.)
    parser.add_argument('--max-pose-delta-ms', type=float, default=30.)
    parser.add_argument('--depth-tolerance-m', type=float, default=.15)
    parser.add_argument('--video', action='store_true')
    args = parser.parse_args()
    for value in (args.max_camera_skew_ms, args.max_pose_delta_ms, args.depth_tolerance_m):
        if not np.isfinite(value) or value < 0:
            parser.error('Tolerances must be finite and nonnegative')
    if args.source_time is not None and (not np.isfinite(args.source_time) or args.source_time < 0):
        parser.error('Source time must be finite and nonnegative')
    try:
        report = build_snapshot(args.run_dir, args.output, target_ns=None if args.source_time is None else round(args.source_time * 1e9),
                                images_only=args.images_only, map_npz=args.map_npz,
                                max_camera_skew_ns=round(args.max_camera_skew_ms * 1e6),
                                max_pose_delta_ns=round(args.max_pose_delta_ms * 1e6),
                                depth_tolerance_m=args.depth_tolerance_m, video=args.video)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, f'Multi-camera snapshot unavailable: {error}\n')
    print(json.dumps(dict(output=str(args.output), camera_source_skew_ns=report['camera_source_skew_ns'],
                          map_projection=report['map_projection'] is not None)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
