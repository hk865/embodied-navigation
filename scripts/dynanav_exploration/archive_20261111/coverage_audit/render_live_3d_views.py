#!/usr/bin/env python3
"""Render measured same-generation LIO map points with current estimated pose.

Example:
  python3 render_live_3d_views.py --cloud current/measured_partial_cloud.npz \
    --observation current_observe.json --output-dir views --prefix s001 \
    --roi-span 20 --z-span 12 --expected-run-id native_exploration_YYYYMMDDTHHMMSSZ

Outputs local XY/XZ/YZ/oblique views, the same four full-domain views, and a
machine-readable provenance/extent record. This is point visualization only:
missing returns are not free space, projections are not clearance certificates,
and measured surface z is distinct from the estimated robot pose-origin z.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sys
import textwrap
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator


def scalar_text(array):
    value = np.asarray(array)
    return str(value.item()) if value.size == 1 else None


def read_inputs(cloud_path, observation_path, expected_run, observation_run):
    with np.load(cloud_path, allow_pickle=False) as data:
        xyz = np.asarray(data['xyz'], dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('cloud xyz must have shape (N,3)')
        stamps = np.asarray(data['source_stamp_ns'], dtype=np.int64)
        trace = np.asarray(data['lio_pose_xyz'], dtype=np.float64) if 'lio_pose_xyz' in data else np.empty((0, 3))
        embedded = {key: scalar_text(data[key]) for key in ('run_id', 'cloud_map_frame', 'cloud_stage') if key in data}
    finite = np.isfinite(xyz).all(axis=1)
    dropped = int((~finite).sum())
    xyz = xyz[finite]
    if not len(xyz):
        raise ValueError('cloud has no finite measured xyz points')
    observation = json.loads(observation_path.read_text())
    pose = observation.get('pose', {})
    position = pose.get('position_xyz_m', observation.get('aligned_position_xyz_m'))
    yaw = pose.get('yaw_rad', observation.get('aligned_yaw_rad'))
    if position is None or yaw is None:
        raise ValueError('observation must contain measured estimated position and yaw')
    position = np.asarray(position, dtype=float)
    yaw = float(yaw)
    if position.shape != (3,) or not np.isfinite(position).all() or not math.isfinite(yaw):
        raise ValueError('estimated position/yaw are invalid')
    summary_path = cloud_path.parent / 'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    run_id = embedded.get('run_id') or summary.get('run') or 'not recorded'
    frame = embedded.get('cloud_map_frame') or summary.get('cloud_map_frame') or summary.get('last_cloud_frame')
    observation_frame = pose.get('map_frame', pose.get('frame', observation.get('frame_id')))
    if expected_run and run_id == 'not recorded':
        raise ValueError('expected-run-id supplied but cloud generation metadata is absent')
    if expected_run and run_id != expected_run:
        raise ValueError(f'cloud generation {run_id!r} differs from expected {expected_run!r}; no overlay made')
    if frame and observation_frame and frame != observation_frame:
        raise ValueError(f'cloud frame {frame!r} differs from pose frame {observation_frame!r}; no transform invented')
    embedded_observation_run = observation.get('generation_id') or observation.get('source_run_id')
    if not embedded_observation_run and observation.get('run_id') not in (None, 'capture'):
        embedded_observation_run = observation['run_id']
    observation_generation = embedded_observation_run or observation_run
    if observation_run and embedded_observation_run and observation_run != embedded_observation_run:
        raise ValueError('declared observation run differs from embedded observation generation')
    if expected_run and observation_generation and observation_generation != expected_run:
        raise ValueError('observation generation differs from current cloud generation; no overlay made')
    stage = embedded.get('cloud_stage') or summary.get('cloud_stage') or 'LIO (collector contract)'
    pose_stamp = pose.get('stamp_s', observation.get('odom_stamp_s'))
    meta = {
        'cloud_file': str(cloud_path.resolve()), 'observation_file': str(observation_path.resolve()),
        'run_id': run_id, 'expected_run_id': expected_run,
        'observation_generation_id': observation_generation,
        'observation_generation_binding': 'embedded' if embedded_observation_run else ('explicit CLI declaration' if observation_run else 'not recorded; must verify current-run source'),
        'cloud_stage': stage, 'cloud_map_frame': frame or 'not embedded; inspect collector provenance',
        'estimated_pose_map_frame': observation_frame, 'estimated_pose_child_frame': pose.get('sensor_frame', pose.get('child_frame', observation.get('child_frame_id'))),
        'estimated_pose_source': pose.get('source', observation.get('pose_source')),
        'cloud_source_first_s': float(stamps.min()/1e9) if stamps.size else None,
        'cloud_source_last_s': float(stamps.max()/1e9) if stamps.size else None,
        'cloud_paired_source_count': int(stamps.size),
        'pose_source_stamp_s': pose_stamp, 'observation_sim_s': observation.get('sim_s', observation.get('current_sim_s')),
        'pose_sim_age_s': pose.get('sim_age_s', observation.get('sim_age_s')),
        'pose_wall_age_s': pose.get('wall_age_s', observation.get('wall_age_s')),
        'pose_fresh_flag': pose.get('fresh'),
        'pose_origin_xyz_m': position.tolist(), 'pose_yaw_rad': yaw,
        'input_point_count': int(len(finite)), 'finite_point_count': int(len(xyz)), 'nonfinite_dropped_for_render': dropped,
        'cloud_full_xyz_min_m': xyz.min(axis=0).tolist(), 'cloud_full_xyz_max_m': xyz.max(axis=0).tolist(),
        'render_semantics': 'Measured LIO map points only; no mesh, occupancy fill, ray carving, unknown-as-free, or pose-floor z substitution.',
        'side_view_semantics': 'Each side view projects all retained ROI depths. Yaw arrows are projected horizontally; pitch is not inferred.',
    }
    return xyz, trace, position, yaw, meta


def sample(points, maximum):
    if len(points) <= maximum:
        return points
    return points[np.random.default_rng(0).choice(len(points), maximum, replace=False)]


def heading_2d(ax, start, direction, length, label=False):
    end = np.asarray(start) + length*np.asarray(direction)
    ax.plot(*start, marker='o', ms=7, color='#e23b35', markeredgecolor='white', markeredgewidth=1.1, zorder=12)
    if np.linalg.norm(end-start) > 0.03:
        ax.annotate('', xy=end, xytext=start, arrowprops={'arrowstyle': '-|>', 'color':'#e23b35', 'lw':2.0, 'mutation_scale':15}, zorder=13)
    if label:
        ax.annotate('estimated pose origin', xy=start, xytext=(5, 7), textcoords='offset points', fontsize=8, color='#a31e19', zorder=14)


def set_2d(ax, bounds_a, bounds_b, xlabel, ylabel, local):
    ax.set_facecolor('#e3e7eb')
    ax.set_xlim(*bounds_a); ax.set_ylim(*bounds_b)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
    ax.grid(color='white', alpha=.7, linewidth=.7)
    if local:
        ax.xaxis.set_major_locator(MultipleLocator(2))
        ax.yaxis.set_major_locator(MultipleLocator(2))
        ax.xaxis.set_minor_locator(MultipleLocator(1))
        ax.yaxis.set_minor_locator(MultipleLocator(1))
        ax.grid(which='minor', color='white', alpha=.35, linewidth=.4)


def fmt(value, decimals=2):
    return f'{value:.{decimals}f}' if isinstance(value, (int, float)) else str(value)


def render(points, trace, robot, yaw, bounds, meta, destination, title, args, local):
    shown = sample(points, args.max_points)
    fig = plt.figure(figsize=(15, 12), facecolor='white')
    ax_xy = fig.add_subplot(2,2,1); ax_xz = fig.add_subplot(2,2,2)
    ax_yz = fig.add_subplot(2,2,3); ax_3d = fig.add_subplot(2,2,4,projection='3d',computed_zorder=False)
    low, high = bounds
    color_low = robot[2]-args.z_span/2; color_high = robot[2]+args.z_span/2
    scatter_kw = dict(c=shown[:,2], cmap='viridis', vmin=color_low, vmax=color_high,
                      s=args.point_size, linewidths=0, rasterized=True)
    sc = ax_xy.scatter(shown[:,0], shown[:,1], **scatter_kw)
    ax_xz.scatter(shown[:,0], shown[:,2], **scatter_kw)
    ax_yz.scatter(shown[:,1], shown[:,2], **scatter_kw)
    set_2d(ax_xy, (low[0],high[0]), (low[1],high[1]), 'map x (m)', 'map y (m)', local)
    set_2d(ax_xz, (low[0],high[0]), (low[2],high[2]), 'map x (m)', 'map z (m)', local)
    set_2d(ax_yz, (low[1],high[1]), (low[2],high[2]), 'map y (m)', 'map z (m)', local)
    dx,dy = math.cos(yaw),math.sin(yaw)
    heading_2d(ax_xy, robot[[0,1]], (dx,dy), args.heading_length, True)
    heading_2d(ax_xz, robot[[0,2]], (dx,0), args.heading_length)
    heading_2d(ax_yz, robot[[1,2]], (dy,0), args.heading_length)
    ax_xy.set_title(f'XY / top view · heading arrow {args.heading_length:g} m')
    ax_xz.set_title('XZ / side projection · yaw projected into plane')
    ax_yz.set_title('YZ / side projection · yaw projected into plane')
    if len(trace):
        valid = np.isfinite(trace).all(axis=1)
        if local:
            valid &= ((trace>=low)&(trace<=high)).all(axis=1)
        masked = np.where(valid[:,None], trace, np.nan)
        for ax, columns in [(ax_xy,(0,1)),(ax_xz,(0,2)),(ax_yz,(1,2))]:
            ax.plot(masked[:,columns[0]],masked[:,columns[1]],color='#bd4058',alpha=.5,lw=.8,zorder=9)
        ax_3d.plot(masked[:,0],masked[:,1],masked[:,2],color='#bd4058',alpha=.8,lw=1.2,zorder=5)
    # Metric scale bar in the top view. It is a measured coordinate length.
    bar = min(2.0, (high[0]-low[0])/5)
    bar_x = low[0]+.06*(high[0]-low[0]); bar_y = low[1]+.06*(high[1]-low[1])
    ax_xy.plot([bar_x,bar_x+bar],[bar_y,bar_y],color='#182b3a',lw=3,zorder=14)
    ax_xy.text(bar_x+bar/2,bar_y+.03*(high[1]-low[1]),f'{bar:g} m',ha='center',fontsize=9,color='#182b3a',zorder=15)
    ax_3d.set_facecolor('#edf0f3')
    ax_3d.scatter(shown[:,0],shown[:,1],shown[:,2],**scatter_kw,alpha=.65,zorder=1)
    ax_3d.scatter(*robot, color='#e23b35',s=55,edgecolors='white',linewidths=.8,depthshade=False,zorder=15)
    ax_3d.quiver(*robot,dx,dy,0,length=args.heading_length,color='#e23b35',linewidth=2,arrow_length_ratio=.3,zorder=16)
    ax_3d.text(*robot,' pose',color='#a31e19',fontsize=8,zorder=17)
    ax_3d.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2],high[2]),xlabel='map x (m)',ylabel='map y (m)',zlabel='map z (m)')
    ax_3d.set_box_aspect(np.maximum(high-low,.01))
    ax_3d.view_init(args.elev,args.azim)
    ax_3d.set_title(f'Oblique 3D / azim {args.azim:g}°, elev {args.elev:g}°')
    first,last = meta['cloud_source_first_s'],meta['cloud_source_last_s']
    freshness = ' | POSE FRESH FLAG FALSE' if meta['pose_fresh_flag'] is False else ''
    if args.recorded_pose:
        freshness += ' | RECORDED POSE (not fresh terminal state)'
    if not meta['observation_generation_id']:
        freshness += ' | OBSERVATION GENERATION NOT EMBEDDED: verify source'
    fig.suptitle(f'{title}\nRun {meta["run_id"]} · measured LIO cloud {fmt(first,1)}–{fmt(last,1)} s · pose {fmt(meta["pose_source_stamp_s"],2)} s{freshness}',fontsize=13,y=.985)
    caption=(f'Frame: {meta["estimated_pose_map_frame"]}; estimated pose origin ({meta["estimated_pose_child_frame"]}) '
             f'= ({robot[0]:.3f}, {robot[1]:.3f}, {robot[2]:.3f}) m; yaw={yaw:.3f} rad. '
             f'Pose age sim/wall at snapshot creation={fmt(meta["pose_sim_age_s"],3)}/{fmt(meta["pose_wall_age_s"],3)} s.\n'
             f'Gray/no points ≠ free space. Point projections do not certify support or clearance. Robot marker is pose origin, not floor. '
             f'Shown {len(shown):,}/{len(points):,} finite points; pink line = recorded same-stage LIO pose trace.')
    caption='\n'.join('\n'.join(textwrap.wrap(line,180)) for line in caption.splitlines())
    fig.text(.035,.012,caption,ha='left',va='bottom',fontsize=8.0,color='#253746',linespacing=1.35)
    fig.subplots_adjust(left=.065,right=.90,bottom=.105,top=.91,wspace=.24,hspace=.29)
    color_ax=fig.add_axes([.93,.20,.015,.60])
    fig.colorbar(sc,cax=color_ax,label='measured return z (m) · colors clipped to local z scale')
    fig.savefig(destination,dpi=args.dpi)
    plt.close(fig)
    return len(shown)


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cloud',type=Path,required=True)
    ap.add_argument('--observation',type=Path,required=True)
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--prefix',default='live')
    ap.add_argument('--expected-run-id')
    ap.add_argument('--observation-run-id',help='explicit current-run source declaration when observe JSON only says run_id=capture')
    ap.add_argument('--roi-span',type=float,default=20.0,help='local XY width/height in metres, centered on robot')
    ap.add_argument('--z-span',type=float,default=12.0,help='local vertical span in metres, centered on pose origin')
    ap.add_argument('--heading-length',type=float,default=1.0)
    ap.add_argument('--max-points',type=int,default=100000)
    ap.add_argument('--point-size',type=float,default=.45)
    ap.add_argument('--elev',type=float,default=28.0)
    ap.add_argument('--azim',type=float,default=-58.0)
    ap.add_argument('--dpi',type=int,default=140)
    ap.add_argument('--recorded-pose',action='store_true',help='mark a historical measured pose snapshot, preserving its original source stamp')
    args=ap.parse_args()
    if min(args.roi_span,args.z_span,args.heading_length,args.max_points,args.dpi)<=0:
        ap.error('metric spans, heading length, max points, and dpi must be positive')
    xyz,trace,robot,yaw,meta=read_inputs(args.cloud,args.observation,args.expected_run_id,args.observation_run_id)
    meta['pose_usage']='recorded snapshot, not a fresh terminal pose' if args.recorded_pose else 'current-run observation snapshot'
    meta['cloud_last_minus_pose_stamp_s']=(meta['cloud_source_last_s']-meta['pose_source_stamp_s']) if meta['cloud_source_last_s'] is not None and meta['pose_source_stamp_s'] is not None else None
    half=np.array([args.roi_span/2,args.roi_span/2,args.z_span/2])
    low,high=robot-half,robot+half
    mask=((xyz>=low)&(xyz<=high)).all(axis=1)
    local=xyz[mask]
    global_low=np.minimum(xyz.min(axis=0),robot);global_high=np.maximum(xyz.max(axis=0),robot)
    padding=np.maximum((global_high-global_low)*.025,.25)
    global_low-=padding;global_high+=padding
    args.output_dir.mkdir(parents=True,exist_ok=True)
    local_file=args.output_dir/f'{args.prefix}_local_multiview.png'
    full_file=args.output_dir/f'{args.prefix}_full_multiview.png'
    local_shown=render(local,trace,robot,yaw,(low,high),meta,local_file,
                       f'{"Recorded sensor context" if args.recorded_pose else "Current sensor context"} / {args.roi_span:g} m × {args.roi_span:g} m local ROI',args,True)
    full_shown=render(xyz,trace,robot,yaw,(global_low,global_high),meta,full_file,
                      'Full retained cloud domain / display subset (distant returns preserved)',args,False)
    meta.update({'local_roi_xyz_min_m':low.tolist(),'local_roi_xyz_max_m':high.tolist(),
                 'local_point_count':int(mask.sum()),'outside_local_point_count':int((~mask).sum()),
                 'local_points_plotted':local_shown,'full_points_plotted':full_shown,
                 'sampling':'deterministic random point subset for display only; full NPZ is unchanged',
                 'local_image':str(local_file.resolve()),'full_image':str(full_file.resolve())})
    metadata_file=args.output_dir/f'{args.prefix}_view_metadata.json'
    metadata_file.write_text(json.dumps(meta,indent=2))
    print(json.dumps({'local':str(local_file),'full':str(full_file),'metadata':str(metadata_file),
                      'run_id':meta['run_id'],'source_window_s':[meta['cloud_source_first_s'],meta['cloud_source_last_s']],
                      'pose_stamp_s':meta['pose_source_stamp_s'],'finite_points':meta['finite_point_count'],
                      'local_points':meta['local_point_count']},ensure_ascii=False))


if __name__=='__main__':
    try:
        main()
    except (ValueError,KeyError,OSError) as exc:
        print(f'render_live_3d_views: {exc}',file=sys.stderr)
        raise SystemExit(2)
