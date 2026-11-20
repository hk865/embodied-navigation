#!/usr/bin/env python3
"""Prepare advisory PCT-route candidate utility without submitting any action."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from embodied_navigation.planning.exploration.waypoint_candidates import prepare_candidates, run_binding


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',required=True,type=Path)
    parser.add_argument('--candidates',type=Path,help='JSON list of {id,goal_xyz_m,role,reason}')
    parser.add_argument('--frontier-json',type=Path)
    parser.add_argument('--frontier-z',type=float,help='Explicit estimator child-origin waypoint Z; never inferred')
    parser.add_argument('--prepare-next',action='store_true')
    parser.add_argument('--offline',action='store_true',help='Explicit same-run frozen replay; never retimestamps evidence')
    parser.add_argument('--snapshot',type=Path,help='Offline same-run NPZ override')
    parser.add_argument('--observation-json',type=Path,help='Offline same-run observation override')
    parser.add_argument('--output',type=Path,help='Optional advisory artifact outside action mailbox; default stdout')
    parser.add_argument('--voxel-m',type=float,default=.5)
    parser.add_argument('--max-range-m',type=float,default=8.)
    parser.add_argument('--route-spacing-m',type=float,default=1.)
    parser.add_argument('--max-route-samples',type=int,default=32)
    parser.add_argument('--azimuth-samples',type=int,default=32)
    parser.add_argument('--lidar-offset',type=float,nargs=3,required=True,
        help='Explicit LiDAR minus route child-origin XYZ, modeled in route frame')
    parser.add_argument('--body-support-offset',type=float,default=.32)
    parser.add_argument('--pct-start-blind-radius',type=float,default=0.)
    parser.add_argument('--pct-binary',type=Path)
    args=parser.parse_args(argv)
    capture,_=run_binding(args.run_dir)
    if args.output is not None and args.output.resolve().is_relative_to((capture/'interactive').resolve()):
        parser.error('--output must be outside the action mailbox')
    if args.candidates is None and args.frontier_json is None:
        parser.error('provide --candidates and/or --frontier-json')
    candidates=json.loads(args.candidates.read_text()) if args.candidates else []
    if not isinstance(candidates,list):
        parser.error('--candidates must contain a JSON list')
    result=prepare_candidates(args.run_dir,candidates,prepare_next=args.prepare_next,
        frontier_path=args.frontier_json,frontier_z=args.frontier_z,
        offline=args.offline,snapshot_path=args.snapshot,observation_path=args.observation_json,
        scorer_options=dict(voxel_m=args.voxel_m,max_range_m=args.max_range_m,
            route_spacing_m=args.route_spacing_m,max_route_samples=args.max_route_samples,
            azimuth_samples=args.azimuth_samples,lidar_offset_in_route_frame=args.lidar_offset),
        planner_options=dict(body_support_offset_m=args.body_support_offset,
            start_blind_radius_m=args.pct_start_blind_radius,native_binary=args.pct_binary))
    content=json.dumps(result,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(content)
    else:
        print(content,end='')
    return 0

if __name__=='__main__':
    raise SystemExit(main())
