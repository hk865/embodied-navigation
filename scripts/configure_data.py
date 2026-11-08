#!/usr/bin/env python3
"""Check or create the three local data-only links used by the cloned project."""
import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=os.environ.get('PROJECT_DATA_ROOT'))
    parser.add_argument('--link', action='store_true', help='Create missing data links; never replace existing content')
    args = parser.parse_args()
    if args.data_root is None:
        parser.error('Set PROJECT_DATA_ROOT or provide --data-root pointing to 6.project_data')
    data = Path(args.data_root).expanduser().resolve()
    if not data.is_dir():
        parser.error('The configured data volume is not mounted or does not exist')
    runtime = data / 'embodied-navigation/runtime_assets'
    result = {}
    for name in ('assets', 'models', 'runs'):
        target = runtime / name
        local = ROOT / name
        if args.link:
            if name == 'runs':
                target.mkdir(parents=True, exist_ok=True)
            if not target.is_dir():
                parser.error(f'Provide the required {name} data package before linking')
            if local.is_symlink():
                if local.resolve() != target:
                    parser.error(f'{name} already points elsewhere; no change made')
            elif local.exists():
                parser.error(f'{name} already exists as real content; no change made')
            else:
                local.symlink_to(target, target_is_directory=True)
        result[name] = dict(data_present=target.is_dir(), linked=local.is_symlink() and local.resolve() == target)
    print(json.dumps(result, indent=2))
    return 0 if all(value['data_present'] for value in result.values()) else 2


if __name__ == '__main__':
    raise SystemExit(main())
