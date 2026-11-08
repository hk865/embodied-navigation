"""Plain process settings for the current navigation instance.

This is configuration, never an approval, frozen source list or test result.
"""
from pathlib import Path
import json

NAME = 'runtime_context.json'


def load(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get('profile'), dict):
        raise ValueError('Runtime configuration must contain a profile object')
    directory = Path(data.get('run_dir', path.parent)).resolve()
    if directory != path.parent:
        raise ValueError('Runtime output directory differs from its settings location')
    return dict(data, path=str(path), run_dir=str(directory), profile=data['profile'])


def write(directory, profile):
    directory = Path(directory).resolve()
    value = dict(run_dir=str(directory), profile=profile, actor_kind='student',
                 controller_kind='student', navigation_ground_truth_used=False,
                 references={}, experiment='navigation')
    (directory/NAME).write_text(json.dumps(value, indent=2)+'\n')
    return value
