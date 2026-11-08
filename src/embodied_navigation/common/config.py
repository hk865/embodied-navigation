"""One editable configuration and project-relative asset locations.

The entry document is ``configs/launch/navigation.json``.  It lists the module
files to merge, in order, and may also carry top-level settings itself.  String
values may embed ``${PROJECT_ROOT}``; every occurrence is expanded to the
repository root; relative runtime paths are anchored there (see ``path()`` for
absolute paths and the ``actor_python='auto'`` exception).
"""

from embodied_navigation.project import project_root as _project_root
from pathlib import Path
import json
import os
import sys

ROOT = _project_root()
CONFIG = ROOT / 'configs/launch/navigation.json'


def expand(value):
    """Recursively replace ``${PROJECT_ROOT}`` inside string values.

    Only ``dict`` values, ``list`` elements and ``str`` leaves are walked;
    dictionary keys and every other JSON type (numbers, booleans, ``None``) are
    returned unchanged.  ``str.replace`` rewrites exactly the literal
    placeholder, so unknown ``${...}`` text survives untouched.
    """
    if isinstance(value, dict):
        return {k: expand(v) for k, v in value.items()}
    if isinstance(value, list):
        return [expand(v) for v in value]
    if isinstance(value, str):
        return value.replace('${PROJECT_ROOT}', str(ROOT))
    return value


def loads(raw, *args, **kwargs):
    """``json.loads`` with extra arguments forwarded, then :func:`expand`."""
    return expand(json.loads(raw, *args, **kwargs))


def read():
    """Merge the navigation configuration modules into one expanded dict.

    ``CONFIG`` must itself be a JSON object.  Its ``modules`` list is merged in
    listed order; each module path is resolved against ``ROOT`` and must stay
    inside it (a ``..``/symlink escape raises ``ValueError``), and each module
    must be a JSON object that does not repeat a top-level key already supplied
    by an earlier module (``ValueError`` on duplicates).  Top-level keys of the
    entry document other than ``modules`` are overlaid last, so they win
    silently over module values.  ``read()`` re-reads the files on every call
    (no caching); the returned tree is fully ``expand()``-ed.
    """
    entry = json.loads(CONFIG.read_text())
    if not isinstance(entry, dict):
        raise ValueError('Navigation configuration must be an object')

    value = {}
    for relative in entry.get('modules', []):
        source = (ROOT / relative).resolve()
        if not source.is_relative_to(ROOT):
            raise ValueError('Module configuration escapes project')
        part = json.loads(source.read_text())
        # Check the type first so a non-object module cannot raise TypeError from
        # the set intersection.  At this point only earlier modules contributed
        # keys, so this rejects cross-module duplicates; entry-level keys are
        # overlaid after the loop and are therefore not part of this check.
        if not isinstance(part, dict) or value.keys() & part.keys():
            raise ValueError('Module configuration contains duplicate top-level settings')
        value.update(part)

    value.update({k: v for k, v in entry.items() if k != 'modules'})
    return expand(value)


def path(name):
    """Resolve one configured runtime path to an absolute :class:`Path`.

    ``actor_python`` has a single sentinel: when its configured value is exactly
    ``'auto'``, the interpreter comes from ``$NAVIGATION_ACTOR_PYTHON`` (falling
    back to ``sys.executable``) and is returned via ``Path.absolute()`` -- made
    absolute without resolving symlinks or normalising ``..``.  Every other
    value is ``expanduser()``-expanded: absolute paths are resolved directly,
    relative ones are anchored at ``ROOT`` and then resolved.
    """
    setting = read()['runtime_paths'][name]
    if name == 'actor_python' and setting == 'auto':
        return Path(os.environ.get('NAVIGATION_ACTOR_PYTHON', sys.executable)).absolute()
    value = Path(setting).expanduser()
    return value.resolve() if value.is_absolute() else (ROOT / value).resolve()
