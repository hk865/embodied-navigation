"""One editable configuration and project-relative asset locations."""

from embodied_navigation.project import project_root as _project_root
from pathlib import Path
import json,os,sys
ROOT=_project_root()
CONFIG=ROOT/'configs/launch/navigation.json'
def expand(value):
    if isinstance(value,dict):return {k:expand(v)for k,v in value.items()}
    if isinstance(value,list):return [expand(v)for v in value]
    if isinstance(value,str):return value.replace('${PROJECT_ROOT}',str(ROOT))
    return value
def loads(raw,*args,**kwargs):return expand(json.loads(raw,*args,**kwargs))
def read():
    entry=json.loads(CONFIG.read_text())
    if not isinstance(entry,dict):raise ValueError('Navigation configuration must be an object')
    value={}
    for relative in entry.get('modules',[]):
        source=(ROOT/relative).resolve()
        if not source.is_relative_to(ROOT):raise ValueError('Module configuration escapes project')
        part=json.loads(source.read_text())
        if not isinstance(part,dict) or value.keys() & part.keys():
            raise ValueError('Module configuration contains duplicate top-level settings')
        value.update(part)
    value.update({k:v for k,v in entry.items() if k!='modules'})
    return expand(value)
def path(name):
    setting=read()['runtime_paths'][name]
    if name=='actor_python'and setting=='auto':
        return Path(os.environ.get('NAVIGATION_ACTOR_PYTHON',sys.executable)).absolute()
    value=Path(setting).expanduser()
    return value.resolve()if value.is_absolute()else(ROOT/value).resolve()
