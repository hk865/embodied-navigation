"""Current project policy and model locations; actual interfaces are validated by the actor."""
from pathlib import Path
import json,sys
from embodied_navigation.common.config import ROOT, CONFIG, path
HERE=Path(__file__).resolve().parent
POLICY=ROOT/"src/embodied_navigation/robot/unitree/policy"
def selection():
    worker=POLICY/'native/worker.py'
    if not worker.is_file():raise FileNotFoundError(worker)
    return dict(candidate_origin=str(worker),selected_destination='policy/worker.py')
def student_sources():
    files=dict(student_adapter=POLICY/'student.py',student_contract=CONFIG,student_export=path('student_model'))
    for value in files.values():
        if not value.is_file():raise FileNotFoundError(value)
    return files
def source_files():return [Path(selection()['candidate_origin']),*student_sources().values(),POLICY/'native/observation.py']
def archive_relative(original,root,data):
    original=Path(original).resolve()
    if original==Path(data['candidate_origin']):return Path('policy/worker.py')
    if original.parent==HERE/'policy_candidate':return Path('policy')/original.name
    return Path('navigation')/original.name
def binding(run):
    result=dict(candidate_origin=selection()['candidate_origin'],selected_snapshot=selection()['candidate_origin'])
    for role,origin in student_sources().items():
        result.update({role+'_origin':str(origin),role+'_snapshot':str(origin)})
    return result
def record_actual_loaded(run,legacy):
    observation=sys.modules[legacy.build_observation.__module__]
    paths=dict(worker=str(Path(legacy.__file__).resolve()),observation=str(Path(observation.__file__).resolve()))
    value=dict(actor_kind='student',actual_paths=paths,selected_binding=binding(run),actual_actor_load_pending=True)
    return value
