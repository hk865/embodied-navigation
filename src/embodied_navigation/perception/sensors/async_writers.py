"""Default-OFF metadata mode with a separate default-OFF body-only extension.

No message/QoS reliability/history/clock changes. AUTO asks pinned rmw_fastrtps
8.4.4 to preserve the single topic XML publication mode while retaining its
original memory policy and data-sharing override. Actual DataWriter QoS is read
once at startup, never inferred from the environment or publish duration.
"""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import time

KEY='R119_METADATA_ASYNC'
OPTION=os.environ.get(KEY,'0')
if OPTION not in ('0','1'):raise ValueError(KEY+' must be 0 or 1')
ENABLED=OPTION=='1'
BODY_KEY='R119_BODY_ASYNC'
BODY_OPTION=os.environ.get(BODY_KEY,'0')
if BODY_OPTION not in ('0','1'):raise ValueError(BODY_KEY+' must be 0 or 1')
BODY_ENABLED=BODY_OPTION=='1'
if BODY_ENABLED and not ENABLED:raise ValueError('R119 body experiment requires existing metadata ASYNC ON')
from embodied_navigation.common.config import ROOT
HERE=ROOT/'src/embodied_navigation/common/communication'
TOPIC='/demo/navigation/localization_bundle'
BODY_TOPIC='/demo/slam/body_odom'
FIELDS=('publish_mode','reliability','durability','history','history_depth',
 'max_samples','max_instances','max_samples_per_instance','allocated_samples',
 'history_memory_policy','data_sharing','max_blocking_seconds','max_blocking_nanosec',
 'topic_matches_original_rmw_name')
_LIB=None
_PREPARED=None


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def library():
    global _LIB
    if _LIB is not None:return _LIB
    lib=ctypes.CDLL(str(ROOT/'install/communication/lib/writer_qos.so'))
    lib.r119_read_writer.argtypes=[ctypes.c_size_t,ctypes.POINTER(ctypes.c_int64),ctypes.c_size_t]
    lib.r119_read_writer.restype=ctypes.c_int
    lib.r119_parse_profile.argtypes=[ctypes.c_char_p,ctypes.c_char_p]
    lib.r119_parse_profile.restype=ctypes.c_int
    _LIB=lib
    return lib



def validated_transport_baseline(run,references=None):
    """Check current middleware settings, not a manifest or source digest."""
    keys=('RMW_IMPLEMENTATION','RMW_FASTRTPS_USE_QOS_FROM_XML','RMW_FASTRTPS_PUBLICATION_MODE')
    expected=('rmw_fastrtps_cpp','0','SYNCHRONOUS')
    if any(os.environ.get(key)!=value for key,value in zip(keys,expected)):
        raise RuntimeError('DDS middleware/publication settings invalid')
    xml=Path(os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE',''))
    if not xml.is_file():raise FileNotFoundError('DDS transport XML missing')
    return dict(xml_path=str(xml),environment={key:os.environ[key]for key in keys})


def configure(run):
    """Before OwnedRosLifecycle/rclpy.init; selected process only."""
    global _PREPARED
    if not ENABLED:return None
    if _PREPARED is not None:raise RuntimeError('R119 process already configured')
    run=Path(run).resolve();scope=run/'runtime_context.json'
    paths=[HERE/'metadata_async.xml', ROOT/'install/communication/lib/writer_qos.so']
    if BODY_ENABLED:paths.append(HERE/'body_async.xml')
    for path in paths:
        if not path.is_file():raise FileNotFoundError(path)
    if os.environ.get('RMW_IMPLEMENTATION','rmw_fastrtps_cpp') not in ('','rmw_fastrtps_cpp'):
        raise RuntimeError('R119 requires pinned rmw_fastrtps_cpp')
    if os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML','0') not in ('','0'):
        raise RuntimeError('R119 does not enable global XML QoS override')
    mode_before=os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE')
    if mode_before!='SYNCHRONOUS':
        raise RuntimeError('R119 conflicting explicit process publication mode')
    baseline=validated_transport_baseline(run)
    xml=HERE/'metadata_async.xml'
    inherited={}
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
        value=os.environ.get(key,'')
        if value:
            path=Path(value).resolve()
            if not path.is_file():raise FileNotFoundError(path)
            inherited[key]=dict(path=str(path))
    # Parse with the actual installed FastDDS library, without creating entities.
    if library().r119_parse_profile(str(xml).encode(),TOPIC.encode())!=1:
        raise RuntimeError('R119 XML did not parse as target ASYNCHRONOUS')
    if BODY_ENABLED and library().r119_parse_profile(str(HERE/'body_async.xml').encode(),BODY_TOPIC.encode())!=1:
        raise RuntimeError('R119 body-only XML did not parse as target ASYNCHRONOUS')
    # Preload only the named writer profile into this process's installed
    # FastDDS profile manager. rmw still loads the original inherited transport
    # XML at participant creation; neither path nor bytes are replaced.
    os.environ['RMW_FASTRTPS_PUBLICATION_MODE']='AUTO'
    _PREPARED=dict(schema='r119_metadata_async_setup/v1',run=str(run),pid=os.getpid(),
        requested=True,
        target_topic=TOPIC,xml=str(xml),before_rclpy_init=True,
        named_profile_preloaded=True,inherited_transport_xml=inherited,
        transport_xml_environment_unchanged=True,transport_baseline=baseline,
        mode_before=mode_before,mode_after=os.environ['RMW_FASTRTPS_PUBLICATION_MODE'],
        only_this_process_publication_mode_changed=True,
        requested_mode='AUTO; named selected topics ASYNCHRONOUS',actual_writer_checked=False,
        body_environment_key=BODY_KEY,body_environment_value=BODY_OPTION,body_async_enabled=BODY_ENABLED,
        body_topic=BODY_TOPIC,body_profile=str(HERE/'body_async.xml')if BODY_ENABLED else None,
        original_metadata_mode_preserved=True)
    return dict(_PREPARED)


def read_writer(publisher):
    """The caller holds a live rclpy Publisher handle; no entity is created."""
    values=(ctypes.c_int64*len(FIELDS))()
    with publisher.handle:
        code=library().r119_read_writer(publisher.handle.pointer,values,len(FIELDS))
    if code:raise RuntimeError('R119 readonly actual writer check failed: '+str(code))
    q=dict(zip(FIELDS,values))
    q.update(topic=publisher.topic_name,requested_history_depth=publisher.qos_profile.depth,
        requested_reliability=int(publisher.qos_profile.reliability),
        requested_history=int(publisher.qos_profile.history),
        requested_durability=int(publisher.qos_profile.durability))
    if q['topic_matches_original_rmw_name']!=1:
        raise RuntimeError('R119 actual DDS topic differs from original rmw topic')
    if q['history_depth']!=publisher.qos_profile.depth:
        raise RuntimeError('R119 actual history depth differs from original rclpy QoS')
    return q


def verify(node,run):
    if not ENABLED:return None
    from rclpy.utilities import get_rmw_implementation_identifier
    if _PREPARED is None or get_rmw_implementation_identifier()!='rmw_fastrtps_cpp':
        raise RuntimeError('R119 actual process middleware differs')
    publishers=dict(metadata=node.bundle_pub,body=node.pubs['body'],lidar=node.pubs['lidar'],
        cloud=node.cloud_pub,status=node.status_pub)
    actual={name:read_writer(pub)for name,pub in publishers.items()}
    for name,q in actual.items():
        if q['publish_mode']!=(1 if name=='metadata' or name=='body' and BODY_ENABLED else 0):
            raise RuntimeError('R119 actual publication mode differs: '+name)
        # These were preserved by rmw_fastrtps 8.4.4 rather than XML override.
        if q['history_memory_policy']!=1 or q['data_sharing']!=3:
            raise RuntimeError('R119 original memory/data-sharing policy differs: '+name)
        if q['reliability']!=(1 if name=='cloud' else 2):
            raise RuntimeError('R119 original writer reliability differs: '+name)
    result=dict(_PREPARED,actual_writer_checked=True,actual=actual,
        actual_middleware=get_rmw_implementation_identifier(),observed_monotonic_ns=time.monotonic_ns(),
        publication_calls_prove_delivery=False,root_component_OFF_ON_comparison_required=True,
        original_source_age_limit_ns=300000000,S1_enabled=False,navigation_PASS=False)
    with (Path(run)/'r119_metadata_async_actual.json').open('x') as stream:
        json.dump(result,stream,allow_nan=False);stream.write('\n')
    return result
