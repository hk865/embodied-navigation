"""Explicit source point/commit verification; geometry remains an independent gate."""
import base64
import hashlib
import json
import struct
import numpy as np

MAX_POINTS=65536
MAX_META=2*1024*1024
def message_sha(msg):
    def text(v):
        data=v.encode('utf-8');return struct.pack('<I',len(data))+data
    h=msg.header;header=struct.pack('<iI',int(h.stamp.sec),int(h.stamp.nanosec))+text(h.frame_id)
    if hasattr(msg,'point_step'):
        data=bytes(msg.data)
        out=bytearray(b'go2_ros_semantic/v1:PointCloud2\0'+header)
        out+=struct.pack('<III',msg.height,msg.width,len(msg.fields))
        for f in msg.fields:out+=text(f.name)+struct.pack('<IBI',f.offset,f.datatype,f.count)
        out+=struct.pack('<BIIQ',bool(msg.is_bigendian),msg.point_step,msg.row_step,len(data))+data+struct.pack('<B',bool(msg.is_dense))
    else:
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;l=msg.twist.twist.linear;a=msg.twist.twist.angular
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,*msg.pose.covariance,l.x,l.y,l.z,a.x,a.y,a.z,*msg.twist.covariance]
        out=b'go2_ros_semantic/v1:Odometry\0'+header+text(msg.child_frame_id)+struct.pack('<'+str(len(values))+'d',*values)
    return hashlib.sha256(out).hexdigest()

def decode(data,schema):
    if len(data)>MAX_META:raise ValueError('identity metadata allocation bound')
    value=json.loads(data)
    if value.get('schema')!=schema:raise ValueError('identity schema')
    return value

def integer(value,name,low=1,high=2**64-1):
    if type(value)is not int or not low<=value<=high:raise ValueError('invalid '+name)
    return value

def epoch(value):
    if not isinstance(value,str) or len(value)!=32 or any(c not in '0123456789abcdef'for c in value):
        raise ValueError('invalid producer epoch')
    return value

def hash_value(value):
    if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef'for c in value):
        raise ValueError('invalid SHA256')
    return value

def unpack(encoded,sha,itemsize):
    if not isinstance(encoded,str) or len(encoded)>4*((MAX_POINTS*itemsize+2)//3):raise ValueError('identity encoded size')
    raw=base64.b64decode(encoded,validate=True)
    if len(raw)>MAX_POINTS*itemsize or len(raw)%itemsize or hashlib.sha256(raw).hexdigest()!=hash_value(sha):
        raise ValueError('identity bytes/hash/layout')
    return raw

def stamp(msg):return int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
def records(msg):
    return np.ndarray((msg.height,msg.width),dtype='V'+str(msg.point_step),buffer=msg.data,
        strides=(msg.row_step,msg.point_step)).reshape(-1)

def source_points(raw,adapted,proof,run_id=None,expected_pid=None):
    if proof.get('schema')!='raw_filtered_point_identity/v1':raise ValueError('raw source identity absent')
    if proof.get('fingerprint_model')!='go2_ros_semantic/v1':raise ValueError('source fingerprint contract')
    epoch(proof['producer_epoch']);integer(proof['producer_pid'],'source pid')
    if proof.get('navigation_ground_truth_used')is not False:raise ValueError('raw source truth authority')
    if run_id is not None and proof.get('run_id')!=run_id:raise ValueError('raw producer run differs')
    if expected_pid is not None and proof['producer_pid']!=expected_pid:raise ValueError('raw producer PID differs')
    if integer(proof['stamp_ns'],'raw stamp')!=stamp(raw) or stamp(raw)!=stamp(adapted):raise ValueError('raw/adapted identity stamp')
    if message_sha(raw)!=hash_value(proof['raw_message_sha256']) or message_sha(adapted)!=hash_value(proof['adapted_message_sha256']):
        raise ValueError('raw/adapted complete semantic message hash differs')
    raw_indices=np.frombuffer(unpack(proof['raw_ordinals_base64'],proof['raw_ordinals_sha256'],4),dtype='<u4')
    r,a=records(raw),records(adapted)
    if len(a)>MAX_POINTS or len(r)>MAX_POINTS or len(raw_indices)!=len(a) or proof['raw_count']!=len(r) or proof['adapted_count']!=len(a):
        raise ValueError('raw/adapted source count')
    if len(raw_indices) and (int(raw_indices.max())>=len(r) or np.any(np.diff(raw_indices.astype(np.int64))<=0)):
        raise ValueError('raw ordinal uniqueness/order')
    if not np.array_equal(a,r[raw_indices]):raise ValueError('adapted bytes do not equal actual source records')
    return raw_indices

def commit_messages(registered,mapper,body,lidar,commit,adapter,run_id=None,expected_pid=None):
    if commit.get('schema')!='post_lio_source_point_commit/v1' or adapter.get('schema')!='mapper_body_lidar_commit/v1':
        raise ValueError('explicit stage commit absent')
    if commit.get('fingerprint_model')!='go2_ros_semantic/v1' or adapter.get('fingerprint_model')!='go2_ros_semantic/v1':raise ValueError('commit fingerprint contract')
    token=(epoch(commit['producer_epoch']),integer(commit['commit'],'stage commit'))
    integer(commit['producer_pid'],'mapper PID')
    if expected_pid is not None and commit['producer_pid']!=expected_pid:raise ValueError('mapper producer PID differs')
    if run_id is not None and (commit.get('run_id')!=run_id or adapter.get('run_id')!=run_id):raise ValueError('commit run differs')
    if commit.get('stage')!='post_lio_dense' or commit.get('navigation_ground_truth_used')is not False:raise ValueError('stage authority differs')
    if token!=(adapter.get('producer_epoch'),adapter.get('commit')) or commit['stamp_ns']!=adapter.get('stamp_ns'):
        raise ValueError('adapter parent commit differs')
    ns=integer(commit['stamp_ns'],'commit stamp')
    if any(stamp(m)!=ns for m in (registered,mapper,body,lidar)):raise ValueError('same commit integer stamp differs')
    for msg,key,meta in [(registered,'registered_message_sha256',commit),(mapper,'mapper_message_sha256',commit),
        (mapper,'mapper_message_sha256',adapter),(body,'body_message_sha256',adapter),(lidar,'lidar_message_sha256',adapter)]:
        if message_sha(msg)!=hash_value(meta[key]):raise ValueError('actual '+key+' differs')
    if adapter.get('navigation_ground_truth_used')is not False:raise ValueError('adapter truth authority')
    return dict(producer_epoch=token[0],commit=token[1],stage=commit['stage'],stamp_ns=ns,
        mapper_message_sha256=commit['mapper_message_sha256'],registered_message_sha256=commit['registered_message_sha256'],
        body_message_sha256=adapter['body_message_sha256'],lidar_message_sha256=adapter['lidar_message_sha256'],
        explicit_message_commit_token=True)

def exact_indices(raw,adapted,proof,commit,registered,eligible,run_id=None,expected_source_pid=None):
    raw_indices=source_points(raw,adapted,proof,run_id,expected_source_pid)
    sources=commit.get('sources')
    # Preserve all propagated source IDs, but no admission of unproved multiscan timing.
    if not isinstance(sources,list) or len(sources)!=1:raise ValueError('multiscan point timing not admitted')
    source=sources[0];packet=integer(source['packet'],'source packet')
    if source['stamp_ns']!=stamp(raw) or source['stamp_ns']!=commit['stamp_ns'] or source['input_count']!=len(raw_indices):
        raise ValueError('exact same input source count/stamp differs')
    if source['adapted_message_sha256']!=proof['adapted_message_sha256']:raise ValueError('actual native input differs from filtered source')
    wire=unpack(commit['point_refs_base64'],commit['point_refs_sha256'],12)
    if commit.get('point_ref_layout')!='LE:u64packet,u32input_ordinal':raise ValueError('point identity wire layout')
    refs=np.frombuffer(wire,dtype=np.dtype([('packet','<u8'),('ordinal','<u4')]))
    n=int(registered.width)*int(registered.height)
    if not n or n>MAX_POINTS or len(refs)!=n or commit.get('point_count')!=n:raise ValueError('registered identity cardinality')
    if not np.all(refs['packet']==packet) or np.any(refs['ordinal']>=len(raw_indices)):raise ValueError('source reference outside packet')
    ordinals=refs['ordinal'].astype(np.int64);expected=np.flatnonzero(eligible)
    if len(ordinals)!=len(expected) or not np.array_equal(np.sort(ordinals),expected):
        raise ValueError('source point identities do not cover original eligible records exactly once')
    # Values may be equal; the producer-owned refs distinguish their real sources.
    return ordinals,raw_indices[ordinals]

def verify_corrected(body,cloud,metadata):
    parent=metadata.get('source_commit')
    if not isinstance(parent,dict) or parent.get('explicit_message_commit_token')is not True:raise ValueError('corrected bundle source commit absent')
    epoch(parent['producer_epoch']);integer(parent['commit'],'corrected parent commit')
    if parent.get('stamp_ns')!=metadata['source_ns'] or stamp(body)!=metadata['source_ns'] or stamp(cloud)!=metadata['source_ns']:
        raise ValueError('corrected bundle source stamp differs')
    if metadata.get('robot_truth_pose_used')is not False:raise ValueError('corrected bundle truth authority')
    return (parent['producer_epoch'],parent['commit'])

def xyz(msg):
    f={x.name:x for x in msg.fields};endian='>' if msg.is_bigendian else '<'
    if len(msg.data)<msg.height*msg.row_step or msg.row_step<msg.width*msg.point_step:raise ValueError('source cloud layout')
    values=[]
    for name in ('x','y','z'):
        field=f[name]
        if field.datatype!=7 or field.count!=1 or field.offset<0 or field.offset+4>msg.point_step:raise ValueError('source XYZ format')
        values.append(np.ndarray((msg.height,msg.width),dtype=endian+'f4',buffer=msg.data,offset=field.offset,
            strides=(msg.row_step,msg.point_step)).reshape(-1))
    return np.column_stack(values).astype(float)

def geometry_bundle(slot,run_id,mapper_pid=None,source_pid=None,mount=(.2,0,.1177)):
    from scipy.spatial.transform import Rotation
    raw,adapted,registered,mapper,body,lidar=(slot[k][0]for k in ('raw','adapted','registered','mapper','body','lidar'))
    commit,adapter,proof=(slot[k][0]for k in ('point_identity','odom_identity','source_identity'))
    parent=commit_messages(registered,mapper,body,lidar,commit,adapter,run_id,mapper_pid)
    if raw.header.frame_id!='velodyne' or adapted.header.frame_id!='velodyne' or registered.header.frame_id!=body.header.frame_id or mapper.header.frame_id!=body.header.frame_id:
        raise ValueError('source commit cloud/pose frames')
    if body.child_frame_id!='demo_slam_body':raise ValueError('source commit body child frame')
    def pose(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        v=np.array([p.x,p.y,p.z]);qv=np.array([q.x,q.y,q.z,q.w])
        if not np.isfinite(v).all() or not np.isfinite(qv).all() or abs(qv@qv-1)>1e-3:raise ValueError('source pose finite/unit')
        return v,Rotation.from_quat(qv).as_matrix()
    p,r=pose(body);mp,mr=pose(mapper)
    if np.max(np.abs(p-mp))>1e-10 or np.max(np.abs(r-mr))>1e-10:raise ValueError('source mapper/body stage pose differs')
    layout=lambda m:[(f.name,f.offset,f.datatype,f.count)for f in m.fields]
    if layout(raw)!=layout(adapted) or raw.point_step!=adapted.point_step:raise ValueError('source adapter layout differs')
    if any(f.name.lower()in ('time','t','timestamp','offset_time')for f in raw.fields):raise ValueError('per point time not admitted')
    points=xyz(adapted);full=xyz(registered)
    eligible=np.isfinite(points).all(1)&(np.linalg.norm(points,axis=1)>=.3)
    indices,raw_ordinals=exact_indices(raw,adapted,proof,commit,registered,eligible,run_id,source_pid)
    reconstructed=(full-p)@r;distance=np.linalg.norm(reconstructed-(points[indices]+np.asarray(mount)),axis=1)
    rms=float(np.sqrt(np.mean(distance**2)));maximum=float(np.max(distance))
    if not np.isfinite(full).all() or not np.isfinite(distance).all() or rms>.006 or maximum>.030:raise ValueError('original source geometry residual gate')
    parent.update(point_identity_coverage_verified=True,source_raw_message_sha256=proof['raw_message_sha256'],
        source_producer_epoch=proof['producer_epoch'],point_refs_sha256=commit['point_refs_sha256'],
        registered_source_rms_m=rms,registered_source_max_m=maximum)
    return parent
