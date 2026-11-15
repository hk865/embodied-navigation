"""Offline shadow pose fusion; no ROS publisher or estimator feedback.

Covariance intersection handles unknown cross-branch correlations only if the
input covariances are consistent. Native ESIKF covariance is a model output,
not a demonstrated bound on real errors; this module never certifies it.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


def _spd(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1] or not np.isfinite(matrix).all():
        raise ValueError('Expected a finite square covariance')
    if not np.allclose(matrix, matrix.T, atol=1e-10, rtol=1e-8):
        raise ValueError('Asymmetric covariance')
    matrix = .5*(matrix+matrix.T)
    try:
        np.linalg.cholesky(matrix)
    except np.linalg.LinAlgError as error:
        raise ValueError('Covariance must be positive definite; no silent regularization') from error
    return matrix


def covariance_intersection(means, covariances, weights=None):
    """Convex combination of information, counting no input as independent."""
    means = np.asarray(means, dtype=float)
    if means.ndim != 2 or not len(means) or not np.isfinite(means).all():
        raise ValueError('Expected finite NxD local means')
    count, dimension = means.shape
    if len(covariances) != count:
        raise ValueError('Mean/covariance count mismatch')
    covariances = [_spd(c) for c in covariances]
    if any(c.shape != (dimension, dimension) for c in covariances):
        raise ValueError('Covariance dimension mismatch')
    weights = np.full(count,1/count) if weights is None else np.asarray(weights,dtype=float)
    if weights.shape != (count,) or not np.isfinite(weights).all() or np.any(weights<0) or not np.isclose(weights.sum(),1.,atol=1e-12,rtol=0):
        raise ValueError('CI weights must be a finite convex combination')
    information = np.zeros((dimension,dimension))
    vector = np.zeros(dimension)
    for mean, covariance, weight in zip(means,covariances,weights):
        precision = np.linalg.solve(covariance,np.eye(dimension))
        information += weight*precision
        vector += weight*(precision@mean)
    covariance = np.linalg.solve(information,np.eye(dimension))
    return np.linalg.solve(information,vector), _spd(covariance)


def right_log_jacobian_inverse(rotation_vector):
    """d Log(Exp(phi) Exp(epsilon))/d epsilon at epsilon=0."""
    phi = np.asarray(rotation_vector,dtype=float)
    if phi.shape != (3,) or not np.isfinite(phi).all():
        raise ValueError('Expected finite body rotation vector')
    theta = np.linalg.norm(phi)
    if theta>.2:
        raise ValueError('Shadow tangent comparison limited to 0.2 rad corrections')
    x,y,z = phi
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
    coefficient = 1/12+theta**2/720 if theta<1e-4 else (1-.5*theta/np.tan(.5*theta))/theta**2
    return np.eye(3)+.5*skew+coefficient*(skew@skew)


@dataclass(frozen=True)
class ShadowPacket:
    camera_id: str
    generation: str
    prior_id: str
    map_id: str
    source_stamp_ns: int
    prior_stamp_ns: int
    map_stamp_ns: int
    calibration_id: str
    delta_rotation_body_position_world: np.ndarray
    posterior_pose_covariance: np.ndarray
    covariance_source: str
    retained_update: bool
    tangent_convention: str = 'right_body_rotation_world_translation'


def fuse_shadow_packets(packets, weights=None):
    packets = list(packets)
    if not packets or len({p.camera_id for p in packets})!=len(packets):
        raise ValueError('Missing or duplicate camera branches')
    reference = packets[0]
    means, covariances = [], []
    for packet in packets:
        for stamp in (packet.source_stamp_ns,packet.prior_stamp_ns,packet.map_stamp_ns):
            if isinstance(stamp,(bool,np.bool_)) or not isinstance(stamp,(int,np.integer)) or not 0<=int(stamp)<=np.iinfo(np.int64).max:
                raise ValueError('Source/prior/map epochs must be nonnegative integer int64 nanoseconds')
        if not packet.camera_id or not packet.calibration_id or not all((packet.generation,packet.prior_id,packet.map_id)):
            raise ValueError('Missing generation/prior/map/calibration identity')
        if (packet.generation,packet.prior_id,packet.map_id,packet.prior_stamp_ns)!=(reference.generation,reference.prior_id,reference.map_id,reference.prior_stamp_ns):
            raise ValueError('Branches do not share the exact same frozen prior/map epoch')
        if packet.source_stamp_ns!=packet.prior_stamp_ns or packet.map_stamp_ns!=packet.prior_stamp_ns:
            raise ValueError('Asynchronous source requires actual IMU propagation/cross-covariance, never retimestamping')
        if packet.covariance_source!='native_internal' or packet.tangent_convention!='right_body_rotation_world_translation':
            raise ValueError('Unavailable ROS covariance or incompatible tangent convention')
        if not packet.retained_update:
            raise ValueError('No retained numerical visual correction; not a fusion measurement')
        delta = np.asarray(packet.delta_rotation_body_position_world,dtype=float)
        if delta.shape!=(6,) or not np.isfinite(delta).all():
            raise ValueError('Expected six finite native pose increments')
        covariance = _spd(packet.posterior_pose_covariance)
        if covariance.shape!=(6,6):
            raise ValueError('Expected rotation-first native pose marginal')
        # Covariance is in each posterior's right tangent; transport it to the
        # common prior's Log chart before CI. World position is additive.
        transform = np.eye(6)
        transform[:3,:3] = right_log_jacobian_inverse(delta[:3])
        means.append(delta)
        covariances.append(transform@covariance@transform.T)
    mean,covariance = covariance_intersection(means,covariances,weights)
    return dict(delta=mean,covariance_in_common_prior_chart=covariance,
        estimator_feedback_enabled=False, confidence_calibrated=False,
        prior_counted_as_independent_measurement=False,
        consistency_condition='CI bound requires consistent input covariances; native model consistency not established')
