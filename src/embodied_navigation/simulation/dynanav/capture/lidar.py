"""RTX rotating LiDAR packets, preserving native firing times and ring IDs.

No depth image, world pose or truth-based point motion correction is used here.
The sole coordinate operation is the fixed sensor-to-IMU installation transform.
Isaac imports are delayed so the wire contract can be tested on CPU.
"""

from dataclasses import dataclass
import hashlib
import os
import uuid
import time

import numpy as np

LIDAR_VARIANT = "OS1_REV6_32ch10hz1024res"
LIDAR_MOUNT = np.array([0.0, 0.0, 0.25])
LIDAR_TOPIC = "/dynanav/lidar_rtx"
MAX_SCAN_POINTS = 65_536  # Native ingress: <=65536 points and <=2 MiB wire data.
# Match the registered ouster_ros::Point field types, including ancillary fields.
OUSTER_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u4", "<u2", "u1", "<u2", "<u4"],
    "offsets": [0, 4, 8, 12, 16, 20, 22, 24, 28], "itemsize": 32,
})


COMPLETE_SCAN_PERIOD_NS = 100_000_000
# Fixed production coordinate-frame pose phase, NOT the source scan origin.
COMPLETE_SCAN_COORD_FRAME_START_PHASE_NS = 15_000_000
COMPLETE_SCAN_ASSET_SHA256 = "ec5b732ef18521efe33ccdfc4e1242e9c3ce066e9482a91d29b3cea91fc7000b"
COMPLETE_SCAN_CONFIG_ID = LIDAR_VARIANT + ":sha256:" + COMPLETE_SCAN_ASSET_SHA256
COMPLETE_SCAN_SENSOR_ID = "go2_rtx_os1"
COMPLETE_SCAN_TOPIC = LIDAR_TOPIC + "/complete_scan"
_COMPLETE_ATTRIBUTES = {
    "omni:sensor:Core:accumulateOutputs": True,
    "omni:sensor:Core:numberOfChannels": 32,
    "omni:sensor:Core:numberOfEmitters": 32,
    "omni:sensor:Core:patternFiringRateHz": 10240,
    "omni:sensor:Core:scanRateBaseHz": 10,
    "omni:sensor:tickRate": 10.0,
}


# These are the original production output-format overrides, not firing/timing
# changes. USD token attributes return Python str; GMO uses separate enum names.
_COMPLETE_RUNTIME_OVERRIDES = {
    "omni:sensor:Core:outputFrameOfReference": "SENSOR",
    "omni:sensor:Core:outputMotionCompensationState": "NONCOMPENSATED",
    "omni:sensor:Core:elementsCoordsType": "CARTESIAN",
}


def attest_complete_scan_asset(asset, runtime_prim=None):
    """Read only: pin the known official frozen bytes AND their USD semantics."""
    from pathlib import Path
    from pxr import Usd
    if hashlib.sha256(Path(asset).read_bytes()).hexdigest() != COMPLETE_SCAN_ASSET_SHA256:
        raise ValueError("Complete-scan asset is not the attested official OS1 config")
    stage = Usd.Stage.Open(str(asset))
    if stage is None:
        raise ValueError("Cannot read complete-scan USD config")
    prim = stage.GetPrimAtPath("/Lidar/sensor")
    attest_complete_scan_prim(prim)
    if runtime_prim is not None:
        attest_complete_scan_prim(runtime_prim)
        # A matching rate/channel count alone cannot attest the firing schedule.
        # Compare all frozen sensor attributes, including every emitter array.
        frozen_names = {a.GetName() for a in prim.GetAttributes()
                        if a.GetName().startswith("omni:sensor:")}
        emitter_prefix = "omni:sensor:Core:emitterState:"
        runtime_emitters = {a.GetName() for a in runtime_prim.GetAttributes()
                            if a.GetName().startswith(emitter_prefix)}
        if runtime_emitters != {n for n in frozen_names if n.startswith(emitter_prefix)}:
            raise ValueError("Runtime RTX emitter schedule differs from frozen asset")
        # Schema fallback attributes may be absent in a schema-less CPU read.
        # Always check all three overrides, even when not in frozen_names.
        for name in frozen_names | _COMPLETE_RUNTIME_OVERRIDES.keys():
            frozen_attr = prim.GetAttribute(name)
            actual = runtime_prim.GetAttribute(name)
            expected = (_COMPLETE_RUNTIME_OVERRIDES[name] if name in _COMPLETE_RUNTIME_OVERRIDES
                        else frozen_attr.Get())
            observed = actual.Get() if actual else None
            actual_usd_type = str(actual.GetTypeName()) if actual else None
            expected_usd_type = ("token" if name in _COMPLETE_RUNTIME_OVERRIDES
                                 else str(frozen_attr.GetTypeName()))
            exact_override = (name not in _COMPLETE_RUNTIME_OVERRIDES
                              or (type(observed) is str and observed == expected
                                  and actual_usd_type == "token"))
            if ((frozen_attr and frozen_attr.GetNumTimeSamples()) or not actual
                    or actual.GetNumTimeSamples() or not exact_override
                    or not np.array_equal(np.asarray(observed), np.asarray(expected))):
                raise ValueError(
                    "Runtime RTX config mismatch: " + name
                    + f" actual={observed!r} ({type(observed).__name__})"
                    + f" expected={expected!r} ({type(expected).__name__})"
                    + f" actual_usd_type={actual_usd_type!r} expected_usd_type={expected_usd_type!r}")
    return COMPLETE_SCAN_CONFIG_ID


def attest_complete_scan_prim(prim):
    if not prim or not prim.IsValid():
        raise ValueError("Missing complete-scan sensor prim")
    for name, expected in _COMPLETE_ATTRIBUTES.items():
        attr = prim.GetAttribute(name)
        if not attr or attr.GetNumTimeSamples() or attr.Get() != expected:
            raise ValueError("Unattested complete-scan sensor attribute: " + name)


class CompleteScanCertificate:
    """Fail-closed session continuity, independent of the legacy point packer."""
    def __init__(self, config_id):
        if config_id != COMPLETE_SCAN_CONFIG_ID:
            raise ValueError("Unknown complete-scan configuration")
        self.session_id = str(uuid.uuid4())
        self.sequence = 0
        self.last = None
        self.failed = False

    def observe(self, gmo):
        if self.failed:
            raise ValueError("Complete-scan certification session is failed")
        if int(gmo.scanComplete) == 0:
            return None
        start, frame = int(gmo.timestampNs), int(gmo.frameId)
        end = start + COMPLETE_SCAN_PERIOD_NS
        offsets = np.asarray(gmo.timeOffsetNs)
        valid = (int(gmo.scanComplete) == 1 and 0 <= start < end < 2**64
                 and 0 <= frame < 2**64
                 and int(gmo.frameStart.timestampNs) == start + COMPLETE_SCAN_COORD_FRAME_START_PHASE_NS
                 and int(gmo.frameEnd.timestampNs) == end
                 and offsets.shape == (int(gmo.numElements),)
                 and np.issubdtype(offsets.dtype, np.integer)
                 and np.all(offsets >= 0) and np.all(offsets < COMPLETE_SCAN_PERIOD_NS))
        if self.last is not None:
            valid = valid and start == self.last[0] + COMPLETE_SCAN_PERIOD_NS and frame > self.last[1]
        if not valid:
            self.failed = True
            raise ValueError("Complete-scan native interval, identity or offsets contradict contract")
        self.last = (start, frame)
        self.sequence += 1
        return dict(contract_version=1, session_id=self.session_id, sequence=self.sequence,
                    native_frame_id=frame, sensor_id=COMPLETE_SCAN_SENSOR_ID,
                    config_id=COMPLETE_SCAN_CONFIG_ID, source_start_ns=start,
                    source_end_ns=end, coordinate_frame_start_ns=int(gmo.frameStart.timestampNs),
                    coordinate_frame_end_ns=int(gmo.frameEnd.timestampNs),
                    accumulate_outputs=True, scan_complete=True)


def complete_scan_message(packet, cloud=None):
    """One typed atomic envelope; never join a separate latest metadata topic."""
    certificate = packet.receipt.get("complete_scan_certificate")
    if certificate is None:
        return None
    from dynanav_sensor_msgs.msg import CompleteLidarScan
    if (packet.stamp_ns != certificate["source_start_ns"]
            or np.any(packet.records["t"] >= COMPLETE_SCAN_PERIOD_NS)
            or packet.receipt.get("header_rebase_ns") != 0):
        raise ValueError("Certificate and retained cloud disagree")
    expected = pointcloud_message(packet)
    if cloud is not None:
        def identity(value):
            return (value.header.stamp.sec, value.header.stamp.nanosec, value.header.frame_id,
                    value.height, value.width, value.point_step, value.row_step,
                    value.is_dense, value.is_bigendian,
                    tuple((f.name, f.offset, f.datatype, f.count) for f in value.fields),
                    bytes(value.data))
        if identity(cloud) != identity(expected):
            raise ValueError("Supplied cloud differs from certified packet")
    msg = CompleteLidarScan()
    for name, value in certificate.items():
        setattr(msg, name, value)
    msg.cloud = expected if cloud is None else cloud
    return msg


@dataclass(frozen=True)
class LidarPacket:
    stamp_ns: int
    records: np.ndarray
    receipt: dict

    @property
    def points(self):
        return np.column_stack([self.records[k] for k in ("x", "y", "z")])


def pack_lidar_scan(*, xyz_sensor, intensity, offsets_ns, ring, timestamp_ns,
                    imu_mount, mount=LIDAR_MOUNT, valid_flags=None, allow_empty=False):
    """Keep absolute firing times, normalize only the Ouster header/t split.

    Input timestamps may be signed offsets around the native GMO timestamp.
    Native nonnegative offsets preserve the authoritative scan-start header
    and each uint32 t exactly. Signed legacy offsets are rebased only as a
    header/t split, without changing absolute firing times. No synthetic
    per-point timing, truncation or decimation is allowed.
    """
    xyz = np.asarray(xyz_sensor, dtype=float)
    offsets = np.asarray(offsets_ns)
    rings = np.asarray(ring)
    intensities = np.asarray(intensity, dtype=float)
    n = len(xyz)
    if xyz.shape != (n, 3) or not (0 <= n <= MAX_SCAN_POINTS if allow_empty else 0 < n <= MAX_SCAN_POINTS):
        raise ValueError("Invalid or over-budget RTX scan shape")
    if any(a.shape != (n,) for a in (offsets, rings, intensities)):
        raise ValueError("RTX point/time/channel arrays are not paired")
    if not np.issubdtype(offsets.dtype, np.integer) or not np.issubdtype(rings.dtype, np.integer):
        raise ValueError("RTX firing offsets and channels must be native integers")
    offsets = offsets.astype(np.int64)
    # Native RTX uses scan-start timestamp and nonnegative ns offsets. Preserve
    # that header exactly; signed legacy buffers are normalized without changing
    # any absolute firing time.
    origin = min(0, int(offsets.min())) if n else 0
    relative = offsets - origin
    if n and (int(relative.max()) > 110_000_000 or (int(relative.max()) <= 0 and not allow_empty)):
        raise ValueError("RTX scan is not a timed 10 Hz rotation")
    if np.any(rings < 0) or np.any(rings >= 32):
        raise ValueError("RTX channels do not match the selected 32-line sensor")
    imu = np.asarray(imu_mount, dtype=float)
    mount = np.asarray(mount, dtype=float)
    if imu.shape != (3,) or mount.shape != (3,) or not np.isfinite([imu, mount]).all():
        raise ValueError("Invalid fixed RTX/IMU installation")
    base = xyz + mount
    own = np.all((base >= [-0.5, -0.3, -0.45]) & (base <= [0.5, 0.3, 0.3]), axis=1)
    distance = np.linalg.norm(xyz, axis=1)
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(intensities) & (distance > 0.3)
    if valid_flags is not None:
        flag_mask = np.asarray(valid_flags, dtype=bool)
        if flag_mask.shape != (n,):
            raise ValueError("RTX validity flags are not paired with points")
        valid &= flag_mask
    keep = valid & ~own
    if not np.any(keep) and not allow_empty:
        raise ValueError("RTX scan contains no valid non-self returns")
    selected = np.flatnonzero(keep)
    selected = selected[np.argsort(relative[selected], kind="stable")]
    records = np.zeros(len(selected), dtype=OUSTER_DTYPE)
    points_imu = xyz[selected] + mount - imu
    for j, name in enumerate(("x", "y", "z")):
        records[name] = points_imu[:, j]
    records["intensity"] = intensities[selected]
    records["t"] = relative[selected].astype("<u4")
    records["ring"] = rings[selected].astype("u1")
    records["range"] = np.rint(distance[selected] * 1000).astype("<u4")
    stamp = int(timestamp_ns) + origin
    if stamp < 0:
        raise ValueError("Negative RTX scan timestamp")
    receipt = {
        "source": "isaac_rtx_lidar", "model": LIDAR_VARIANT,
        "topic": LIDAR_TOPIC, "frame_id": "go2_imu", "stamp_ns": stamp,
        "native_timestamp_ns": int(timestamp_ns), "native_offset_min_ns": int(offsets.min()) if n else None,
        "header_rebase_ns": origin,
        "native_offset_max_ns": int(offsets.max()) if n else None,
        "first_retained_ns": stamp + int(records["t"][0]) if len(records) else None,
        "last_retained_ns": stamp + int(records["t"][-1]) if len(records) else None,
        "scan_firing_span_ns": int(offsets.max() - offsets.min()) if n else 0, "raw_points": n,
        "points": len(records), "invalid_returns": int((~valid).sum()),
        "self_returns": int((valid & own).sum()),
        "rings": np.unique(records["ring"]).astype(int).tolist(),
        "time_unit": "relative nanoseconds", "point_step": OUSTER_DTYPE.itemsize,
        "point_sha256": hashlib.sha256(records.tobytes()).hexdigest(),
        "motion_compensation": False, "truth_pose_used": False,
        "transform": "p_imu = p_lidar + lidar_origin_in_base - imu_origin_in_base",
        "decimation": False,
        "ancillary_fields": "reflectivity and ambient unavailable (zero); range is native sensor distance in millimeters",
    }
    return LidarPacket(stamp, records, receipt)


def pointcloud_message(packet):
    """Serialize the verified Ouster layout, without changing source stamp."""
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(packet.stamp_ns, 1_000_000_000)
    msg.header.frame_id = "go2_imu"
    msg.height, msg.width = 1, len(packet.records)
    typecodes = {"f": 7, "u1": 2, "u2": 4, "u4": 6}
    for name in OUSTER_DTYPE.names:
        dtype, offset = OUSTER_DTYPE.fields[name]
        code = typecodes["f" if dtype.kind == "f" else "u" + str(dtype.itemsize)]
        msg.fields.append(PointField(name=name, offset=offset, datatype=code, count=1))
    msg.point_step = OUSTER_DTYPE.itemsize
    msg.row_step = msg.width * msg.point_step
    msg.is_dense = True
    msg.is_bigendian = False
    msg.data = packet.records.tobytes()
    return msg


def prepare_lidar_asset(output):
    """Resolve and freeze the official composed OS1 asset before opening the scene."""
    import json
    from pathlib import Path
    import omni.usd
    from isaacsim.sensors.experimental.rtx import Lidar
    from isaacsim.storage.native import get_assets_root_path

    destination = Path(output) / "sensor_config"
    destination.mkdir(exist_ok=False)
    # The first successful official load is frozen locally. Reuse its verified
    # bytes rather than checking the public asset server on every experiment.
    from embodied_navigation.project import data_root
    import os
    import shutil
    import sys
    version_file = None
    isaac_root = os.environ.get("ISAAC_SIM_ROOT")
    if isaac_root:
        candidate = Path(isaac_root) / "VERSION"
        if candidate.exists():
            version_file = candidate
    if version_file is None:
        for parent in Path(sys.executable).resolve().parents:
            candidate = parent / "VERSION"
            if candidate.exists():
                version_file = candidate
                break
    version = version_file.read_text().strip() if version_file else ""
    cache = data_root() / "runtime_assets/sensors/isaacsim_6_0_1_rc7"
    cached = cache / (LIDAR_VARIANT + ".usd")
    cached_meta = cache / "os1_32ch10hz1024res.provenance.json"
    if version.startswith("6.0.1-rc.7") and cached.exists() and cached_meta.exists():
        provenance = json.loads(cached_meta.read_text())
        digest = hashlib.sha256(cached.read_bytes()).hexdigest()
        if (provenance.get("variant") != LIDAR_VARIANT or provenance.get("config") != "OS1"
                or digest != provenance.get("asset_sha256")):
            raise ValueError("Cached official RTX sensor asset provenance mismatch")
        asset = destination / cached.name
        shutil.copy2(cached, asset)
        receipt = dict(provenance, asset=str(asset), asset_sha256=digest,
                       resolved_from_verified_cache=str(cached), isaac_version=version)
        (destination / "provenance.json").write_text(json.dumps(receipt, indent=2))
        return asset, receipt
    omni.usd.get_context().new_stage()
    lidar = Lidar.create("/Lidar", config="OS1", variant=LIDAR_VARIANT,
                         aux_output_level="BASIC", accumulate_outputs=True, tick_rate=10.0)
    temporary = omni.usd.get_context().get_stage()
    temporary.SetDefaultPrim(temporary.GetPrimAtPath("/Lidar"))
    asset = destination / (LIDAR_VARIANT + ".usd")
    temporary.Flatten().Export(str(asset))
    receipt = {
        "source": "official NVIDIA OS1 USD and named variant",
        "asset_root": "official configured NVIDIA Isaac assets root", "config": "OS1", "variant": LIDAR_VARIANT,
        "asset": str(asset), "asset_sha256": hashlib.sha256(asset.read_bytes()).hexdigest(),
        "channel_to_ring": "native GMO channelId already zero-based 0..31, verified by real probe",
        "firing_pattern": "official OS1 REV6 32ch 10Hz 1024res",
        "local_json_conversion_used": False,
    }
    (destination / "provenance.json").write_text(json.dumps(receipt, indent=2))
    return asset, receipt


class RtxLidar:
    """One physical GPU ray-traced scanner, with no RGB-D fallback."""

    def __init__(self, rig, asset):
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
        import omni.replicator.core as rep

        self.complete_certificate = None
        if os.environ.get("DYNANAV_RTX_COMPLETE_SCAN") == "1":
            self.complete_certificate = CompleteScanCertificate(attest_complete_scan_asset(asset))

        self.base_path = rig.base_path
        self.imu_mount = np.asarray(rig.imu_mount, dtype=float).copy()
        self.lidar = Lidar.create(
            rig.base_path + "/RtxLidar", usd_path=str(asset),
            aux_output_level="BASIC", accumulate_outputs=True, tick_rate=10.0,
            translations=LIDAR_MOUNT,
            attributes={"omni:sensor:Core:outputFrameOfReference": "SENSOR",
                        "omni:sensor:Core:outputMotionCompensationState": "NONCOMPENSATED",
                        "omni:sensor:Core:elementsCoordsType": "CARTESIAN"},
        )
        if self.complete_certificate is not None:
            attest_complete_scan_asset(asset, runtime_prim=self.lidar.prims[0])
        self.sensor = LidarSensor(self.lidar, annotators=[])
        self.writer_error = None
        self.writer_callbacks = 0
        self.read_calls = 0
        self.queue_peak_depth = 0
        owner = self
        class DynanavNativeLidarWriter(rep.Writer):
            def __init__(self):
                self.data_structure = "renderProduct"
                self.annotators = [rep.annotators.get("GenericModelOutput")]
            def write(self, data):
                callback_monotonic_ns = time.monotonic_ns()
                owner.writer_callbacks += 1
                callback_sequence = owner.writer_callbacks
                try:
                    for product in data.get("renderProducts", {}).values():
                        raw = product.get("GenericModelOutput")
                        if isinstance(raw, dict):
                            raw = raw.get("data")
                        if raw is not None:
                            owner.accept_native_output(
                                raw, callback_monotonic_ns=callback_monotonic_ns,
                                callback_sequence=callback_sequence,
                            )
                except BaseException as error:
                    owner.writer_error = error
        rep.WriterRegistry.register(DynanavNativeLidarWriter)
        self.writer = self.sensor.attach_writer("DynanavNativeLidarWriter")
        from collections import deque
        self.pending = deque()
        self.last_key = None
        self.minimum_scan_start_ns = 0
        self.scans = 0

    def installation(self):
        from pxr import Gf, UsdGeom
        prim = self.lidar.prims[0]
        # Compose authored local transforms only, stopping before base. This is
        # fixed assembly calibration, never a world-pose measurement or deskew.
        relative = Gf.Matrix4d(1)
        part = prim
        while str(part.GetPath()) != self.base_path:
            if not part.IsValid() or part.IsPseudoRoot():
                raise ValueError("RTX sensor is not parented to the actual robot base")
            local = UsdGeom.Xformable(part)
            if local.GetResetXformStack() or any(op.MightBeTimeVarying() for op in local.GetOrderedXformOps()):
                raise ValueError("RTX installation must be a fixed inherited local transform")
            relative = relative * local.GetLocalTransformation()
            part = part.GetParent()
        expected = np.eye(4)
        expected[3, :3] = LIDAR_MOUNT
        if not np.allclose(np.asarray(relative), expected, atol=1e-7):
            raise ValueError("RTX authored sensor-to-base installation differs from calibration")
        keys = ["scanRateBaseHz", "patternFiringRateHz", "numberOfChannels", "numberOfEmitters",
                "outputFrameOfReference", "outputMotionCompensationState", "elementsCoordsType", "accumulateOutputs"]
        return {
            "model": LIDAR_VARIANT, "type": "RTX rotating LiDAR, not Livox and not depth projection",
            "parent": str(prim.GetPath().GetParentPath()), "prim": str(prim.GetPath()),
            "lidar_translation_in_base": LIDAR_MOUNT.tolist(),
            "lidar_R_in_base": np.eye(3).tolist(),
            "sensor_to_base_row_matrix": np.asarray(relative).tolist(),
            "installation_verified_without_world_pose": True,
            "lidar_translation_in_IMU": (LIDAR_MOUNT - self.imu_mount).tolist(),
            "published_frame": "go2_imu", "topic": LIDAR_TOPIC,
            "coordinates": "native sensor-local noncompensated rays plus fixed lidar-to-IMU transform",
            "native_attributes": {k: prim.GetAttribute("omni:sensor:Core:" + k).Get() for k in keys},
            "truth_pose_used": False, "true_per_point_timing": True,
        }

    def capture(self):
        """Surface callback failures; the Writer captures every native frame."""
        if self.writer_error is not None:
            raise RuntimeError("Native LiDAR writer failed") from self.writer_error

    def accept_native_output(self, raw, *, callback_monotonic_ns=None, callback_sequence=None):
        """Observe host receipt/queue boundaries, never change native ray times.

        All *_monotonic_ns values use this process's host monotonic clock, not
        simulation time, UTC, or a GPU completion timestamp. The Writer supplies
        its entry time/sequence (shared by products in one callback). Direct
        calls use this method's entry instead. Only accepted scans carry rows;
        callback-sequence gaps may be empty/partial/duplicate/startup outputs,
        not evidence of lost scans. No callback logging or additional copy.
        """
        if callback_monotonic_ns is None:
            callback_monotonic_ns = time.monotonic_ns()
        if callback_sequence is None:
            self.writer_callbacks += 1
            callback_sequence = self.writer_callbacks
        packet = self._parse_native(raw)
        if packet is not None:
            depth_before = len(self.pending)
            if depth_before >= 4:
                raise RuntimeError("RTX scan queue overflow; refusing silent scan loss")
            packet.receipt.update(
                host_timing_schema=1,
                writer_callback_monotonic_ns=callback_monotonic_ns,
                writer_callback_sequence=callback_sequence,
                queue_depth_before_enqueue=depth_before,
                queue_capacity_scans=4,
                # Four complete 10 Hz rotations; this is not a wall-time or
                # downstream IMU/RGB retention guarantee, nor extra capacity.
                queue_nominal_coverage_ns=400_000_000,
            )
            # Immediately before append, after the existing parse/pack/hash.
            packet.receipt["enqueue_monotonic_ns"] = time.monotonic_ns()
            self.pending.append(packet)
            depth_after = len(self.pending)
            self.queue_peak_depth = max(self.queue_peak_depth, depth_after)
            packet.receipt.update(queue_depth_after_enqueue=depth_after,
                                  queue_peak_depth=self.queue_peak_depth)

    def read(self):
        read_monotonic_ns = time.monotonic_ns()
        self.read_calls += 1
        self.capture()
        if not self.pending:
            return None
        depth_before = len(self.pending)
        packet = self.pending.popleft()
        dequeue_monotonic_ns = time.monotonic_ns()
        depth_after = len(self.pending)
        packet.receipt.update(
            read_monotonic_ns=read_monotonic_ns,
            read_sequence=self.read_calls,
            dequeue_monotonic_ns=dequeue_monotonic_ns,
            queue_depth_before_dequeue=depth_before,
            queue_depth_after_dequeue=depth_after,
            queue_residence_ns=dequeue_monotonic_ns - packet.receipt["enqueue_monotonic_ns"],
        )
        return packet

    def _parse_native(self, raw):
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
        if raw is None:
            return None
        # GMO array fields borrow the underlying bytes. Own them through packing.
        copy_begin_monotonic_ns = time.monotonic_ns()
        raw_copy = np.array(raw.numpy() if hasattr(raw, "numpy") else raw, copy=True)
        copy_end_monotonic_ns = time.monotonic_ns()
        if raw_copy.size == 0:
            return None
        gmo = parse_generic_model_output_data(raw_copy)
        certifier = getattr(self, "complete_certificate", None)
        if not gmo.numElements and certifier is None:
            return None
        if (gmo.frameOfReference.name != "SENSOR" or gmo.motionCompensationState.name != "NONCOMPENSATED"
                or gmo.elementsCoordsType.name != "CARTESIAN"):
            raise ValueError("RTX output must be sensor-local, Cartesian and noncompensated")
        if int(gmo.timestampNs) < self.minimum_scan_start_ns:
            return None
        certificate = certifier.observe(gmo) if certifier is not None else None
        key = (int(gmo.frameId), int(gmo.timestampNs))
        if self.last_key is not None:
            if key[1] == self.last_key[1]:
                return None
            if key[1] < self.last_key[1]:
                raise ValueError("RTX scan clock moved backwards")
        if not int(gmo.scanComplete):
            return None
        model_transform = np.asarray(gmo.modelToAppTransform).reshape(4, 4)
        if not np.allclose(model_transform, np.eye(4), atol=1e-7):
            raise ValueError("Unexpected nonidentity RTX model-to-app axes")
        packet = pack_lidar_scan(
            xyz_sensor=np.column_stack([gmo.x, gmo.y, gmo.z]), intensity=np.asarray(gmo.scalar),
            offsets_ns=np.asarray(gmo.timeOffsetNs), ring=np.asarray(gmo.channelId),
            timestamp_ns=int(gmo.timestampNs), imu_mount=self.imu_mount,
            valid_flags=(np.asarray(gmo.flags) & 64) != 0,
            allow_empty=certificate is not None,
        )
        if certificate is not None:
            if packet.stamp_ns != certificate["source_start_ns"] or np.any(packet.records["t"] >= COMPLETE_SCAN_PERIOD_NS):
                certifier.failed = True
                raise ValueError("Complete-scan retained cloud contradicts certificate")
            else:
                packet.receipt["complete_scan_certificate"] = certificate
        # These native renderer poses are retained only for independent scoring.
        # They never enter pack_lidar_scan or the ROS cloud coordinates.
        packet.receipt["native_world_pose_scoring_only"] = {
            "start": {"timestamp_ns": int(gmo.frameStart.timestampNs), "position": list(gmo.frameStart.posM), "quaternion_xyzw": list(gmo.frameStart.orientation)},
            "end": {"timestamp_ns": int(gmo.frameEnd.timestampNs), "position": list(gmo.frameEnd.posM), "quaternion_xyzw": list(gmo.frameEnd.orientation)},
            "used_for_point_transform": False,
        }
        packet.receipt.update(native_frame_id=int(gmo.frameId), scan_complete=int(gmo.scanComplete),
                              native_motion_compensation=gmo.motionCompensationState.name,
                              native_frame_reference=gmo.frameOfReference.name)
        # Includes raw.numpy() materialization, when present. Parse/pack/hash
        # follow this interval and precede enqueue; GPU->callback is unobserved.
        packet.receipt.update(
            raw_copy_begin_monotonic_ns=copy_begin_monotonic_ns,
            raw_copy_end_monotonic_ns=copy_end_monotonic_ns,
            raw_copy_duration_ns=copy_end_monotonic_ns - copy_begin_monotonic_ns,
            raw_copy_bytes=int(raw_copy.nbytes),
        )
        packet.receipt["native_scan_start_delta_ns"] = None if self.last_key is None else key[1] - self.last_key[1]
        self.last_key = key
        self.scans += 1
        packet.receipt["scan_index"] = self.scans
        return packet
