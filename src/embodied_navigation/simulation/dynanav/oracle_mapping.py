"""Simulator-only mask/pose oracle for checking map plumbing, not learned perception.

The renderer supplies a mask for every RGB-D observation. Semantic metadata is
refreshed sparsely. Unknown pixels are quarantined; the navigation cloud is not
filtered. Geometry is persisted only on selected sensor keyframes.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    DYNAMIC,
    STATIC,
    UNKNOWN,
    LayeredMap,
    Observation,
    PointMask,
    SemanticScheduler,
)


class OracleMapping:
    def __init__(self, stage, output, profile):
        self.stage, self.output, self.profile = stage, Path(output), profile
        self.output.mkdir()
        self.frames = self.output / "keyframes"
        self.frames.mkdir()
        self.store = LayeredMap(max_observations=301)
        self.scheduler = SemanticScheduler(
            interval_ns=profile["semantic_interval_ns"],
            event_min_interval_ns=500_000_000,
        )
        self.last_keyframe_ns = None
        self.labels, self.geometry = {}, {}
        self.frame_count = self.keyframes = self.mask_failures = 0
        self.counts = {"static": 0, "dynamic": 0, "unknown": 0, "navigation": 0}
        self.empty_frames = self.low_coverage_frames = 0

    def _geometry(self, path):
        if path in self.geometry:
            return self.geometry[path]
        from pxr import UsdPhysics

        prim = self.stage.GetPrimAtPath(path)
        if (
            not prim
            or path.startswith(("/World/Go2", "/__Prototype_"))
            or prim.IsInPrototype()
        ):
            return UNKNOWN
        state = STATIC
        # Authored movable ownership is a per-frame oracle safety prior, even
        # while the sparse semantic label cache has not refreshed yet.
        if self._label(path).lower() in self.store.movable_labels:
            state = DYNAMIC
        cursor = prim
        roots = self.profile.get("pedestrian_roots", [])
        if any(path == root or path.startswith(root + "/") for root in roots):
            state = DYNAMIC
        while cursor and not cursor.IsPseudoRoot():
            if cursor.HasAPI(UsdPhysics.RigidBodyAPI) or any(
                attr.GetName().startswith("xformOp:") and attr.GetNumTimeSamples() > 1
                for attr in cursor.GetAttributes()
            ):
                state = DYNAMIC
            cursor = cursor.GetParent()
        self.geometry[path] = state
        return state

    def _label(self, path):
        if any(
            path == root or path.startswith(root + "/")
            for root in self.profile.get("pedestrian_roots", [])
        ):
            return "person"
        prim = self.stage.GetPrimAtPath(path)
        while prim and not prim.IsPseudoRoot():
            for attr in prim.GetAttributes():
                if attr.GetName().endswith(":semanticData") and attr.Get():
                    return str(attr.Get())
            prim = prim.GetParent()
        return "scene_geometry"

    def process(self, *, packet, segmentation, source_ns, transform, rgb, depth):
        self.frame_count += 1
        sequence = self.frame_count
        states = np.zeros(len(packet.points), dtype=np.uint8)
        instances = np.full(len(packet.points), "", dtype=object)
        valid = False
        labels = {}
        if isinstance(segmentation, dict):
            image = np.asarray(segmentation.get("data"))
            if image.ndim == 3 and image.shape[-1] == 1:
                image = image[..., 0]
            mapping = segmentation.get("info", {}).get("idToLabels", {})
            if (
                image.shape == depth.shape
                and np.issubdtype(image.dtype, np.integer)
                and mapping
            ):
                ids = image.reshape(-1)[packet.pixel_flat_indices]
                for instance in np.unique(ids):
                    path = mapping.get(
                        str(int(instance)), mapping.get(int(instance), "")
                    )
                    if not isinstance(path, str) or not path.startswith("/"):
                        continue
                    selected = ids == instance
                    states[selected] = self._geometry(path)
                    instances[selected] = path
                valid = True
                visible = set(instances) - {""}
                newly_visible = visible - self.labels.keys()
                if self.scheduler.request(
                    source_ns, epoch=1, event=bool(newly_visible)
                ):
                    for path in visible:
                        self.labels[path] = self._label(path)
                    self.scheduler.complete(source_ns, epoch=1)
                labels = {
                    path: self.labels[path]
                    for path in set(instances)
                    if path in self.labels
                }
            if sequence == 1:
                (self.output / "renderer_identity.json").write_text(
                    json.dumps(
                        {
                            "provider": "isaac_instance_oracle",
                            "data_shape": list(image.shape),
                            "info": segmentation.get("info", {}),
                            "valid": valid,
                        },
                        indent=2,
                    )
                )
        if not valid:
            self.mask_failures += 1
        counts = {
            "static": int(np.sum(states == STATIC)),
            "dynamic": int(np.sum(states == DYNAMIC)),
            "unknown": int(np.sum(states == UNKNOWN)),
            "navigation": len(states),
            "mask_valid": valid,
        }
        self.empty_frames += len(states) == 0
        self.low_coverage_frames += len(states) <= 1000
        for name in self.counts:
            self.counts[name] += counts[name]
        due = (
            self.last_keyframe_ns is None
            or source_ns - self.last_keyframe_ns
            >= self.profile["map_keyframe_interval_ns"]
        )
        if due:
            observation = Observation(
                epoch=1,
                sequence=sequence,
                source_ns=source_ns,
                pose_source_ns=source_ns,
                points_sensor=packet.points,
                T_map_sensor=transform,
                anchor_id=f"rgbd:{sequence}",
                frame_id="go2_imu",
                calibration_id="rgbd:v1",
                pose_source="sim_oracle",
            )
            mask = (
                PointMask(
                    epoch=1,
                    sequence=sequence,
                    source_ns=source_ns,
                    produced_ns=source_ns,
                    frame_id="go2_imu",
                    calibration_id="rgbd:v1",
                    states=states,
                    instance_ids=tuple(instances),
                    labels=labels,
                    provenance={
                        "kind": "sim_oracle",
                        "producer": "isaac.instance_id_segmentation",
                        "instance_id_scope": "epoch",
                        "geometry_source": "USD static/rigid/animated ownership",
                        "semantic_calls": self.scheduler.requested,
                    },
                )
                if valid
                else None
            )
            result = self.store.integrate(observation, mask, now_ns=source_ns)
            self.keyframes += 1
            self.last_keyframe_ns = source_ns
            np.savez_compressed(
                self.frames / f"rgbd_{sequence:06d}.npz",
                depth=depth,
                rgb=rgb,
                points_sensor=packet.points,
                pixel_flat_indices=packet.pixel_flat_indices,
                states=states,
                instance_paths=instances.astype(str),
                T_map_sensor=transform,
                source_ns=np.int64(source_ns),
                labels_json=np.asarray(json.dumps(labels)),
                mask_provenance_json=np.asarray(
                    json.dumps(mask.provenance if mask else None)
                ),
                semantic_oracle=np.bool_(True),
                pose_oracle=np.bool_(True),
            )
            # The persistent mapper may strengthen a movable semantic label.
            # Apply the same result to the frontend branch for this observation.
            states[result["dynamic_indices"]] = DYNAMIC
            counts.update(
                keyframe=True,
                mask_status=result["mask_status"],
                persisted=result["persisted"],
            )
        with (self.output / "frames.jsonl").open("a") as stream:
            stream.write(
                json.dumps(dict(sequence=sequence, source_ns=source_ns, **counts))
                + "\n"
            )
        return states, counts

    def finish(self):
        points = self.store.static_points()
        np.savez_compressed(
            self.output / "static_map.npz",
            points=points,
            pose_source="sim_oracle",
            mask_source="sim_oracle",
            voxel_size=self.store.voxel_size,
        )
        self.store.save(self.output / "scene_map.json", include_points=False)
        receipt = {
            "frames": self.frame_count,
            "keyframes": self.keyframes,
            "oracle_semantic_refreshes": self.scheduler.requested,
            "learned_model_calls": 0,
            "mask_failures": self.mask_failures,
            "point_totals": self.counts,
            "static_map_points": len(points),
            "pose_source": "sim_oracle",
            "mask_source": "sim_oracle",
            "learned_perception": False,
            "static_frontend_topic": "/dynanav/lidar_static",
            "navigation_topic": "/dynanav/lidar_filtered",
            "SLAM_dynamic_filter_verified": False,
            "camera_visual_frontend_masked": False,
            "empty_rgbd_frames": self.empty_frames,
            "low_coverage_rgbd_frames": self.low_coverage_frames,
            "full_scene_coverage_verified": False,
        }
        (self.output / "receipt.json").write_text(json.dumps(receipt, indent=2))
        return receipt
