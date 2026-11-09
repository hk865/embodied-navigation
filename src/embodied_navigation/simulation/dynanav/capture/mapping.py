"""Oracle/learned mapping, scoring and raw-RGB archive wiring.

This module owns the two optional mapping sinks (simulator oracle and learned
RGB person mapping), the optional evaluation-only learned scorer, the instance
segmentation annotator they need, and the raw RGB archive. Mapping frames are
processed in the original order and the module keeps the exact receipt keys,
journal payloads and error handling for a failed finalisation. It never imports
Isaac modules at import time.
"""

import numpy as np

from embodied_navigation.project import data_root


def open_rgb_archive(out, experiment, result):
    """Open the raw RGB archive, refusing paths outside the project data root."""
    rgb_archive = None
    if experiment.get("archive_rgb", False):
        from embodied_navigation.simulation.dynanav.rgb_archive import RawRgbArchive

        if not out.resolve().is_relative_to(data_root().resolve()):
            raise ValueError(
                "Raw RGB archive must be below the external project data root"
            )
        rgb_archive = RawRgbArchive(out / "rgb_raw")
        result["rgb_archive"] = rgb_archive.receipt(acquired_frames=0)
    return rgb_archive


def process_oracle_frame(
    *,
    oracle_mapper,
    packet,
    instance_ann,
    it,
    Rb,
    sensor_position,
    rgb,
    zz,
    xyz,
    actual_time,
    dynamic_guard,
    result,
    journal,
):
    """Update the oracle mapping sink and the oracle-cloud dynamic hold."""
    states = None
    if oracle_mapper is not None:
        transform = np.eye(4)
        transform[:3, :3] = Rb
        transform[:3, 3] = sensor_position
        states, frame_receipt = oracle_mapper.process(
            packet=packet,
            segmentation=instance_ann.get_data(),
            source_ns=round(it * 1e9),
            transform=transform,
            rgb=rgb,
            depth=zz,
        )
        result["mapping_frame"] = frame_receipt
        if dynamic_guard is not None:
            from embodied_navigation.localization_mapping.semantic_mapping import (
                DYNAMIC,
            )

            physics_ns = round(actual_time * 1e9)
            held, hold_receipt = dynamic_guard.update(
                xyz[states == DYNAMIC],
                source_ns=physics_ns,
                mask_source_ns=physics_ns,
                now_ns=physics_ns,
                mask_valid=frame_receipt["mask_valid"]
                and frame_receipt["unknown"] == 0,
                sampled_point_count=len(xyz),
            )
            hold_receipt["ros_sensor_stamp_ns"] = round(it * 1e9)
            hold_receipt["clock_binding"] = (
                "same validated physics tick; ROS IMU stamp retains float32 quantization"
            )
            result["dynamic_hold"] = hold_receipt
            journal("dynamic_hold_frames.jsonl", hold_receipt)
    return states


def process_learned_frame(
    *,
    learned_mapper,
    learned_scorer,
    person_oracle_labels,
    dynamic_guard,
    packet,
    xyz,
    rgb,
    zz,
    it,
    actual_time,
    latest_odom,
    instance_ann,
    result,
    journal,
):
    """Update the learned mapping sink, evaluation scorer and RGB-evidence hold."""
    states = None
    frontend_keep = None
    if learned_mapper is not None:
        states, frontend_keep, frame_receipt, quarantined = (
            learned_mapper.process(
                packet=packet,
                source_ns=round(it * 1e9),
                rgb=rgb,
                depth=zz,
                latest_odom=latest_odom[0],
            )
        )
        result["mapping_frame"] = frame_receipt
        if learned_scorer is not None:
            # Evaluation labels never enter online inference, tracking,
            # candidate filtering, route control or geometric votes.
            scoring_segmentation = instance_ann.get_data()
            oracle_states = person_oracle_labels(
                scoring_segmentation, packet.pixel_flat_indices
            )
            learned_scorer.observe(
                round(it * 1e9),
                oracle_states,
                states,
                frontend_keep,
                segmentation=scoring_segmentation,
                pixels=packet.pixel_flat_indices,
            )
        if dynamic_guard is not None and frame_receipt["fresh_inference"]:
            physics_ns = round(actual_time * 1e9)
            held, hold_receipt = dynamic_guard.update(
                xyz[(states == 2) | quarantined],
                source_ns=physics_ns,
                mask_source_ns=physics_ns,
                now_ns=physics_ns,
                mask_valid=frame_receipt["quarantine_pose_valid"],
                sampled_point_count=len(xyz),
            )
            hold_receipt["ros_sensor_stamp_ns"] = round(it * 1e9)
            hold_receipt["evidence_scope"] = (
                "fresh successful RGB detector plus bounded prior quarantine; negative detections are provisional, not certified absence"
            )
            result["dynamic_hold"] = hold_receipt
            journal("dynamic_hold_frames.jsonl", hold_receipt)
    return states, frontend_keep


class MappingSession:
    """Own the oracle/learned mappers, evaluation scorer and instance annotator."""

    def __init__(
        self,
        experiment,
        stage,
        out,
        rep,
        product,
        context,
        gt_diagnostic,
        slam_control,
    ):
        self.experiment = experiment
        self.oracle_mapper = None
        self.learned_mapper = None
        self.learned_scorer = None
        self.instance_ann = None
        self.person_oracle_labels = None
        self.context = context
        context.mapping = self
        if experiment.get("semantic_provider") == "isaac_instance_oracle":
            if not (gt_diagnostic or slam_control):
                raise ValueError("Oracle diagnostic maps require an explicit experiment")
            from embodied_navigation.simulation.dynanav.oracle_mapping import OracleMapping

            self.oracle_mapper = OracleMapping(stage, out / "mapping", experiment)
        elif experiment.get("semantic_provider") == "learned_rgb_person":
            from embodied_navigation.simulation.dynanav.learned_mapping import (
                LearnedMapping,
            )

            self.learned_mapper = LearnedMapping(out / "mapping", experiment)
            if experiment.get("oracle_evaluation"):
                from embodied_navigation.simulation.dynanav.learned_scoring import (
                    LearnedScoring,
                    person_oracle_labels,
                )

                self.learned_scorer = LearnedScoring(out / "learned_scoring")
        if self.oracle_mapper is not None or self.learned_scorer is not None:
            self.instance_ann = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation")
            self.instance_ann.attach([product])

        if self.learned_scorer is not None:
            self.person_oracle_labels = person_oracle_labels

    def process_frame(
        self,
        *,
        packet,
        xyz,
        rgb,
        zz,
        it,
        actual_time,
        Rb,
        sensor_position,
        latest_odom,
        dynamic_guard,
        result,
        journal,
    ):
        """Process one 10 Hz RGB-D frame through every enabled mapping sink."""
        states = process_oracle_frame(
            oracle_mapper=self.oracle_mapper,
            packet=packet,
            instance_ann=self.instance_ann,
            it=it,
            Rb=Rb,
            sensor_position=sensor_position,
            rgb=rgb,
            zz=zz,
            xyz=xyz,
            actual_time=actual_time,
            dynamic_guard=dynamic_guard,
            result=result,
            journal=journal,
        )
        frontend_keep = None
        if self.learned_mapper is not None:
            states, frontend_keep = process_learned_frame(
                learned_mapper=self.learned_mapper,
                learned_scorer=self.learned_scorer,
                person_oracle_labels=self.person_oracle_labels,
                dynamic_guard=dynamic_guard,
                packet=packet,
                xyz=xyz,
                rgb=rgb,
                zz=zz,
                it=it,
                actual_time=actual_time,
                latest_odom=latest_odom,
                instance_ann=self.instance_ann,
                result=result,
                journal=journal,
            )
        return states, frontend_keep

    def finish(self, result):
        """Finalize every enabled mapping sink into result.json (success path)."""
        if self.oracle_mapper is not None:
            result["mapping"] = self.oracle_mapper.finish()
        if self.learned_mapper is not None:
            result["mapping"] = self.learned_mapper.finish()
        if self.learned_scorer is not None:
            result["learned_scoring"] = self.learned_scorer.finish(
                self.learned_mapper.geometry.confirmed_records()
            )

    def finalize_failure(self, result):
        """Finalize any partially built mapping sink without masking the primary error."""
        if self.oracle_mapper is not None:
            try:
                result["mapping"] = self.oracle_mapper.finish()
            except Exception as mapping_error:  # noqa: BLE001 - retain the primary simulation failure
                result["mapping_finalize_error"] = repr(mapping_error)
        if self.learned_mapper is not None:
            try:
                result["mapping"] = self.learned_mapper.finish()
                if self.learned_scorer is not None:
                    result["learned_scoring"] = self.learned_scorer.finish(
                        self.learned_mapper.geometry.confirmed_records()
                    )
            except Exception as mapping_error:  # noqa: BLE001 - preserve the primary simulation failure
                result["learned_mapping_finalize_error"] = repr(mapping_error)
