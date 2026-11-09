"""Oracle-only evaluation sink; never supplies labels to online perception."""

from __future__ import annotations

import itertools
import json
from pathlib import Path

import numpy as np


def person_oracle_labels(
    segmentation, pixels, *, person_root="/World/DynamicPedestrian"
):
    """Renderer identity evaluation only: inserted person vs known non-person.

    Non-person is a binary negative, not proof of permanent static ownership.
    This function reads no stage, native pose, model output or object tracker.
    """
    pixels = np.asarray(pixels)
    states = np.zeros(len(pixels), dtype=np.uint8)
    if not isinstance(segmentation, dict):
        return states
    image = np.asarray(segmentation.get("data"))
    if image.ndim == 3 and image.shape[-1] == 1:
        image = image[..., 0]
    mapping = segmentation.get("info", {}).get("idToLabels", {})
    if (
        image.ndim != 2
        or not np.issubdtype(image.dtype, np.integer)
        or np.any(pixels < 0)
        or np.any(pixels >= image.size)
    ):
        return states
    identities = image.reshape(-1)[pixels]
    for ident in np.unique(identities):
        path = mapping.get(str(int(ident)), mapping.get(int(ident), ""))
        if (
            not isinstance(path, str)
            or not path.startswith("/")
            or path.startswith(("/World/Go2", "/__Prototype_"))
        ):
            continue
        states[identities == ident] = (
            2 if path == person_root or path.startswith(person_root + "/") else 1
        )
    return states


class LearnedScoring:
    def __init__(self, output, *, person_root="/World/DynamicPedestrian"):
        self.output = Path(output)
        self.output.mkdir()
        (self.output / "labels").mkdir()
        self.person_root = person_root
        self.labels = {}
        self.persons = {}
        self.frontend = {
            "frames": 0,
            "raw_dynamic": 0,
            "retained_dynamic": 0,
            "raw_person": 0,
            "retained_person": 0,
            "raw_static": 0,
            "removed_static": 0,
        }
        self.admitted_dynamic = self.admitted_person = self.admitted_total = 0
        self.admitted_unknown_labels = 0
        self.id_frames = []

    def observe(
        self, source_ns, oracle_states, candidate_states, keep, *, segmentation, pixels
    ):
        oracle = np.asarray(oracle_states, dtype=np.uint8).copy()
        person = np.zeros(len(oracle), dtype=bool)
        segmentation = segmentation if isinstance(segmentation, dict) else {}
        image = np.asarray(segmentation.get("data"))
        if image.ndim == 3 and image.shape[-1] == 1:
            image = image[..., 0]
        mapping = segmentation.get("info", {}).get("idToLabels", {})
        if image.ndim == 2 and np.issubdtype(image.dtype, np.integer):
            identities = image.reshape(-1)[pixels]
            for ident in np.unique(identities):
                path = mapping.get(str(int(ident)), mapping.get(int(ident), ""))
                if isinstance(path, str) and (
                    path == self.person_root or path.startswith(self.person_root + "/")
                ):
                    person |= identities == ident
        self.labels[source_ns], self.persons[source_ns] = oracle, person
        np.savez_compressed(
            self.output / "labels" / f"{source_ns}.npz",
            source_ns=np.int64(source_ns),
            oracle_states=oracle,
            oracle_person=person,
        )
        counts = {
            "source_ns": source_ns,
            "raw_dynamic": int(np.sum(oracle == 2)),
            "retained_dynamic": int(np.sum((oracle == 2) & keep)),
            "raw_person": int(person.sum()),
            "retained_person": int(np.sum(person & keep)),
            "raw_static": int(np.sum(oracle == 1)),
            "removed_static": int(np.sum((oracle == 1) & ~keep)),
            "direct_person_candidate_points": int(np.sum(candidate_states == 2)),
            "raw_points": len(oracle),
            "retained_points": int(keep.sum()),
        }
        self.frontend["frames"] += 1
        for key in self.frontend:
            if key != "frames":
                self.frontend[key] += counts[key]
        self._journal("frontend.jsonl", counts)

    def _journal(self, name, value):
        with (self.output / name).open("a") as stream:
            stream.write(json.dumps(value, allow_nan=False) + "\n")

    def observe_completion(self, completed):
        ns = completed["source_ns"]
        indices = np.asarray(completed["static_indices"], dtype=np.int64)
        self.admitted_total += len(indices)
        if ns not in self.labels:
            self.admitted_unknown_labels += len(indices)
            self._journal(
                "missing_labels.jsonl",
                {"source_ns": ns, "admitted_unknown": len(indices)},
            )
            return
        oracle, person = self.labels[ns], self.persons[ns]
        dynamic = int(np.sum(oracle[indices] == 2))
        people = int(person[indices].sum())
        self.admitted_unknown_labels += int(np.sum(oracle[indices] == 0))
        self.admitted_dynamic += dynamic
        self.admitted_person += people
        row = {
            "source_ns": ns,
            "admitted": len(indices),
            "admitted_oracle_dynamic": dynamic,
            "admitted_oracle_person": people,
            "fresh_inference": completed["fresh_inference"],
        }
        ids = np.asarray(completed["tracked_ids"], dtype=str)
        selected = person & (np.asarray(completed["tracked_states"]) == 2) & (ids != "")
        values, counts = np.unique(ids[selected], return_counts=True)
        row.update(
            visible_person_points=int(person.sum()),
            tracked_person_points=int(selected.sum()),
            overlapping_ids={str(k): int(v) for k, v in zip(values, counts)},
            dominant_id=str(values[np.argmax(counts)]) if len(values) else None,
        )
        self.id_frames.append(row)
        self._journal("persistent_admissions.jsonl", row)

    def finish(self, static_records):
        labelled = dynamic = person = unknown = 0
        # The independent ledger identifies the original source point chosen
        # as representative; evaluation cannot alter cache votes or geometry.
        for record in static_records:
            ns, index = (
                record["representative_source_ns"],
                record["representative_point_index"],
            )
            if ns not in self.labels or not 0 <= index < len(self.labels[ns]):
                unknown += 1
                continue
            label = self.labels[ns][index]
            labelled += int(label in (1, 2))
            dynamic += int(label == 2)
            person += int(self.persons[ns][index])
            unknown += int(label == 0)
        support_total = support_dynamic = support_person = support_unknown = (
            contaminated_support_voxels
        ) = 0
        for record in static_records:
            has_dynamic = False
            for support in record.get("supports", []):
                ns, index = support["source_ns"], support["point_index"]
                support_total += 1
                if ns not in self.labels or not 0 <= index < len(self.labels[ns]):
                    support_unknown += 1
                    continue
                label = self.labels[ns][index]
                support_dynamic += int(label == 2)
                support_person += int(self.persons[ns][index])
                support_unknown += int(label == 0)
                has_dynamic |= bool(label == 2)
            contaminated_support_voxels += int(has_dynamic)
        fresh = [
            x
            for x in self.id_frames
            if x["fresh_inference"] and x["visible_person_points"]
        ]
        matched = [x for x in fresh if x["dominant_id"] is not None]
        switches = sum(
            left["dominant_id"] != right["dominant_id"]
            for left, right in itertools.pairwise(matched)
        )
        result = {
            "schema": "online_learned_perception_oracle_scoring/v1",
            "evaluation_only": True,
            "label_protocol": "inserted pedestrian renderer identity vs known non-person pixels; binary negatives do not certify permanent static ownership",
            "labels_returned_to_algorithm": False,
            "frontend": self.frontend,
            "frontend_dynamic_retained_fraction": self.frontend["retained_dynamic"]
            / self.frontend["raw_dynamic"]
            if self.frontend["raw_dynamic"]
            else None,
            "frontend_person_retained_fraction": self.frontend["retained_person"]
            / self.frontend["raw_person"]
            if self.frontend["raw_person"]
            else None,
            "persistent_admissions": {
                "total": self.admitted_total,
                "unknown_or_missing_labels": self.admitted_unknown_labels,
                "oracle_dynamic": self.admitted_dynamic,
                "oracle_person": self.admitted_person,
            },
            "final_geometry_representatives": {
                "total": len(static_records),
                "known_labels": labelled,
                "oracle_dynamic": dynamic,
                "oracle_person": person,
                "unknown_or_unmatched": unknown,
                "oracle_dynamic_fraction_of_known": dynamic / labelled
                if labelled
                else None,
            },
            "final_retained_geometry_supports": {
                "total": support_total,
                "oracle_dynamic": support_dynamic,
                "oracle_person": support_person,
                "unknown_or_missing_labels": support_unknown,
                "voxels_with_any_dynamic_support": contaminated_support_voxels,
                "scope": "all retained support votes, separate from final representative point labels",
            },
            "single_inserted_person_identity": {
                "fresh_exact_pose_person_visible_frames": len(fresh),
                "frames_with_confirmed_tracked_person_points": len(matched),
                "dominant_ids": sorted({x["dominant_id"] for x in matched}),
                "dominant_ID_changes_across_matched_frames": switches,
                "scope": "single inserted actor overlap, not multi-person IDF1; missed/unscheduled frames are not inferred successes",
            },
            "limits": [
                "Renderer ownership marks potential movable/person points, including standing people.",
                "Input filtering is not a direct audit of native FAST-LIVO internal map state.",
                "Repeated geometric support is not permanent static ownership; pollution is measured, not assumed absent.",
                "No oracle pose aligns or corrects the online map in this scoring sink.",
            ],
        }
        (self.output / "receipt.json").write_text(json.dumps(result, indent=2))
        return result
