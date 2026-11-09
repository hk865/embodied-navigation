"""Bidirectional privileged Actor map selection; actual SLAM owns selection.

Native pose is used solely to compare the frozen Actor rays on the shared
landing. It is never returned as navigation pose or used to choose a layer.
"""

import copy
import json
import numpy as np
from embodied_navigation.simulation.terrain.provider import (
    SwitchingTerrainProvider,
    canonical,
    digest,
)

LAYERS = {"lower12": "initial", "upper23": "alternate"}
TRANSITIONS = {
    "exploring": ("exploration:11", "lower12", "upper23"),
    "returning": ("return_origin:5", "upper23", "lower12"),
    "navigating": ("navigation_f1_f3:7", "lower12", "upper23"),
}


class MissionTerrainProvider(SwitchingTerrainProvider):
    def __init__(self, run, scope, map_class, observation):
        copied = copy.deepcopy(scope)
        # Historical manifest names kept for the original scenario. They are not
        # in this checkout (tests/terrain_targets/ is absent, and only
        # configs/scenarios/terrain_targets/lower12.json exists). In
        # policy/worker.py, ``mission46_required`` alone selects this class;
        # ``terrain_layer_switch`` without it selects the generic
        # SwitchingTerrainProvider. Neither switch is enabled by the default
        # profile.
        copied["profile"]["terrain_layer_switch"] = dict(
            initial_manifest="tests/terrain_targets/lower12.json",
            alternate_manifest="tests/terrain_targets/upper23.json",
            alternate_include_models=["floor_2", "ramp_23", "floor_3"],
            completed_goal_id="connector_mid",
        )
        self.completed_transitions = []
        super().__init__(run, copied, map_class, observation)

    def contract(self):
        """Return the mission-46 switch contract with base geometry metadata."""
        result = super().contract()
        result.update(
            schema="teacher_mission46_actor_terrain/v1",
            transitions=TRANSITIONS,
            completed_goal_id=None,
            trigger="original46 actual SLAM arrival plus matching mission intent",
        )
        return result

    def evidence(self):
        """Return base evidence plus an ordered completed-transition list."""
        result = super().evidence()
        result.update(completed_transitions=list(self.completed_transitions))
        return result

    def check_switch(self):
        """Apply one pending mission-46 layer switch request, fail-closed.

        Returns False when there is nothing to do (no request, state not ready,
        transition already completed, or a verified switch that was committed)
        and True when motion must hold: a latched/refused switch or a causal wait
        for an arrival stamp up to 50 ms ahead of the current native clock.
        The short future-stamp wait does not latch ``self.failure``.
        """
        if self.failure is not None:
            return True
        if self.state is None:
            return False
        path = self.run / "mission46_terrain_request.json"
        if not path.exists():
            return False
        proof = {}
        try:
            intent, read = self._read_json(path.name)
            key = (intent["request_id"], intent["goal_id"])
            if list(key) in self.completed_transitions:
                return False
            goal_id, before, after = TRANSITIONS[intent["phase"]]
            if (
                intent["run_id"] != self.run.name
                or intent["goal_id"] != goal_id
                or intent["from_layer"] != before
                or intent["to_layer"] != after
                or LAYERS[before] != self.current_id
            ):
                raise ValueError("Mission terrain intent does not match original transition order")
            status, status_read = self._read_json("navigation_status.json")
            request, request_read = self._read_json("navigation_request.json")
            rid, goals = self.goal_regions.parse_request(request)
            goal_hash = self.goal_regions.definitions_sha256(goals)
            definitions = [g.definition() for g in goals]
            index = next(i for i, g in enumerate(goals) if g.goal_id == goal_id)
            if (
                rid != intent["request_id"]
                or not rid.startswith(self.run.name + ":")
                or status.get("request_id") != rid
                or status.get("state") != "running"
                or status.get("waypoint_index") != index + 1
                or status.get("navigation_ground_truth_used") is not False
                or status.get("mode") != "teacher"
                or status.get("scope") != self.profile["experiment"]
                or status.get("goals_definition_sha256") != goal_hash
                or intent["goals_definition_sha256"] != goal_hash
                or status.get("goals_definitions") != definitions
                or status.get("frame_id") != "camera_init"
                or len(status.get("region_arrivals", [])) != index + 1
            ):
                raise ValueError(
                    "Terrain selection lacks matching actual SLAM request/connector hold"
                )
            arrival = status["region_arrivals"][index]
            goal = goals[index]
            stamp, begin, dwell, gap = [
                arrival.get(k)
                for k in ("stamp_ns", "start_stamp_ns", "dwell_ns", "max_observation_gap_ns")
            ]
            if (
                arrival != intent["arrival_receipt"]
                or arrival.get("goal_id") != goal_id
                or arrival.get("request_id") != rid
                or arrival.get("goals_definition_sha256") != goal_hash
                or not all(type(v) is int for v in (stamp, begin, dwell, gap))
                or not 0 <= begin <= stamp
                or dwell != stamp - begin
                or dwell < round(goal.dwell_sim_s * 1e9)
                or not 0 < gap <= 200000000
                or arrival.get("protected") is not False
                or arrival.get("reason") != "arrived"
                or arrival.get("region_inside") is not True
                or arrival.get("control_region_inside") is not True
                or not self.goal_regions.contains_control(goal, arrival.get("raw_position"))
            ):
                raise ValueError("Terrain intent has no valid original measured dwell receipt")
            clock = round(float(self.state[0]) * 1e9)
            if clock < stamp:
                if stamp - clock > 50000000:
                    raise ValueError("Terrain arrival exceeds causal future tolerance")
                return True
            rotation = self.observation.quaternion_rotation(self.state[4:8])
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            c, s = np.cos(yaw), np.sin(yaw)
            xy = self.observation.SCAN_GRID_XY @ np.asarray([[c, s], [-s, c]]) + self.state[1:3]
            starts = np.column_stack((xy, np.full(187, self.state[3] + 20.0)))
            a, an = self.maps[LAYERS[before]].raycast(starts)
            b, bn = self.maps[LAYERS[after]].raycast(starts)
            if (
                a.shape != (187,)
                or b.shape != (187,)
                or not np.isfinite(a).all()
                or not np.isfinite(b).all()
                or len(an) != 187
                or len(bn) != 187
                or not all(n.split("/")[0] == "floor_2" for n in list(an) + list(bn))
                or np.max(np.abs(a - b)) > 1e-9
            ):
                raise ValueError("All187 Actor rays must agree on shared floor2; no layer fallback")
            proof = dict(
                intent=intent,
                intent_read=read,
                actual_original_navigation_status=status,
                status_read=status_read,
                request_read=request_read,
                native_clock=clock,
                effective_sim_ns=clock,
                native_state_use="privileged Actor ray equality only",
                ray_starts=starts.tolist(),
                ray_heights_before=a.tolist(),
                ray_heights_after=b.tolist(),
                collision_names_before=an,
                collision_names_after=bn,
                world_sha256=digest(self.world.read_bytes()),
                manifest_sha256=self.manifest_hashes,
                navigation_ground_truth_used=False,
            )
            evidence_file = self.run / (
                "mission46_terrain_evidence_%d.json" % (len(self.completed_transitions) + 1)
            )
            evidence_file.write_text(canonical(proof) + "\n")
            self.current_id = LAYERS[after]
            self.switched = True
            self.completed_transitions.append(list(key))
            self.event = proof
            checks = (
                "actual_slam_arrival",
                "causal_provider_application",
                "frozen_sdf_geometry",
                "privileged_actor_source_declared",
                "shared_landing_187_rays_equal",
            )
            ack = dict(
                intent,
                schema="teacher_mission46_terrain_switch/v1",
                passed=True,
                binding_verified=True,
                checks={k: dict(status="passed", passed=True) for k in checks},
                source_evidence_file=str(evidence_file),
                source_evidence_sha256=digest(evidence_file.read_bytes()),
                effective_sim_ns=clock,
                ray_count=187,
                maximum_ray_difference_m=float(np.max(np.abs(a - b))),
                navigation_ground_truth_used=False,
            )
            tmp = self.run / "mission46_terrain_ack.tmp"
            tmp.write_text(canonical(ack) + "\n")
            tmp.replace(self.run / "mission46_terrain_ack.json")
            self._record(dict(event="mission46_switched", **proof, provider_after=self.current_id))
            return False
        except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration) as error:
            self.failure = type(error).__name__ + ": " + str(error)
            self.event = proof
            self._record(
                dict(event="mission46_switch_refused_latched", failure=self.failure, proof=proof)
            )
            return True
