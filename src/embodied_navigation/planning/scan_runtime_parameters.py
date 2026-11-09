"""Bounded native SCAN envelope; R85 adds only the external mode3 protocol."""


def parameters(profile):
    horizon = float(profile["continuous_route_contract"]["local_planning_horizon_m"])
    if horizon != 0.65:
        raise ValueError("Only reviewed bounded planning horizon .65m permitted")
    session = profile.get("r85_session", {})
    if not isinstance(session, dict) or not isinstance(session.get("enabled", False), bool):
        raise ValueError("r85_session.enabled must be an explicit boolean")
    result = {
        "use_sim_time": True,
        "fsm.navi_mode": 3,
        "fsm.planning_horizon": horizon,
        "fsm.thresh_replan": 0.35,
        "fsm.max_replan_fail_count": 50,
        "fsm.measured_max_speed": profile["max_speed_mps"],
        "fsm.measured_velocity_filter_tau": 0.4,
        "grid_map.frame_id": "camera_init",
        "grid_map.sliding_map_frame_id": "teacher_scan_sliding_map",
        "grid_map.sensor_type": "lidar",
        "grid_map.cloud_is_world": True,
        "grid_map.need_extrinsic": False,
        "grid_map.resolution": 0.08,
        "grid_map.body_height": 0.4,
        "grid_map.obstacles_inflation_z_down": 0.12,
        "grid_map.obstacles_inflation_z_up": 0.12,
        "grid_map.double_cylinder_radius": 0.25,
        "grid_map.double_cylinder_offset": 0.18,
        "grid_map.sliding_map_size_x": 10.0,
        "grid_map.sliding_map_size_y": 10.0,
        "grid_map.sliding_map_size_z": 5.0,
        "grid_map.local_update_range_x": 5.0,
        "grid_map.local_update_range_y": 5.0,
        "grid_map.local_update_range_z": 2.5,
        "manager.max_vel": profile["max_speed_mps"],
        "manager.max_acc": 0.15,
        "manager.planning_horizon": horizon,
        "optimization.max_vel": profile["max_speed_mps"],
        "optimization.max_acc": 0.15,
    }
    if session.get("enabled", False):
        # Run/epoch/generation must be supplied by the R85 epoch supervisor.
        # The v3 C++ initializer rejects a missing/unsafe run identity.
        result["fsm.external_reference_protocol"] = True
    from embodied_navigation.common.observation import api as _observer_api

    protocol_observation_enabled = _observer_api("r74_protocol_observation_python").enabled
    if protocol_observation_enabled(profile):
        result["fsm.r74_protocol_observation"] = True
    return result
