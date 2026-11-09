"""三段物理平台路线的预注册到达区域。

原始点数组仍是规划器权威中心；区域边界只描述机体位置到达，不代表接触验证
或自由空间地图。本模块只做纯数据装配与校验，不建 ROS 节点、不发命令、不写文件。
"""

import copy
import math

from embodied_navigation.navigation.mission.goal_regions import IDENTITY, RigidTransform, parse_goal

# 三条任务路线名；装配、校验与坐标变换都按这个固定集合遍历。
ROUTES = ("exploration", "return_origin", "navigation_f1_f3")


def add_route_regions(scenario, profile="three_platform_body_arrival_v2"):
    """把预注册到达体积挂到三段平台场景上，返回深拷贝。

    只接受两个已审阅 profile：v2 使用内层机体体积并额外生成 ``control_band``，
    v1 保留原始外层体积。规划中心不变——区域只描述机体位置到达，绝不表示实测
    接触。区域数值（0.4 s dwell、0.2 s 最大间隔、平面半径 0.35 m、半高 0.10 m、
    斜面半宽 0.30 m、原点球 0.22 m 等）在本文件硬编码；场景 JSON 的
    ``goal_region_profile`` 必须与这里生成的结果逐字段一致（由
    ``validate_route_regions`` 校验）。
    """
    # profile 名是契约的一部分；未知名字直接拒绝，避免静默换几何。
    if profile not in ("three_platform_body_arrival_v1", "three_platform_body_arrival_v2"):
        raise ValueError("unknown arrival profile")
    inner = profile == "three_platform_body_arrival_v2"
    result = copy.deepcopy(scenario)
    result["goal_region_profile"] = {
        "schema_version": 2,
        "name": profile,
        "dwell_sim_s": 0.4,
        "max_gap_sim_s": 0.2,
        "flat_radius_m": 0.35,
        "flat_height_half_span_m": 0.10,
        "ramp_half_extents_m": [0.35, 0.30, 0.10],
        "origin_radius_m": 0.22,
        "description": "Preregistered body-position volumes; SCAN plans to unchanged centers. Surface IDs are metadata, not measured contacts.",
    }
    if inner:
        result["goal_region_profile"]["control_band"] = {
            "flat_radius_m": 0.25,
            "flat_height_half_span_m": 0.10,
            "ramp_half_extents_m": [0.25, 0.20, 0.07],
            "origin_radius_m": 0.17,
        }
    result["route_goals"] = {}
    # 斜面轴归一化因子：斜率 0.1 -> 1/sqrt(1+0.1²)。
    scale = math.sqrt(1.01)
    for name in ROUTES:
        goals = []
        for index, point in enumerate(result[name]):
            x, y, z = point
            arrival = {
                "type": "disc_prism",
                "radius_m": 0.35,
                "height_half_span_m": 0.10,
                "axes": IDENTITY,
                "dwell_sim_s": 0.4,
            }
            # 平台面先按 z 高度粗分（硬编码容差 0.01 m）：0 / 1.2 / 2.4 m 三层。
            # 斜面点随后由 slope 改写；其余点保持粗分结果。
            surface = "floor_1" if z < 0.01 else "floor_2" if abs(z - 1.2) < 0.01 else "floor_3"
            # 两条斜面缝（y==2 / y==7、2<=x<=14）同时使用有向走廊与其内部区域。
            slope = (
                0.1
                if y == 2 and 2 <= x <= 14 and 0 <= z <= 1.2
                else -0.1
                if y == 7 and 2 <= x <= 14 and 1.2 <= z <= 2.4
                else None
            )
            if slope is not None:
                surface = "ramp_12" if slope > 0 else "ramp_23"
                arrival = {
                    "type": "oriented_box",
                    "dwell_sim_s": 0.4,
                    "axes": [
                        [1 / scale, 0, slope / scale],
                        [0, 1, 0],
                        [-slope / scale, 0, 1 / scale],
                    ],
                    "half_extents_m": [0.35, 0.30, 0.10],
                }
            # 返航最后一点必须回到初始化原点，使用预注册的 .22 m 球。
            if name == "return_origin" and index == len(result[name]) - 1:
                if point != [0, 0, 0]:
                    raise ValueError("return_origin must end at the initialization origin")
                surface = "origin"
                arrival = {"type": "sphere", "radius_m": 0.22, "dwell_sim_s": 0.4}
            # v2 控制带按面类型收窄（m）；与 goal_region_profile.control_band 对应。
            if inner:
                arrival["control_band"] = (
                    {"radius_m": 0.17}
                    if surface == "origin"
                    else {"half_extents_m": [0.25, 0.20, 0.07]}
                    if slope is not None
                    else {"radius_m": 0.25, "height_half_span_m": 0.10}
                )
            # 单区域进度期限 90 s（硬编码，仿真秒），同时写入每个目标。
            raw = {
                "goal_id": f"{name}:{index}",
                "center": point,
                "arrival": arrival,
                "timeout_sim_s": 90.0,
                "route_surface_id": surface,
            }
            goals.append(parse_goal(raw).definition())
        result["route_goals"][name] = goals
    return result


def validate_route_regions(scenario):
    """在任何快照或运动前拒绝歧义/不一致的路线配置。

    没有 ``route_goals`` 的历史纯航点场景仍然支持（直接返回）。否则要求恰好
    三条路线、profile 名与声明一致、目标数与规划中心一致、goal_id 唯一、中心与
    规划中心逐点相等，并且几何与按声明 profile 重新生成的结果完全一致；最后
    校验返航终点是 .22 m 球。任何不符都抛 ``ValueError``。
    """
    if "route_goals" not in scenario:
        return  # 历史纯航点场景仍受支持，不要求区域定义。
    configured = scenario["route_goals"]
    if not isinstance(configured, dict) or set(configured) != set(ROUTES):
        raise ValueError("route_goals must define exactly all three mission routes")
    # 用声明 profile 重新生成期望值，再与场景中携带的副本逐字段比较。
    profile = scenario.get("goal_region_profile", {}).get("name")
    expected = add_route_regions({name: scenario[name] for name in ROUTES}, profile=profile)
    if scenario.get("goal_region_profile") != expected["goal_region_profile"]:
        raise ValueError("arrival profile does not match its declared contract")
    for name in ROUTES:
        raw = configured[name]
        if not isinstance(raw, list) or not raw or len(raw) != len(scenario[name]):
            raise ValueError(name + " goal count differs from planner centers")
        goals = tuple(parse_goal(item) for item in raw)
        if len({g.goal_id for g in goals}) != len(goals):
            raise ValueError(name + " duplicate goal ID")
        for goal, point in zip(goals, scenario[name]):
            if tuple(point) != goal.center:
                raise ValueError(name + " arrival center differs from planner center")
        if [g.definition() for g in goals] != expected["route_goals"][name]:
            raise ValueError(name + " geometry differs from its declared profile")
    final = parse_goal(configured["return_origin"][-1])
    if final.center != (0.0, 0.0, 0.0) or final.kind != "sphere" or final.radius != 0.22:
        raise ValueError("return origin requires the preregistered .22 m sphere")


def transformed_route_goals(scenario, name, origin, heading_alignment):
    """返回该路线在 camera_init 内的已验证目标（元组）。

    先 ``validate_route_regions``，再按场景声明的 waypoint reference 变换：
    ``relative_world_axes`` 用一次测量的 world->camera_init 航向旋转，
    ``relative_initial_body`` 只平移 origin，其他值视为已是绝对坐标（恒等变换）。
    路线不存在时返回空元组。
    """
    validate_route_regions(scenario)
    raw = scenario.get("route_goals", {}).get(name)
    if raw is None:
        return ()
    reference = scenario.get("waypoint_reference", "relative_initial_body")
    if reference == "relative_world_axes":
        from embodied_navigation.navigation.mission.goal_regions import relative_world_transform

        transform = relative_world_transform(
            origin, heading_alignment["yaw_camera_init_from_world"]
        )
    else:
        transform = RigidTransform(
            IDENTITY, origin if reference == "relative_initial_body" else (0.0, 0.0, 0.0)
        )
    return tuple(transform.goal(parse_goal(item)) for item in raw)
