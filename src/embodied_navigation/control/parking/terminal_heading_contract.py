"""由当前 route approach 几何推导终端航向（rad）。

计划路线里仍保留 endpoint support connectors；当 start 与 goal 两个命名连接件
都存在时，它们的人工短 bearing 会被排除在 approach 之外。本模块不读取任何来源
档案、版本、哈希、注册信息或证据文件，只根据传入的路线文档计算航向。开关键为
r85_auto_reference_contract.terminal_heading_from_pct_approach。
"""

import copy, math

FLAG = "terminal_heading_from_pct_approach"


def enabled(profile):
    """开关是否打开；取值必须是 bool，否则 raise（不静默降级）。"""
    config = profile.get("r85_auto_reference_contract", {})
    value = config.get(FLAG, False)
    if type(value) is not bool:
        raise ValueError("Terminal heading option must be boolean")
    return value


def derive(profile, doc):
    """由路线几何推导终端航向，返回 (heading_rad, 说明 dict)。

    doc["ground_points"] 必须是 2..4096 个有限 XYZ 点。若
    doc["endpoint_support_connectors"] 同时含 start 与 goal 角色，则去掉首末点
    （连接件的人工短 bearing）；否则使用全部点。随后从终点沿路线反向累计
    direction_audit_arc_m（默认 0.8 m，只允许 0.2..1.2 m）定位锚点，取
    “锚点→终点”的平面方向 atan2(dy, dx)。

    返回的说明 dict 含 method/window_m/anchor/end，便于证据核对。路线点不足、
    坐标非有限、窗口非法或平面方向退化时 raise ValueError。
    """
    points = doc.get("ground_points")
    if not isinstance(points, list) or not 2 <= len(points) <= 4096:
        raise ValueError("Route point count invalid")
    for p in points:
        if (
            not isinstance(p, list)
            or len(p) != 3
            or not all(type(x) in (int, float) and math.isfinite(x) for x in p)
        ):
            raise ValueError("Route XYZ must be finite")
    connectors = doc.get("endpoint_support_connectors", [])
    named = {x.get("role"): x for x in connectors if isinstance(x, dict)}
    # 首末点是 endpoint support connector 的人工连接段，会给出非行驶方向的短 bearing。
    approach = points[1:-1] if set(named) == {"start", "goal"} else points
    if len(approach) < 2:
        raise ValueError("Route approach is too short")
    # 方向审计弧长（米）：用多长的一段接近几何来求终端航向。
    arc = float(
        profile.get("global_route_reference_contract", {}).get("direction_audit_arc_m", 0.8)
    )
    if not math.isfinite(arc) or not 0.2 <= arc <= 1.2:
        raise ValueError("Terminal approach window invalid")
    # 从终点反向累计弧长，定位距终点 arc 米的锚点；必要时在两点之间线性插值。
    remaining = arc
    for i in range(len(approach) - 2, -1, -1):
        length = math.dist(approach[i], approach[i + 1])
        if length < 1e-6:
            continue
        if remaining <= length:
            fraction = 1.0 - remaining / length
            anchor = [
                approach[i][j] + fraction * (approach[i + 1][j] - approach[i][j]) for j in range(3)
            ]
            break
        remaining -= length
    else:
        raise ValueError("Route approach shorter than direction window")
    dx, dy = approach[-1][0] - anchor[0], approach[-1][1] - anchor[1]
    if math.hypot(dx, dy) < 1e-6:
        raise ValueError("Route approach has no planar direction")
    heading = math.atan2(dy, dx)
    return heading, dict(
        method="current_route_approach", window_m=arc, anchor=anchor, end=approach[-1]
    )


def attach_reference_heading(profile, doc):
    """开关启用时，返回带 task_terminal_heading_rad 的路线文档副本。

    返回的是 deepcopy，调用方原文档不被修改；开关关闭时原样返回同一对象。
    推导失败时由 derive 抛 ValueError。
    """
    if not enabled(profile):
        return doc
    result = copy.deepcopy(doc)
    result["task_terminal_heading_rad"] = derive(profile, doc)[0]
    return result


def validate_reference_heading(profile, doc):
    """开关启用时，要求文档里的终端航向与当前路线推导值一致。

    比对按 2*pi 取模，容差 1e-9 rad；值缺失、非数值或非有限同样 raise。
    开关关闭时立即返回，不做任何检查。
    """
    if not enabled(profile):
        return
    value = doc.get("task_terminal_heading_rad")
    expected = derive(profile, doc)[0]
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or abs(math.atan2(math.sin(value - expected), math.cos(value - expected))) > 1e-9
    ):
        raise ValueError("Terminal heading does not match current route")
