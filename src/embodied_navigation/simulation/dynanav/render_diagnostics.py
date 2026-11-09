"""Read existing USD/Fabric render state without refreshing or editing the stage.

Importing this module does not import Isaac or start an application. Receipts are
observations of scene state, not proof that a mesh reached the renderer. A camera
RenderProduct's CameraParams annotator provides the separate rendered view matrix.
"""

from __future__ import annotations

DEFAULT_PATHS = (
    "/Root",
    "/Root/SM_StairsMain_2",
    "/Root/SM_StairsMain_2/SM_StairsMain/Section0",
    "/Root/SM_StairsCarpet2",
    "/Root/SM_StairsCarpet2/SM_StairsCarpet",
    "/Root/SM_Floor_6m2",
    "/Root/SM_Floor_6m2/SM_Floor_6m/SM_Floor_6m",
    "/World/Go2",
    "/World/Go2/Geometry/base",
    "/World/Go2/Geometry/base/DiagnosticCamera",
    "/World/Go2/Geometry/base/PIDOverviewCamera",
)


def _serial(value):
    """Return JSON-compatible values without discarding matrix row order."""
    if value is None or isinstance(value, (bool, str, int, float)):
        return value
    if isinstance(value, dict):
        return {str(k): _serial(v) for k, v in value.items()}
    if hasattr(value, "item"):
        try:
            return _serial(value.item())
        except ValueError:
            pass
    try:
        return [_serial(x) for x in value]
    except TypeError:
        return str(value)


def _unsupported(exc):
    return {"status": "unsupported", "reason": f"{type(exc).__name__}: {exc}"}


def capture_stage_receipt(stage, paths=DEFAULT_PATHS):
    """Read USD and existing Fabric matrices for an already initialized stage.

    Call after the capture's completed render. No update_world_xforms, transform
    setters, material edits, layer saves or application updates are performed.
    Each backend/error is recorded independently; USD is never used as a silent
    substitute for unavailable Fabric state. Matrices use Gf row-vector order.
    """
    import hashlib
    import math

    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetRootLayer()
    session = stage.GetSessionLayer()
    session_text = session.ExportToString() if session else ""
    report = {
        "schema": "dynanav-stage-receipt/v1",
        "method": "existing USD and Fabric hierarchy; no refresh or scene edits",
        "matrix_convention": "Gf row vectors; translation in row 3",
        "root_layer": {"identifier": root.identifier, "dirty": bool(root.dirty)},
        "edit_target_layer": stage.GetEditTarget().GetLayer().identifier,
        "muted_layers": list(stage.GetMutedLayers()),
        "session_layer": {
            "identifier": session.identifier if session else None,
            "bytes": len(session_text.encode()),
            "sha256": hashlib.sha256(session_text.encode()).hexdigest(),
        },
        "fabric": {"status": "unsupported", "reason": "not initialized"},
        "paths": [],
    }
    fabric_stage = hierarchy = None
    try:
        import carb.settings
        import isaacsim.core.experimental.utils.stage as stage_utils
        import usdrt

        fsd = carb.settings.get_settings().get("/app/useFabricSceneDelegate")
        report["fabric"]["FSD_enabled"] = fsd
        if fsd is not True:
            report["fabric"]["reason"] = "Fabric Scene Delegate setting is not true"
        else:
            fabric_stage = stage_utils.get_current_stage(backend="fabric")
            hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
                fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId()
            )
            report["fabric"] = {"status": "ok", "FSD_enabled": fsd}
    except Exception as exc:  # noqa: BLE001 - optional SDK diagnostics report unsupported APIs without hiding capture failures.
        report["fabric"].update(_unsupported(exc))

    for path in paths:
        path = str(path)
        item = {"path": path}
        prim = stage.GetPrimAtPath(path)
        item["usd_exists"] = bool(prim)
        if not prim:
            item["usd"] = {"status": "unsupported", "reason": "USD prim absent"}
        else:
            try:
                state = {
                    "status": "ok",
                    "type": prim.GetTypeName(),
                    "active": prim.IsActive(),
                    "loaded": prim.IsLoaded(),
                    "instance": prim.IsInstance(),
                    "instance_proxy": prim.IsInstanceProxy(),
                    "variants": {
                        n: prim.GetVariantSet(n).GetVariantSelection()
                        for n in prim.GetVariantSets().GetNames()
                    },
                }
                imageable = UsdGeom.Imageable(prim)
                if imageable:
                    state["computed_visibility"] = str(imageable.ComputeVisibility())
                    state["computed_purpose"] = str(imageable.ComputePurpose())
                if prim.IsA(UsdGeom.Xformable):
                    state["world_matrix"] = _serial(
                        UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                            Usd.TimeCode.Default()
                        )
                    )
                visual = {}
                for name in (
                    "visibility",
                    "purpose",
                    "doubleSided",
                    "orientation",
                    "primvars:displayColor",
                    "primvars:displayOpacity",
                ):
                    attr = prim.GetAttribute(name)
                    if attr:
                        visual[name] = {
                            "value": _serial(attr.Get()),
                            "authored": attr.HasAuthoredValueOpinion(),
                            "sources": [
                                {"layer": p.layer.identifier, "path": str(p.path)}
                                for p in attr.GetPropertyStack()
                            ],
                        }
                state["visual_attributes"] = visual
                if prim.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(prim)
                    state["mesh_sizes"] = {
                        "points": len(mesh.GetPointsAttr().Get() or []),
                        "faces": len(mesh.GetFaceVertexCountsAttr().Get() or []),
                    }
                    material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[
                        0
                    ]
                    state["bound_material_path"] = (
                        str(material.GetPath()) if material else None
                    )
                    material_inputs = []
                    if material:
                        for shader_prim in Usd.PrimRange(
                            material.GetPrim(), Usd.TraverseInstanceProxies()
                        ):
                            for attr in shader_prim.GetAttributes():
                                name = attr.GetName()
                                if name.startswith("inputs:") and any(
                                    part in name.lower()
                                    for part in (
                                        "opacity",
                                        "cutout",
                                        "alpha",
                                        "transmission",
                                        "thin_walled",
                                    )
                                ):
                                    material_inputs.append(
                                        {
                                            "path": str(attr.GetPath()),
                                            "value": _serial(attr.Get()),
                                            "connections": [
                                                str(x) for x in attr.GetConnections()
                                            ],
                                        }
                                    )
                    state["material_opacity_inputs"] = material_inputs
                item["usd"] = state
            except Exception as exc:  # noqa: BLE001 - optional SDK diagnostics report unsupported APIs without hiding capture failures.
                item["usd"] = _unsupported(exc)
        if hierarchy is None:
            item["fabric"] = {
                "status": "unsupported",
                "reason": report["fabric"].get("reason"),
            }
        else:
            try:
                fp = fabric_stage.GetPrimAtPath(path)
                matrix = _serial(hierarchy.get_world_xform(usdrt.Sdf.Path(path)))
                if (
                    len(matrix) != 4
                    or any(len(row) != 4 for row in matrix)
                    or not all(math.isfinite(x) for row in matrix for x in row)
                    or abs(matrix[3][3] - 1.0) > 1e-6
                ):
                    raise ValueError(
                        "Fabric returned no valid finite homogeneous 4x4 matrix"
                    )
                state = {
                    "status": "ok" if fp else "unproved",
                    "prim_exists": bool(fp),
                    "world_matrix": matrix,
                }
                if not fp:
                    state["reason"] = (
                        "Hierarchy returned a matrix, but Fabric prim is absent; not verified"
                    )
                else:
                    visual = {}
                    for name in (
                        "visibility",
                        "purpose",
                        "doubleSided",
                        "orientation",
                        "primvars:displayColor",
                        "primvars:displayOpacity",
                    ):
                        attr = fp.GetAttribute(name)
                        if attr:
                            visual[name] = _serial(attr.Get())
                    state["visual_attributes"] = visual
                usd_matrix = item.get("usd", {}).get("world_matrix")
                if usd_matrix:
                    state["minus_usd_matrix_max_abs"] = max(
                        abs(matrix[i][j] - usd_matrix[i][j])
                        for i in range(4)
                        for j in range(4)
                    )
                item["fabric"] = state
            except Exception as exc:  # noqa: BLE001 - optional SDK diagnostics report unsupported APIs without hiding capture failures.
                item["fabric"] = _unsupported(exc)
        report["paths"].append(item)
    return report
