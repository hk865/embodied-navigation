#!/usr/bin/env python3
"""Static finite-face/OBB validation of a PCT polyline, never a runtime permit.

No ROS, PCT, simulator, policy or candidate controller is imported. Only the
candidate arrays, frozen geometry metadata/reference and explicit limits enter
the geometry. Original mission intermediate regions/routes are never read.
"""
from __future__ import annotations
from embodied_navigation.common.config import loads as _project_loads

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np


HERE = Path(__file__).resolve().parent
REFERENCE_SHA = 'f37a0693f6ad463a5ba2edbb8415b035d0dfef115470319dae92885257de4914'
SUPPORTS = ("floor_1", "floor_2", "floor_3", "ramp_12", "ramp_23")
LAYERS = {"lower12": ("floor_1", "ramp_12", "floor_2"),
          "upper23": ("floor_2", "ramp_23", "floor_3")}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source(path):
    path = Path(path).resolve()
    return {"path": str(path), "sha256": sha(path), "bytes": path.stat().st_size}


def limits(path):
    d = _project_loads(Path(path).read_text())
    if d.get("schema") != "pct_static_support_validation_limits/v1":
        raise ValueError("Wrong explicit validation-limits schema")
    required = ("sample_spacing_m", "body_radius_m", "body_offset_m", "body_below_m",
                "body_above_m", "position_uncertainty_m", "yaw_half_range_rad",
                "body_ground_height_m", "nominal_route_body_ground_offset_m",
                "support_height_tolerance_m", "max_support_slope_rad",
                "actor_grid_half_x_m", "actor_grid_half_y_m", "actor_grid_spacing_m",
                "actor_ray_start_above_body_m", "switch_height_equality_m", "numeric_epsilon_m")
    for key in required:
        if isinstance(d.get(key), bool) or not isinstance(d.get(key), (int, float)) or not math.isfinite(d[key]) or d[key] <= 0:
            raise ValueError("Missing/invalid explicit limit: " + key)
    lower_bounds = {"body_radius_m": .25, "body_offset_m": .18, "body_below_m": .12,
                    "body_above_m": .12, "position_uncertainty_m": .08}
    if any(d[k] < v for k, v in lower_bounds.items()):
        raise ValueError("Chassis envelope/uncertainty may not be reduced below authorized C64 values")
    if (d["sample_spacing_m"] > .025 or d["support_height_tolerance_m"] > .10
            or d["max_support_slope_rad"] > .35 or d["switch_height_equality_m"] > 1e-9
            or d["numeric_epsilon_m"] > 1e-10):
        raise ValueError("Sampling/support/equality limits may not be relaxed")
    frozen = {"yaw_half_range_rad": math.pi, "body_ground_height_m": .32,
              "nominal_route_body_ground_offset_m": .3, "actor_grid_half_x_m": .8,
              "actor_grid_half_y_m": .5, "actor_grid_spacing_m": .1,
              "actor_ray_start_above_body_m": 20.}
    if any(not math.isclose(d[k], v, abs_tol=1e-12, rel_tol=0) for k, v in frozen.items()):
        raise ValueError("Authorized yaw, Actor grid, or live/nominal body-height contract changed")
    if d.get("initial_provider") not in LAYERS or not 1 <= d.get("max_report_examples", 0) <= 100:
        raise ValueError("Explicit initial provider and bounded report size required")
    return d


def top_level_speed(path):
    # Read only this top-level scalar, never scenario/46-region point values.
    pattern = re.compile(r'^  "max_speed_mps"\s*:\s*([0-9.eE+-]+)\s*,?\s*$')
    found = []
    with Path(path).open() as stream:
        for number, line in enumerate(stream, 1):
            match = pattern.fullmatch(line.rstrip("\n"))
            if match:
                found.append((float(match.group(1)), number))
                break
    if len(found) != 1 or not math.isfinite(found[0][0]) or found[0][0] <= 0:
        raise ValueError("Cannot read a positive top-level profile max_speed_mps")
    return {**source(path), "field": "max_speed_mps", "value_mps": found[0][0], "line": found[0][1],
            "used_for_time_parameterization": False, "intermediate_mission_points_used": False}


def array(value, base):
    if isinstance(value, str):
        p = (base / value).resolve()
        a = np.load(p, allow_pickle=False)
    else:
        a = value
    a = np.asarray(a, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != 3 or not 2 <= len(a) <= 20000 or not np.isfinite(a).all():
        raise ValueError("Candidate arrays must be finite Nx3, 2<=N<=20000")
    return a


def read_route(args, d):
    inputs, connector_notes = {}, []
    if args.route:
        doc = _project_loads(args.route.read_text())
        if doc.get("coordinate_frame") != "camera_init":
            raise ValueError("Candidate route must explicitly use canonical camera_init")
        ground = array(doc["ground_xyz"], args.route.parent)
        body = array(doc["body_xyz"], args.route.parent)
        inputs["route"] = source(args.route)
        for role in ("ground_xyz", "body_xyz"):
            if isinstance(doc[role], str):
                inputs[role + "_npy"] = source(args.route.parent / doc[role])
        connectors = doc.get("endpoint_connectors_requiring_validation")
        if connectors is not None:
            begin = np.asarray(connectors["start_body_xyz_pair"], dtype=float)
            end = np.asarray(connectors["goal_body_xyz_pair"], dtype=float)
            if (begin.shape != (2, 3) or end.shape != (2, 3) or not np.isfinite(begin).all()
                    or not np.isfinite(end).all() or not np.allclose(begin[1], body[0], atol=1e-12, rtol=0)
                    or not np.allclose(end[0], body[-1], atol=1e-12, rtol=0)):
                raise ValueError("Task/grid connector identity mismatch")
            if not np.array_equal(begin[0], body[0]):
                body = np.vstack((begin[0], body))
                g = begin[0].copy(); g[2] -= d["nominal_route_body_ground_offset_m"]
                ground = np.vstack((g, ground))
                connector_notes.append("Exact supplied task-start→first-grid connector included; ground derived using explicit0.3m offset")
            if not np.array_equal(end[1], body[-1]):
                body = np.vstack((body, end[1]))
                g = end[1].copy(); g[2] -= d["nominal_route_body_ground_offset_m"]
                ground = np.vstack((ground, g))
                connector_notes.append("Last-grid→exact supplied task-goal connector included; ground derived using explicit0.3m offset")
        else:
            connector_notes.append("No task connectors supplied; claim covers arrays only")
    else:
        if not args.ground_npy or not args.body_npy:
            raise ValueError("Supply --route or both --ground-npy and --body-npy")
        ground = array(str(args.ground_npy.resolve()), Path("/"))
        body = array(str(args.body_npy.resolve()), Path("/"))
        inputs = {"ground_npy": source(args.ground_npy), "body_npy": source(args.body_npy)}
        connector_notes.append("NPY-only interface; no separate task connectors claimed")
    if ground.shape != body.shape or not np.allclose(body - ground, [0, 0, d["nominal_route_body_ground_offset_m"]], atol=1e-8, rtol=0):
        raise ValueError("Body/ground arrays must retain the explicit nominal0.3m offset")
    return ground, body, inputs, connector_notes


def make_face(box, axis, sign):
    r, c, h = box["r"], box["c"], box["h"]
    normal = r[:, axis] * sign
    tangential = [k for k in range(3) if k != axis]
    center = c + normal * h[axis]
    corners = np.array([center + u*r[:, tangential[0]]*h[tangential[0]] + v*r[:, tangential[1]]*h[tangential[1]]
                        for u, v in ((-1, -1), (-1, 1), (1, -1), (1, 1))])
    return {"model": box["model"], "box": box, "axis": axis, "sign": sign,
            "tangential": tangential, "n": normal, "center": center,
            "xy_lo": corners[:, :2].min(0), "xy_hi": corners[:, :2].max(0),
            "slope_rad": math.acos(float(np.clip(normal[2], -1, 1)))}


def geometry(path):
    meta = _project_loads(path.read_text())
    if meta.get("schema") != "known_scene43_pct_surface_cloud/v1" or meta.get("coordinate_frame") != "camera_init":
        raise ValueError("Wrong frozen geometry metadata schema/frame")
    p = Path(meta["inputs"]["reference"]["path"])
    ref = _project_loads(p.read_text())
    if len(ref["boxes"]) != 43:
        raise ValueError("Expected all43 static boxes")
    frame = np.asarray(meta["T_navigation_world"], dtype=float)
    if not np.allclose(frame, [[1,0,0,0],[0,1,0,0],[0,0,1,-.3],[0,0,0,1]], atol=1e-12, rtol=0):
        raise ValueError("Reviewed canonical transform changed")
    boxes, uppers, tops = [], [], []
    for value in ref["boxes"]:
        t = np.asarray(value["T_world_box"], dtype=float)
        box = {"model": value["model"], "r": t[:3,:3], "c": t[:3,3],
               "h": np.asarray(value["size_m"], dtype=float)/2,
               "lo": np.asarray(value["world_min"]), "hi": np.asarray(value["world_max"])}
        boxes.append(box)
        for axis in range(3):
            for sign in (-1, 1):
                face = make_face(box, axis, sign)
                if face["n"][2] > 1e-12:
                    uppers.append(face)
                if value["model"] in SUPPORTS and axis == 2 and sign == 1:
                    tops.append(face)
    # All reviewed upward projected faces are XY-axis rectangles; generic SDF
    # rotation would require polygon clipping rather than this specialization.
    for face in uppers:
        r = face["box"]["r"]
        if not (np.allclose(r[1], [0,1,0], atol=1e-12, rtol=0)):
            raise ValueError("Non-reviewed projected-face orientation needs a polygon resolver")
    return boxes, uppers, tops, frame, {"metadata": source(path), "reference": source(p)}


def clip_nonnegative(lo, hi, a, b):
    """Keep a+b*t>=0, closed float64 interval; no physical gap filling."""
    moving = np.abs(b) > 1e-15
    lo = np.where(moving & (b > 0), np.maximum(lo, np.divide(-a, b, out=np.zeros_like(a), where=moving)), lo)
    hi = np.where(moving & (b < 0), np.minimum(hi, np.divide(-a, b, out=np.zeros_like(a), where=moving)), hi)
    invalid = (~moving) & (a < 0)
    return np.where(invalid, 1., lo), np.where(invalid, 0., hi)


def face_intervals(xy0, xy1, face):
    n, c = face["n"], face["center"]
    z0 = c[2] - ((xy0-c[:2]) @ n[:2]) / n[2]
    z1 = c[2] - ((xy1-c[:2]) @ n[:2]) / n[2]
    p0, p1 = np.column_stack((xy0,z0)), np.column_stack((xy1,z1))
    q0 = (p0-face["box"]["c"]) @ face["box"]["r"]
    q1 = (p1-face["box"]["c"]) @ face["box"]["r"]
    lo, hi = np.zeros(len(xy0)), np.ones(len(xy0))
    for axis in face["tangential"]:
        half = face["box"]["h"][axis]
        lo, hi = clip_nonnegative(lo,hi,half-q0[:,axis],q0[:,axis]-q1[:,axis])
        lo, hi = clip_nonnegative(lo,hi,half+q0[:,axis],q1[:,axis]-q0[:,axis])
    return lo, hi, z0, z1


def covers(lo, hi, eps):
    lo, hi = np.asarray(lo), np.asarray(hi)
    if lo.ndim == 1:
        lo, hi = lo[:,None], hi[:,None]
    order = np.argsort(lo,axis=0)
    lo, hi = np.take_along_axis(lo,order,axis=0), np.take_along_axis(hi,order,axis=0)
    end = np.zeros(lo.shape[1]); good = np.ones(lo.shape[1],dtype=bool)
    for a,b in zip(lo,hi):
        valid = a <= b
        good &= ~(valid & (a > end+eps) & (end < 1-eps))
        end = np.where(valid & (a <= end+eps), np.maximum(end,b),end)
    return good & (end >= 1-eps)


def center_support(g0,g1,tops,d):
    lows, highs, names = [], [], []
    for face in tops:
        if face["slope_rad"] > d["max_support_slope_rad"]:
            continue
        lo,hi,z0,z1 = face_intervals(g0[None,:2],g1[None,:2],face)
        diff0, diff1 = z0-g0[2], z1-g1[2]
        tol = d["support_height_tolerance_m"]
        lo,hi = clip_nonnegative(lo,hi,tol-diff0,diff0-diff1)
        lo,hi = clip_nonnegative(lo,hi,tol+diff0,diff1-diff0)
        lows.append(lo);highs.append(hi)
        if lo[0] <= hi[0]: names.append(face["model"])
    return bool(covers(lows,highs,d["numeric_epsilon_m"])[0]),names


def union_rect_covers(qlo,qhi,rectangles,eps):
    """Exact rectangle union partition; never its convex hull or corners only."""
    cuts = [qlo[0],qhi[0]]
    for a,b in rectangles:
        cuts.extend((max(qlo[0],min(qhi[0],a[0])),max(qlo[0],min(qhi[0],b[0]))))
    cuts = sorted(set(cuts))
    probes = cuts + [(a+b)/2 for a,b in zip(cuts[:-1],cuts[1:]) if b>a]
    for x in probes:
        intervals = [(max(qlo[1],a[1]),min(qhi[1],b[1])) for a,b in rectangles if a[0]-eps<=x<=b[0]+eps]
        reach = qlo[1]
        for a,b in sorted(intervals):
            if a>b:continue
            if a>reach+eps:return False
            reach=max(reach,b)
        if reach<qhi[1]-eps:return False
    return True


def filled_support(g0,g1,faces,radius,d):
    qlo=np.minimum(g0[:2],g1[:2])-radius
    qhi=np.maximum(g0[:2],g1[:2])+radius
    rectangles=[]
    for face in faces:
        if face["slope_rad"]>d["max_support_slope_rad"]:continue
        a,b=np.maximum(qlo,face["xy_lo"]),np.minimum(qhi,face["xy_hi"])
        if np.any(a>b):continue
        centers=np.array([g0[:2],g1[:2]])
        predicted=face["center"][2]-(centers-face["center"][:2])@face["n"][:2]/face["n"][2]
        # Keep the exact0.10m residual budget: compare this face's affine
        # continuation at BOTH centerline endpoints to the ground line. The
        # whole subsegment has the same affine residual bound. Across-footprint
        # physical height variation comes from this actual plane, not from
        # enlarging the permitted residual tolerance.
        if np.max(np.abs(predicted-np.array([g0[2],g1[2]])))>d["support_height_tolerance_m"]:continue
        rectangles.append((a,b))
    return union_rect_covers(qlo,qhi,rectangles,d["numeric_epsilon_m"])


def aabb_obb_disjoint(lo,hi,box,eps):
    center=(lo+hi)/2; half=(hi-lo)/2; delta=box["c"]-center
    axes=[*np.eye(3),*box["r"].T]
    axes += [np.cross(a,b) for a in np.eye(3)for b in box["r"].T]
    for axis in axes:
        norm=np.linalg.norm(axis)
        if norm<1e-12:continue
        axis=axis/norm
        radius=np.abs(axis)@half+np.abs(axis@box["r"])@box["h"]
        if abs(axis@delta)>radius+eps:return True
    return False


def collision_sweep(b0,b1,boxes,d):
    xy=d["body_radius_m"]+d["body_offset_m"]+d["position_uncertainty_m"]
    lower=np.array([xy,xy,d["body_below_m"]+d["position_uncertainty_m"]])
    upper=np.array([xy,xy,d["body_above_m"]+d["position_uncertainty_m"]])
    lo=np.minimum(b0,b1)-lower;hi=np.maximum(b0,b1)+upper
    hits=[b["model"]for b in boxes if not aabb_obb_disjoint(lo,hi,b,d["numeric_epsilon_m"])]
    return not hits,hits,lo,hi


def ray_tables(b0,b1,yaw,faces,d,translation=(0.,0.)):
    gx,gy=np.meshgrid(np.linspace(-.8,.8,17),np.linspace(-.5,.5,11),indexing="xy")
    grid=np.column_stack((gx.ravel(),gy.ravel()))
    r=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    offsets=np.vstack((grid@r.T,np.zeros((1,2))))+translation
    x0,x1=b0[:2]+offsets,b1[:2]+offsets
    rows=[face_intervals(x0,x1,f)for f in faces]
    return tuple(np.array([x[i]for x in rows])for i in range(4))


def provider_segment(g0,g1,b0,b1,yaw,faces,d):
    lo,hi,z0,z1=ray_tables(b0,b1,yaw,faces,d)
    eps=d["numeric_epsilon_m"]; valid=lo<=hi
    complete=covers(lo[:,:187],hi[:,:187],eps)
    # Every potential top face is <= body+tol: therefore the actual highest
    # first-hit surface is too. This never treats a lower hit as the winner.
    h0=z0-b0[2]-d["support_height_tolerance_m"]
    h1=z1-b1[2]-d["support_height_tolerance_m"]
    worst=np.maximum(h0+(h1-h0)*lo,h0+(h1-h0)*hi)
    overhead=np.any(valid[:,:187]&(worst[:,:187]>eps))
    # The center's maximum first-hit envelope must remain within ground band.
    delta0=z0[:,-1]-g0[2]; delta1=z1[:,-1]-g1[2]
    upper=np.maximum(delta0+(delta1-delta0)*lo[:,-1],delta0+(delta1-delta0)*hi[:,-1])
    center_upper=not np.any(valid[:,-1]&(upper>d["support_height_tolerance_m"]+eps))
    l,h=clip_nonnegative(lo[:,-1].copy(),hi[:,-1].copy(),delta0+d["support_height_tolerance_m"],delta1-delta0)
    center_lower=bool(covers(l,h,eps)[0])
    return bool(complete.all()and not overhead and center_upper and center_lower),{
        "continuous_187_ray_coverage":bool(complete.all()),"uncovered_ray_count":int((~complete).sum()),
        "potential_highest_hit_above_body_band":bool(overhead),
        "center_highest_hit_matches_support_band":bool(center_upper and center_lower)}


def point_hits(body,yaw,faces,d,translation=(0.,0.)):
    lo,hi,z0,_=ray_tables(body,body,yaw,faces,d,translation)
    ok=lo[:,:187]<=hi[:,:187]
    values=np.where(ok,z0[:,:187],-np.inf)
    winners=np.argmax(values,axis=0)
    heights=values[winners,np.arange(187)]
    names=[faces[i]["model"]if np.isfinite(h)else None for i,h in zip(winners,heights)]
    return heights,names


def shared_landing(body,yaw,layer_faces,d):
    a,an=point_hits(body,yaw,layer_faces["lower12"],d)
    b,bn=point_hits(body,yaw,layer_faces["upper23"],d)
    passed=np.isfinite(a).all()and np.isfinite(b).all()and all(n=="floor_2"for n in an+bn)
    difference=float(np.max(np.abs(a-b)))if passed else None
    return bool(passed and difference<=d["switch_height_equality_m"]),difference


def shared_landing_full(body,layer_faces,layer_boxes,d):
    """Both maps must hit the SAME floor2 for every permitted yaw/XY error."""
    radius=math.hypot(.8,.5)+d["position_uncertainty_m"]
    lo,hi=body[:2]-radius,body[:2]+radius
    deck=next(b for b in layer_boxes["lower12"]if b["model"]=="floor_2")
    if np.any(lo<deck["lo"][:2])or np.any(hi>deck["hi"][:2]):return False
    height=deck["hi"][2]
    for faces in layer_faces.values():
        for face in faces:
            if face["model"]=="floor_2":continue
            a,b=np.maximum(lo,face["xy_lo"]),np.minimum(hi,face["xy_hi"])
            if np.any(a>b):continue
            xy=np.array([[x,y]for x in(a[0],b[0])for y in(a[1],b[1])])
            z=face["center"][2]-(xy-face["center"][:2])@face["n"][:2]/face["n"][2]
            # Reject ties too: native argmin order may choose another model.
            if z.max()>=height-d["numeric_epsilon_m"]:return False
    return True


def full_yaw_domain(b0,b1,faces,boxes,d):
    radius=math.hypot(.8,.5)+d["position_uncertainty_m"]
    lo=np.minimum(b0[:2],b1[:2])-radius;hi=np.maximum(b0[:2],b1[:2])+radius
    # Filled axis-aligned square encloses every187-ray yaw and every declared
    # XY position error. Rectangle union check preserves domain holes.
    rects=[(b["lo"][:2],b["hi"][:2])for b in boxes]
    coverage=union_rect_covers(lo,hi,rects,d["numeric_epsilon_m"])
    overhead=False
    for face in faces:
        a,b=np.maximum(lo,face["xy_lo"]),np.minimum(hi,face["xy_hi"])
        if np.any(a>b):continue
        xy=np.array([[x,y]for x in(a[0],b[0])for y in(a[1],b[1])])
        z=face["center"][2]-(xy-face["center"][:2])@face["n"][:2]/face["n"][2]
        if z.max()>min(b0[2],b1[2])-d["position_uncertainty_m"]+d["support_height_tolerance_m"]+d["numeric_epsilon_m"]:
            overhead=True
    return bool(coverage and not overhead),{"enclosing_xy_radius_m":radius,
        "continuous_filled_outer_bound_coverage":bool(coverage),"outer_bound_overhead":bool(overhead)}


def counterexample(body,faces,d):
    theta=math.atan2(.8,.5)
    yaws=sorted(set([0.,math.pi/2,math.pi,-math.pi/2,theta,-theta,math.pi-theta,-math.pi+theta]))
    u=d["position_uncertainty_m"]
    for yaw in yaws:
        for dx,dy in ((0,0),(u,0),(-u,0),(0,u),(0,-u),(u,u),(u,-u),(-u,u),(-u,-u)):
            heights,names=point_hits(body,yaw,faces,d,(dx,dy))
            bad=~np.isfinite(heights)
            if bad.any():
                index=int(np.flatnonzero(bad)[0]);grid=np.array([-.8+.1*(index%17),-.5+.1*(index//17)])
                r=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
                xy=body[:2]+r@grid+np.array([dx,dy])
                return {"yaw_rad":yaw,"position_error_xy_m":[dx,dy],"missing_ray_count":int(bad.sum()),
                        "first_missing_ray_index":index,"first_missing_world_ray_xy":xy.tolist(),
                        "kind":"actual187_ray_counterexample_within_declared_bounds"}
    return None


def run(args,receipt):
    d=limits(args.config); ground,body,route_sources,connector_notes=read_route(args,d)
    boxes,uppers,tops,frame,geometry_sources=geometry(args.geometry_metadata)
    receipt.update(inputs={**route_sources,**geometry_sources,"limits":source(args.config),"validator":source(__file__)},
                   limits=d,speed_profile=top_level_speed(args.profile),endpoint_connectors=connector_notes)
    # Work in frozen world geometry; no native/truth robot measurement is used.
    ground=(ground-frame[:3,3])@frame[:3,:3];body=(body-frame[:3,3])@frame[:3,:3]
    if min(body[:,2])+d["actor_ray_start_above_body_m"]<=max(b["hi"][2]for b in boxes):
        raise ValueError("Ray-start-above-all-geometry prerequisite failed")
    layers={k:[f for f in uppers if f["model"]in names]for k,names in LAYERS.items()}
    layer_boxes={k:[b for b in boxes if b["model"]in names]for k,names in LAYERS.items()}
    counts=Counter();provider_counts={k:Counter()for k in LAYERS};examples=[];first_per_reason={};edges=[];max_spacing=0.;duplicates=0
    def example(value):
        if len(examples)<d["max_report_examples"]:examples.append(value)
    for original,(g0,g1,b0,b1) in enumerate(zip(ground[:-1],ground[1:],body[:-1],body[1:])):
        length=max(float(np.linalg.norm(g1-g0)),float(np.linalg.norm(b1-b0)))
        if length<1e-12:duplicates+=1;continue
        horizontal=float(np.linalg.norm(g1[:2]-g0[:2]))
        if horizontal<1e-12:
            counts["vertical_nonzero_connector_or_edge"]+=1
            example({"original_edge":original,"reason":"nonzero vertical-only segment; tangent yaw unobservable"})
            continue
        slope=math.atan2(abs(float(g1[2]-g0[2])),horizontal)
        yaw=math.atan2(float(g1[1]-g0[1]),float(g1[0]-g0[0]))
        pieces=max(1,math.ceil(length/d["sample_spacing_m"]))
        if len(edges)+pieces>200000:raise ValueError("Validation sample budget exceeded")
        for j in range(pieces):
            t0,t1=j/pieces,(j+1)/pieces
            ga,gb=g0+t0*(g1-g0),g0+t1*(g1-g0)
            ba,bb=b0+t0*(b1-b0),b0+t1*(b1-b0)
            spacing=float(np.linalg.norm(bb-ba));max_spacing=max(max_spacing,spacing)
            supported,support_names=center_support(ga,gb,tops,d)
            live_a=ba.copy();live_b=bb.copy();live_a[2]-=d["body_ground_height_m"];live_b[2]-=d["body_ground_height_m"]
            live_supported,_=center_support(live_a,live_b,tops,d)
            clear,hits,_,_=collision_sweep(ba,bb,boxes,d)
            xy=d["body_radius_m"]+d["body_offset_m"]+d["position_uncertainty_m"]
            footprint=filled_support(ga,gb,tops,xy,d)
            basic=supported and live_supported and clear and footprint and slope<=d["max_support_slope_rad"]
            if not basic:
                reasons={"ground_center_not_supported":not supported,"live032_height_not_supported":not live_supported,
                         "swept_chassis_outer_bound_intersects_static_box":not clear,
                         "filled_chassis_footprint_support_not_certified":not footprint,
                         "route_grade_exceeds_limit":slope>d["max_support_slope_rad"]}
                counts.update(k for k,v in reasons.items()if v)
                detail={"original_edge":original,"fraction":[t0,t1],"reasons":reasons,"collision_boxes":hits,
                        "ground_world_xyz":ga.tolist(),"body_world_xyz":ba.tolist(),"slope_rad":slope}
                example(detail)
                for reason,bad in reasons.items():
                    if bad:first_per_reason.setdefault(reason,detail)
            nominal,ray_only,full,diagnostics={}, {}, {}, {}
            for layer in LAYERS:
                ok,diag=provider_segment(ga,gb,ba,bb,yaw,layers[layer],d)
                layer_tops=[f for f in tops if f["model"]in LAYERS[layer]]
                layer_support=filled_support(ga,gb,layer_tops,xy,d)
                ray_only[layer]=bool(ok)
                nominal[layer]=bool(ok and layer_support)
                provider_counts[layer].update({"continuous187_coverage_failures":int(not diag["continuous_187_ray_coverage"]),
                    "center_highest_support_band_failures":int(not diag["center_highest_hit_matches_support_band"]),
                    "potential_overhead_failures":int(diag["potential_highest_hit_above_body_band"]),
                    "filled_layer_support_failures":int(not layer_support),"compatible_subsegments":int(nominal[layer])})
                robust,robust_diag=full_yaw_domain(ba,bb,layers[layer],layer_boxes[layer],d)
                full[layer]=bool(nominal[layer]and robust)
                diagnostics[layer]={"nominal":diag,"filled_layer_support":layer_support,"full_yaw":robust_diag}
            switch,difference=shared_landing(ba,yaw,layers,d)
            switch_full=shared_landing_full(ba,layers,layer_boxes,d)
            edges.append({"original_edge":original,"fraction":[t0,t1],"world_body_start":ba.tolist(),
                          "world_body_end":bb.tolist(),"nominal_yaw_rad":yaw,"basic":bool(basic),
                          "nominal":nominal,"nominal_ray_domain":ray_only,"full":full,"shared_landing_start":switch,
                          "shared_landing_full_yaw_start":switch_full,
                          "shared_landing_max_difference_m":difference})
            if not any(nominal.values()):
                counts["no_compatible_nominal_provider"]+=1
                detail={"original_edge":original,"fraction":[t0,t1],"reason":"no compatible nominal provider",
                        "world_body_xyz":ba.tolist(),"nominal_yaw_rad":yaw,"details":diagnostics}
                example(detail);first_per_reason.setdefault("no_compatible_nominal_provider",detail)
    if not edges:raise ValueError("No nondegenerate candidate segments to validate")
    def classify(mode):
        states={d["initial_provider"]:(0,[])}
        for index,e in enumerate(edges):
            next_states={}
            for layer in LAYERS:
                if not e[mode][layer]:continue
                for previous,(cost,history) in states.items():
                    switch_key="shared_landing_full_yaw_start"if mode=="full"else"shared_landing_start"
                    if previous!=layer and not e[switch_key]:continue
                    candidate=(cost+int(previous!=layer),history+([{"edge_index":index,"original_edge":e["original_edge"],
                        "from_provider":previous,"to_provider":layer,"world_body_xyz":e["world_body_start"],
                        "nominal_yaw_rad":e["nominal_yaw_rad"],"shared_floor2_187_difference_m":e["shared_landing_max_difference_m"],
                        "runtime_ack_observed":False,"actual_slam_dwell_intent_and_causal_ack_required":True}]if previous!=layer else[]))
                    if layer not in next_states or candidate[0]<next_states[layer][0]:next_states[layer]=candidate
            if not next_states:return {"compatible_provider_sequence_exists":False,"first_unclassified_edge":index,
                                        "first_unclassified_original_edge":e["original_edge"],"first_unclassified_fraction":e["fraction"],
                                        "first_unclassified_world_body_xyz":e["world_body_start"],
                                        "compatible_final_providers":[],"pending_ack_landings":[]}
            states=next_states
        last=min(states,key=lambda k:states[k][0])
        return {"compatible_provider_sequence_exists":True,"compatible_final_providers":list(states),
                "minimum_switch_count":states[last][0],"pending_ack_landings":states[last][1]}
    nominal=classify("nominal");ray_only=classify("nominal_ray_domain");full=classify("full")
    landing_runs=[]
    for index,e in enumerate(edges):
        if not e["shared_landing_start"]:continue
        if landing_runs and landing_runs[-1]["last_edge_index"]==index-1:
            landing_runs[-1].update(last_edge_index=index,last_world_body_xyz=e["world_body_start"],
                full_yaw_all_points_certified=landing_runs[-1]["full_yaw_all_points_certified"]and e["shared_landing_full_yaw_start"])
        else:
            landing_runs.append({"first_edge_index":index,"last_edge_index":index,
                "first_world_body_xyz":e["world_body_start"],"last_world_body_xyz":e["world_body_start"],
                "nominal_187_both_maps_hit_floor2":True,"maximum_hit_difference_m":e["shared_landing_max_difference_m"],
                "full_yaw_all_points_certified":e["shared_landing_full_yaw_start"],
                "actual_slam_dwell_intent_and_causal_ack_observed":False})
    basic_ok=not any(counts[k]for k in ("vertical_nonzero_connector_or_edge","ground_center_not_supported","live032_height_not_supported",
                  "swept_chassis_outer_bound_intersects_static_box","filled_chassis_footprint_support_not_certified","route_grade_exceeds_limit"))
    witnesses=[]
    for i,e in enumerate(edges):
        if len(witnesses)>=d["max_report_examples"]:break
        for layer in LAYERS:
            if e["nominal"][layer]and not e["full"][layer]:
                witness=counterexample(np.asarray(e["world_body_start"]),layers[layer],d)
                if witness:witnesses.append({"edge_index":i,"original_edge":e["original_edge"],"fraction":e["fraction"],
                    "world_body_xyz":e["world_body_start"],"provider":layer,**witness})
    status=("STATIC_GEOMETRY_REJECTED"if not basic_ok or not nominal["compatible_provider_sequence_exists"]else
            "NOMINAL_ONLY_FULL_YAW_NOT_CERTIFIED"if not full["compatible_provider_sequence_exists"]else
            "STATIC_GEOMETRY_SEGMENTED_PENDING_TERRAIN_ACK"if full.get("minimum_switch_count",0)>0 else
            "STATIC_POLYLINE_GEOMETRY_ONLY")
    receipt.update(status=status,ready_for_control=False,navigation_PASS=False,
        original_body_point_count=len(body),checked_subsegments=len(edges),duplicate_zero_edges=duplicates,
        maximum_3d_sample_spacing_m=max_spacing,basic_geometry_conditions_passed=bool(basic_ok),
        sample_spacing_numeric_tolerance_m=d["numeric_epsilon_m"],
        maximum_true_support_face_slope_rad=max(f["slope_rad"]for f in tops),
        raw_polyline_grade_failure_semantics="Conservative grade of supplied ground polyline, including derived task connectors; this is distinct from true support-plane slope. Endpoint measurement height residual can trigger it; no endpoint or tolerance was altered.",
        failure_counts=dict(counts),first_failure_per_reason=first_per_reason,bounded_examples=examples,
        provider_predicate_failure_counts={k:dict(v)for k,v in provider_counts.items()},
        nominal_187_provider_domain={**ray_only,"filled_chassis_support_and_collision_included":False,
            "scope":"Continuous187 coverage, highest-center support band and no overhead at nominal fixed segment yaw only; not actual heading/foot load"},
        shared_floor2_geometric_landing_candidates=landing_runs,
        nominal_tangent_yaw={**nominal,"actual_heading_bound_verified":False,
                            "status":"CONDITIONAL_ON_NOMINAL_YAW_ONLY"if nominal["compatible_provider_sequence_exists"]else"FAILED"},
        conservative_full_yaw={**full,"yaw_half_range_rad":math.pi,"position_uncertainty_m":d["position_uncertainty_m"],
            "actor_all_yaw_corner_radius_m":math.hypot(.8,.5),"enclosing_radius_with_uncertainty_m":math.hypot(.8,.5)+d["position_uncertainty_m"],
            "switch_guard":"Independent full-yaw+uncertainty outer square must hit common floor2 only in BOTH maps; nominal guard never reused",
            "counterexamples":witnesses,"status":"PROVED_PROVIDER_DOMAIN_OUTER_BOUND_ONLY"if full["compatible_provider_sequence_exists"]else
            "FAILED_WITH_COUNTEREXAMPLE"if witnesses else"NOT_CERTIFIED_CONSERVATIVE_BOUND"},
        body_height_contract={"candidate_nominal_m":.3,"live_guard_m":.32,"difference_m":.02,
                              "silently_adjusted":False,"both_support_height_checks_performed":True},
        continuous_evidence={"center_support":"Union of exact finite-z+ face line intervals and explicit height bands",
            "chassis_collision":"Full-yaw C64 swept AABB outer bound vs every frozen OBB using SAT; overlap rejects certification, may be conservative",
            "filled_chassis_support":"Rectangle-union coverage of whole swept XY outer bound, five eligible finite top faces and declared height/slope band",
            "filled_support_height_rule":"Each eligible actual plane's affine centerline residual stays within unchanged0.10m at both endpoints; no slope×radius addition to tolerance",
            "nominal187":"Exact upper-facet parameter-interval union per ray along each fixed-yaw linear subsegment, highest-hit center/overhead bounds; endcaps not extrapolated as slope",
            "full_yaw187":"Filled XY outer square encloses all187 yaws plus position uncertainty; domain union and upper-facet height bounds",
            "numeric_arithmetic":"float64 with explicit1e-10 interval tolerance; engineering evidence, not exact-arithmetic formal proof",
            "discrete_limit":"<=.025m samples alone are not continuity proof. Certificates apply only to the provided piecewise-linear geometry and declared outer bounds."},
        limitations=["No candidate BSpline/SCAN curve or tracking-error certificate; verify every future curve independently.",
            "No Actor per-foot contact/load, friction, gait, real-robot or stair guarantee.",
            "Envelope is authorized C64 chassis model; parts/roll/pitch outside that envelope are not certified.",
            "No dynamic-obstacle timing, live map freshness, control/latency or actual yaw evidence.",
            "Nominal yaw can jump at polyline vertices; actual rotation/reachable-yaw envelopes remain unverified. Full-yaw result is retained separately.",
            "A multi-provider classification only identifies static segments and candidate shared-floor2 landings. It does not authorize a provider switch or supply runtime ACK.",
            "Any future restricted-heading adoption must bind original0.2rad drive/handoff gates, actualSLAM-yaw187allhit and future command reachable-yaw evidence; no bound was narrowed here."])
    if max_spacing>d["sample_spacing_m"]+d["numeric_epsilon_m"]:
        raise ValueError("Actual subdivision spacing exceeded the explicit limit")
    for item in list(receipt["inputs"].values())+[receipt["speed_profile"]]:
        if sha(item["path"])!=item["sha256"]:raise ValueError("Input/source changed during support validation")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    choice=parser.add_mutually_exclusive_group(required=True)
    choice.add_argument("--route",type=Path,help="route_candidate.json with inline or relative-NPY body_xyz/ground_xyz")
    choice.add_argument("--body-npy",type=Path)
    parser.add_argument("--ground-npy",type=Path)
    parser.add_argument("--geometry-metadata",type=Path,default=HERE/"data/known_scene43_camera_init_h005/metadata.json")
    parser.add_argument("--config",type=Path,default=HERE/"support_validation_config.json")
    parser.add_argument("--profile",type=Path,required=True,help="Explicit navigation speed profile; no historical run default")
    parser.add_argument("--output",type=Path,required=True,help="New JSON receipt; never overwrite an existing result")
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError("Refusing to replace an existing support receipt")
    receipt={"schema":"pct_static_polyline_support_receipt/v1","created_at_utc":datetime.now(timezone.utc).isoformat(),
             "status":"STARTED","ready_for_control":False,"navigation_PASS":False,
             "scope":"One PCT polyline plus its explicit endpoint connectors; no original intermediate mission route read",
             "control_ros_pct_or_simulation_started":False}
    code=0
    try:
        run(args,receipt)
    except Exception as error:
        receipt.update(status="VALIDATION_ERROR",error=type(error).__name__+":"+str(error));code=2
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(receipt,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"status":receipt["status"],"receipt":str(args.output.resolve()),"ready_for_control":False},indent=2))
    raise SystemExit(code)


if __name__=="__main__":
    main()
