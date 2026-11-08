"""Terminal heading calculated from the current route's approach geometry.

Endpoint support connectors remain in the planned route; their short artificial
bearing is excluded when a named goal connector is present. No source archive,
version, hash, registration, or evidence file is consulted.
"""
import copy,math

FLAG='terminal_heading_from_pct_approach'

def enabled(profile):
    config=profile.get('r85_auto_reference_contract',{})
    value=config.get(FLAG,False)
    if type(value)is not bool:raise ValueError('Terminal heading option must be boolean')
    return value

def derive(profile,doc):
    points=doc.get('ground_points')
    if not isinstance(points,list)or not 2<=len(points)<=4096:raise ValueError('Route point count invalid')
    for p in points:
        if not isinstance(p,list)or len(p)!=3 or not all(type(x)in(int,float)and math.isfinite(x)for x in p):raise ValueError('Route XYZ must be finite')
    connectors=doc.get('endpoint_support_connectors',[])
    named={x.get('role'):x for x in connectors if isinstance(x,dict)}
    approach=points[1:-1]if set(named)=={'start','goal'}else points
    if len(approach)<2:raise ValueError('Route approach is too short')
    arc=float(profile.get('global_route_reference_contract',{}).get('direction_audit_arc_m',.8))
    if not math.isfinite(arc)or not .2<=arc<=1.2:raise ValueError('Terminal approach window invalid')
    remaining=arc
    for i in range(len(approach)-2,-1,-1):
        length=math.dist(approach[i],approach[i+1])
        if length<1e-6:continue
        if remaining<=length:
            fraction=1.-remaining/length;anchor=[approach[i][j]+fraction*(approach[i+1][j]-approach[i][j])for j in range(3)];break
        remaining-=length
    else:raise ValueError('Route approach shorter than direction window')
    dx,dy=approach[-1][0]-anchor[0],approach[-1][1]-anchor[1]
    if math.hypot(dx,dy)<1e-6:raise ValueError('Route approach has no planar direction')
    heading=math.atan2(dy,dx)
    return heading,dict(method='current_route_approach',window_m=arc,anchor=anchor,end=approach[-1])

def attach_reference_heading(profile,doc):
    if not enabled(profile):return doc
    result=copy.deepcopy(doc);result['task_terminal_heading_rad']=derive(profile,doc)[0];return result

def validate_reference_heading(profile,doc):
    if not enabled(profile):return
    value=doc.get('task_terminal_heading_rad');expected=derive(profile,doc)[0]
    if type(value)not in(int,float)or not math.isfinite(value)or abs(math.atan2(math.sin(value-expected),math.cos(value-expected)))>1e-9:raise ValueError('Terminal heading does not match current route')
