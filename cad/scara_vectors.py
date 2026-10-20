"""2D vector import and adaptive cubic Bezier sampling for SCARA CAD.

All imported geometry is stored as local XY strokes in millimetres. No file
references are used at playback/export time. SVG external resources/scripts
are never loaded. Unsupported drawing objects are reported, not fabricated.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

MAX_BYTES = 8_000_000
MAX_POINTS = 40000
MAX_PATHS = 2000


def _dist_line(p, a, b):
    length = math.dist(a, b)
    if length < 1e-12:
        return math.dist(p, a)
    return abs((b[0]-a[0])*(a[1]-p[1])-(a[0]-p[0])*(b[1]-a[1]))/length


def cubic_points(p0, p1, p2, p3, tolerance=0.10, max_depth=14):
    """Adaptive subdivision with conservative control-point/chord deviation."""
    if not 0.005 <= tolerance <= 5:
        raise ValueError('Tolerance musi byt 0.005..5 mm')
    pts = [tuple(map(float, p0))]

    def split(a,b,c,d,depth):
        if max(_dist_line(b,a,d),_dist_line(c,a,d)) <= tolerance:
            pts.append(tuple(d)); return
        if depth >= max_depth:
            raise ValueError('BEZIER: nutna jemnejsi segmentace, limit rekurze')
        ab=((a[0]+b[0])/2,(a[1]+b[1])/2)
        bc=((b[0]+c[0])/2,(b[1]+c[1])/2)
        cd=((c[0]+d[0])/2,(c[1]+d[1])/2)
        abc=((ab[0]+bc[0])/2,(ab[1]+bc[1])/2)
        bcd=((bc[0]+cd[0])/2,(bc[1]+cd[1])/2)
        m=((abc[0]+bcd[0])/2,(abc[1]+bcd[1])/2)
        split(a,ab,abc,m,depth+1); split(m,bcd,cd,d,depth+1)
        if len(pts) > MAX_POINTS:
            raise ValueError('BEZIER: prilis mnoho G1 segmentu')
    split(*[tuple(map(float,p)) for p in (p0,p1,p2,p3)],0)
    return pts


def transform_paths(paths, x, y, angle_deg=0, scale=1):
    """Rigid XY transform of local-mm paths; local positive Y points up."""
    if not (math.isfinite(angle_deg) and math.isfinite(scale) and 0.001 <= scale <= 1000):
        raise ValueError('Neplatne meritko nebo natoceni')
    ca,sa=math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    return [[(x+scale*(ca*px-sa*py),y+scale*(sa*px+ca*py)) for px,py in path]
            for path in paths]


def _clean_paths(paths):
    cleaned=[]
    count=0
    for path in paths:
        pts=[]
        for p in path:
            x,y=map(float,p)
            if not (math.isfinite(x) and math.isfinite(y)) or max(abs(x),abs(y))>1e7:
                raise ValueError('Import: neplatne nebo extremni souradnice')
            if not pts or math.dist(pts[-1],(x,y))>1e-8:
                pts.append((x,y))
        if len(pts)>=2:
            cleaned.append(pts); count+=len(pts)
        if len(cleaned)>MAX_PATHS or count>MAX_POINTS:
            raise ValueError('Import prilis rozsahly (max 2000 kontur / 40000 bodu)')
    if not cleaned:
        raise ValueError('Soubor neobsahuje podporovane geometricke kontury')
    return cleaned


def _matrix_mult(a,b):
    return (a[0]*b[0]+a[2]*b[1],a[1]*b[0]+a[3]*b[1],
            a[0]*b[2]+a[2]*b[3],a[1]*b[2]+a[3]*b[3],
            a[0]*b[4]+a[2]*b[5]+a[4],a[1]*b[4]+a[3]*b[5]+a[5])


def _map_pt(m,p):
    return (m[0]*p[0]+m[2]*p[1]+m[4],m[1]*p[0]+m[3]*p[1]+m[5])


def _svg_transform(spec):
    m=(1.,0.,0.,1.,0.,0.)
    for kind,rest in re.findall(r'([A-Za-z]+)\s*\(([^)]*)\)',spec or ''):
        values=[float(v) for v in re.findall(r'[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?',rest)]
        k=kind.lower()
        if k=='translate' and len(values) in (1,2):
            v=(1,0,0,1,values[0],values[1] if len(values)==2 else 0)
        elif k=='scale' and len(values) in (1,2):
            v=(values[0],0,0,values[1] if len(values)==2 else values[0],0,0)
        elif k=='rotate' and len(values) in (1,3):
            c,s=math.cos(math.radians(values[0])),math.sin(math.radians(values[0]))
            v=(c,s,-s,c,0,0)
            if len(values)==3:
                cx,cy=values[1:];v=_matrix_mult(_matrix_mult((1,0,0,1,cx,cy),v),(1,0,0,1,-cx,-cy))
        elif k=='matrix' and len(values)==6: v=tuple(values)
        elif k=='skewx' and len(values)==1: v=(1,0,math.tan(math.radians(values[0])),1,0,0)
        elif k=='skewy' and len(values)==1: v=(1,math.tan(math.radians(values[0])),0,1,0,0)
        else: raise ValueError(f'SVG: nepodporovana transformace {kind}')
        m=_matrix_mult(m,v)
    return m


NUM=r'[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?'
PATH_TOKEN=re.compile(r'[a-zA-Z]|'+NUM)


def _arc_svg(p0,p1,rx,ry,rot,large,sweep,tolerance):
    rx,ry=abs(rx),abs(ry)
    if rx<1e-9 or ry<1e-9: return [p1]
    phi=math.radians(rot); cp,sp=math.cos(phi),math.sin(phi)
    x1p=cp*(p0[0]-p1[0])/2+sp*(p0[1]-p1[1])/2
    y1p=-sp*(p0[0]-p1[0])/2+cp*(p0[1]-p1[1])/2
    lam=(x1p/rx)**2+(y1p/ry)**2
    if lam>1: rx*=math.sqrt(lam);ry*=math.sqrt(lam)
    sq=max(0,(rx*rx*ry*ry-rx*rx*y1p*y1p-ry*ry*x1p*x1p)/(rx*rx*y1p*y1p+ry*ry*x1p*x1p+1e-25))
    f=(-1 if large==sweep else 1)*math.sqrt(sq)
    cxp=f*rx*y1p/ry;cyp=-f*ry*x1p/rx
    cx=cp*cxp-sp*cyp+(p0[0]+p1[0])/2
    cy=sp*cxp+cp*cyp+(p0[1]+p1[1])/2
    def ang(u,v):return math.atan2(u[0]*v[1]-u[1]*v[0],u[0]*v[0]+u[1]*v[1])
    u=((x1p-cxp)/rx,(y1p-cyp)/ry)
    v=((-x1p-cxp)/rx,(-y1p-cyp)/ry)
    a0=math.atan2(u[1],u[0]); delta=ang(u,v)
    if not sweep and delta>0: delta-=2*math.pi
    elif sweep and delta<0: delta+=2*math.pi
    rmax=max(rx,ry)
    theta=2*math.acos(max(-1,min(1,1-tolerance/max(rmax,tolerance))))
    n=max(1,math.ceil(abs(delta)/max(theta,0.02)))
    if n>MAX_POINTS:raise ValueError('SVG: prilis detailni oblouk')
    res=[]
    for i in range(1,n+1):
        a=a0+delta*i/n
        res.append((cx+cp*rx*math.cos(a)-sp*ry*math.sin(a),cy+sp*rx*math.cos(a)+cp*ry*math.sin(a)))
    res[-1]=p1
    return res


def _svg_path(d, tolerance):
    toks=PATH_TOKEN.findall(d);i=0;cmd=None;pos=(0.,0.);origin=pos;cur=[];out=[]
    prior_ctrl=None;prior_kind=None
    def read(n):
        nonlocal i
        if i+n>len(toks) or any(re.fullmatch('[A-Za-z]',v) for v in toks[i:i+n]):
            raise ValueError('SVG path: neuplne parametry')
        vals=[float(v) for v in toks[i:i+n]];i+=n;return vals
    def vec(v,rel):return (v[0]+pos[0],v[1]+pos[1]) if rel else (v[0],v[1])
    while i<len(toks):
        if toks[i].isalpha():
            cmd=toks[i];i+=1
        if cmd is None:raise ValueError('SVG path: chybi prikaz')
        k=cmd.upper();rel=cmd.islower()
        if k=='Z':
            if cur:
                cur.append(origin);out.append(cur);cur=[]
            pos=origin;prior_ctrl=None;prior_kind='Z';cmd=None;continue
        if k=='M':
            q=vec(read(2),rel)
            if cur:out.append(cur)
            cur=[q];pos=origin=q;cmd='l' if rel else 'L';prior_ctrl=None;prior_kind='M';continue
        if not cur: cur=[pos];origin=pos
        if k=='L':q=vec(read(2),rel);cur.append(q);pos=q;prior_ctrl=None
        elif k=='H':
            v=read(1)[0];q=(pos[0]+v if rel else v,pos[1]);cur.append(q);pos=q;prior_ctrl=None
        elif k=='V':
            v=read(1)[0];q=(pos[0],pos[1]+v if rel else v);cur.append(q);pos=q;prior_ctrl=None
        elif k in ('C','S','Q','T'):
            if k=='C':
                vals=read(6);a=vec(vals[:2],rel);b=vec(vals[2:4],rel);q=vec(vals[4:6],rel);ctrl=b
            elif k=='S':
                vals=read(4);a=(2*pos[0]-prior_ctrl[0],2*pos[1]-prior_ctrl[1]) if prior_kind in ('C','S') and prior_ctrl else pos
                b=vec(vals[:2],rel);q=vec(vals[2:],rel);ctrl=b
            elif k=='Q':
                vals=read(4);qc=vec(vals[:2],rel);q=vec(vals[2:],rel)
                a=(pos[0]+2*(qc[0]-pos[0])/3,pos[1]+2*(qc[1]-pos[1])/3)
                b=(q[0]+2*(qc[0]-q[0])/3,q[1]+2*(qc[1]-q[1])/3);ctrl=qc
            else:
                vals=read(2);qc=(2*pos[0]-prior_ctrl[0],2*pos[1]-prior_ctrl[1]) if prior_kind in ('Q','T') and prior_ctrl else pos
                q=vec(vals,rel)
                a=(pos[0]+2*(qc[0]-pos[0])/3,pos[1]+2*(qc[1]-pos[1])/3)
                b=(q[0]+2*(qc[0]-q[0])/3,q[1]+2*(qc[1]-q[1])/3);ctrl=qc
            cur.extend(cubic_points(pos,a,b,q,tolerance)[1:]);pos=q;prior_ctrl=ctrl
        elif k=='A':
            rx,ry,rot,large,sweep,x,y=read(7)
            q=vec((x,y),rel)
            cur.extend(_arc_svg(pos,q,rx,ry,rot,round(large),round(sweep),tolerance));pos=q;prior_ctrl=None
        else:raise ValueError(f'SVG path: nepodporovany prikaz {cmd}')
        prior_kind=k
        if len(cur)>MAX_POINTS:raise ValueError('SVG path: prilis mnoho bodu')
    if cur:out.append(cur)
    return out


def _length_mm(s):
    m=re.fullmatch(r'\s*('+NUM+r')\s*(mm|cm|in|pt|pc|px)?\s*',str(s),re.I)
    if not m:raise ValueError('SVG: nepodporovana jednotka delky '+str(s))
    return float(m[1])*{'mm':1,'cm':10,'in':25.4,'pt':25.4/72,'pc':25.4/6,'px':25.4/96}[str(m[2] or 'px').lower()]


def import_svg(filename,tolerance=0.10):
    if Path(filename).stat().st_size>MAX_BYTES:raise ValueError('SVG presahuje 8 MB')
    root=ET.parse(filename).getroot()
    if root.tag.split('}')[-1]!='svg':raise ValueError('Soubor neni SVG')
    box=[float(v) for v in re.findall(NUM,root.get('viewBox',''))]
    if len(box)==4:
        vb_x,vb_y,vb_w,vb_h=box
        wmm=_length_mm(root.get('width',f'{vb_w}px'))
        hmm=_length_mm(root.get('height',f'{vb_h}px'))
        sx,sy=wmm/vb_w,hmm/vb_h
    else:
        sx=sy=25.4/96;vb_x=vb_y=0
    units=(sx,0,0,sy,-vb_x*sx,-vb_y*sy)
    paths=[];skipped={}
    def walk(el,m):
        tag=el.tag.split('}')[-1].lower()
        if tag in ('defs','symbol','clipPath','mask','pattern','marker'):
            return
        m=_matrix_mult(m,_svg_transform(el.get('transform','')))
        def nums(s):return [float(v) for v in re.findall(NUM,s or '')]
        local=[]
        if tag=='path':local=_svg_path(el.get('d',''),tolerance/max(sx,sy))
        elif tag=='line':local=[[(float(el.get('x1',0)),float(el.get('y1',0))),
                                 (float(el.get('x2',0)),float(el.get('y2',0)))]]
        elif tag in ('polyline','polygon'):
            v=nums(el.get('points',''));local=[list(zip(v[::2],v[1::2]))]
            if tag=='polygon' and local[0]:local[0].append(local[0][0])
        elif tag=='rect':
            x=float(el.get('x',0));y=float(el.get('y',0));w=float(el.get('width',0));h=float(el.get('height',0))
            local=[[(x,y),(x+w,y),(x+w,y+h),(x,y+h),(x,y)]]
        elif tag in ('circle','ellipse'):
            cx=float(el.get('cx',0));cy=float(el.get('cy',0))
            rx=float(el.get('r',el.get('rx',0)));ry=float(el.get('r',el.get('ry',0)))
            n=max(12,min(MAX_POINTS,math.ceil(2*math.pi*max(rx,ry)/max(tolerance/max(sx,sy),0.1))))
            local=[[(cx+rx*math.cos(2*math.pi*i/n),cy+ry*math.sin(2*math.pi*i/n)) for i in range(n+1)]]
        elif tag in ('text','image','use','foreignobject'):
            skipped[tag]=skipped.get(tag,0)+1
        for p in local:
            paths.append([_map_pt(units,_map_pt(m,xy)) for xy in p])
        for child in el:walk(child,m)
    walk(root,(1,0,0,1,0,0))
    # CAD uses mathematical XY with Y upwards. Translate min bounding point to local 0,0.
    paths=[[(x,-y) for x,y in path] for path in _clean_paths(paths)]
    minx=min(x for p in paths for x,_ in p);miny=min(y for p in paths for _,y in p)
    paths=_clean_paths([[(x-minx,y-miny) for x,y in p] for p in paths])
    return {'paths':paths,'skipped':skipped,'units':'mm','source':Path(filename).name}


def import_dxf(filename,tolerance=0.10):
    if Path(filename).stat().st_size>MAX_BYTES:raise ValueError('DXF presahuje 8 MB')
    try:
        import ezdxf
        from ezdxf.path import make_path
        from ezdxf.disassemble import recursive_decompose
    except ImportError as exc:
        raise ValueError('DXF vyzaduje knihovnu ezdxf: py -m pip install ezdxf') from exc
    doc=ezdxf.readfile(str(filename))
    units=int(doc.header.get('$INSUNITS',4))
    factor={0:1.,1:25.4,2:304.8,3:1609344.,4:1.,5:10.,6:1000.,7:1e6,8:25.4/1e6,9:25.4/1000,10:914.4,14:100.,15:10000.}.get(units)
    if factor is None:raise ValueError(f'DXF: nepodporovane $INSUNITS={units}')
    paths=[];skipped={}
    for e in recursive_decompose(doc.modelspace()):
        kind=e.dxftype()
        if kind in ('LINE','ARC','CIRCLE','ELLIPSE','SPLINE','LWPOLYLINE','POLYLINE'):
            try:
                p=make_path(e)
                pts=[(float(v.x)*factor,float(v.y)*factor) for v in p.flattening(distance=max(tolerance/factor,1e-6),segments=8)]
                if pts:paths.append(pts)
            except Exception as exc:
                raise ValueError(f'DXF {kind}: nelze nacist geometrii: {exc}') from exc
        elif kind not in ('POINT',):
            skipped[kind]=skipped.get(kind,0)+1
    paths=_clean_paths(paths)
    minx=min(x for p in paths for x,_ in p);miny=min(y for p in paths for _,y in p)
    paths=_clean_paths([[(x-minx,y-miny) for x,y in p] for p in paths])
    return {'paths':paths,'skipped':skipped,'units':'mm','source':Path(filename).name}


def import_vectors(filename,tolerance=0.10):
    suffix=Path(filename).suffix.lower()
    if suffix=='.svg':return import_svg(filename,tolerance)
    if suffix=='.dxf':return import_dxf(filename,tolerance)
    raise ValueError('Import podporuje pouze soubory .svg a .dxf')
