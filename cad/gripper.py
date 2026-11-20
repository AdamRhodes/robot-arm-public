import numpy as np, manifold3d as m, trimesh
from math import *

# ---------------- house thread ----------------
D=8.0;P=2.0;DEPTH=0.6;CREST=0.4;ROOT=0.4;FIT=0.2
def thread(length,offset=0.0,npts=120):
    rmaj=D/2+offset;rmin=rmaj-DEPTH;a=CREST;b=a+DEPTH;c=b+ROOT
    def f(t):
        t=t%P
        if t<a: return rmaj
        if t<b: return rmaj-(t-a)
        if t<c: return rmin
        return rmin+(t-c)
    th=np.linspace(0,2*np.pi,npts,endpoint=False)
    cs=m.CrossSection([[(f(t*P/(2*np.pi))*np.cos(t),f(t*P/(2*np.pi))*np.sin(t)) for t in th]])
    return m.Manifold.extrude(cs,length,int(length/P*npts/3)+1,360.0*length/P)
def thread_hole(x,y,z_top,depth):
    """threaded hole entering from z_top going down, with entry chamfer"""
    h=thread(depth+1,offset=FIT/2).translate((x,y,z_top-depth))
    ch=m.Manifold.cylinder(1.0,D/2-0.4,D/2+0.6,64).translate((x,y,z_top-1.0+0.001))
    return h+ch

def circ(r,x=0,y=0,n=96): return m.CrossSection.circle(r,n).translate((x,y))
def cyl(r,h,x=0,y=0,z=0,n=64): return m.Manifold.cylinder(h,r,r,n).translate((x,y,z))
def box(x0,x1,y0,y1,z0,z1): return m.Manifold.cube((x1-x0,y1-y0,z1-z0)).translate((x0,y0,z0))
def rotz(man,deg,cx,cy): return man.translate((-cx,-cy,0)).rotate((0,0,deg)).translate((cx,cy,0))

# ---------------- involute gear ----------------
def gear2d(mod,z,pa=20,backlash=0.15,n=10):
    pa=radians(pa); rp=mod*z/2; rb=rp*cos(pa); ra=rp+mod; rf=rp-1.25*mod
    inv=lambda a: tan(a)-a
    half=pi/(2*z)-backlash/2/rp
    r0=max(rb,rf)
    def th(r): return half+inv(pa)-inv(acos(min(1,rb/r)))
    pts=[]
    for i in range(z):
        c=2*pi*i/z
        rs=np.linspace(r0,ra,n)
        pts.append((rf*cos(c-th(r0)),rf*sin(c-th(r0))))
        for r in rs: pts.append((r*cos(c-th(r)),r*sin(c-th(r))))
        for r in rs[::-1]: pts.append((r*cos(c+th(r)),r*sin(c+th(r))))
        pts.append((rf*cos(c+th(r0)),rf*sin(c+th(r0))))
    return m.CrossSection([pts])

# ---------------- parameters ----------------
MOD=1.5; Z=16; RP=MOD*Z/2; RA=RP+MOD          # pitch r 12, tip r 13.5
CD=2*RP+0.2                                    # centre distance with a little play
DRV=(0.0,0.0); IDL=(0.0,CD); MID=CD/2          # driven pivot = servo shaft
PLATE=4.0                                      # base plate, z -4..0 (tabs rest on z=0)
# servo stack above the plate face (ASSUMED - measure before printing)
TAB_T=2.5; CASE_TOP=TAB_T+4.5; HUMP_TOP=CASE_TOP+4.0; ARM_TOP=15.55  # measured
SLOT=1.8; JAW_T=6.0
JAW_Z0=ARM_TOP-SLOT; JAW_Z1=JAW_Z0+JAW_T       # 13.4 .. 19.4
TOP_Z0=JAW_Z1+0.5; TOP_T=3.0; TOP_Z1=TOP_Z0+TOP_T
TIP_X=45.0
FA=(-16.0,MID-8,MID+8)                          # forearm mount holes x, y1, y2
PILLAR=(-17.0,36.0)
ALPHA=35                                        # open angle per jaw (deg)

# servo: long axis along y, shaft at origin, body toward -y
CL=0.4; BW=22.8; BD=12.2; SOFF=BW/2-5.9; TABP=27.8
BCY=-SOFF                                       # body centre y

def servo_env():
    s=box(-BD/2,BD/2,BCY-BW/2,BCY+BW/2,-15.9,CASE_TOP)
    s+=box(-BD/2,BD/2,BCY-16.15,BCY+16.15,0,TAB_T)
    s+=cyl(6.0,HUMP_TOP-CASE_TOP,0,0,CASE_TOP)
    s+=cyl(3.5,ARM_TOP-(HUMP_TOP+0.5),0,0,HUMP_TOP+0.5)
    return s
def horn(ang):
    a=m.CrossSection.batch_hull([circ(2.35,-2.65+2.35+0),circ(2.35,16.6-2.35)])+circ(3.5)
    h=m.Manifold.extrude(a,1.36).translate((0,0,ARM_TOP-1.36))
    h+=cyl(2.2,1.5,0,0,ARM_TOP)                 # horn screw head
    return h.rotate((0,0,ang))

# ---------------- jaws ----------------
th_dir=degrees(atan2(MID-0.5-3.5,TIP_X))       # finger direction, closed pose
def finger2d():
    d=radians(th_dir)
    p1=(10*cos(d),10*sin(d)); tip=(TIP_X,MID-0.5-3.5)
    f=m.CrossSection.batch_hull([circ(5.5,*p1),circ(3.5,*tip)])
    return f
def driven_jaw():
    g=gear2d(MOD,Z).rotate(90)                  # tooth points at the idler
    sec=g+finger2d()+circ(8)
    j=m.Manifold.extrude(sec,JAW_T).translate((0,0,JAW_Z0))
    slot=m.CrossSection.batch_hull([circ(2.6,-2.65),circ(2.6,16.6-2.35)])+circ(3.75)
    slot=slot.rotate(th_dir)
    j-=m.Manifold.extrude(slot,SLOT+0.01).translate((0,0,JAW_Z0-0.01))
    j-=cyl(2.8,JAW_T+2,0,0,JAW_Z0-1)            # horn screw access
    return j
def idler_jaw():
    g=gear2d(MOD,Z).rotate(90)
    body=(finger2d()+circ(8)).mirror((0,1))
    g=g.mirror((0,1)).rotate(180/Z)             # half-tooth phase so teeth interleave
    j=m.Manifold.extrude(body+g,JAW_T).translate((0,CD,JAW_Z0))
    j-=cyl(6.7,JAW_T+2,IDL[0],IDL[1],JAW_Z0-1)  # bore on the post (0.2 clearance, from gear test)
    return j

# ---------------- plates ----------------
def base_plate():
    out=m.CrossSection.batch_hull([circ(8,FA[0],FA[1]),circ(8,FA[0],FA[2]),
        circ(7,0,BCY-13.9-1),circ(9.5,*IDL),circ(7,*PILLAR),
        m.CrossSection.square((BD+6,BW+6)).translate((-(BD+6)/2,BCY-(BW+6)/2))])
    p=m.Manifold.extrude(out,PLATE).translate((0,0,-PLATE))
    p+=cyl(9,JAW_Z0,IDL[0],IDL[1],0)            # shoulder the idler jaw rests on
    p+=cyl(6.5,TOP_Z0-JAW_Z0,IDL[0],IDL[1],JAW_Z0)  # pivot post
    p+=cyl(6.5,TOP_Z0,PILLAR[0],PILLAR[1],0)    # top plate pillar
    # cuts
    p-=box(-(BD+CL)/2,(BD+CL)/2,BCY-(BW+CL)/2,BCY+(BW+CL)/2,-PLATE-1,1)
    for dy in (-TABP/2,TABP/2): p-=cyl(0.9,PLATE+2,0,BCY+dy,-PLATE-1,24)
    # wire notches at both ends of the pocket (wire runs down the servo's end face)
    for y0 in (BCY+(BW+CL)/2-0.1, BCY-(BW+CL)/2-1.6):
        p-=box(-2.25,2.25,y0,y0+1.7,-PLATE-1,1)
    for y in FA[1:]:
        p-=cyl(4.35,PLATE+2,FA[0],y,-PLATE-1)
    p-=thread_hole(IDL[0],IDL[1],TOP_Z0,8)
    p-=thread_hole(PILLAR[0],PILLAR[1],TOP_Z0,8)
    return p
def top_plate():
    out=m.CrossSection.batch_hull([circ(9,*DRV),circ(9.5,*IDL),circ(7,*PILLAR)])
    t=m.Manifold.extrude(out,TOP_T).translate((0,0,TOP_Z0))
    t-=cyl(2.8,TOP_T+2,0,0,TOP_Z0-1)
    for p in (IDL,PILLAR): t-=cyl(4.35,TOP_T+2,p[0],p[1],TOP_Z0-1)
    return t

def tomesh(man):
    mm=man.to_mesh()
    return trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=False)

if __name__=='__main__':
    parts={'base':base_plate(),'top':top_plate()}
    fl=trimesh.load('/mnt/user-data/outputs/forearm.stl')
    fore=m.Manifold(m.Mesh(vert_properties=np.asarray(fl.vertices,np.float32),tri_verts=np.asarray(fl.faces,np.uint32)))
    fore=fore.translate((FA[0]-58,MID,-PLATE-5))
    srv=servo_env()
    for pose,a in (('closed',0),('open',ALPHA)):
        dj=rotz(driven_jaw(),-a,*DRV); ij=rotz(idler_jaw(),a,*IDL); hn=horn(th_dir-a)
        items={'base':parts['base'],'top':parts['top'],'driven':dj,'idler':ij,'servo':srv,'horn':hn,'forearm':fore}
        names=list(items)
        print('==',pose)
        for i in range(len(names)):
            for k in range(i+1,len(names)):
                a1,b1=names[i],names[k]
                if {a1,b1} in ({'driven','horn'},{'servo','horn'}): continue
                v=(items[a1]^items[b1]).volume()
                if v>0.01: print(f'  overlap {a1}-{b1}: {v:.2f} mm3')
        # closed tip gap
    print('finger dir',round(th_dir,2),'CD',CD,'jaw z',JAW_Z0,JAW_Z1,'top',TOP_Z0,TOP_Z1)
