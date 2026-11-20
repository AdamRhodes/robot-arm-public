import numpy as np, manifold3d as m, trimesh
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
def circ(r,x=0,y=0,n=96): return m.CrossSection.circle(r,n).translate((x,y))
def cyl(r,h,x=0,y=0,z=0,n=64): return m.Manifold.cylinder(h,r,r,n).translate((x,y,z))

T=5.0
GX=58.0; GY=8.0   # gripper mount holes at (58, +-8)
outline=m.CrossSection.batch_hull([circ(10,1,0),circ(8,GX,GY),circ(8,GX,-GY)])
body=m.Manifold.extrude(outline,T)

# single-arm horn slot on the servo face (z=0), same as upper arm
slot2d=m.CrossSection.batch_hull([circ(2.6,-5+2.35),circ(2.6,16.6-2.35)])+circ(3.75)
cut=m.Manifold.extrude(slot2d,1.3+0.01).translate((0,0,-0.01))
cut+=cyl(2.8,T+2,z=-1)                        # horn screw access
for x in (27,43): cut+=cyl(4.35,T+2,x,z=-1)    # backplate screws
for y in (GY,-GY):                             # threaded gripper holes
    th=thread(T+2,offset=FIT/2).translate((GX,y,-1))
    ch=m.Manifold.cylinder(1.0,D/2+0.6,D/2-0.4,64)
    cut+=th+ch.translate((GX,y,T-1.0+0.001))   # entry chamfer, front face
    cut+=ch.mirror((0,0,1)).translate((GX,y,1.0-0.001))  # chamfer, back face
part=body-cut
mesh=part.to_mesh()
t=trimesh.Trimesh(mesh.vert_properties[:,:3],mesh.tri_verts,process=False)
print(t.bounds,t.is_watertight,round(t.volume),'g PLA ~',round(t.volume*1.24/1000,1))
t.export('/mnt/user-data/outputs/forearm.stl')
