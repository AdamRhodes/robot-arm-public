import numpy as np, trimesh
from PIL import Image, ImageDraw, ImageFont
from trimesh.ray.ray_pyembree import RayMeshIntersector

def render(parts, view_dir, up=(0,0,1), size=(900,650), pad=6, title=None, light=None):
    """parts: list of (trimesh, rgb). Orthographic ray-cast render."""
    meshes=[p[0] for p in parts]; cols=np.array([p[1] for p in parts],float)
    allm=trimesh.util.concatenate(meshes)
    owner=np.concatenate([np.full(len(mm.faces),i) for i,mm in enumerate(meshes)])
    d=np.array(view_dir,float); d/=np.linalg.norm(d)
    u=np.array(up,float); r=np.cross(d,u); r/=np.linalg.norm(r); u=np.cross(r,d)
    V=allm.vertices; pr=V@r; pu=V@u
    W,H=size
    span=max((pr.max()-pr.min())/(W-2*pad*4),(pu.max()-pu.min())/(H-2*pad*4))
    cr=(pr.max()+pr.min())/2; cu=(pu.max()+pu.min())/2
    xs=(np.arange(W)-W/2)*span+cr; ys=(H/2-np.arange(H))*span+cu
    X,Y=np.meshgrid(xs,ys)
    c=allm.bounds.mean(0); back=-d*(np.linalg.norm(allm.extents)*2)
    O=c+back+(X.ravel()[:,None]*r)+(Y.ravel()[:,None]*u)-((c@r)*r)-((c@u)*u)
    D=np.tile(d,(len(O),1))
    ri=RayMeshIntersector(allm)
    tri=ri.intersects_first(O,D)
    img=np.full((H*W,3),255.0)
    hit=tri>=0
    n=allm.face_normals[tri[hit]]
    n=np.where((n@d)[:,None]>0,-n,n)
    L=np.array(light if light is not None else (-d+u*0.8+r*0.4)); L/=np.linalg.norm(L)
    shade=0.35+0.65*np.clip(n@L,0,1)
    img[hit]=cols[owner[tri[hit]]]*shade[:,None]
    # simple outline from part id / depth edges
    pid=np.full(H*W,-1); pid[hit]=owner[tri[hit]]; pid=pid.reshape(H,W)
    edge=np.zeros((H,W),bool)
    edge[:-1]|=pid[:-1]!=pid[1:]; edge[:,:-1]|=pid[:,:-1]!=pid[:,1:]
    img=img.reshape(H,W,3); img[edge]*=0.35
    im=Image.fromarray(np.clip(img,0,255).astype(np.uint8))
    if title:
        dr=ImageDraw.Draw(im)
        f=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',20)
        dr.text((14,10),title,fill=(20,20,20),font=f)
    return im
