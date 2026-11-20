from gripper import *
from render_asm import render
from PIL import Image
scr=trimesh.load('/mnt/user-data/outputs/screwplate_screw_hex.stl')
def screw_at(x,y,zb):
    s=scr.copy(); T=np.eye(4); T[2,2]=-1; T[:3,3]=(x,y,zb+4.5); s.apply_transform(T); s.invert(); return s
fl=trimesh.load('/mnt/user-data/outputs/forearm.stl').copy(); fl.apply_translation((FA[0]-58,MID,-PLATE-5))
srv=servo_env()
C=dict(base=(80,170,150),top=(150,205,190),drv=(235,135,60),idl=(240,165,90),servo=(70,110,200),horn=(245,245,245),fore=(165,165,170),screw=(200,110,70))
def scene(a, explode=0.0):
    e=explode
    P=[(tomesh(base_plate()),C['base']),
       (tomesh(top_plate()).apply_translation((0,0,3*e)),C['top']),
       (tomesh(rotz(driven_jaw(),-a,*DRV)).apply_translation((0,0,1.6*e)),C['drv']),
       (tomesh(rotz(idler_jaw(),a,*IDL)).apply_translation((0,0,1.6*e)),C['idl']),
       (tomesh(srv),C['servo']),(tomesh(horn(th_dir-a)).apply_translation((0,0,0.6*e)),C['horn']),
       (fl.copy().apply_translation((0,0,-1.5*e)),C['fore'])]
    for y in FA[1:]: P.append((screw_at(FA[0],y,0+0.8*e),C['screw']))
    for p in (IDL,PILLAR): P.append((screw_at(p[0],p[1],TOP_Z1+3.8*e),C['screw']))
    return P
v=(-0.55,0.75,-0.62)
a=render(scene(0),v,title='Gripper on forearm, closed')
b=render(scene(30),v,title='Open (30 deg per jaw)')
c=render(scene(0,12),v,title='Exploded: forearm, base plate + servo, jaws, top plate')
W,H=a.size
out=Image.new('RGB',(W*2,H*2),'white'); out.paste(a,(0,0)); out.paste(b,(W,0)); out.paste(c,(W//2,H))
out.save('/mnt/user-data/outputs/gripper_assembly_preview.png')
