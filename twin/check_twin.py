import json, numpy as np, trimesh
from render_asm import render
from PIL import Image
meta=json.load(open('twin/parts.json'))
def Rz(d):
    a=np.radians(d); T=np.eye(4); T[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]; return T
def Tr(v): T=np.eye(4); T[:3,3]=v; return T
Rx_m90=np.eye(4); Rx_m90[:3,:3]=[[1,0,0],[0,0,1],[0,-1,0]]
def frames(yaw,sh,el,gr):
    W=np.eye(4)
    T=W@Tr((0,0,meta['T_Z']))@Rz(yaw)
    U=T@Tr(meta['U_MOUNT'])@Rx_m90@Rz(sh-90)
    F=U@Tr(meta['F_MOUNT'])@Rz(el)
    G=F@Tr(meta['G_OFF'])
    JD=G@Rz(-gr); JI=G@Tr((meta['IDL'][0],meta['IDL'][1],0))@Rz(gr)
    return dict(W=W,T=T,U=U,F=F,G=G,JD=JD,JI=JI)
col=dict(print=(80,170,150),print2=(150,205,190),servo=(70,110,200),screw=(200,110,70),horn=(240,240,240),jaw=(235,135,60),jaw2=(240,165,90))
def scene(*pose):
    fr=frames(*pose); P=[]
    for n,info in meta['parts'].items():
        t=trimesh.load(f'twin/parts/{n}.stl'); t.apply_transform(fr[info['frame']]); P.append((t,col[info['role']]))
    tip=fr['G']@np.r_[meta['TIP'],1]
    return P,tip
ims=[]
for pose,title in [((0,0,90,0),'home: upright, elbow 90'),((30,45,60,30),'reaching, gripper open'),((0,70,80,0),'reaching low')]:
    P,tip=scene(*pose); print(title,'tip',tip[:3].round(1))
    ims.append(render(P,(-0.5,0.85,-0.35),size=(620,620),title=title))
out=Image.new('RGB',(620*3,620),'white')
for i,im in enumerate(ims): out.paste(im,(620*i,0))
out.save('/tmp/claude-0/twin_check.png') if False else out.save('/home/claude/arm/twin_check.png')
