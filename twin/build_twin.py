"""Export every arm part in its own joint frame for the 3D twin page.
Frames (all mm, z up):
  W  world: desk at z=0, base axis at x=y=0
  T  turntable: origin on base axis at disc underside, rotates about z (base yaw)
  U  upper arm: origin on shoulder shaft at arm's servo face, x along arm, rotates about z
  F  forearm: origin on elbow shaft at forearm's servo face, rotates about z
  G  gripper: fixed in F at (74,-12.1,9); jaws pivot about z
"""
import sys, os, json
sys.path.insert(0, '/home/claude/arm')
from gripper import *
import gripper_side as GS
import trimesh, numpy as np

OUT = '/home/claude/arm/twin/parts'
os.makedirs(OUT, exist_ok=True)
SRC = '/mnt/user-data/outputs/'
HORN_TOP = ARM_TOP  # 15.55 measured: tab underside to top of horn arm
SLOT_D = 1.3        # horn slot depth in the arm parts

def load(name, faces=None):
    t = trimesh.load(SRC + name + '.stl')
    if faces and len(t.faces) > faces:
        t = t.simplify_quadric_decimation(face_count=faces)
    return t

def M(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None: T[:3, :3] = np.array(R, float)
    T[:3, 3] = t
    return T

FLIP_X = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]   # 180 deg about x

def save(name, mesh, T=None):
    m_ = mesh.copy()
    if T is not None: m_.apply_transform(T)
    m_.export(f'{OUT}/{name}.stl')
    return m_

def man2tri(man): return tomesh(man)

def simple_screw(head_r=6.5, head_h=4.5, shank=9.0):
    s = cyl(head_r, head_h, n=40) + cyl(3.9, shank, 0, 0, -shank, 32)
    s -= m.Manifold.extrude(m.CrossSection.circle(5.25 / np.sqrt(3), 6), 3.2).translate((0, 0, head_h - 3.2 + 0.01))
    return man2tri(s)   # bearing face at z=0, head up, shank down

def single_horn():
    a = m.CrossSection.batch_hull([circ(2.35, -2.65 + 2.35), circ(2.35, 16.6 - 2.35)]) + circ(3.5)
    h = m.Manifold.extrude(a, 1.36).translate((0, 0, SLOT_D - 1.36))
    h += cyl(3.5, 2.4, 0, 0, SLOT_D - 1.36 - 2.4)
    return man2tri(h)

def double_horn():
    a = m.CrossSection.batch_hull([circ(2.35, -(16.6 - 2.35)), circ(2.35, 16.6 - 2.35)]) + circ(3.5)
    h = m.Manifold.extrude(a, 1.36).translate((0, 0, SLOT_D - 1.36))
    h += cyl(3.5, 2.4, 0, 0, SLOT_D - 1.36 - 2.4)
    return man2tri(h)

def servo_body():
    """servo in its own frame: shaft on z axis, tab underside at z=0, body toward -y (no horn)"""
    s = box(-BD/2, BD/2, BCY - BW/2, BCY + BW/2, -15.9, CASE_TOP)
    s += box(-BD/2, BD/2, BCY - 16.15, BCY + 16.15, 0, TAB_T)
    s += cyl(6.0, HUMP_TOP - CASE_TOP, 0, 0, CASE_TOP)
    s += cyl(2.4, HORN_TOP - 1.36 - 0.5 - HUMP_TOP, 0, 0, HUMP_TOP)   # output spline
    return man2tri(s)

parts = {}   # name -> frame, colour role
def add(name, frame, role, mesh, T=None):
    save(name, mesh, T); parts[name] = dict(frame=frame, role=role)

# ---- world: lid + base + base servo
add('lid', 'W', 'print', load('arm_base_lid', 8000))
add('base', 'W', 'print', load('arm_base'), M(FLIP_X, (0, 0, 22.4)))
srv = servo_body()
add('servo_base', 'W', 'servo', srv, M([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], (0, 0, 22.4)))   # body toward -x
scr = simple_screw(shank=19)
for i, (sx, sy) in enumerate([(39, 39), (-39, 39), (39, -39), (-39, -39)]):
    add(f'screw_base{i}', 'W', 'screw', scr, M(None, (sx, sy, 22.4)))

# ---- T: turntable (disc underside at horn top - slot depth)
T_Z = 22.4 + HORN_TOP - SLOT_D
add('turntable', 'T', 'print', load('shoulder_turntable'))
add('turntable_backplate', 'T', 'print2', load('turntable_backplate', 6000), M(FLIP_X))
add('horn_base', 'T', 'horn', double_horn())
s9 = simple_screw()
for i, sx in enumerate((29, -29)):
    add(f'screw_tt{i}', 'T', 'screw', s9, M(None, (sx, 0, 5)))
# shoulder servo: tabs on wall front face y=-13, shaft +y at z=26.5, body downward
add('servo_shoulder', 'T', 'servo', srv, M([[-1, 0, 0], [0, 0, 1], [0, 1, 0]], (0, -13, 26.5)))
U_MOUNT = (0, -13 + HORN_TOP - SLOT_D, 26.5)

# ---- U: upper arm
add('upper_arm', 'U', 'print', load('upper_arm'))
add('upper_backplate', 'U', 'print2', load('upper_arm_backplate', 6000), M(FLIP_X))
add('horn_shoulder', 'U', 'horn', single_horn())
add('screw_ua', 'U', 'screw', s9, M(None, (27, 0, 5)))
add('servo_elbow', 'U', 'servo', srv, M([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], (70, 0, 5)))  # body toward shoulder
F_MOUNT = (70, 0, 5 + HORN_TOP - SLOT_D)

# ---- F: forearm + gripper (G = F + offset)
add('forearm', 'F', 'print', load('forearm', 6000))
add('forearm_backplate', 'F', 'print2', load('upper_arm_backplate', 6000), M(FLIP_X))
add('horn_elbow', 'F', 'horn', single_horn())
for i, x in enumerate((27, 43)):
    add(f'screw_fa{i}', 'F', 'screw', s9, M(None, (x, 0, 5)))
G_OFF = tuple(GS.T)                         # gripper frame origin in forearm frame (rotated -90 about x)
add('gripper_base', 'G', 'print', man2tri(GS.base_plate_side()).simplify_quadric_decimation(face_count=9000))
add('gripper_top', 'G', 'print2', man2tri(top_plate()))
add('servo_gripper', 'G', 'servo', srv)
for i, y in enumerate((GS.HOLE_Y, -GS.HOLE_Y)):
    add(f'screw_gm{i}', 'F', 'screw', s9, M(None, (58, y, GS.FOOT_Z[1])))
for i, p in enumerate((IDL, PILLAR)):
    add(f'screw_gt{i}', 'G', 'screw', s9, M(None, (p[0], p[1], TOP_Z1)))
add('jaw_driven', 'JD', 'jaw', man2tri(driven_jaw()))
add('horn_gripper', 'JD', 'horn', man2tri(horn(th_dir)))
add('jaw_idler', 'JI', 'jaw2', man2tri(idler_jaw().translate((-IDL[0], -IDL[1], 0))))

TIP_G = np.array([TIP_X + 2.0, MID, (JAW_Z0 + JAW_Z1) / 2])
TIP_F = (GS.R @ TIP_G + GS.T).tolist()
meta = dict(parts=parts, T_Z=T_Z, U_MOUNT=U_MOUNT, F_MOUNT=F_MOUNT, G_OFF=G_OFF, G_ROTX=-90, TIP_F=TIP_F,
            IDL=list(IDL), TIP=[TIP_X + 2.0, MID, (JAW_Z0 + JAW_Z1) / 2], MAX_OPEN=35)
json.dump(meta, open('/home/claude/arm/twin/parts.json', 'w'), indent=1)
tot = sum(os.path.getsize(f'{OUT}/{f}') for f in os.listdir(OUT))
print(len(parts), 'parts,', round(tot / 1e6, 2), 'MB')
print(json.dumps({k: v for k, v in meta.items() if k != 'parts'}))
