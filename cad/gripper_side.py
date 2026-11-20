"""Gripper turned 90 deg about the forearm's long axis so the jaws open sideways.
Gripper frame G (same as gripper.py) maps into the forearm frame F by
   p_f = R @ p_g + T,   R = rot_x(-90): x_g -> x_f, y_g -> -z_f, z_g -> +y_f
The new base plate grows an L-shaped foot that lies on the forearm's front face (z_f = 5..9)
and takes the forearm's two threaded holes at (58, +-8)."""
import sys; sys.path.insert(0, '/home/claude/arm')
from gripper import *

R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)      # rot_x(-90)
T = np.array([83.0, -18.25, MID + 2.5])                      # jaws centred on the forearm plate's mid-plane
FOOT_X = (50.5, 66.0); FOOT_Z = (5.0, 9.0); HOLE_Y = 8.0

def f_to_g(man):
    """move a manifold built in forearm coords into gripper coords: g = R^T (f - T)"""
    Rt = R.T; off = -Rt @ T
    return man.transform([[*Rt[0], off[0]], [*Rt[1], off[1]], [*Rt[2], off[2]]])
def g_to_f(man):
    return man.transform([[*R[0], T[0]], [*R[1], T[1]], [*R[2], T[2]]])

def foot_f():
    """foot + gusset, built in forearm coords"""
    y_plate_back = T[1] - PLATE           # -22.25 (base plate back face)
    y_plate_front = T[1]                  # -18.25
    out = m.CrossSection.batch_hull([circ(7.5, 58, HOLE_Y), circ(7.5, 58, -HOLE_Y),
        m.CrossSection.square((FOOT_X[1] - FOOT_X[0], 6.0)).translate((FOOT_X[0], y_plate_back))])
    foot = m.Manifold.extrude(out, FOOT_Z[1] - FOOT_Z[0]).translate((0, 0, FOOT_Z[0]))
    # triangular gusset in the inside corner of the L (between foot top and plate front)
    tri = m.CrossSection([[(y_plate_front - 0.01, FOOT_Z[1] - 0.01), (y_plate_front + 3.5, FOOT_Z[1] - 0.01),
                           (y_plate_front - 0.01, FOOT_Z[1] + 11)]])
    g = m.Manifold.extrude(tri, FOOT_X[1] - FOOT_X[0] - 1.0)          # extruded along +z of the 2D frame
    # 2D (u,v)=(y,z), extrude along w -> map (u,v,w) to (x=w, y=u, z=v)
    g = g.transform([[0, 0, 1, FOOT_X[0] + 0.5], [1, 0, 0, 0], [0, 1, 0, 0]])
    foot += g
    # teardrop holes: the part prints with +y_f pointing up, so the top of each hole comes to a 45 deg point
    r = 4.35
    for y in (HOLE_Y, -HOLE_Y):
        pts = [(58 + r*np.cos(a), y + r*np.sin(a)) for a in np.linspace(0, 2*np.pi, 96, endpoint=False)] + [(58, y + r*np.sqrt(2))]
        td = m.CrossSection.hull_points(pts)
        foot -= m.Manifold.extrude(td, 10).translate((0, 0, FOOT_Z[0] - 2))
    return foot

def base_plate_side():
    # plate outline in gripper coords: the old outline minus the forearm-mount lobes, plus a tab out to the foot
    out = m.CrossSection.batch_hull([circ(7, 0, BCY - 13.9 - 1), circ(9.5, *IDL), circ(7, *PILLAR),
        m.CrossSection.square((BD + 6, BW + 6)).translate((-(BD + 6) / 2, BCY - (BW + 6) / 2))])
    foot_x_g = (FOOT_X[0] - T[0], FOOT_X[1] - T[0])                 # -29.5 .. -14
    yg_hi = T[2] - FOOT_Z[0] + 0.0                                  # foot underside in y_g (9.6)
    tab = m.CrossSection.square((0 - foot_x_g[0], yg_hi + 6.4)).translate((foot_x_g[0], -7.0))   # stops 0.6 inside the foot (no coincident faces)   # wide enough to carry the gusset
    out = out + tab
    p = m.Manifold.extrude(out, PLATE).translate((0, 0, -PLATE))
    p += cyl(9, JAW_Z0, IDL[0], IDL[1], 0)
    p += cyl(6.5, TOP_Z0 - JAW_Z0, IDL[0], IDL[1], JAW_Z0)
    p += cyl(6.5, TOP_Z0, PILLAR[0], PILLAR[1], 0)
    p += f_to_g(foot_f())
    p -= box(-(BD + CL) / 2, (BD + CL) / 2, BCY - (BW + CL) / 2, BCY + (BW + CL) / 2, -PLATE - 1, 1)
    for dy in (-TABP / 2, TABP / 2): p -= cyl(0.9, PLATE + 2, 0, BCY + dy, -PLATE - 1, 24)
    for y0 in (BCY + (BW + CL) / 2 - 0.1, BCY - (BW + CL) / 2 - 1.6):
        p -= box(-2.25, 2.25, y0, y0 + 1.7, -PLATE - 1, 1)
    p -= thread_hole(IDL[0], IDL[1], TOP_Z0, 8)
    p -= thread_hole(PILLAR[0], PILLAR[1], TOP_Z0, 8)
    return p

if __name__ == '__main__':
    fl = trimesh.load('/mnt/user-data/outputs/forearm.stl').simplify_quadric_decimation(face_count=6000)
    fore_f = m.Manifold(m.Mesh(vert_properties=np.asarray(fl.vertices, np.float32), tri_verts=np.asarray(fl.faces, np.uint32)))
    fore = f_to_g(fore_f)
    scr = trimesh.load('/mnt/user-data/outputs/screwplate_screw_hex.stl')
    def screw_f(x, y, zb):
        s = scr.copy(); M_ = np.eye(4); M_[2, 2] = -1; M_[:3, 3] = (x, y, zb + 4.5); s.apply_transform(M_); s.invert()
        return m.Manifold(m.Mesh(vert_properties=np.asarray(s.vertices, np.float32), tri_verts=np.asarray(s.faces, np.uint32)))
    def screw_g(x, y, zb):
        s = scr.copy(); M_ = np.eye(4); M_[2, 2] = -1; M_[:3, 3] = (x, y, zb + 4.5); s.apply_transform(M_); s.invert()
        return m.Manifold(m.Mesh(vert_properties=np.asarray(s.vertices, np.float32), tri_verts=np.asarray(s.faces, np.uint32)))
    screws = {f'foot{i}': f_to_g(screw_f(58, y, FOOT_Z[1])) for i, y in enumerate((HOLE_Y, -HOLE_Y))}
    screws.update({'top_idler': screw_g(*IDL, TOP_Z1), 'top_pillar': screw_g(*PILLAR, TOP_Z1)})
    fa_scr = {f'fa{i}': f_to_g(screw_f(x, 0, 5)) for i, x in enumerate((27, 43))}
    base = base_plate_side(); top = top_plate(); srv = servo_env()
    tabheads = cyl(2.5, 1.5, 0, BCY - TABP / 2, TAB_T) + cyl(2.5, 1.5, 0, BCY + TABP / 2, TAB_T)
    worst = {}
    for a in np.linspace(0, 35, 8):
        items = {'base': base, 'top': top, 'driven': rotz(driven_jaw(), -a, *DRV), 'idler': rotz(idler_jaw(), a, *IDL),
                 'servo': srv, 'tabheads': tabheads, 'horn': horn(th_dir - a), 'forearm': fore, **screws, **fa_scr}
        n = list(items)
        for i in range(len(n)):
            for k in range(i + 1, len(n)):
                pair = {n[i], n[k]}
                if pair in ({'driven', 'horn'}, {'servo', 'horn'}, {'servo', 'tabheads'}): continue
                v = (items[n[i]] ^ items[n[k]]).volume()
                if v > 0.01: worst[(n[i], n[k])] = max(worst.get((n[i], n[k]), 0), v)
    print('overlaps:', {k: round(v, 2) for k, v in worst.items()} or 'none')
    # tightest gaps
    def gap(a, b):
        A = tomesh(a); B = tomesh(b); return B.nearest.on_surface(A.vertices[::25])[1].min()
    print('servo - foot screw heads', round(min(gap(srv, screws['foot0']), gap(srv, screws['foot1'])), 2))
    print('servo - forearm', round(gap(srv, fore), 2))
    print('base plate - forearm', round(gap(base, fore), 2))
    print('idler jaw (open) - forearm', round(gap(rotz(idler_jaw(), 35, *IDL), fore), 2))
    print('top plate - foot screws', round(min(gap(top, screws['foot0']), gap(top, screws['foot1'])), 2))
    print('foot - forearm screw heads (27,43)', round(min(gap(base, fa_scr['fa0']), gap(base, fa_scr['fa1'])), 2))
    t = tomesh(base); print('base watertight', t.is_watertight, 'volume g', round(t.volume * 1.24 / 1000, 1))
