"""Summarise a PrusaSlicer G-code: extrusion by feature type, and where overhangs/bridges happen."""
import re, sys, collections
path = sys.argv[1]
typ = None; z = 0.0; x = y = None; e_abs = None
length = collections.Counter(); where = collections.defaultdict(list)
for line in open(path):
    if line.startswith(';TYPE:'): typ = line[6:].strip(); continue
    if line.startswith(';Z:'): z = float(line[3:]); continue
    if not line.startswith('G1'): continue
    m = dict(re.findall(r'([XYZE])(-?[\d.]+)', line))
    nx = float(m['X']) if 'X' in m else x; ny = float(m['Y']) if 'Y' in m else y
    if 'E' in m and float(m['E']) > 0 and x is not None and typ:
        d = ((nx - x)**2 + (ny - y)**2) ** .5; length[typ] += d
        if typ in ('Overhang perimeter', 'Bridge infill'): where[typ].append((z, nx, ny, d))
    x, y = nx, ny
tot = sum(length.values())
for k, v in length.most_common(): print(f'{k:24s} {v/1000:7.2f} m  {100*v/tot:5.1f}%')
for k, pts in where.items():
    zs = sorted({round(p[0], 2) for p in pts})
    print(f'\n{k}: {sum(p[3] for p in pts):.0f} mm over {len(zs)} layers, z = {zs[:12]}{" ..." if len(zs) > 12 else ""}')
    xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
    print(f'   x {min(xs):.1f}..{max(xs):.1f}  y {min(ys):.1f}..{max(ys):.1f}')
