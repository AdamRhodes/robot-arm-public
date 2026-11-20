"""Flatten PrusaSlicer vendor presets (with 'inherits') into one config file for the CLI."""
import re, sys
SRC = '/usr/share/PrusaSlicer/profiles/PrusaResearch.ini'
sections, cur = {}, None
for line in open(SRC, encoding='utf-8'):
    line = line.rstrip('\n')
    m = re.match(r'^\[(\w+):(.+)\]$', line)
    if m: cur = (m.group(1), m.group(2)); sections[cur] = {}; continue
    if cur and '=' in line and not line.lstrip().startswith('#'):
        k, v = line.split('=', 1); sections[cur][k.strip()] = v.strip()
def resolve(kind, name, depth=0):
    sec = sections[(kind, name)]; out = {}
    for parent in [p.strip() for p in sec.get('inherits', '').split(';') if p.strip()]:
        out.update(resolve(kind, parent, depth + 1))
    out.update({k: v for k, v in sec.items() if k != 'inherits'})
    return out
cfg = {}
for kind, name in [('printer', sys.argv[1]), ('print', sys.argv[2]), ('filament', sys.argv[3])]:
    d = resolve(kind, name)
    for drop in ('compatible_printers', 'compatible_printers_condition', 'compatible_prints', 'compatible_prints_condition', 'renamed_from'):
        d.pop(drop, None)
    cfg.update(d)
with open(sys.argv[4], 'w') as f:
    for k, v in cfg.items(): f.write(f'{k} = {v}\n')
print(len(cfg), 'settings ->', sys.argv[4])
