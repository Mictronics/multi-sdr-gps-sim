#!/usr/bin/env python3
# Generates geoid_egm96.h from GeographicLib's EGM96 15' grid, see docs/development.md:
#   python3 docs/mkgeoid.py geoids/egm96-15.pgm geoid_egm96.h
import sys, struct, math
pgm, out = sys.argv[1], sys.argv[2]
d = open(pgm, 'rb').read()
pos = 0; fields = []
while len(fields) < 4:
    nl = d.index(b'\n', pos); line = d[pos:nl].decode(); pos = nl + 1
    if line.startswith('#'):
        if line.startswith('# Offset'): off = float(line.split()[2])
        if line.startswith('# Scale'): sc = float(line.split()[2])
        continue
    fields += line.split()
w, h = int(fields[1]), int(fields[2])
raw = struct.unpack('>%dH' % (w * h), d[pos:pos + 2 * w * h])
N = lambda r, c: off + sc * raw[r * w + c]
rows, cols = (h - 1) // 4 + 1, w // 4
g = [[round(N(4 * r, 4 * c) * 10) for c in range(cols)] for r in range(rows)]
# bilinear on the 1 deg grid vs the full 15' grid
mx = ss = 0.0
for r in range(h):
    for c in range(w):
        y, x = r / 4, c / 4; r0 = min(int(y), rows - 2); c0 = int(x); fy, fx = y - r0, x - c0; c1 = (c0 + 1) % cols
        v = ((1-fy)*((1-fx)*g[r0][c0]+fx*g[r0][c1]) + fy*((1-fx)*g[r0+1][c0]+fx*g[r0+1][c1])) / 10
        e = abs(v - N(r, c)); mx = max(mx, e); ss += e * e
print('max %.2f m rms %.3f m' % (mx, math.sqrt(ss / (w * h))), file=sys.stderr)
with open(out, 'w') as f:
    f.write('/* EGM96 geoid height above the WGS84 ellipsoid in decimetres, generated; do not edit.\n'
            ' * Every 4th point of GeographicLib\'s egm96-15.pgm (NGA EGM96 15\' grid), rows 90N to 90S,\n'
            ' * columns 0E to 359E. See docs/development.md to regenerate.\n */\n\n')
    f.write('#define GEOID_STEP 1.0\n#define GEOID_ROWS %d\n#define GEOID_COLS %d\n#define GEOID_SCALE 10.0\n\n' % (rows, cols))
    f.write('static const short geoid_egm96[GEOID_ROWS][GEOID_COLS] = {\n')
    for row in g:
        f.write('    {' + ','.join(map(str, row)) + '},\n')
    f.write('};\n')
