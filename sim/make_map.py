#!/usr/bin/env python3
"""Generate a 10x10 m test map (0.05 m/px): border walls + 0.4x0.4 m box at (5,5)."""
import os

W = H = 200  # 10 m / 0.05
FREE, OCC = 254, 0
grid = [[FREE] * W for _ in range(H)]

# border walls, 2 cells thick
for i in range(H):
    for j in range(W):
        if i < 2 or i >= H - 2 or j < 2 or j >= W - 2:
            grid[i][j] = OCC

# box obstacle centered at world (5.0, 5.0): 0.4 x 0.4 m => cells 96..103
for i in range(96, 104):
    for j in range(96, 104):
        grid[i][j] = OCC

out = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(out, "map.pgm"), "wb") as f:
    f.write(b"P5\n%d %d\n255\n" % (W, H))
    # PGM row 0 = top of image = max y in map convention
    for i in range(H - 1, -1, -1):
        f.write(bytes(grid[i]))

with open(os.path.join(out, "map.yaml"), "w") as f:
    f.write(
        "image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
print("map written to", out)
