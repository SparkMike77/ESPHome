"""LCARS elbow corners for the CYD (schlafzimmer.yaml), as alpha masks LVGL tints at runtime.

LVGL 8 can't draw an elbow's concave inner curve without a black cut-out block, and a cut-out
on the top layer would hide page content. These 40x40 masks are the whole corner instead:
white = filled, alpha = coverage (4x4 supersampled). The firmware shows them as GRAYSCALE
(LVGL ALPHA_8BIT) images with image_recolor, so the top elbow can take the air-quality colour.

  elbow_top.png     header arm 18 px tall, sidebar 12 px wide, outer radius 20, inner radius 10
  elbow_bottom.png  footer 26 px tall, sidebar 12 px wide, same radii (sits at y 200..240)

Run with any Python that has Pillow + numpy. Outputs are committed (original shapes).
"""
from pathlib import Path

import numpy as np
from PIL import Image

OUT = Path(__file__).parent
SIZE, SS = 40, 4
SIDE = 12          # sidebar width
R_OUT, R_IN = 20, 10


def coverage(filled) -> np.ndarray:
    n = SIZE * SS
    ys, xs = np.mgrid[0:n, 0:n]
    x = (xs + 0.5) / SS
    y = (ys + 0.5) / SS
    m = filled(x, y).astype(np.float32)
    return m.reshape(SIZE, SS, SIZE, SS).mean(axis=(1, 3))


def top(x, y):
    arm = 18
    outer = ~((x < R_OUT) & (y < R_OUT) & (np.hypot(x - R_OUT, y - R_OUT) > R_OUT))
    cx, cy = SIDE + R_IN, arm + R_IN
    shape = (x < SIDE) | (y < arm) | ((x < cx) & (y < cy) & (np.hypot(x - cx, y - cy) > R_IN))
    return outer & shape


def bottom(x, y):
    foot = 26
    top_of_foot = SIZE - foot           # 14
    cy_out = SIZE - R_OUT
    outer = ~((x < R_OUT) & (y > cy_out) & (np.hypot(x - R_OUT, y - cy_out) > R_OUT))
    cx, cy = SIDE + R_IN, top_of_foot - R_IN
    shape = (x < SIDE) | (y >= top_of_foot) | ((x < cx) & (y >= cy) & (np.hypot(x - cx, y - cy) > R_IN))
    return outer & shape


def save(name, fn):
    a = (coverage(fn) * 255).round().astype(np.uint8)
    Image.fromarray(np.dstack([a, a]), "LA").save(OUT / name)


if __name__ == "__main__":
    save("elbow_top.png", top)
    save("elbow_bottom.png", bottom)
    print("ok")
