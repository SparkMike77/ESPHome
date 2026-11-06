"""Pre-render Disc's Eye of Agamotto frames from a source render (art/source/eye.png, 800x800).

Only the centre disc of the amulet (inside the gold ring) is used, scaled to fill the 360x360
round screen, so the enclosure can supply the gold frame.

The device can't rotate images fast enough (LVGL took 120-225 ms a frame), so every frame is
rendered here and the firmware just swaps full-screen pictures:
  f00..f{N-1}.png  closed+aligned -> bands turned + eye open (green). Played backwards to close.
  open_yellow.png, open_yellow2.png   processing (two brightnesses, alternated as a pulse)
  open_red.png     error
  open_purple.png  timer ringing
  preview_sheet.png  contact sheet for checking

Sequencing inside the opening: the bands start turning first, the lids follow, so playing it
backwards closes the eye first and then turns the bands back - the mechanism locking again.

The source image and outputs are gitignored (third-party design); only this script is committed.
Run with the ESPHome venv python (has Pillow; needs numpy).
"""
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).parent
SRC = HERE / "source" / "eye.png"
OUT = HERE / "out"

# Geometry in source pixels (800x800 image). The disc centre is measured, not the image centre.
SRC_CX, SRC_CY = 410.0, 405.0
R_OUT = 247.0     # inner edge of the gold ring = edge of the screen
R_MID = 182.0     # split between the outer and inner bands
R_EYE = 88.0      # eye socket
DISPLAY = 360

FRAMES = 14       # opening sequence length (f00 closed .. f13 open)
TURN_OUTER = 40.0  # degrees, clockwise
TURN_INNER = -30.0
LID_ARC = 0.38    # lid edge bow, fraction of socket radius
LID_OPEN = 0.95   # lid slide when fully open, fraction of socket radius

PAD = 8
SIZE = int(2 * R_OUT) + 2 * PAD     # working canvas (source px), disc centred
C = SIZE / 2.0


def ease(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def load_disc() -> Image.Image:
    """Source cropped to the disc, centre at (C, C)."""
    im = Image.open(SRC).convert("RGBA")
    dx, dy = C - SRC_CX, C - SRC_CY
    return im.transform((SIZE, SIZE), Image.AFFINE, (1, 0, -dx, 0, 1, -dy), resample=Image.BICUBIC)


def radial(size, r_in, r_out, feather=1.2):
    c = (size - 1) / 2.0
    y, x = np.mgrid[0:size, 0:size]
    r = np.hypot(x - c, y - c)
    a = np.clip((r_out - r) / feather + 0.5, 0, 1)
    if r_in > 0:
        a = a * np.clip((r - r_in) / feather + 0.5, 0, 1)
    return a


def with_mask(img: Image.Image, mask: np.ndarray) -> Image.Image:
    a = np.asarray(img).astype(np.float32)
    a[..., 3] = 255.0 * mask
    return Image.fromarray(a.astype(np.uint8), "RGBA")


def make_stone(d: int, core, mid, edge) -> Image.Image:
    c = (d - 1) / 2.0
    y, x = np.mgrid[0:d, 0:d].astype(np.float32)
    r = np.hypot(x - c, y - c) / (d / 2.0)
    core, mid, edge = (np.array(v, np.float32) for v in (core, mid, edge))
    t = np.clip(r, 0, 1)[..., None]
    col = np.where(t < 0.45, core + (mid - core) * (t / 0.45), mid + (edge - mid) * ((t - 0.45) / 0.55))
    ang = np.arctan2(y - c, x - c)
    col = col * (1.0 - 0.07 * np.clip(r * 1.6, 0, 1) * (0.5 + 0.5 * np.cos(ang * 6 + r * 5.0)))[..., None]
    hl = np.exp(-(((x - c * 0.72) ** 2 + (y - c * 0.62) ** 2) / (2 * (d * 0.09) ** 2)))
    col = col + hl[..., None] * 110
    return Image.fromarray(np.clip(col, 0, 255).astype(np.uint8), "RGB").convert("RGBA")


STONES = {
    "green":   ((215, 255, 200), (40, 205, 95), (4, 48, 22)),
    "yellow":  ((255, 250, 205), (245, 190, 30), (70, 40, 0)),
    "yellow2": ((255, 255, 235), (255, 215, 70), (110, 70, 5)),
    "red":     ((255, 215, 205), (230, 45, 35), (60, 5, 5)),
    "purple":  ((250, 215, 255), (200, 70, 230), (45, 5, 60)),
}


def make_lids(disc: Image.Image, d: int):
    """Upper/lower lids from the dome texture: d wide, d + 2h tall, socket centre at row h + d/2."""
    r = d / 2.0
    h = int(math.ceil(LID_ARC * r))
    H = d + 2 * h
    k = int(R_EYE) + 2
    dome = disc.crop((int(C) - k, int(C) - k, int(C) + k, int(C) + k)).resize((H, H), Image.LANCZOS)
    tex = np.asarray(dome).astype(np.float32)[:, h:h + d]
    tex[..., 3] = 255.0
    cx, cy = (d - 1) / 2.0, (H - 1) / 2.0
    y, x = np.mgrid[0:H, 0:d].astype(np.float32)
    u = np.clip((x - cx) / r, -1, 1)
    edge_top = cy - h * (1 - u * u)
    edge_bot = cy + h * (1 - u * u)
    lids = []
    for dist in (edge_top - y, y - edge_bot):
        t = tex.copy()
        shade = np.clip(dist / (0.12 * r), 0, 1)
        t[..., :3] *= (0.40 + 0.60 * shade)[..., None]
        t[..., 3] = 255.0 * np.clip(dist + 0.5, 0, 1)
        lids.append(Image.fromarray(t.astype(np.uint8), "RGBA"))
    return lids[0], lids[1], h


class Scene:
    def __init__(self):
        self.disc = load_disc()
        self.outer = with_mask(self.disc, radial(SIZE, R_MID, R_OUT + PAD))
        self.inner = with_mask(self.disc, radial(SIZE, R_EYE - 2, R_MID + 1.5))
        self.d = int(round(2 * R_EYE))
        self.d += self.d % 2
        self.lid_top, self.lid_bot, self.h = make_lids(self.disc, self.d)
        self.stones = {k: make_stone(self.d, *v) for k, v in STONES.items()}
        self.screen_mask = Image.fromarray(
            (radial(DISPLAY, 0, DISPLAY / 2.0 - 0.5) * 255).astype(np.uint8), "L")

    def render(self, turn: float, opening: float, stone: str = "green") -> Image.Image:
        canvas = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 255))
        canvas.alpha_composite(self.outer.rotate(-TURN_OUTER * turn, resample=Image.BICUBIC))
        canvas.alpha_composite(self.inner.rotate(-TURN_INNER * turn, resample=Image.BICUBIC))
        d, h = self.d, self.h
        socket = Image.new("RGBA", (d, d), (0, 0, 0, 255))
        socket.alpha_composite(self.stones[stone])
        travel = h + LID_OPEN * d / 2.0
        layer = Image.new("RGBA", (d, d * 4), (0, 0, 0, 0))
        base = 2 * d
        layer.alpha_composite(self.lid_bot, (0, base - h + int(round(-h + opening * travel))))
        layer.alpha_composite(self.lid_top, (0, base - h + int(round(h - opening * travel))))
        socket.alpha_composite(layer.crop((0, base, d, base + d)))
        m = Image.fromarray((radial(d, 0, d / 2.0 - 0.5) * 255).astype(np.uint8), "L")
        off = int(round(C - d / 2.0))
        canvas.paste(socket, (off, off), m)
        # crop the disc (R_OUT) and fit it to the screen
        k = R_OUT
        disc = canvas.crop((int(C - k), int(C - k), int(C + k), int(C + k))).resize(
            (DISPLAY, DISPLAY), Image.LANCZOS)
        out = Image.new("RGB", (DISPLAY, DISPLAY), (0, 0, 0))
        out.paste(disc.convert("RGB"), (0, 0), self.screen_mask)
        return out


def main():
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*.png"):
        old.unlink()
    s = Scene()
    frames = []
    for i in range(FRAMES):
        t = i / (FRAMES - 1)
        turn = ease(t / 0.75)
        opening = ease((t - 0.35) / 0.65)
        img = s.render(turn, opening)
        img.save(OUT / f"f{i:02d}.png")
        frames.append(img)
    for name in ("yellow", "yellow2", "red", "purple"):
        s.render(1.0, 1.0, name).save(OUT / f"open_{name}.png")
    print(f"{FRAMES} frames + 4 variants, socket d={s.d}px source")

    thumbs = frames + [Image.open(OUT / f"open_{n}.png") for n in ("yellow", "yellow2", "red", "purple")]
    tw = 180
    cols = 6
    rows = math.ceil(len(thumbs) / cols)
    sheet = Image.new("RGB", (cols * (tw + 6), rows * (tw + 6)), (40, 40, 40))
    for i, im in enumerate(thumbs):
        sheet.paste(im.resize((tw, tw), Image.LANCZOS), ((i % cols) * (tw + 6), (i // cols) * (tw + 6)))
    sheet.save(OUT / "preview_sheet.png")


if __name__ == "__main__":
    sys.exit(main())
