#!/usr/bin/env python3
"""Render a Kalibr AprilGrid pixel-exact for a laptop screen.

The layout follows kalibr_create_target_pdf: t36h11 tags with 2 border bits,
tag code rotated by 180 degrees, tag id = cols * row + col with row 0 at the
bottom, and black squares of tagSpacing * tagSize at every tag corner.
Every edge falls on a whole pixel, so the screen shows sharp corners.

Outputs a PNG, the Kalibr target YAML, and an HTML page that shows the PNG
with one image pixel per CSS device pixel. macOS display scaling other than
the default renders at a different size and resamples, so the shown board is
not the design size. Always measure the full board width on the screen and
pass the derived tag size with --measured-tag-mm: tag = width * tag_px /
(cols * tag_px + (cols + 1) * gap_px). Kalibr distances scale with it.
"""
from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

# First t36h11 codes of kalibr_create_target_pdf (AprilTagCodes.t36h11).
T36H11 = [
    0xd5d628584, 0xd97f18b49, 0xdd280910e, 0xe479e9c98, 0xebcbca822, 0xf31dab3ac,
    0x056a5d085, 0x10652e1d4, 0x22b1dfead, 0x265ad0472, 0x34fe91b86, 0x3ff962cd5,
    0x43a25329a, 0x474b4385f, 0x4e9d243e9, 0x5246149ae, 0x5997f5538, 0x683bb6c4c,
    0x6be4a7211, 0x7e3158eea, 0x81da494af, 0x858339a74, 0x8cd51a5fe, 0x9f21cc2d7,
    0xa2cabc89c, 0xadc58d9eb, 0xb16e7dfb0, 0xb8c05eb3a, 0xd25ef139d, 0xd607e1962,
    0xe4aba3076, 0x2dde6a3da, 0x43d40c678, 0x5620be351, 0x64c47fa65, 0x686d7002a,
    0x6c16605ef, 0x6fbf50bb4, 0x8d06d39dc, 0x9f53856b5,
]
CODE_BITS = 6
BORDER_BITS = 2


def tag_bits(code: int) -> list[list[bool]]:
    """Black cells of one tag including the border, row 0 at the top."""
    n = CODE_BITS + 2 * BORDER_BITS
    black = [[True] * n for _ in range(n)]
    code_black = [[not (code >> (CODE_BITS * i + j)) & 1 for j in range(CODE_BITS)] for i in range(CODE_BITS)]
    rotated = [row[::-1] for row in code_black[::-1]]
    for i in range(CODE_BITS):
        for j in range(CODE_BITS):
            black[BORDER_BITS + i][BORDER_BITS + j] = rotated[i][j]
    return black


def render(cols: int, rows: int, bit_px: int, spacing: float, margin_px: int, width: int, height: int):
    n = CODE_BITS + 2 * BORDER_BITS
    tag_px = n * bit_px
    gap_px = round(spacing * tag_px)
    if abs(gap_px - spacing * tag_px) > 1e-9:
        raise SystemExit(f"tagSpacing * tag size must be a whole number of pixels, got {spacing * tag_px}")
    board_w = cols * tag_px + (cols + 1) * gap_px
    board_h = rows * tag_px + (rows + 1) * gap_px
    width = width or board_w + 2 * margin_px
    height = height or board_h + 2 * margin_px
    if board_w > width or board_h > height:
        raise SystemExit(f"board {board_w}x{board_h} px does not fit in {width}x{height} px")
    if cols * rows > len(T36H11):
        raise SystemExit(f"at most {len(T36H11)} tags are supported")

    img = [bytearray(b"\xff" * width) for _ in range(height)]
    x0 = (width - board_w) // 2 + gap_px
    bottom = (height - board_h) // 2 + gap_px + rows * tag_px + (rows - 1) * gap_px

    def fill(x: int, y: int, w: int, h: int) -> None:
        for yy in range(y, y + h):
            img[yy][x:x + w] = b"\x00" * w

    pitch = tag_px + gap_px
    for r in range(rows):
        for c in range(cols):
            left = x0 + c * pitch
            top = bottom - r * pitch - tag_px
            bits = tag_bits(T36H11[cols * r + c])
            for i in range(n):
                for j in range(n):
                    if bits[i][j]:
                        fill(left + j * bit_px, top + i * bit_px, bit_px, bit_px)
            for cx, cy in ((left - gap_px, top - gap_px), (left + tag_px, top - gap_px),
                           (left - gap_px, top + tag_px), (left + tag_px, top + tag_px)):
                fill(cx, cy, gap_px, gap_px)
    return img, width, height, tag_px


def write_png(path: Path, img: list[bytearray], width: int, height: int) -> None:
    raw = b"".join(b"\x00" + bytes(row) for row in img)

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9))
                     + chunk(b"IEND", b""))


HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>AprilGrid {cols}x{rows}</title>
<style>
html,body{{margin:0;height:100%;background:#fff;overflow:hidden;cursor:none}}
img{{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);image-rendering:pixelated}}
#info{{position:absolute;left:8px;bottom:8px;font:12px monospace;color:#888}}
</style></head><body>
<img id="g" src="{png}">
<div id="info"></div>
<script>
const g=document.getElementById('g');
function fit(){{
  const dpr=window.devicePixelRatio||1;
  g.style.width=({width}/dpr)+'px';
  g.style.height=({height}/dpr)+'px';
  document.getElementById('info').textContent=
    'devicePixelRatio='+dpr+' tag='+{tag_px}+'px  press f for fullscreen';
}}
fit();
window.addEventListener('resize',fit);
document.addEventListener('keydown',e=>{{if(e.key==='f')document.documentElement.requestFullscreen();}});
</script></body></html>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--bit-px", type=int, default=32, help="pixels per tag bit; a tag is 10 bits wide")
    ap.add_argument("--spacing", type=float, default=0.3, help="Kalibr tagSpacing, ratio of the tag size")
    ap.add_argument("--pixel-pitch-mm", type=float, default=25.4 / 254.0,
                    help="screen pixel pitch; MacBook Pro 14 and 16 inch are 254 ppi")
    ap.add_argument("--measured-tag-mm", type=float, default=None,
                    help="tag edge measured on the screen, overrides the pixel pitch")
    ap.add_argument("--margin-px", type=int, default=60)
    ap.add_argument("--width", type=int, default=0, help="canvas width, defaults to the board plus margin")
    ap.add_argument("--height", type=int, default=0)
    args = ap.parse_args()

    img, width, height, tag_px = render(args.cols, args.rows, args.bit_px, args.spacing,
                                        args.margin_px, args.width, args.height)
    tag_m = (args.measured_tag_mm if args.measured_tag_mm else tag_px * args.pixel_pitch_mm) / 1000.0
    stem = f"aprilgrid_screen_{args.cols}x{args.rows}_{tag_px}px"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    png = args.out_dir / f"{stem}.png"
    write_png(png, img, width, height)
    yaml_path = args.out_dir / f"{stem}.yaml"
    yaml_path.write_text(
        "target_type: 'aprilgrid'\n"
        f"tagCols: {args.cols}\n"
        f"tagRows: {args.rows}\n"
        f"tagSize: {tag_m:.6f}\n"
        f"tagSpacing: {args.spacing}\n"
    )
    html = args.out_dir / f"{stem}.html"
    html.write_text(HTML.format(cols=args.cols, rows=args.rows, png=png.name,
                                width=width, height=height, tag_px=tag_px))
    print(f"png  {png} ({width}x{height} px, tag {tag_px} px)")
    print(f"yaml {yaml_path} (tagSize {tag_m * 1000:.3f} mm)")
    print(f"html {html}")


if __name__ == "__main__":
    main()
