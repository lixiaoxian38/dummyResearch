#!/usr/bin/env python3
"""Generate an A4 printable ArUco calibration board (PDF + PNG).

Default matches dummy_vision:
  DICT_ARUCO_ORIGINAL, marker id 0, black square side = 50 mm.

Usage:
  python3 scripts/vision/generate_aruco_a4_pdf.py
  python3 scripts/vision/generate_aruco_a4_pdf.py --marker-id 0 --marker-mm 50 --out scripts/vision/aruco_markers/

Print: **100% scale / Actual size** (do NOT fit to page).
After printing, measure the black square with a ruler — it must be exactly 50 mm.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as e:
    raise SystemExit("需要 Pillow：sudo apt install python3-pil 或 pip install pillow") from e

ARUCO_DICTS = {
    "ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "6X6_50": cv2.aruco.DICT_6X6_50,
}

# A4 @ 300 DPI
A4_W_MM = 210.0
A4_H_MM = 297.0
DEFAULT_DPI = 300


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def generate_marker_image(aruco_dict_name: str, marker_id: int, side_px: int) -> np.ndarray:
    if aruco_dict_name not in ARUCO_DICTS:
        raise ValueError(f"Unknown dict {aruco_dict_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[aruco_dict_name])
    # OpenCV 4.7+
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, side_px, borderBits=1)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, side_px, borderBits=1)
    return img


def build_a4_page(
    aruco_dict: str,
    marker_id: int,
    marker_mm: float,
    dpi: int,
) -> Image.Image:
    page_w = mm_to_px(A4_W_MM, dpi)
    page_h = mm_to_px(A4_H_MM, dpi)
    marker_px = mm_to_px(marker_mm, dpi)

    marker = generate_marker_image(aruco_dict, marker_id, marker_px)
    # Marker is grayscale; compose RGB page
    page = np.ones((page_h, page_w, 3), dtype=np.uint8) * 255
    mx = (page_w - marker_px) // 2
    my = (page_h - marker_px) // 2 - mm_to_px(8, dpi)  # slightly above center for caption
    page[my : my + marker_px, mx : mx + marker_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    pil = Image.fromarray(page)
    draw = ImageDraw.Draw(pil)

    # Scale verification bar (50 mm horizontal under marker)
    bar_y = my + marker_px + mm_to_px(6, dpi)
    bar_x0 = mx
    bar_x1 = mx + marker_px
    draw.line([(bar_x0, bar_y), (bar_x1, bar_y)], fill=(0, 0, 0), width=max(2, dpi // 150))
    draw.line([(bar_x0, bar_y - 4), (bar_x0, bar_y + 4)], fill=(0, 0, 0), width=2)
    draw.line([(bar_x1, bar_y - 4), (bar_x1, bar_y + 4)], fill=(0, 0, 0), width=2)

    caption_y = bar_y + mm_to_px(5, dpi)
    lines = [
        f"Dummy Research — ArUco {aruco_dict}  id={marker_id}",
        f"Black square side = {marker_mm:.0f} mm  (set marker_length={marker_mm / 1000:.3f} in ROS)",
        "Print: 100% / Actual size — verify black square with a ruler before use.",
        "Mount flat on table for hand-eye calibration; hold in hand for tracking demo.",
    ]
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", mm_to_px(3.5, dpi))
        font_sm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", mm_to_px(3.0, dpi))
    except OSError:
        font = ImageFont.load_default()
        font_sm = font

    for i, line in enumerate(lines):
        f = font if i == 0 else font_sm
        tw = draw.textlength(line, font=f) if hasattr(draw, "textlength") else len(line) * 8
        tx = (page_w - int(tw)) // 2
        ty = caption_y + i * mm_to_px(5, dpi)
        draw.text((tx, ty), line, fill=(0, 0, 0), font=f)

    # Corner crop marks (optional visual guide)
    m = mm_to_px(10, dpi)
    for cx, cy in [(m, m), (page_w - m, m), (m, page_h - m), (page_w - m, page_h - m)]:
        draw.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], outline=(180, 180, 180))

    return pil


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate A4 ArUco calibration board PDF")
    p.add_argument("--dict", default="ORIGINAL", choices=sorted(ARUCO_DICTS.keys()))
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--marker-mm", type=float, default=50.0, help="Black square side length (mm)")
    p.add_argument("--dpi", type=int, default=DEFAULT_DPI)
    p.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "aruco_markers",
        help="Output directory",
    )
    p.add_argument("--basename", default="", help="Output base name (default auto)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    base = args.basename or f"aruco_{args.dict.lower()}_id{args.marker_id}_{int(args.marker_mm)}mm_a4"
    png_path = args.out / f"{base}.png"
    pdf_path = args.out / f"{base}.pdf"

    page = build_a4_page(args.dict, args.marker_id, args.marker_mm, args.dpi)
    page.save(png_path, "PNG", dpi=(args.dpi, args.dpi))
    page.save(pdf_path, "PDF", resolution=args.dpi)

    print(f"Generated:\n  {pdf_path}\n  {png_path}")
    print(f"Dict={args.dict}  id={args.marker_id}  marker={args.marker_mm}mm  dpi={args.dpi}")
    print("Print PDF at 100% scale; measure black square = {:.0f} mm.".format(args.marker_mm))
    return 0


if __name__ == "__main__":
    sys.exit(main())
