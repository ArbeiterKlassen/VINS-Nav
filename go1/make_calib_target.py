#!/usr/bin/env python3
"""make_calib_target.py — 生成可打印的双目标定棋盘格（A4, 实际尺寸）。

默认：内角点 9×6（方格 10×7），方格 20.0 mm → 图案 200×140 mm，A4 可 1:1 打印。
输出 targets/checkerboard_9x6_20mm_A4.png + .pdf，并附 100 mm 校验尺。

用法:
  python3 make_calib_target.py [--cols 9 --rows 6 --square-mm 20 --out targets]
打印：务必选择 "实际大小 / 100%"，不要"适合页面"；打印后用尺子量 10 个方格的
总长度验证（应为 cols * square-mm 附近），必要时把实测方格尺寸传给标定脚本
（--square）。方格尺寸只影响平移/基线尺度，不影响内参。
"""
from __future__ import annotations

import argparse
import os

from PIL import Image, ImageDraw, ImageFont

DPI = 300


def mm2px(mm: float) -> int:
    return int(round(mm * DPI / 25.4))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cols", type=int, default=9, help="内角点数（列）")
    ap.add_argument("--rows", type=int, default=6, help="内角点数（行）")
    ap.add_argument("--square-mm", type=float, default=20.0, help="方格边长（毫米）")
    ap.add_argument("--out", default="targets")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    sq = args.square_mm
    n_x, n_y = args.cols + 1, args.rows + 1  # 方格数
    board_w_mm, board_h_mm = n_x * sq, n_y * sq

    pw, ph = mm2px(210), mm2px(297)  # A4
    if mm2px(board_w_mm) > pw - mm2px(10) or mm2px(board_h_mm) > ph - mm2px(60):
        raise SystemExit("板子放不进 A4，请减小 --square-mm 或换 A3")

    img = Image.new("RGB", (pw, ph), "white")
    d = ImageDraw.Draw(img)
    ox = (pw - mm2px(board_w_mm)) // 2
    oy = mm2px(18)
    s = mm2px(sq)
    for j in range(n_y):
        for i in range(n_x):
            if (i + j) % 2 == 0:
                d.rectangle([ox + i * s, oy + j * s, ox + (i + 1) * s - 1, oy + (j + 1) * s - 1],
                            fill="black")

    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 34)
        font_s = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 26)
    except OSError:
        font = font_s = ImageFont.load_default()

    y = oy + mm2px(board_h_mm) + mm2px(8)
    d.text((ox, y), f"Checkerboard: inner corners {args.cols}x{args.rows}, square {sq:g} mm",
           fill="black", font=font)
    d.text((ox, y + 44), "Print at 100% / Actual size (do NOT fit to page).", fill="black", font=font_s)
    d.text((ox, y + 80), "Verify: 10 squares = %.1f mm after printing." % (10 * sq),
           fill="black", font=font_s)

    # 100 mm 校验尺
    ry = ph - mm2px(28)
    rx0 = mm2px(20)
    d.line([rx0, ry, rx0 + mm2px(100), ry], fill="black", width=3)
    for k in range(11):
        x = rx0 + mm2px(10 * k)
        d.line([x, ry - mm2px(3), x, ry + mm2px(3)], fill="black", width=2)
    d.text((rx0 + mm2px(104), ry - 14), "100 mm", fill="black", font=font_s)

    base = os.path.join(args.out, f"checkerboard_{args.cols}x{args.rows}_{sq:g}mm_A4")
    img.save(base + ".png", dpi=(DPI, DPI))
    img.save(base + ".pdf", resolution=DPI)
    print("saved:", base + ".png / .pdf")
    print("board physical size: %.1f x %.1f mm; inner corners %dx%d"
          % (board_w_mm, board_h_mm, args.cols, args.rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
