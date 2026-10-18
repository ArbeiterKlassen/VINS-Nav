#!/usr/bin/env python3
"""stereo_depth_demo.py — 双目测距演示（SGBM 视差 → 深度图 + 距离估计）。

对一对已校正（rectified）左右目图像:
  1) 可选 180° 旋转（Go1 SDK 的校正输出是倒的；两幅一起转，几何不变，
     视差符号会翻转——脚本会自动检测并交换/取反）
  2) SGBM 计算视差 → depth = fx * baseline / disparity
  3) 输出彩色深度图 PNG + 中心区域的距离统计

用法:
  python3 stereo_depth_demo.py --left rect_left_00.png --right rect_right_00.png \
      --out /tmp/depth_demo [--rotate180 yes] [--fx 464] [--baseline 0.02509]

注意: 内参为估计值（hFov≈90°），标定替换后测距才准；本脚本用于能力演示。
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--left", required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--out", default="/tmp/depth_demo")
    ap.add_argument("--rotate180", default="yes", choices=["yes", "no"])
    ap.add_argument("--fx", type=float, default=None,
                    help="焦距（像素）；默认按 hFov≈90°: fx = width/2")
    ap.add_argument("--baseline", type=float, default=0.02509, help="基线（米）")
    ap.add_argument("--num-disparities", type=int, default=64)
    ap.add_argument("--block-size", type=int, default=9)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    l = cv2.imread(args.left)
    r = cv2.imread(args.right)
    if l is None or r is None:
        print("failed to read images", file=sys.stderr)
        return 1
    if args.rotate180 == "yes":
        l = cv2.rotate(l, cv2.ROTATE_180)
        r = cv2.rotate(r, cv2.ROTATE_180)
    gl = cv2.cvtColor(l, cv2.COLOR_BGR2GRAY)
    gr = cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
    h, w = gl.shape
    fx = args.fx if args.fx else w / 2.0

    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=args.num_disparities,
        blockSize=args.block_size,
        P1=8 * 3 * args.block_size ** 2,
        P2=32 * 3 * args.block_size ** 2,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        disp12MaxDiff=1,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    disp = sgbm.compute(gl, gr).astype(np.float32) / 16.0  # 亚像素

    valid = disp > 0.5
    if valid.sum() > 0 and float(np.median(disp[valid])) < 0:
        print("[warn] median disparity < 0 — 左右目可能互换/方向相反，自动交换")
        disp = -disp
        valid = disp > 0.5

    depth = np.zeros_like(disp)
    depth[valid] = fx * args.baseline / disp[valid]

    # 彩色深度图（0.2–5 m 截断）
    dvis = np.clip(depth, 0.2, 5.0)
    dn = ((dvis - 0.2) / (5.0 - 0.2) * 255).astype(np.uint8)
    dn[~valid] = 0
    color = cv2.applyColorMap(dn, cv2.COLORMAP_TURBO)
    color[~valid] = (0, 0, 0)
    cv2.imwrite(os.path.join(args.out, "depth_color.png"), color)
    cv2.imwrite(os.path.join(args.out, "disparity_raw.png"), (np.clip(disp, 0, 64) / 64 * 255).astype(np.uint8))
    cv2.imwrite(os.path.join(args.out, "left_used.png"), l)
    cv2.imwrite(os.path.join(args.out, "right_used.png"), r)

    ch, cw = h // 4, w // 4
    cy0, cy1, cx0, cx1 = h // 2 - ch, h // 2 + ch, w // 2 - cw, w // 2 + cw
    d_roi = depth[cy0:cy1, cx0:cx1]
    v_roi = valid[cy0:cy1, cx0:cx1]
    if v_roi.sum() > 100:
        vals = d_roi[v_roi]
        print(f"valid disparity: {valid.mean() * 100:.1f}% of image")
        print(f"center ROI depth: median={np.median(vals):.2f} m, "
              f"p10={np.percentile(vals, 10):.2f} m, p90={np.percentile(vals, 90):.2f} m")
    else:
        print(f"valid disparity: {valid.mean() * 100:.1f}% (center ROI too sparse)")
    print(f"saved to {args.out}/ (depth_color.png, disparity_raw.png, left/right_used.png)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
