#!/usr/bin/env python3
"""
Post-process v5: GT-driven optimization.
- All white-gray boundaries → occupied (walls extend into unknown)
- Aggressive denoising (3+ neighbors required)
- No Hough lines (they produce FP)
"""
import numpy as np, cv2, yaml, sys, os

def load(yp):
    with open(yp) as f: info = yaml.safe_load(f)
    pgm = info['image']
    if not os.path.isabs(pgm): pgm = os.path.join(os.path.dirname(yp), pgm)
    return cv2.imread(pgm, cv2.IMREAD_GRAYSCALE), info

def save(img, info, out):
    cv2.imwrite(out, img)
    yp = out.replace('.pgm','.yaml')
    info['image'] = os.path.abspath(out)
    with open(yp,'w') as f: yaml.dump(info, f)

def process(img):
    occupied = (img < 100).astype(np.uint8)
    free = (img > 200).astype(np.uint8)
    unknown = (img == 205).astype(np.uint8)

    # Step 1: Denoise — remove isolated cells (< 3 neighbors)
    kernel = np.ones((3,3), np.uint8); kernel[1,1] = 0
    nbr = cv2.filter2D(occupied, -1, kernel)
    clean = occupied.copy()
    clean[(occupied > 0) & (nbr < 3)] = 0
    print(f"  Denoise: removed {occupied.sum() - clean.sum()} isolated cells")

    # Step 1b: Remove drift artifacts — horizontal clusters wider than tall
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(clean, connectivity=8)
    clean2 = clean.copy()
    removed = 0
    for i in range(1, n_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        # Drift artifacts: horizontally elongated (w/h>3, h<40) OR small (<100 cells)
        aspect = w / max(h, 1)
        is_drift = (aspect > 3 and h < 40) or area < 100
        if is_drift:
            clean2[labels == i] = 0
            removed += area
    clean = clean2
    print(f"  Drift+cluster filter: removed {removed} cells ({n_labels-1} clusters)")

    # Step 2: Close 1px wall gaps
    closed = cv2.morphologyEx(clean, cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2)))

    # Step 3: Fill white-gray boundaries EXCEPT narrow passages
    free_dil = cv2.dilate(free, np.ones((3,3), np.uint8))
    boundary = (unknown > 0) & (free_dil > 0)
    # Require occupied structure nearby
    occ_dil_3 = cv2.dilate(clean, np.ones((7,7), np.uint8))
    boundary = boundary & occ_dil_3
    # Exclude narrow passages: doorways 1-4 cells wide, flanked by free
    narrow_gap = np.zeros_like(free, dtype=bool)
    for d in [1, 2, 3, 4]:
        fl = np.roll(free, d, axis=1); fr = np.roll(free, -d, axis=1)
        fu = np.roll(free, d, axis=0); fd = np.roll(free, -d, axis=0)
        narrow_gap = narrow_gap | (fl & fr) | (fu & fd)
    boundary_fill = boundary & ~narrow_gap
    print(f"  Boundary fill: {boundary_fill.sum()} cells (skipped {boundary.sum() - boundary_fill.sum()} narrow gaps)")

    # Step 3b: Manual drift artifact mask (user-identified regions)
    # These are VIO drift residuals that appear as black blobs outside walls
    drift_mask = np.zeros_like(closed)
    drift_mask[36:58, 139:298] = 1  # upper horizontal stripe
    removed = (closed & drift_mask).sum()
    closed = closed & ~drift_mask
    print(f"  Manual drift mask: removed {removed} cells")

    # Step 4: Combine
    result = np.maximum(closed, boundary_fill.astype(np.uint8))
    out = np.full_like(img, 254, np.uint8)
    out[unknown > 0] = 205
    out[result > 0] = 0
    return out

if __name__ == '__main__':
    yp = sys.argv[1] if len(sys.argv) > 1 else 'map.yaml'
    out = sys.argv[2] if len(sys.argv) > 2 else yp.replace('.yaml','_v5.pgm')
    img, info = load(yp)
    occ_b = (img < 100).sum()
    print(f"Input: {img.shape[1]}x{img.shape[0]}, occ={occ_b}")
    result = process(img)
    occ_a = (result < 100).sum()
    print(f"Output: occ={occ_a} (+{occ_a - occ_b})")
    save(result, info, out)
