#!/usr/bin/env python3
"""
Post-process occupancy grid map: fill gaps, smooth edges, detect walls.
Input:  .pgm + .yaml map file
Output: processed .pgm
"""
import numpy as np
import cv2
import yaml
import sys
import os
def load_map(yaml_path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    pgm_path = info['image']
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise IOError("Cannot read %s" % pgm_path)
    # Convert ROS occupancy grid values:
    # -1 (unknown)=205, 0 (free)=254, 100 (occupied)=0
    # Invert to standard: 0=unknown, 128=free, 255=occupied
    # Actually PGM stores 0-255. ROS map_saver uses:
    #   0 = occupied, 254 = free, 205 = unknown
    return img, info

def save_map(img, info, output_path):
    cv2.imwrite(output_path, img)
    # Also save yaml
    yaml_path = output_path.replace('.pgm', '.yaml')
    info['image'] = os.path.abspath(output_path)
    with open(yaml_path, 'w') as f:
        yaml.dump(info, f)
    print("Saved: %s, %s" % (output_path, yaml_path))

def process_map(img):
    """Clean up occupancy grid: remove noise only. Preserve doorframes and passages."""
    h, w = img.shape

    # ROS map_saver convention: 0=occupied(black), 254=free(white), 205=unknown(gray)
    occupied = (img < 100).astype(np.uint8) * 255
    free = ((img > 200) & (img < 255)).astype(np.uint8) * 255
    unknown = ((img > 100) & (img < 200)).astype(np.uint8) * 255

    # Step 1: Remove isolated noise pixels (< 3 occupied neighbors)
    occupied_neighbors = cv2.filter2D(occupied, -1, np.ones((3,3))/255).astype(np.uint8)
    occupied_clean = occupied.copy()
    occupied_clean[(occupied > 0) & (occupied_neighbors < 3)] = 0

    # Step 2: Minimal close (1px) to fill tiny sensor gaps only
    kernel_1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
    occupied_final = cv2.morphologyEx(occupied_clean, cv2.MORPH_CLOSE, kernel_1)

    # Step 3: Reconstruct output — keep original free/unknown, only clean occupied
    output = img.copy()
    output[occupied_final > 128] = 0        # cleaned occupied
    output[(occupied_final == 0) & (img < 100)] = 254  # removed noise → free

    return output

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: postprocess_map.py <map.yaml> [output.pgm]")
        sys.exit(1)

    yaml_path = sys.argv[1]
    output = sys.argv[2] if len(sys.argv) > 2 else yaml_path.replace('.yaml', '_processed.pgm')

    img, info = load_map(yaml_path)
    print("Input: %dx%d" % (img.shape[1], img.shape[0]))
    print("  Occupied: %d, Free: %d, Unknown: %d" % (
        (img < 100).sum(), ((img > 200) & (img < 255)).sum(),
        ((img > 100) & (img < 200)).sum()))

    result = process_map(img)
    save_map(result, info, output)
    print("Done.")
