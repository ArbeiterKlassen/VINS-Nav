#!/usr/bin/env python3
"""go1_stereo_calibrate.py — Go1 双目标定（用录制视频/图像对直接标定）。

输入（二选一）:
  --bag  BAG     录制得到的 rosbag（含 /go1/cam0/image_raw、/go1/cam1/image_raw）
  --dir  DIR     接收端保存的 PNG 对目录（left_*.png/right_*.png 或 rect_*.png）
模型:
  --model fisheye  原始鱼眼图像（推荐；cv2.fisheye / Kannala-Brandt）→ 输出可映射到
                   OpenVINS `equidistant`；同时给出校正后的 pinhole 参数
  --model pinhole  已校正图像（radtan；接收端保存的 rect_* 默认 180° 倒置，用
                   --rotate180 yes 校正）
标定板: --cols 9 --rows 6（内角点） --square 0.020（米）

输出（--out 目录）:
  stereo_calib.yaml        K/D（两目）、R/T、基线、RMS、板参数、帧数
  rectified_camera_info.yaml  校正后 pinhole 参数（fx/fy/cx/cy、基线）——可直接更新
                              接收端 camera_info 的估计值
  openvins_imucam_chain.yaml  OpenVINS/Kalibr 风格的草稿（T_imu_cam 未知，占位 TODO）
  rectified_check.png     校正效果检查图（左右目 + 水平极线）
  report.txt              摘要与质量指标
  calib_pairs/            参与标定的图像对（少量抽样，便于复查）

用法示例:
  python3 go1_stereo_calibrate.py --selftest
  python3 go1_stereo_calibrate.py --bag ~/go1_recordings/calib.bag --model fisheye \
      --cols 9 --rows 6 --square 0.020 --out ~/go1_recordings/calib_fisheye
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2
import numpy as np


# ---------------------------------------------------------------------------
def load_pairs_from_bag(path: str, max_pairs: int):
    import rosbag
    from sensor_msgs.msg import Image

    bag = rosbag.Bag(path)
    left, right = {}, {}
    n = 0
    for topic, msg, _ in bag.read_messages(topics=["/go1/cam0/image_raw", "/go1/cam1/image_raw"]):
        key = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if topic.endswith("cam0/image_raw"):
            left[key] = img.copy()
        else:
            right[key] = img.copy()
        n += 1
        if len(left) >= max_pairs and len(right) >= max_pairs:
            break
    bag.close()
    pairs = [(left[k], right[k]) for k in sorted(set(left) & set(right))]
    return pairs


def load_pairs_from_dir(path: str, max_pairs: int):
    globs = [("left_*.png", "right_*.png"),
             ("rect_left_*.png", "rect_right_*.png"),
             ("raw_left_*.png", "raw_right_*.png")]
    for lg, rg in globs:
        ls = sorted(glob.glob(os.path.join(path, lg)))
        rs = sorted(glob.glob(os.path.join(path, rg)))
        if ls and len(ls) == len(rs):
            out = []
            for lp, rp in list(zip(ls, rs))[:max_pairs]:
                l = cv2.imread(lp)
                r = cv2.imread(rp)
                if l is not None and r is not None:
                    out.append((l, r))
            return out
    return []


def detect_board(gray, cols, rows, flags):
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (7, 7), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    return ok, corners


# ---------------------------------------------------------------------------
def selftest() -> int:
    """用已知参数的合成鱼眼双目数据验证标定求解链路（不依赖相机）。"""
    size = (928, 800)
    K0 = np.array([[300.0, 0, 464.0], [0, 300.0, 400.0], [0, 0, 1]])
    D0 = np.array([0.06, -0.03, 0.008, -0.002])
    K1 = np.array([[305.0, 0, 466.0], [0, 305.0, 398.0], [0, 0, 1]])
    D1 = np.array([0.05, -0.025, 0.006, -0.0015])
    ang = np.deg2rad(2.0)
    R_gt = cv2.Rodrigues(np.array([0.0, ang, 0.0]))[0]
    T_gt = np.array([-0.02509, 0.0, 0.0])

    cols, rows, sq = 9, 6, 0.02
    objp = np.zeros((1, cols * rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq

    rng = np.random.default_rng(1)
    obj_list, pts0, pts1 = [], [], []
    for i in range(18):
        rvec = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.6, 0.6)])
        tvec = np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), rng.uniform(0.35, 0.9)])
        R_b = cv2.Rodrigues(rvec)[0]
        t_b1 = R_gt @ tvec + T_gt
        R_b1 = R_gt @ R_b
        rvec1 = cv2.Rodrigues(R_b1)[0]
        p0, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, K0, D0)
        p1, _ = cv2.fisheye.projectPoints(objp, rvec1, t_b1, K1, D1)
        p0 = p0.reshape(-1, 2) + rng.normal(0, 0.15, (cols * rows, 2))
        p1 = p1.reshape(-1, 2) + rng.normal(0, 0.15, (cols * rows, 2))
        obj_list.append(objp.copy())
        pts0.append(p0.reshape(1, -1, 2).astype(np.float32))
        pts1.append(p1.reshape(1, -1, 2).astype(np.float32))

    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    K0e = np.eye(3); D0e = np.zeros((4, 1)); K1e = np.eye(3); D1e = np.zeros((4, 1))
    rms0, K0e, D0e, _, _ = cv2.fisheye.calibrate(
        obj_list, pts0, size, K0e, D0e,
        flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
        criteria=crit)
    rms1, K1e, D1e, _, _ = cv2.fisheye.calibrate(
        obj_list, pts1, size, K1e, D1e,
        flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
        criteria=crit)
    rms, K0e, D0e, K1e, D1e, R_e, T_e = cv2.fisheye.stereoCalibrate(
        obj_list, pts0, pts1, K0e, D0e, K1e, D1e, size,
        flags=cv2.fisheye.CALIB_FIX_INTRINSIC, criteria=crit)

    fx_err = abs(K0e[0, 0] - K0[0, 0]) / K0[0, 0]
    base_err = abs(np.linalg.norm(T_e) - np.linalg.norm(T_gt)) / np.linalg.norm(T_gt)
    ang_err = np.rad2deg(np.arccos(np.clip((np.trace(R_e.T @ R_gt) - 1) / 2, -1, 1)))
    print("== selftest (synthetic fisheye stereo) ==")
    print(f"  rms mono: {rms0:.3f} / {rms1:.3f} px; stereo rms: {rms:.3f} px")
    print(f"  recovered K0 fx={K0e[0,0]:.2f} (gt {K0[0,0]:.2f}, err {fx_err*100:.2f}%)")
    print(f"  baseline: {np.linalg.norm(T_e)*1000:.2f} mm (gt {np.linalg.norm(T_gt)*1000:.2f} mm, err {base_err*100:.2f}%)")
    print(f"  relative rotation error: {ang_err:.2f} deg")
    ok = (rms < 1.0 and fx_err < 0.02 and base_err < 0.05 and ang_err < 1.0)
    print("SELFTEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--bag", default="")
    ap.add_argument("--dir", default="")
    ap.add_argument("--model", choices=["fisheye", "pinhole"], default="fisheye")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.020, help="方格边长（米）")
    ap.add_argument("--rotate180", choices=["auto", "yes", "no"], default="auto",
                    help="校正图（rect_* 或来自 rect 推流的 bag）是倒置的，需旋转")
    ap.add_argument("--max-pairs", type=int, default=300)
    ap.add_argument("--out", default="calib_out")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.bag:
        pairs = load_pairs_from_bag(args.bag, args.max_pairs)
    elif args.dir:
        pairs = load_pairs_from_dir(args.dir, args.max_pairs)
    else:
        print("need --bag or --dir (or --selftest)")
        return 2
    if not pairs:
        print("no stereo pairs loaded")
        return 1
    print(f"loaded {len(pairs)} stereo pairs")

    rotate = args.rotate180 == "yes" or (args.rotate180 == "auto" and args.model == "pinhole")

    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    objp = np.zeros((1, args.cols * args.rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    obj_list, pts0, pts1, sample_pairs = [], [], [], []
    for idx, (l, r) in enumerate(pairs):
        if rotate:
            l, r = cv2.rotate(l, cv2.ROTATE_180), cv2.rotate(r, cv2.ROTATE_180)
        g0 = cv2.cvtColor(l, cv2.COLOR_BGR2GRAY)
        g1 = cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
        ok0, c0 = detect_board(g0, args.cols, args.rows, flags)
        ok1, c1 = detect_board(g1, args.cols, args.rows, flags)
        if ok0 and ok1:
            obj_list.append(objp.copy())
            pts0.append(c0.reshape(1, -1, 2).astype(np.float32))
            pts1.append(c1.reshape(1, -1, 2).astype(np.float32))
            sample_pairs.append((l, r))
        if (idx + 1) % 50 == 0:
            print(f"  scanned {idx+1}/{len(pairs)}, detections={len(obj_list)}")
    print(f"detections (both eyes): {len(obj_list)}")
    if len(obj_list) < 8:
        print("ERROR: 有效帧太少（<8）——请确保整块板同时出现在左右目、覆盖画面各处")
        return 1

    size = (pairs[0][0].shape[1], pairs[0][0].shape[0])
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    os.makedirs(args.out, exist_ok=True)

    if args.model == "fisheye":
        K0 = np.eye(3); D0 = np.zeros((4, 1)); K1 = np.eye(3); D1 = np.zeros((4, 1))
        rms0, K0, D0, _, _ = cv2.fisheye.calibrate(
            obj_list, pts0, size, K0, D0,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
            criteria=crit)
        rms1, K1, D1, _, _ = cv2.fisheye.calibrate(
            obj_list, pts1, size, K1, D1,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
            criteria=crit)
        rms, K0, D0, K1, D1, R, T = cv2.fisheye.stereoCalibrate(
            obj_list, pts0, pts1, K0, D0, K1, D1, size,
            flags=cv2.fisheye.CALIB_FIX_INTRINSIC, criteria=crit)
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K0, D0, K1, D1, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, balance=0.0, fov_scale=1.0)
        model_name = "fisheye(Kannala-Brandt)"
        dist0 = D0.reshape(-1).tolist()
        dist1 = D1.reshape(-1).tolist()
        max_d = 4
    else:
        rms0, K0, D0, _, _ = cv2.calibrateCamera(obj_list, pts0, size, None, None, criteria=crit)
        rms1, K1, D1, _, _ = cv2.calibrateCamera(obj_list, pts1, size, None, None, criteria=crit)
        rms, K0, D0, K1, D1, R, T, E, F = cv2.stereoCalibrate(
            obj_list, pts0, pts1, K0, D0, K1, D1, size,
            flags=cv2.CALIB_FIX_INTRINSIC, criteria=crit)
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            K0, D0, K1, D1, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0)
        model_name = "pinhole(radtan)"
        dist0 = D0.reshape(-1).tolist()
        dist1 = D1.reshape(-1).tolist()
        max_d = 5

    baseline = float(np.linalg.norm(T))
    fx_rect = float(P1[0, 0]); fy_rect = float(P1[1, 1])
    cx_rect = float(P1[0, 2]); cy_rect = float(P1[1, 2])

    # 校正效果检查图（鱼眼模型用 fisheye 的 remap）
    l0, r0 = sample_pairs[0]
    if args.model == "fisheye" and hasattr(cv2.fisheye, "initUndistortRectifyMap"):
        m1a, m1b = cv2.fisheye.initUndistortRectifyMap(K0, D0, R1, P1, size, cv2.CV_16SC2)
        m2a, m2b = cv2.fisheye.initUndistortRectifyMap(K1, D1, R2, P2, size, cv2.CV_16SC2)
    else:
        m1a, m1b = cv2.initUndistortRectifyMap(K0, D0, R1, P1, size, cv2.CV_16SC2)
        m2a, m2b = cv2.initUndistortRectifyMap(K1, D1, R2, P2, size, cv2.CV_16SC2)
    lr = cv2.remap(l0, m1a, m1b, cv2.INTER_LINEAR)
    rr = cv2.remap(r0, m2a, m2b, cv2.INTER_LINEAR)
    vis = cv2.hconcat([lr, rr])
    for y in range(0, vis.shape[0], vis.shape[0] // 10):
        cv2.line(vis, (0, y), (vis.shape[1], y), (0, 255, 0), 1)
    cv2.imwrite(os.path.join(args.out, "rectified_check.png"), vis)

    # 输出 YAML / 报告
    def fmt_mat(m):
        return np.array2string(np.asarray(m), precision=6, separator=", ", max_line_width=200).replace("\n", " ")

    with open(os.path.join(args.out, "stereo_calib.yaml"), "w") as f:
        f.write("model: %s\n" % model_name)
        f.write("image_width: %d\nimage_height: %d\n" % size)
        f.write("board: {cols: %d, rows: %d, square_m: %.6f}\n" % (args.cols, args.rows, args.square))
        f.write("frames_used: %d\n" % len(obj_list))
        f.write("rms_mono: [%.4f, %.4f]\nrms_stereo: %.4f\n" % (rms0, rms1, rms))
        f.write("cam0_K: %s\ncam0_D: %s\n" % (fmt_mat(K0), fmt_mat(D0)))
        f.write("cam1_K: %s\ncam1_D: %s\n" % (fmt_mat(K1), fmt_mat(D1)))
        f.write("R: %s\nT: %s\n" % (fmt_mat(R), fmt_mat(T)))
        f.write("baseline_m: %.6f\n" % baseline)
        f.write("rectified_K: [%.4f, %.4f, %.4f, %.4f]\n" % (fx_rect, fy_rect, cx_rect, cy_rect))

    with open(os.path.join(args.out, "rectified_camera_info.yaml"), "w") as f:
        f.write("# 已校正图像的 pinhole 参数（可直接替换接收端 camera_info 的估计值）\n")
        f.write("image_width: %d\nimage_height: %d\n" % size)
        f.write("fx: %.6f\nfy: %.6f\ncx: %.6f\ncy: %.6f\n" % (fx_rect, fy_rect, cx_rect, cy_rect))
        f.write("baseline_m: %.6f\n" % baseline)

    with open(os.path.join(args.out, "openvins_imucam_chain.yaml"), "w") as f:
        f.write("# OpenVINS/Kalibr 草稿。T_imu_cam 尚未标定（相机-IMU 外参），先用单位阵占位；\n")
        f.write("# 需要 IMU-相机外参时可以：①用 Kalibr 标定，或 ②在 OpenVINS 中开启在线外参估计。\n")
        f.write("cam0:\n  T_imu_cam: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]\n")
        f.write("  camera_model: pinhole\n  distortion_model: %s\n" %
                ("equidistant" if args.model == "fisheye" else "radtan"))
        f.write("  intrinsics: [%.6f, %.6f, %.6f, %.6f]\n" % (K0[0, 0], K0[1, 1], K0[0, 2], K0[1, 2]))
        f.write("  distortion_coeffs: %s\n" % dist0[:max_d])
        f.write("  resolution: [%d, %d]\n  rostopic: /go1/cam0/image_raw\n" % size)
        f.write("cam1:\n  T_imu_cam: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]\n")
        f.write("  camera_model: pinhole\n  distortion_model: %s\n" %
                ("equidistant" if args.model == "fisheye" else "radtan"))
        f.write("  intrinsics: [%.6f, %.6f, %.6f, %.6f]\n" % (K1[0, 0], K1[1, 1], K1[0, 2], K1[1, 2]))
        f.write("  distortion_coeffs: %s\n" % dist1[:max_d])
        f.write("  resolution: [%d, %d]\n  rostopic: /go1/cam1/image_raw\n" % size)

    with open(os.path.join(args.out, "report.txt"), "w") as f:
        f.write("model: %s\n" % model_name)
        f.write("pairs: %d, detections: %d\n" % (len(pairs), len(obj_list)))
        f.write("rms mono: %.4f / %.4f px\n" % (rms0, rms1))
        f.write("rms stereo: %.4f px\n" % rms)
        f.write("baseline: %.2f mm\n" % (baseline * 1000))
        f.write("rectified K: fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n" % (fx_rect, fy_rect, cx_rect, cy_rect))

    # 抽样保存参与标定的图像对
    sp = os.path.join(args.out, "calib_pairs")
    os.makedirs(sp, exist_ok=True)
    for i, (l, r) in enumerate(sample_pairs[:5]):
        cv2.imwrite(os.path.join(sp, f"pair_{i:02d}_left.png"), l)
        cv2.imwrite(os.path.join(sp, f"pair_{i:02d}_right.png"), r)

    print("== calibration ==")
    print(f"  model: {model_name}; detections: {len(obj_list)}")
    print(f"  mono rms: {rms0:.3f} / {rms1:.3f} px; stereo rms: {rms:.3f} px")
    print(f"  baseline: {baseline*1000:.2f} mm")
    print(f"  rectified K: fx={fx_rect:.2f} fy={fy_rect:.2f} cx={cx_rect:.2f} cy={cy_rect:.2f}")
    print(f"  outputs -> {args.out}/")
    return 0 if rms < 2.0 else 1


if __name__ == "__main__":
    sys.exit(main())
