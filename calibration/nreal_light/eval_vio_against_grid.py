#!/usr/bin/env python3
"""Compare Basalt relative motion with AprilGrid PnP on a recorded dataset.

The screen AprilGrid is static, so solvePnP on cam0 gives the camera motion
between two frames. For every frame pair (i, i + step) with the board in view,
the Basalt trajectory (T_world_imu) is turned into cam0 motion with the
calibration's own T_imu_cam0 and compared with the PnP motion. World frame and
origin cancel out. Prints rotation and translation error statistics.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def board_points(cols: int, rows: int, tag_m: float, spacing: float) -> dict[int, np.ndarray]:
    """Tag corners in aruco order, board frame x right, y down, z into the screen.

    The Kalibr board draws tag codes rotated by 180 degrees, so aruco's first
    corner is the bottom-right corner of the tag as seen on the screen. Row 0
    is the bottom row.
    """
    pitch = tag_m * (1 + spacing)
    pts = {}
    for r in range(rows):
        for c in range(cols):
            left = c * pitch
            top = (rows - 1 - r) * pitch
            s = tag_m
            pts[cols * r + c] = np.array([[left + s, top + s, 0], [left, top + s, 0], [left, top, 0], [left + s, top, 0]])
    return pts


def load_traj(path: Path) -> tuple[np.ndarray, list[np.ndarray]]:
    ts, Ts = [], []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#") or len(row) < 8:
                continue
            # capture_basalt_backend writes timestamp, px, py, pz, qw, qx, qy, qz.
            v = [float(x) for x in row[1:8]]
            ts.append(int(row[0]))
            Ts.append(make_T(quat_to_R(v[4], v[5], v[6], v[3]), v[:3]))
    return np.array(ts), Ts


def rot_angle_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--target", type=Path, required=True, help="Kalibr aprilgrid YAML with the measured tag size")
    ap.add_argument("--gt-calib", type=Path, required=True, help="Basalt calibration whose cam0 intrinsics run PnP")
    ap.add_argument("--traj", type=Path, nargs="+", required=True)
    ap.add_argument("--calib", type=Path, nargs="+", required=True, help="Basalt calibration of each trajectory")
    ap.add_argument("--step", type=int, default=10, help="frame distance of compared pairs")
    ap.add_argument("--min-tags", type=int, default=8)
    ap.add_argument("--max-reproj-px", type=float, default=1.0)
    args = ap.parse_args()
    if len(args.traj) != len(args.calib):
        raise SystemExit("--traj and --calib need the same count")

    target = {}
    for line in args.target.read_text().splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            target[k.strip()] = v.strip().strip("'")
    pts = board_points(int(target["tagCols"]), int(target["tagRows"]), float(target["tagSize"]), float(target["tagSpacing"]))

    gt = json.loads(args.gt_calib.read_text())["value0"]["intrinsics"][0]["intrinsics"]
    K = np.array([[gt["fx"], 0, gt["cx"]], [0, gt["fy"], gt["cy"]], [0, 0, 1]])
    dist = np.array([gt["k1"], gt["k2"], gt["p1"], gt["p2"], gt.get("k3", 0.0)])

    params = cv2.aruco.DetectorParameters()
    params.markerBorderBits = 2
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)

    frames = []
    with open(args.dataset / "camera_timestamps.csv", newline="") as f:
        for row in csv.DictReader(f):
            frames.append((int(row["timestamp_ns"]), row["cam0_file"]))

    pnp: dict[int, np.ndarray] = {}
    for idx, (ts, rel) in enumerate(frames):
        img = cv2.imread(str(args.dataset / rel), cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = detector.detectMarkers(img)
        if ids is None or len(ids) < args.min_tags:
            continue
        obj = np.concatenate([pts[int(i)] for i in ids.flatten() if int(i) in pts])
        imgp = np.concatenate([c.reshape(4, 2) for c, i in zip(corners, ids.flatten()) if int(i) in pts])
        # Screen glow shrinks the black squares, so edge-fitted tag corners
        # sit about 2 px inside. The X junction with the corner square does
        # not move; refine onto it like Kalibr does.
        imgp = cv2.cornerSubPix(img, imgp.astype(np.float32).reshape(-1, 1, 2), (7, 7), (-1, -1),
                                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)).reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgp) ** 2, axis=1))))
        if err > args.max_reproj_px:
            continue
        T_c_b = make_T(cv2.Rodrigues(rvec)[0], tvec)
        pnp[idx] = np.linalg.inv(T_c_b)  # T_board_cam
    print(f"PnP frames: {len(pnp)} of {len(frames)}")

    for traj_path, calib_path in zip(args.traj, args.calib):
        p = json.loads(calib_path.read_text())["value0"]["T_imu_cam"][0]
        T_i_c = make_T(quat_to_R(p["qx"], p["qy"], p["qz"], p["qw"]), [p["px"], p["py"], p["pz"]])
        ts, Ts = load_traj(traj_path)
        rot_err, trans_err, trans_gt, scale = [], [], [], []
        for i in sorted(pnp):
            j = i + args.step
            if j not in pnp:
                continue
            ki = np.searchsorted(ts, frames[i][0])
            kj = np.searchsorted(ts, frames[j][0])
            if ki >= len(ts) or kj >= len(ts):
                continue
            if abs(ts[ki] - frames[i][0]) > 2_000_000 or abs(ts[kj] - frames[j][0]) > 2_000_000:
                continue
            rel_vio = np.linalg.inv(T_i_c) @ np.linalg.inv(Ts[ki]) @ Ts[kj] @ T_i_c
            rel_gt = np.linalg.inv(pnp[i]) @ pnp[j]
            d = np.linalg.inv(rel_gt) @ rel_vio
            rot_err.append(rot_angle_deg(d[:3, :3]))
            trans_err.append(float(np.linalg.norm(rel_vio[:3, 3] - rel_gt[:3, 3])))
            trans_gt.append(float(np.linalg.norm(rel_gt[:3, 3])))
            if trans_gt[-1] > 0.03:
                scale.append(float(np.linalg.norm(rel_vio[:3, 3])) / trans_gt[-1])
        if not rot_err:
            print(f"{traj_path.name}: no matched pairs")
            continue
        r = np.array(rot_err)
        t = np.array(trans_err) * 1000
        g = np.array(trans_gt) * 1000
        print(f"{traj_path.name}: pairs={len(r)} step={args.step} "
              f"rot_err_deg mean={r.mean():.3f} median={np.median(r):.3f} p90={np.percentile(r, 90):.3f}  "
              f"trans_err_mm mean={t.mean():.2f} median={np.median(t):.2f} p90={np.percentile(t, 90):.2f}  "
              f"gt_motion_mm median={np.median(g):.1f} rel_trans_err={t.sum() / g.sum() * 100:.1f}%  "
              f"scale_vio/gt median={np.median(scale) if scale else float('nan'):.3f}")


if __name__ == "__main__":
    main()
