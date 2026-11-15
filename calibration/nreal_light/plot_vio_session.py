#!/usr/bin/env python3
"""Plot a Basalt session in 3D against the static screen AprilGrid.

The session starts at the board, leaves it, and returns several times. PnP on
cam0 gives the true camera pose while the board is in view. The first board
segment fixes the transform from the Basalt world to the board; every later
return then shows how far Basalt drifted. The plot uses a gravity-up frame
with the origin at the board center and the board in the x-z plane.

Writes an interactive plotly HTML and a PNG overview, and prints the drift of
every board segment.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import plotly.graph_objects as go  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_vio_against_grid import (  # noqa: E402
    board_poses,
    load_frames,
    load_T_imu_cam0,
    load_target,
    load_traj,
    rot_angle_deg,
)


def mean_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    U, _, Vt = np.linalg.svd(sum(Rs))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def segments(times_s: np.ndarray, gap_s: float) -> list[np.ndarray]:
    """Split sorted board-frame times into runs separated by more than gap_s."""
    if len(times_s) == 0:
        return []
    cuts = np.where(np.diff(times_s) > gap_s)[0] + 1
    return np.split(np.arange(len(times_s)), cuts)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--target", type=Path, required=True)
    ap.add_argument("--calib", type=Path, required=True, help="Basalt calibration of the trajectory")
    ap.add_argument("--traj", type=Path, required=True, help="capture_basalt_backend --trajectory CSV")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--min-tags", type=int, default=4)
    ap.add_argument("--max-reproj-px", type=float, default=1.0)
    ap.add_argument("--segment-gap-s", type=float, default=3.0)
    ap.add_argument("--min-segment-frames", type=int, default=10)
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    frames = load_frames(args.dataset)
    pts = load_target(args.target)
    pnp = board_poses(args.dataset, frames, pts, args.calib, args.min_tags, args.max_reproj_px)
    T_i_c = load_T_imu_cam0(args.calib)
    ts, Ts = load_traj(args.traj)
    t0 = frames[0][0]

    # Board frames that have a Basalt pose at the same time.
    matched = []
    for idx in sorted(pnp):
        k = int(np.searchsorted(ts, frames[idx][0]))
        if k < len(ts) and abs(ts[k] - frames[idx][0]) <= 2_000_000:
            matched.append((idx, Ts[k] @ T_i_c))
    if not matched:
        raise SystemExit("no board frame has a Basalt pose")
    times = np.array([(frames[i][0] - t0) * 1e-9 for i, _ in matched])
    segs = [s for s in segments(times, args.segment_gap_s) if len(s) >= args.min_segment_frames]
    if len(segs) < 2:
        raise SystemExit(f"need at least two board segments, found {len(segs)}")

    # T_board_world from the first segment: average of T_board_cam * inv(T_world_cam).
    A = [pnp[matched[k][0]] @ np.linalg.inv(matched[k][1]) for k in segs[0]]
    R_bw = mean_rotation([a[:3, :3] for a in A])
    t_bw = np.mean([a[:3, 3] for a in A], axis=0)
    T_bw = np.eye(4)
    T_bw[:3, :3] = R_bw
    T_bw[:3, 3] = t_bw
    T_wb = np.linalg.inv(T_bw)

    # Display frame: Basalt world (z up), origin at the board center, yaw so that
    # the board x axis is the display x axis.
    xs = [p[:, 0] for p in pts.values()]
    ys = [p[:, 1] for p in pts.values()]
    corners_b = np.array([[min(map(min, xs)), min(map(min, ys)), 0], [max(map(max, xs)), min(map(min, ys)), 0],
                          [max(map(max, xs)), max(map(max, ys)), 0], [min(map(min, xs)), max(map(max, ys)), 0]])
    center_w = T_wb[:3, :3] @ corners_b.mean(axis=0) + T_wb[:3, 3]
    bx = T_wb[:3, 0]
    yaw = math.atan2(bx[1], bx[0])
    Rz = np.array([[math.cos(-yaw), -math.sin(-yaw), 0], [math.sin(-yaw), math.cos(-yaw), 0], [0, 0, 1]])

    def to_disp(p_w: np.ndarray) -> np.ndarray:
        return (Rz @ (np.asarray(p_w).T - center_w.reshape(3, 1))).T

    vio_w = np.array([(T @ T_i_c)[:3, 3] for T in Ts])
    vio = to_disp(vio_w)
    vio_t = (ts - t0) * 1e-9
    board = to_disp((T_wb[:3, :3] @ corners_b.T).T + T_wb[:3, 3])
    gt = to_disp(np.array([(T_wb @ pnp[i])[:3, 3] for i, _ in matched]))
    est = to_disp(np.array([T[:3, 3] for _, T in matched]))
    err_mm = np.linalg.norm(est - gt, axis=1) * 1000
    rot_err = np.array([rot_angle_deg((np.linalg.inv(T_wb @ pnp[i]) @ T)[:3, :3]) for i, T in matched])

    print(f"poses {len(ts)} over {vio_t[-1] - vio_t[0]:.1f} s, path {np.sum(np.linalg.norm(np.diff(vio, axis=0), axis=1)):.2f} m")
    print(f"board frames {len(matched)}, segments {len(segs)}; segment 1 defines the board-to-world transform")
    rows = []
    for n, s in enumerate(segs, 1):
        row = (n, times[s[0]], times[s[-1]], len(s), np.median(err_mm[s]), np.max(err_mm[s]), np.median(rot_err[s]))
        rows.append(row)
        print(f"segment {n}: {row[1]:5.1f}-{row[2]:5.1f} s frames={row[3]:4d} "
              f"position error median={row[4]:6.1f} mm max={row[5]:6.1f} mm rotation error median={row[6]:.2f} deg")

    # Interactive HTML.
    fig = go.Figure()
    fig.add_trace(go.Mesh3d(x=board[:, 0], y=board[:, 1], z=board[:, 2], i=[0, 0], j=[1, 2], k=[2, 3],
                            color="lightgray", opacity=0.8, name="screen board"))
    fig.add_trace(go.Scatter3d(x=vio[:, 0], y=vio[:, 1], z=vio[:, 2], mode="lines",
                               line=dict(color=vio_t, colorscale="Viridis", width=4,
                                         colorbar=dict(title="time s")),
                               name="Basalt cam0", text=[f"{t:.2f} s" for t in vio_t]))
    for n, s in enumerate(segs, 1):
        fig.add_trace(go.Scatter3d(x=gt[s, 0], y=gt[s, 1], z=gt[s, 2], mode="markers",
                                   marker=dict(size=2, color="red"), name=f"board PnP {n}"))
        fig.add_trace(go.Scatter3d(x=est[s, 0], y=est[s, 1], z=est[s, 2], mode="markers",
                                   marker=dict(size=2, color="blue"), name=f"Basalt at board {n}"))
        g, e = gt[s].mean(axis=0), est[s].mean(axis=0)
        fig.add_trace(go.Scatter3d(x=[g[0], e[0]], y=[g[1], e[1]], z=[g[2], e[2]], mode="lines+text",
                                   line=dict(color="black", width=6), text=["", f"{rows[n - 1][4]:.0f} mm"],
                                   name=f"drift {n}"))
    fig.update_layout(scene=dict(aspectmode="data", xaxis_title="x m (board right)",
                                 yaxis_title="y m (horizontal)", zaxis_title="z m (up)"),
                      title=f"{args.dataset.name}: Basalt vs screen board PnP")
    html = args.out_dir / "session3d.html"
    fig.write_html(str(html), include_plotlyjs=True)

    # PNG overview.
    figp = plt.figure(figsize=(15, 10))
    ax = figp.add_subplot(2, 2, 1, projection="3d")
    ax.plot(*board[[0, 1, 2, 3, 0]].T, color="gray")
    sc = ax.scatter(*vio[::3].T, c=vio_t[::3], cmap="viridis", s=1)
    for s in segs:
        ax.scatter(*gt[s].T, color="red", s=3)
    ax.set_xlabel("x m")
    ax.set_ylabel("y m")
    ax.set_zlabel("z m (up)")
    ax.set_title("3D, red = board PnP")
    extent = np.ptp(np.vstack([vio, board]), axis=0)
    ax.set_box_aspect(np.maximum(extent, 1e-3))
    figp.colorbar(sc, ax=ax, shrink=0.6, label="time s")
    for k, (a, b, name) in enumerate([(0, 1, "top view, x-y"), (0, 2, "front view, x-z")], start=2):
        axp = figp.add_subplot(2, 2, k)
        axp.plot(board[[0, 1, 2, 3, 0], a], board[[0, 1, 2, 3, 0], b], color="gray", label="board")
        axp.scatter(vio[::3, a], vio[::3, b], c=vio_t[::3], cmap="viridis", s=1)
        for n, s in enumerate(segs, 1):
            axp.scatter(gt[s, a], gt[s, b], color="red", s=4, label="board PnP" if n == 1 else None)
            axp.scatter(est[s, a], est[s, b], color="blue", s=4, label="Basalt at board" if n == 1 else None)
        axp.set_aspect("equal", adjustable="datalim")
        axp.set_xlabel("xyz"[a] + " m")
        axp.set_ylabel("xyz"[b] + " m")
        axp.set_title(name)
        axp.legend(loc="best", fontsize=8)
    axe = figp.add_subplot(2, 2, 4)
    axe.plot(times, err_mm, ".", ms=3, color="black")
    axe.set_xlabel("time s")
    axe.set_ylabel("position error vs board PnP mm")
    axe.set_title("drift at each return; segment 1 is the anchor")
    axe.grid(True)
    figp.tight_layout()
    png = args.out_dir / "session3d.png"
    figp.savefig(png, dpi=110)
    print(f"wrote {html}\nwrote {png}")


if __name__ == "__main__":
    main()
