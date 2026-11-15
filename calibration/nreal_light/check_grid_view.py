#!/usr/bin/env python3
"""Live AprilGrid check on the capture_service_cpp stereo streams.

Detects t36h11 tags with 2 border bits, the layout of Kalibr AprilGrids, in
camera0 and camera1 and prints the number of tags, image brightness and the
board coverage of each camera. With --show, opens a window with both images
and the detections. With --save-dir, writes the latest pair as PNG on exit.
With --beep-min-tags, the Mac plays a short sound for every detection with at
least that many tags in camera0, so the board view can be held without
looking at the terminal.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from capture_client.client import CaptureClient  # noqa: E402


def to_gray(frame) -> np.ndarray:
    return np.frombuffer(frame.data, dtype=np.uint8, count=frame.width * frame.height).reshape(frame.height, frame.width)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--registry", default="/tmp/capture_service_streams.json")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this many seconds, 0 runs until Ctrl+C")
    ap.add_argument("--rate", type=float, default=5.0, help="detections per second")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--save-dir", type=Path)
    ap.add_argument("--beep-min-tags", type=int, default=0, help="0 disables the sound")
    ap.add_argument("--beep-sound", default="/System/Library/Sounds/Tink.aiff")
    args = ap.parse_args()

    params = cv2.aruco.DetectorParameters()
    params.markerBorderBits = 2
    # A laptop screen loses contrast at oblique angles; wider threshold windows
    # and bit tolerance keep those tags detectable.
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.errorCorrectionRate = 0.8
    params.maxErroneousBitsInBorderRate = 0.5
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
    client = CaptureClient.from_shm_registry(args.registry, required_streams=["camera0", "camera1"])
    start = time.monotonic()
    last_seq = -1
    images: list[np.ndarray] = []
    try:
        while args.seconds <= 0 or time.monotonic() - start < args.seconds:
            seq = client.latest_sequence("camera0")
            if seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq
            frames = [client.read_image_sequence(s, seq) for s in ("camera0", "camera1")]
            if any(f is None for f in frames):
                continue
            images = [to_gray(f).copy() for f in frames]
            parts = []
            views = []
            for name, img in zip(("cam0", "cam1"), images):
                corners, ids, _ = detector.detectMarkers(img)
                n = 0 if ids is None else len(ids)
                if name == "cam0" and 0 < args.beep_min_tags <= n:
                    subprocess.Popen(["afplay", args.beep_sound])
                cover = 0.0
                if n:
                    pts = np.concatenate([c.reshape(-1, 2) for c in corners])
                    x0, y0 = pts.min(0)
                    x1, y1 = pts.max(0)
                    cover = (x1 - x0) * (y1 - y0) / (img.shape[0] * img.shape[1])
                parts.append(f"{name} tags={n:2d} mean={img.mean():5.1f} sat={np.mean(img >= 250) * 100:4.1f}% cover={cover * 100:4.0f}%")
                if args.show:
                    view = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    if n:
                        cv2.aruco.drawDetectedMarkers(view, corners, ids)
                    views.append(view)
            print(f"seq={seq} " + "  ".join(parts), flush=True)
            if args.show:
                cv2.imshow("nreal light aprilgrid", np.hstack(views))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            time.sleep(max(0.0, 1.0 / args.rate - 0.01))
    except KeyboardInterrupt:
        pass
    finally:
        if args.save_dir and images:
            args.save_dir.mkdir(parents=True, exist_ok=True)
            for name, img in zip(("cam0", "cam1"), images):
                cv2.imwrite(str(args.save_dir / f"grid_view_{name}.png"), img)
            print(f"saved {args.save_dir}")
        client.close()


if __name__ == "__main__":
    main()
