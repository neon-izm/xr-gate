#!/usr/bin/env python3
"""Replay a recorded stereo+IMU dataset into capture-service SHM streams.

Reads a dataset written by calibration/common/linux/record_unified_kalibr_dataset.py
(camera_timestamps.csv, cam0/cam1 PNGs, imu.csv) and publishes camera0,
camera1 and imu0 with the recorded device timestamps, paced in real time, so
that capture_basalt_backend can run on identical input with different
calibrations.
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_client.shm_ring import ShmRingPublisher, StreamRegistry  # noqa: E402
from capture_client.stream_defs import FormatCode, StreamKind, StreamSpec  # noqa: E402


def load_events(dataset: Path, skip_s: float, limit_s: float) -> list[tuple[int, str, object]]:
    events: list[tuple[int, str, object]] = []
    with open(dataset / "camera_timestamps.csv", newline="") as f:
        for row in csv.DictReader(f):
            events.append((int(row["timestamp_ns"]), "cam", (row["cam0_file"], row["cam1_file"])))
    imu_path = dataset / "imu.csv"
    if imu_path.exists():
        with open(imu_path, newline="") as f:
            for row in csv.DictReader(f):
                values = tuple(float(row[k]) for k in ("gx", "gy", "gz", "ax", "ay", "az"))
                events.append((int(row["timestamp_ns"]), "imu", values))
    # IMU first at equal timestamps so the frame's IMU window is complete.
    events.sort(key=lambda e: (e[0], e[1] == "cam"))
    t0 = events[0][0] + int(skip_s * 1e9)
    t1 = t0 + int(limit_s * 1e9) if limit_s > 0 else None
    return [e for e in events if e[0] >= t0 and (t1 is None or e[0] < t1)]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--registry", default="/tmp/capture_service_streams.json")
    ap.add_argument("--namespace", default="replay")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed, 1.0 is real time")
    ap.add_argument("--skip-s", type=float, default=0.0)
    ap.add_argument("--limit-s", type=float, default=0.0)
    ap.add_argument("--start-delay-s", type=float, default=2.0, help="wait after publishing the registry")
    args = ap.parse_args()

    events = load_events(args.dataset, args.skip_s, args.limit_s)
    first = cv2.imread(str(args.dataset / events[next(i for i, e in enumerate(events) if e[1] == "cam")][2][0]),
                       cv2.IMREAD_GRAYSCALE)
    h, w = first.shape
    pubs = {
        name: ShmRingPublisher(StreamSpec(name, StreamKind.IMAGE, w, h, int(FormatCode.GRAY8), w * h, 8, name,
                                          f"replay of {args.dataset.name}"), namespace=args.namespace)
        for name in ("camera0", "camera1")
    }
    pubs["imu0"] = ShmRingPublisher(StreamSpec("imu0", StreamKind.IMU, 0, 0, int(FormatCode.IMU_F32_LE), 24, 4096,
                                               "imu0", "gyro rad/s, accel m/s^2"), namespace=args.namespace)
    StreamRegistry(args.registry).write(namespace=args.namespace, publishers=pubs, config_path=str(args.dataset))
    print(f"[replay] {len(events)} events from {args.dataset}", flush=True)
    time.sleep(args.start_delay_s)

    wall0 = time.monotonic()
    data0 = events[0][0]
    pairs = 0
    try:
        for ts, kind, value in events:
            due = wall0 + (ts - data0) / 1e9 / args.speed
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if kind == "imu":
                pubs["imu0"].publish(struct.pack("<ffffff", *value), timestamp_ns=ts)
                continue
            for name, rel in zip(("camera0", "camera1"), value):
                img = cv2.imread(str(args.dataset / rel), cv2.IMREAD_GRAYSCALE)
                pubs[name].publish(img.tobytes(), timestamp_ns=ts)
            pairs += 1
            if pairs % 300 == 0:
                print(f"[replay] pairs={pairs} t={(ts - data0) / 1e9:.1f}s", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[replay] done pairs={pairs}", flush=True)
        for pub in pubs.values():
            try:
                pub.close(unlink=True)
            except KeyError:
                # The segment was unregistered from the resource tracker, which
                # rejects the unlink notification after the unlink itself.
                pass


if __name__ == "__main__":
    main()
