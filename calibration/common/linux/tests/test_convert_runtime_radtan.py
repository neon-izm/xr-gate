#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    converter = root / "kalibr_to_basalt_unified_json.py"
    camchain = {
        "cam0": {
            "camera_model": "pinhole",
            "distortion_model": "radtan",
            "intrinsics": [236.4, 236.7, 311.4, 247.5],
            "distortion_coeffs": [0.066, -0.025, 0.0024, 0.0026],
            "resolution": [640, 480],
        },
        "cam1": {
            "camera_model": "pinhole",
            "distortion_model": "radtan",
            "intrinsics": [236.1, 236.2, 317.0, 244.3],
            "distortion_coeffs": [0.072, -0.028, 0.0048, -0.0025],
            "resolution": [640, 480],
            "T_cn_cnm1": [
                [1.0, 0.0, 0.0, -0.104],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    }

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        camchain_path = tmp_path / "camchain.yaml"
        out_path = tmp_path / "runtime.json"
        camchain_path.write_text(yaml.safe_dump(camchain), encoding="utf-8")
        subprocess.run(
            [
                sys.executable,
                str(converter),
                "--camchain", str(camchain_path),
                "--out", str(out_path),
                "--imu-update-rate", "1000",
                "--accel-noise-density", "0.014",
                "--accel-random-walk", "0.001",
                "--gyro-noise-density", "0.0004",
                "--gyro-random-walk", "0.0001",
                "--no-imu",
            ],
            check=True,
        )
        value = json.loads(out_path.read_text(encoding="utf-8"))["value0"]
        cam0 = value["intrinsics"][0]
        assert cam0["camera_type"] == "pinhole-radtan8"
        intr = cam0["intrinsics"]
        assert intr["fx"] == 236.4 and intr["cy"] == 247.5
        assert (intr["k1"], intr["k2"], intr["p1"], intr["p2"]) == (0.066, -0.025, 0.0024, 0.0026)
        assert intr["k3"] == intr["k4"] == intr["k5"] == intr["k6"] == 0.0
        assert intr["rpmax"] == -1.0
        assert value["intrinsics"][1]["intrinsics"]["p2"] == -0.0025


if __name__ == "__main__":
    main()
