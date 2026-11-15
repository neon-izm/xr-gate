#!/usr/bin/env python3
"""Print XR_MONADO_* display variables for the Nreal Light from its factory blob.

The factory JSON stores a pinhole matrix per display (k_left_display,
k_right_display) for the 1920x1080 panel of each eye, and the display poses in
the IMU frame. The half-angles of the view frustum follow from the principal
point and focal length; the IPD from the distance between the display origins.
Both displays are within about 2 degrees of the left SLAM camera, which is the
runtime head frame, so no per-eye rotation is emitted.

    eval "$(python3 calibration/nreal_light/factory_display_env.py)"
"""

import argparse
import math
from typing import List

from factory_to_basalt import extract_factory_json


def half_angles_deg(k: List[float], width: float, height: float) -> List[float]:
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return [
        math.degrees(math.atan(cx / fx)),
        math.degrees(math.atan((width - cx) / fx)),
        math.degrees(math.atan(cy / fy)),
        math.degrees(math.atan((height - cy) / fy)),
    ]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--factory", default="/tmp/nreal_light_factory.bin", help="factory blob from capture_service_cpp")
    ap.add_argument("--refresh-hz", type=float, default=60.0)
    args = ap.parse_args()

    with open(args.factory, "rb") as f:
        display = extract_factory_json(f.read())["display"]
    width, height = (float(v) for v in display["resolution"])
    left = display["target_p_left_display"]
    right = display["target_p_right_display"]
    ipd = math.dist(left, right)

    exports = {
        "XR_MONADO_EYE_WIDTH": int(width),
        "XR_MONADO_EYE_HEIGHT": int(height),
        "XR_MONADO_WINDOW_WIDTH": int(2 * width),
        "XR_MONADO_WINDOW_HEIGHT": int(height),
        "XR_MONADO_IPD_M": f"{ipd:.4f}",
        "XR_MONADO_REFRESH_HZ": f"{args.refresh_hz:g}",
    }
    for eye in ("left", "right"):
        names = ("LEFT", "RIGHT", "UP", "DOWN")
        angles = half_angles_deg([float(v) for v in display[f"k_{eye}_display"]], width, height)
        for name, angle in zip(names, angles):
            exports[f"XR_MONADO_{eye.upper()}_EYE_FOV_{name}_DEG"] = f"{angle:.4f}"

    for key, value in exports.items():
        print(f"export {key}={value}")


if __name__ == "__main__":
    main()
