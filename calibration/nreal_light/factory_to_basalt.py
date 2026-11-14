#!/usr/bin/env python3
"""Convert the Nreal Light factory config to an XR Gate Basalt calibration.

capture_service_cpp writes the OV580 factory blob to
nreal_light.factory_config_path (default /tmp/nreal_light_factory.bin) when it
opens the glasses. This script turns that blob into the same Basalt JSON layout
that the XR Gate Kalibr converter writes, so a later Kalibr calibration can
replace the file without changing the runtime.

Conventions taken from the factory JSON:
  * SLAM_camera.device_N.imu_q_cam is JPL (qx, qy, qz, qw). Negating the
    vector part gives the Hamilton rotation camera -> IMU. With this choice,
    inverse(T_imu_left) * T_imu_right reproduces leftcam_q_rightcam and
    leftcam_p_rightcam, which the factory stores in Hamilton form.
  * kc is radial k1, k2, p1, p2, k3. Basalt reads it as pinhole-radtan8 with
    k4 = k5 = k6 = 0.
  * IMU bias, scale, skew, g-sensitivity and accel_q_gyro are applied by
    capture_service_cpp, so the Basalt IMU models stay at zero here.
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Optional


def extract_factory_json(blob: bytes) -> Dict:
    text = blob.decode("latin-1")
    begin = text.find("\n\n{")
    if begin < 0:
        raise ValueError("factory blob has no JSON object")
    end = text.find("\n\n", begin + 2)
    if end < 0:
        raise ValueError("factory blob JSON is not terminated")
    return json.loads(text[begin + 2 : end])


def trailer_value(blob: bytes, key: str) -> Optional[str]:
    text = blob.decode("latin-1")
    marker = key + " = "
    at = text.rfind(marker)
    if at < 0:
        return None
    return text[at + len(marker) :].split("\n", 1)[0].strip()


def hamilton_matrix(x: float, y: float, z: float, w: float) -> List[List[float]]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def transpose(m: List[List[float]]) -> List[List[float]]:
    return [list(row) for row in zip(*m)]


def matmul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a: List[List[float]], v: List[float]) -> List[float]:
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def camera_pose(cam: Dict) -> Dict[str, float]:
    qx, qy, qz, qw = [float(v) for v in cam["imu_q_cam"]]
    px, py, pz = [float(v) for v in cam["imu_p_cam"]]
    if qw < 0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    return {"px": px, "py": py, "pz": pz, "qx": -qx, "qy": -qy, "qz": -qz, "qw": qw}


def pose_rotation(pose: Dict[str, float]) -> List[List[float]]:
    return hamilton_matrix(pose["qx"], pose["qy"], pose["qz"], pose["qw"])


def check_stereo(slam: Dict, left: Dict[str, float], right: Dict[str, float]) -> None:
    r_left = pose_rotation(left)
    r_right = pose_rotation(right)
    baseline = matvec(
        transpose(r_left),
        [right["px"] - left["px"], right["py"] - left["py"], right["pz"] - left["pz"]],
    )
    expected_p = [float(v) for v in slam.get("leftcam_p_rightcam", baseline)]
    rel = matmul(transpose(r_left), r_right)
    expected_r = hamilton_matrix(*[float(v) for v in slam["leftcam_q_rightcam"]]) if "leftcam_q_rightcam" in slam else rel
    p_err = max(abs(a - b) for a, b in zip(baseline, expected_p))
    r_err = max(abs(rel[i][j] - expected_r[i][j]) for i in range(3) for j in range(3))
    print(f"stereo baseline in left camera: {[round(v, 5) for v in baseline]} m")
    print(f"stereo check against leftcam_*_rightcam: position {p_err:.2e} m, rotation {r_err:.2e}")
    if p_err > 1e-3 or r_err > 1e-3:
        raise ValueError("factory imu_q_cam does not reproduce leftcam_q_rightcam; check the JPL convention")


def intrinsics(cam: Dict) -> Dict:
    fx, fy = [float(v) for v in cam["fc"]]
    cx, cy = [float(v) for v in cam["cc"]]
    kc = [float(v) for v in cam["kc"]] + [0.0] * 5
    if cam.get("camera_model", "radial") != "radial":
        raise ValueError(f"unsupported factory camera_model {cam.get('camera_model')!r}")
    return {
        "camera_type": "pinhole-radtan8",
        "intrinsics": {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "k1": kc[0],
            "k2": kc[1],
            "p1": kc[2],
            "p2": kc[3],
            "k3": kc[4],
            "k4": 0.0,
            "k5": 0.0,
            "k6": 0.0,
            "rpmax": -1.0,
        },
    }


def matrix_quaternion(m: List[List[float]]) -> List[float]:
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = [0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = [(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s]
    if q[3] < 0:
        q = [-v for v in q]
    return q


# Basalt world (z up) -> runtime world (y up, -z forward). This is the matrix
# of the XREAL Ultra transform: axis_map x,z,y with invert_x, and the same
# rotation as basis_rotation rx -90, ry 180.
BASALT_TO_RUNTIME = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]
# Runtime head axes (x right, y up, z back) seen from the camera (x right, y down, z forward).
CAMERA_TO_HEAD = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]


def runtime_transform(left: Dict[str, float]) -> Dict:
    """HMD transform for xr_runtime_adapter.

    The adapter outputs B * q_world_imu * B^-1 * offset. The Ultra IMU is
    mounted so that the head frame is B^-1 and needs no offset. The Light IMU
    is the camera frame tilted by about 15 degrees, so the offset is
    B * R_imu_cam * R_cam_head, which makes the runtime head follow the left
    camera.
    """
    r_imu_head = matmul(pose_rotation(left), CAMERA_TO_HEAD)
    offset = matrix_quaternion(matmul(BASALT_TO_RUNTIME, r_imu_head))
    hmd = {
        "coordinate_transform": {
            "axis_map": ["x", "z", "y"],
            "invert_x": True,
            "invert_y": False,
            "invert_z": False,
            "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
            "scale": 1.0,
            "offset_m": {"x": 0.0, "y": 1.6, "z": 0.0},
        },
        "hmd_relative": {"enabled": False, "offset_m": {"x": 0.0, "y": 0.0, "z": 0.0}},
        "orientation_transform": {
            "enabled": True,
            "basis_rotation": {"rx_deg": -90.0, "ry_deg": 180.0, "rz_deg": 0.0},
        },
        "orientation_offset": {
            "enabled": True,
            "multiply_order": "post",
            "quaternion_xyzw": offset,
        },
    }
    return {"enabled": True, "streams": {"hmd": hmd, "hmd_3dof": json.loads(json.dumps(hmd))}}


def identity_pose() -> Dict[str, float]:
    return {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--factory", default="/tmp/nreal_light_factory.bin", help="factory blob from capture_service_cpp")
    ap.add_argument("--out", required=True, help="Basalt calibration JSON to write")
    ap.add_argument("--imu-update-rate", type=float, default=1000.0)
    ap.add_argument(
        "--cam-time-offset-ns",
        type=int,
        default=0,
        help="Basalt cam_time_offset_ns. capture_service_cpp already adds camera_imu_timestamp_offset_us.",
    )
    ap.add_argument("--accel-noise", type=float, help="override factory imu_noises[2]")
    ap.add_argument("--gyro-noise", type=float, help="override factory imu_noises[0]")
    ap.add_argument("--runtime-transform-out", help="also write the xr_runtime_adapter HMD transform JSON")
    args = ap.parse_args()

    blob = Path(args.factory).read_bytes()
    factory = extract_factory_json(blob)
    slam = factory["SLAM_camera"]
    cams = [slam["device_1"], slam["device_2"]]
    poses = [camera_pose(cam) for cam in cams]
    check_stereo(slam, poses[0], poses[1])

    gyro_noise, gyro_bias, accel_noise, accel_bias = [float(v) for v in factory["IMU"]["device_1"]["imu_noises"]]
    if args.accel_noise is not None:
        accel_noise = args.accel_noise
    if args.gyro_noise is not None:
        gyro_noise = args.gyro_noise

    resolution = [[int(cam["resolution"][0]), int(cam["resolution"][1])] for cam in cams]
    data = {
        "value0": {
            "T_imu_cam": poses,
            "intrinsics": [intrinsics(cam) for cam in cams],
            "resolution": resolution,
            "calib_accel_bias": [0.0] * 9,
            "calib_gyro_bias": [0.0] * 12,
            "imu_update_rate": args.imu_update_rate,
            "accel_noise_std": [accel_noise] * 3,
            "gyro_noise_std": [gyro_noise] * 3,
            "accel_bias_std": [accel_bias] * 3,
            "gyro_bias_std": [gyro_bias] * 3,
            "T_mocap_world": identity_pose(),
            "T_imu_marker": identity_pose(),
            "mocap_time_offset_ns": 0,
            "mocap_to_imu_offset_ns": 0,
            "cam_time_offset_ns": args.cam_time_offset_ns,
            "vignette": [
                {"value0": 0, "value1": 10000000000, "value2": [[1.0] for _ in range(67)]},
                {"value0": 0, "value1": 10000000000, "value2": [[1.0] for _ in range(67)]},
            ],
        }
    }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=4) + "\n", encoding="utf-8")
    print("written:", out)
    if args.runtime_transform_out:
        transform_out = Path(args.runtime_transform_out)
        transform_out.parent.mkdir(parents=True, exist_ok=True)
        transform = runtime_transform(poses[0])
        transform_out.write_text(json.dumps(transform, indent=2) + "\n", encoding="utf-8")
        print("written:", transform_out)
        print("hmd orientation_offset xyzw:", [round(v, 6) for v in transform["streams"]["hmd"]["orientation_offset"]["quaternion_xyzw"]])
    print("glasses:", factory.get("glasses_ID", "?"), "calibration:", factory.get("slam_calibration_method", "?"))
    print("factory camera_imu_timestamp_offset_us:", trailer_value(blob, "camera_imu_timestamp_offset_us"))
    print("factory sensor_orientation (not applied):", trailer_value(blob, "sensor_orientation"))


if __name__ == "__main__":
    main()
