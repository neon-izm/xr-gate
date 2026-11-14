# Nreal Light on macOS

Branch `nreal-light`. Runs the XR Gate tracking stack on an Apple Silicon Mac
with Nreal Light glasses:

```text
capture_service_cpp (libusb nreal_light driver, 30 Hz stereo 640x480 + 1 kHz IMU)
  -> POSIX SHM (/tmp/capture_service_streams.json)
  -> capture_basalt_backend (Monado Basalt fork, stereo VIO)
  -> hmd_pose SHM (/tmp/tracking_streams.json)
  -> xr_runtime_adapter (logging adapter, application TCP JSON Lines)
```

SteamVR has no macOS build and Monado does not run on macOS, so this branch
stops at the runtime adapter. Applications read the head pose from the
application TCP server (`RUNTIME_APP_TCP=1`, see
[application_network_api.md](../runtime_adapters/xr_runtime_adapter/docs/application_network_api.md))
or from the `hmd_pose` SHM stream.

## Build

```bash
brew install cmake ninja pkgconf libusb hidapi opencv@4 tbb eigen fmt glew lz4 boost nlohmann-json
devices/nreal_light/scripts/macos/build.sh
```

`build.sh` builds `capture_service_cpp` with the `nreal_light` driver (needs
libusb-1.0 through pkg-config), clones the Monado Basalt fork
(`gitlab.freedesktop.org/mateosss/basalt`, `df6e970`) into `third_party/basalt`
and builds `capture_basalt_backend` against it, then builds
`xr_runtime_adapter`. Upstream Basalt builds its dependencies with vcpkg; the
Monado fork builds with Homebrew packages, which is why macOS uses it. The
backend source switches to the fork's queue names with `XR_BASALT_MONADO_FORK`.

## Run

```bash
RUNTIME_APP_TCP=1 devices/nreal_light/scripts/macos/start.sh
python3 tools/xr_app_client.py --host 127.0.0.1
```

`start.sh` asks for the sudo password. macOS only lets root detach the kernel
UVC and HID drivers from the OV580 (libusb PR 911, issue 1014), so the capture
process starts as root, claims USB, and then switches back to `SUDO_UID`. The
SHM segments are created by the user, so Basalt and the adapter run without
root. Logs go to `/tmp/xr_gate_nreal_light`.

Keep the glasses still for the first second so that Basalt can align gravity.

## Device notes

USB devices of the glasses:

| Device | ID | Use |
| --- | --- | --- |
| MCU | HID `0486:573c` | `@3 1` once, then `@K` heartbeat every 250 ms |
| OV580 | `05a9:0680` | UVC interface 1 bulk `0x81` stereo frames, HID interrupt IN IMU and factory config |
| Audio | `0bda:4b77` | not used |

Stereo frames are one 615908-byte bulk transfer: a 12-byte UVC header every
0x8000 bytes, then 640x481 rows where even rows are the left camera, odd rows
the right camera, and the u64 after the pixels the OV580 time in ns. Transfers
are read into 2 MiB buffers so that a transfer that starts mid-frame ends at the
short packet that closes the frame and the next transfer is aligned again.

IMU report 1 carries gyro in degrees/s and accel in g with their own
multiplier and divisor. The driver publishes the OV580 IMU frame without axis
flips. That is the frame of the factory `imu_q_cam`. ar-drivers-rs flips y and
z, which turns the factory camera extrinsics by 180 degrees about x.

Timestamps use `timestamp_mode: device`. IMU device times train the same affine
device-to-host clock model as XREAL Ultra. Camera times are the OV580 frame
time plus `camera_imu_timestamp_offset_us` from the factory blob (39000 on the
tested unit), mapped through that model.

## Calibration

On every start `capture_service_cpp` reads the factory blob (about 9.3 KB)
from the OV580 and writes it to `/tmp/nreal_light_factory.bin`.
[factory_to_basalt.py](../calibration/nreal_light/factory_to_basalt.py) turns it
into a Basalt calibration and an `xr_runtime_adapter` transform:

```bash
python3 calibration/nreal_light/factory_to_basalt.py \
  --factory /tmp/nreal_light_factory.bin \
  --out ~/.config/xr_tracking/nreal_light/basalt_calib_factory_640x480.json \
  --runtime-transform-out ~/.config/xr_tracking/nreal_light/xr_runtime_transform.json
```

- Cameras: `pinhole-radtan8` with the factory `fc`, `cc`, and `kc` = k1 k2 p1 p2 k3.
- Extrinsics: the factory `imu_q_cam` is JPL. Negating its vector part gives the
  Hamilton camera-to-IMU rotation. The script checks the result against the
  factory `leftcam_q_rightcam` and `leftcam_p_rightcam`; the tested unit has a
  103.7 mm baseline and the cameras are tilted about 15 degrees about x from
  the IMU.
- IMU: bias, scale, skew, g-sensitivity and `accel_q_gyro` are applied in
  `capture_service_cpp` (`nreal_light.apply_factory_imu`), so the Basalt IMU
  models stay at zero, as with the Ultra Kalibr calibration. Noise densities
  come from the factory `imu_noises`.
- `cam_time_offset_ns` is 0 because the capture service already adds the
  factory camera offset.

The factory values were not estimated together with this pipeline's timing.
For better tracking, record an Aprilgrid dataset and run Kalibr as for the
Ultra (`calibration/common/linux`), then pass the result with
`BASALT_CALIB=/path/basalt_calib.json`. The Kalibr converter in this repository
only handles pinhole-equi; Nreal Light lenses fit pinhole-radtan.

## Coordinate transform

Basalt publishes `T_world_imu` in a gravity-aligned z-up world. The runtime
transform uses the Ultra mapping for positions (`axis_map x,z,y`, `invert_x`,
1.6 m height) and the same `basis_rotation` (rx -90, ry 180). The Ultra IMU
sits so that this basis alone yields the head frame. The Light IMU is the
left camera frame (x right, y down, z forward) tilted by about 15 degrees, so
`factory_to_basalt.py` adds a post-multiplied `orientation_offset`
`B * R_imu_cam * diag(1, -1, -1)` that makes the runtime head follow the left
camera. The offset depends on each unit's factory extrinsics.

## Files

| Path | Content |
| --- | --- |
| `capture_service_cpp/src/vendor/nreal_light_*.cpp` | USB session, MCU heartbeat, frame, IMU, factory parsing |
| `capture_service_cpp/src/sources/nreal_light_*_source.cpp` | camera and IMU sources |
| `devices/nreal_light/configs/capture_service/nreal_light_macos_device_time_shm.yaml` | capture config |
| `devices/nreal_light/configs/basalt/basalt_vio_config_640x480.json` | Basalt VIO config for 640x480 at 30 Hz |
| `calibration/nreal_light/factory_to_basalt.py` | factory blob to Basalt calibration and runtime transform |
| `devices/nreal_light/scripts/macos/build.sh`, `start.sh` | build and launch |
| `backends/basalt_vio/scripts/macos/install_basalt.sh` | Basalt fork build |
