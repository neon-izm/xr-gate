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

SteamVR has no macOS build. OpenXR applications run on a patched Monado that
loads into the application process, see [OpenXR](#openxr). Other applications
read the head pose from the application TCP server (`RUNTIME_APP_TCP=1`, see
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

### Capture daemon

To avoid the password and a fresh OV580 open on every run, install the
capture service once as a launchd daemon:

```bash
devices/nreal_light/scripts/macos/capture_daemon.sh install   # password once
devices/nreal_light/scripts/macos/capture_daemon.sh start     # no password
devices/nreal_light/scripts/macos/capture_daemon.sh status
```

`install` copies `capture_service_cpp` and its config to the root-owned
`/usr/local/libexec/xr-gate/nreal_light` and loads
`/Library/LaunchDaemons/com.xr-gate.nreal-light-capture.plist`. launchd keeps
the process alive while `~/.config/xr_tracking/nreal_light/capture.enabled`
exists, and restarts it when the glasses are plugged in again. The process
drops to the installing user after claiming USB, so `start`, `stop`, and
`restart` run without sudo. When the daemon is installed, `start.sh` and
`CAPTURE_ONLY=1` use it instead of `sudo`; `CAPTURE_DAEMON=0` forces the old
path. Rerun `install` after rebuilding `capture_service_cpp` or editing the
capture config; `status` reports a mismatch. The daemon log is
`/Library/Logs/xr-gate-nreal-light-capture.log`. The process still loads the
Homebrew libusb, hidapi, and OpenCV libraries while it is root, as with
`sudo`.

Leave the daemon running between measurements. Each `stop` and `start` opens
the OV580 again, and on the tested Mac three quick cycles left it off the bus.
A port reset, a port power cycle, and a power cycle of all ports of the
glasses hub (`04b4:6572`, ganged power) did not bring it back; only a replug
did.

## OpenXR

```bash
brew install molten-vk vulkan-loader vulkan-headers glslang shaderc
drivers/monado_driver/scripts/macos/build.sh
devices/nreal_light/scripts/macos/capture_daemon.sh install   # after rebuilding capture
devices/nreal_light/scripts/macos/run_openxr.sh               # hello_xr -G Vulkan2
devices/nreal_light/scripts/macos/run_openxr.sh /path/to/app [args]
```

`drivers/monado_driver/scripts/macos/build.sh` resets `third_party/monado_driver`
to the pinned Monado commit, applies `drivers/monado_driver/patches/monado/0001`
to `0003`, copies the `xr_tracking_runtime` driver, and builds only the
`openxr_monado` target with `XRT_FEATURE_SERVICE=OFF`. `monado-service` and the
IPC client use epoll and are not ported, so the runtime and its compositor run
inside the application. The runtime manifest is
`third_party/monado_driver/build/macos/openxr_monado-dev.json`.

`run_openxr.sh` sets the glasses to 3840x1080 side by side, starts `start.sh`
with `RUNTIME_POSE_SHM=1` so that the adapter publishes `runtime_hmd_pose`,
exports the display geometry from the factory blob with
`calibration/nreal_light/factory_display_env.py`, and runs the application
with `XR_TRACKING_RUNTIME_ENABLE=1` and `XRT_COMPOSITOR_MACOS_SCREEN=Light`.
Without `XR_TRACKING_RUNTIME_ENABLE=1` Monado uses its simulated HMD.

Vulkan applications must use `XR_KHR_vulkan_enable2` (hello_xr `-G Vulkan2`),
so that the runtime creates the `VkInstance`. MoltenVK only enumerates with
`VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR`, which the runtime adds.

Patch `0003-macos-metal-compositor.patch`:

- Swapchain images are shared between the application and compositor
  `VkDevice`s as `MTLTexture` handles (`VK_EXT_external_memory_metal`). The
  importing image must also chain `VkImportMetalTextureInfoEXT`
  (`VK_EXT_metal_objects`), otherwise it is not backed by the shared texture.
- `comp_window_macos.m` is a compositor target with a `CAMetalLayer` window.
  `XRT_COMPOSITOR_MACOS_SCREEN` takes an `NSScreen` index or a name substring
  and covers that screen borderless; without it the window is 1920x1080 on the
  main screen.
- `VK_GOOGLE_display_timing` present times are off on macOS. MoltenVK reads
  them as `mach_absolute_time`, which excludes sleep, while Monado uses
  `CLOCK_MONOTONIC`, which includes it, so frames were queued days ahead and
  never shown.

`0001` also sets the tracking origin offset to identity. The static system
devices helper zero-fills it, and the zero quaternion removed the head
rotation from `xrLocateViews`. The view stayed level while the compositor
reprojected to the real rotation.

### Display mode

The MCU sets the glasses display with category `1` command `3`: `1` 2D
1920x1080, `2` 3D 1920x540, `3` 3D 3840x1080 side by side, `4` 3D 3840x1080 at
72 Hz. hidapi opens the MCU exclusively on macOS, so `capture_service_cpp`
sends the mode after `@3 1`. It reads the digit from
`~/.config/xr_tracking/nreal_light/display_mode` and falls back to
`nreal_light.display_mode` in the capture config, where 0 leaves the mode as
is.

```bash
devices/nreal_light/scripts/macos/capture_daemon.sh display-mode sbs   # or 2d, 0-4
```

The command writes the file and restarts a running capture when the mode
changes. macOS then lists the `Light` display at 3840x1080.

### Clocks and SHM on macOS

- `std::chrono::steady_clock` in libc++ is `CLOCK_MONOTONIC_RAW`, which differs
  from `CLOCK_MONOTONIC` by tens of seconds on the tested Mac. All stream
  timestamps use `steady_clock`. `tools/replay_dataset_shm.py --host-time`
  shifts recorded timestamps onto that clock, so that the adapter does not
  drop replayed poses as stale.
- `fstat` on a POSIX SHM descriptor reports `st_nlink` 0, so the Monado driver
  readers do not use it to detect an unlinked segment on macOS. They reopen
  after 500 ms without a new sequence instead.
- `capture_basalt_backend` creates its pose SHM again after an internal
  restart. `start.sh` passes `--hmd-reattach-on-stale-ms 500` to the adapter so
  that it follows the new segment.

## Device notes

USB devices of the glasses:

| Device | ID | Use |
| --- | --- | --- |
| MCU | HID `0486:573c` | `@3 1` once, then `@K` heartbeat every 250 ms |
| OV580 | `05a9:0680` | UVC interface 1 bulk `0x81` stereo frames, HID interrupt IN IMU and factory config |
| Audio | `0bda:4b77` | not used |

Stereo frames are one 615908-byte bulk transfer: a 12-byte UVC header every
0x8000 bytes, then 640x481 rows where even rows are the left camera, odd rows
the right camera, and the u64 after the pixels the OV580 time in ns. Four
exact-size bulk transfers stay queued. A transfer that ends with OVERFLOW,
STALL, or ERROR is not resubmitted; clearing the halt or recommitting UVC in the
same session made the OV580 drop off the bus.

### OV580 on macOS

Every libusb open and close re-enumerates the OV580, and on the tested Mac
consecutive sessions alternate between working, timing out with no frames, and
overflowing. The driver therefore waits up to 2.5 s for three good frames after
the UVC commit and, if they do not arrive, closes the device and reopens it, up
to four attempts. `start.sh` additionally restarts `capture_service_cpp` when
it exits with an error. If the OV580 (`05a9:0680`) disappears from
`system_profiler SPUSBDataType`, unplug and replug the glasses.

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

### Kalibr with an AprilGrid on the laptop screen

The `nreal_light` target of `calibration/common/linux` runs Kalibr in Docker
(amd64 image under emulation on Apple Silicon) and converts `pinhole-radtan`
results to Basalt `pinhole-radtan8`. The AprilGrid is shown on the MacBook
screen:

```bash
python3 calibration/nreal_light/make_screen_aprilgrid.py \
  --out-dir ~/xr_calib/nreal_light/targets --cols 5 --rows 3 --bit-px 40
open ~/xr_calib/nreal_light/targets/aprilgrid_screen_5x3_400px.html   # f: full screen
```

The cameras see about 100 degrees, so the board must fill a large part of the
image at 15 to 25 cm; 32 mm tags on a 6x4 board were detected unreliably. The
screen loses contrast at oblique angles. Measure the full board width on the
screen and write the measured tag size into the target YAML with
`--measured-tag-mm`. With a non-default macOS display scaling the tested
board was 0.831 of its design size, and Kalibr scales all distances with it.

```bash
./calibration/common/linux/calibrate.sh --target nreal_light install
CAPTURE_ONLY=1 devices/nreal_light/scripts/macos/start.sh        # terminal 1
~/.venvs/xreal_capture_service/bin/python calibration/nreal_light/check_grid_view.py  # optional
SECONDS_TOTAL=90 PYTHON_BIN=~/.venvs/xreal_capture_service/bin/python \
  ./calibration/common/linux/calibrate.sh --target nreal_light record   # terminal 2
./calibration/common/linux/calibrate.sh --target nreal_light bag
./calibration/common/linux/calibrate.sh --target nreal_light camera
./calibration/common/linux/calibrate.sh --target nreal_light save-camera
./calibration/common/linux/calibrate.sh --target nreal_light imu-camera
./calibration/common/linux/calibrate.sh --target nreal_light convert-runtime
```

The recorder needs numpy and OpenCV, for example in
`~/.venvs/xreal_capture_service`. Copy the result to
`~/.config/xr_tracking/nreal_light/basalt_calib_kalibr_640x480.json`;
`start.sh` then uses it instead of the factory calibration. `BASALT_CALIB`
still overrides both.

### Evaluating against the screen board

[eval_vio_against_grid.py](../calibration/nreal_light/eval_vio_against_grid.py)
compares Basalt relative motion with PnP on the static screen board. `--loop`
pairs board views before and after a walk around the room, so it measures the
drift accumulated while the board was out of view. A 3x2 board
(`--cols 3 --rows 2 --bit-px 65`, 54 mm measured tags) is detected at 25 to
40 cm, where the 5x3 board is too small. `check_grid_view.py
--beep-min-tags 4` plays a sound while camera 0 sees the board, so the view
can be held without looking at the terminal. A recording is replayed into SHM
with [replay_dataset_shm.py](../tools/replay_dataset_shm.py) and run through
`capture_basalt_backend --trajectory`.

On the tested unit, the glasses were held at the board, looked around the room
for about a minute, and returned. Drift between board views 15 to 60 s apart,
with 67 mm of camera motion on the board:

| Calibration | Rotation error | Translation error | Scale |
| --- | --- | --- | --- |
| Factory | 0.7 deg | 97 mm | 1.94 |
| Kalibr | 1.0 deg | 22 mm | 1.27 |
| Kalibr, matching depth 0.2 m | 1.1 deg | 20 mm | 1.09 |

Pairs with board views 25 to 50 s apart gave 296 mm, 53 mm and 22 mm.

[plot_vio_session.py](../calibration/nreal_light/plot_vio_session.py) draws a
session in 3D: the first board segment fixes the Basalt world to the board,
and every later return shows the PnP pose next to the Basalt pose. It writes
an interactive `session3d.html` and a `session3d.png`. In a live 112 s session
with the default Kalibr calibration, worn on the head and returned to the board
at 20 cm about every 30 s, the drift at the returns was 35, 61 and 58 mm and
1.8 to 2.8 degrees.

`config.optical_flow_matching_default_depth` is the depth at which a new
point is searched in the right image. At the default 1.0 m, points on a near
repetitive pattern such as the screen board match a neighbouring tag and the
scale grows. The Nreal Light VIO config uses 0.2 m.

## Measured on the tested unit

M1 Mac, factory calibration, glasses resting still on a desk for 90 s:

- Capture: 3000 stereo pairs at 30 Hz, 99000 IMU samples at 1 kHz, about
  29 ms camera latency, clock drift about -5 ppm.
- Basalt: 2701 poses over 90 s, about 30 Hz.
- Drift: 0.13 m in position and about 3 degrees in orientation over 90 s.
- `tools/xr_app_client.py` receives `hmd_valid=True` poses at y = 1.62 m.

The drift is the main limit of the factory calibration. See the board
evaluation above for the Kalibr calibration.

## Coordinate transform

Basalt publishes `T_world_imu` in a gravity-aligned z-up world. The runtime
transform uses the Ultra mapping for positions (`axis_map x,z,y`, `invert_x`,
1.6 m height) and the same `basis_rotation` (rx -90, ry 180). The Ultra IMU
sits so that this basis alone yields the head frame. The Light IMU is the
left camera frame (x right, y down, z forward) tilted by about 15 degrees, so
`factory_to_basalt.py` adds a post-multiplied `orientation_offset`
`B * R_imu_cam * diag(1, -1, -1)` that makes the runtime head follow the left
camera. The offset depends on each unit's factory extrinsics.

## Fork Basalt notes

The Monado fork differs from upstream in ways the backend has to handle under
`XR_BASALT_MONADO_FORK`:

- The optical flow thread starts only on `start()`, and its destructor joins
  that thread, so a backend reset without `start()` crashes.
- The optical flow reads depth guesses, VIO state, and landmark bundles from
  the VIO, so those queues are connected before `start()`.
- `OpticalFlowInput` indexes `masks` per camera, so inputs are built with
  `OpticalFlowInput(2)`.
- The VIO config must contain every key of the fork's `VioConfig`.

## Files

| Path | Content |
| --- | --- |
| `capture_service_cpp/src/vendor/nreal_light_*.cpp` | USB session, MCU heartbeat, frame, IMU, factory parsing |
| `capture_service_cpp/src/sources/nreal_light_*_source.cpp` | camera and IMU sources |
| `devices/nreal_light/configs/capture_service/nreal_light_macos_device_time_shm.yaml` | capture config |
| `devices/nreal_light/configs/basalt/basalt_vio_config_640x480.json` | Basalt VIO config for 640x480 at 30 Hz |
| `calibration/nreal_light/factory_to_basalt.py` | factory blob to Basalt calibration and runtime transform |
| `devices/nreal_light/scripts/macos/build.sh`, `start.sh` | build and launch |
| `devices/nreal_light/scripts/macos/run_openxr.sh` | OpenXR application on the glasses |
| `calibration/nreal_light/factory_display_env.py` | factory display calibration to `XR_MONADO_*` display variables |
| `drivers/monado_driver/scripts/macos/build.sh` | Monado OpenXR runtime build |
| `drivers/monado_driver/patches/monado/0003-macos-metal-compositor.patch` | Monado MoltenVK buffer sharing and macOS window target |
| `backends/basalt_vio/scripts/macos/install_basalt.sh` | Basalt fork build |
