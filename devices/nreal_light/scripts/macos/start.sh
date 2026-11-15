#!/usr/bin/env bash
# Run the Nreal Light stack on macOS:
#   capture_service_cpp (sudo, drops to the user after claiming USB)
#   -> factory_to_basalt.py (factory calibration of this unit)
#   -> capture_basalt_backend
#   -> xr_runtime_adapter (logging adapter, optional application TCP)
#
# SteamVR and Monado do not run on macOS. Poses are available through
# RUNTIME_APP_TCP=1 (JSON Lines on 127.0.0.1:45810) and the hmd_pose SHM.
#
# Environment:
#   NREAL_CALIB_DIR      where the per-unit Basalt calibration is written
#                        (default ~/.config/xr_tracking/nreal_light)
#   BASALT_CALIB         use this Basalt calibration; defaults to
#                        $NREAL_CALIB_DIR/basalt_calib_kalibr_640x480.json when
#                        present, else the factory calibration
#   RUNTIME_TRANSFORM    use this xr_runtime_adapter transform JSON
#   RUNTIME_APP_TCP=1    enable the application TCP server
#   BASALT_TRAJECTORY    write the Basalt trajectory CSV here
#   DURATION_SEC         stop after this many seconds (default 0, until Ctrl+C)
#   CAPTURE_ONLY=1       run only capture_service_cpp, for calibration recording
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_PROJECT="${ROOT_PROJECT:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"
DEVICE_DIR="$ROOT_PROJECT/devices/nreal_light"

CAPTURE_BIN="${CAPTURE_BIN:-$ROOT_PROJECT/bin/capture_service_cpp/capture_service_cpp}"
BASALT_BIN="${BASALT_BIN:-$ROOT_PROJECT/bin/backends/basalt_vio/capture_basalt_backend}"
ADAPTER_BIN="${ADAPTER_BIN:-$ROOT_PROJECT/bin/runtime_adapters/xr_runtime_adapter/xr_runtime_adapter}"
CAPTURE_CONFIG="${CAPTURE_CONFIG:-$DEVICE_DIR/configs/capture_service/nreal_light_macos_device_time_shm.yaml}"
BASALT_VIO_CONFIG="${BASALT_VIO_CONFIG:-$DEVICE_DIR/configs/basalt/basalt_vio_config_640x480.json}"
CAPTURE_REGISTRY="${CAPTURE_REGISTRY:-/tmp/capture_service_streams.json}"
TRACKING_REGISTRY="${TRACKING_REGISTRY:-/tmp/tracking_streams.json}"
FACTORY_PATH="${FACTORY_PATH:-/tmp/nreal_light_factory.bin}"
NREAL_CALIB_DIR="${NREAL_CALIB_DIR:-$HOME/.config/xr_tracking/nreal_light}"
LOG_DIR="${LOG_DIR:-/tmp/xr_gate_nreal_light}"
DURATION_SEC="${DURATION_SEC:-0}"
RUNTIME_APP_TCP="${RUNTIME_APP_TCP:-0}"

for bin in "$CAPTURE_BIN" "$BASALT_BIN" "$ADAPTER_BIN"; do
  if [[ ! -x "$bin" ]]; then
    echo "[nreal_light] missing $bin; run devices/nreal_light/scripts/macos/build.sh first" >&2
    exit 2
  fi
done
mkdir -p "$LOG_DIR" "$NREAL_CALIB_DIR"
: >"$LOG_DIR/capture.log"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in ${pids[@]+"${pids[@]}"}; do
    kill "$pid" 2>/dev/null || true
  done
  # The capture process belongs to the user after it drops root, so a plain
  # signal from this shell reaches it.
  pkill -f "$CAPTURE_BIN --config $CAPTURE_CONFIG" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rm -f "$CAPTURE_REGISTRY" "$FACTORY_PATH"
echo "[nreal_light] capture_service_cpp needs root to detach the OV580 drivers"
sudo -v
# capture_service_cpp exits when the stereo stream stalls or loses frame
# alignment. Restart it; capture_basalt_backend reconnects to the new streams.
(
  while true; do
    sudo -E "$CAPTURE_BIN" --config "$CAPTURE_CONFIG" --duration "$DURATION_SEC" >>"$LOG_DIR/capture.log" 2>&1 && break
    echo "[nreal_light] capture exited with $?; restarting" >>"$LOG_DIR/capture.log"
    sleep 2
  done
) &
pids+=("$!")

for _ in $(seq 1 300); do
  [[ -s "$CAPTURE_REGISTRY" && -s "$FACTORY_PATH" ]] && break
  if ! kill -0 "${pids[0]}" 2>/dev/null; then
    cat "$LOG_DIR/capture.log" >&2
    exit 1
  fi
  sleep 0.1
done
if [[ ! -s "$CAPTURE_REGISTRY" ]]; then
  echo "[nreal_light] capture did not publish $CAPTURE_REGISTRY; see $LOG_DIR/capture.log" >&2
  exit 1
fi

if [[ "${CAPTURE_ONLY:-0}" == "1" ]]; then
  echo "[nreal_light] capture only; streams in $CAPTURE_REGISTRY, log in $LOG_DIR/capture.log"
  wait "${pids[0]}"
  exit 0
fi

if [[ -z "${BASALT_CALIB:-}" && -s "$NREAL_CALIB_DIR/basalt_calib_kalibr_640x480.json" ]]; then
  BASALT_CALIB="$NREAL_CALIB_DIR/basalt_calib_kalibr_640x480.json"
  echo "[nreal_light] using Kalibr calibration $BASALT_CALIB"
fi
if [[ -z "${BASALT_CALIB:-}" ]]; then
  BASALT_CALIB="$NREAL_CALIB_DIR/basalt_calib_factory_640x480.json"
  RUNTIME_TRANSFORM="${RUNTIME_TRANSFORM:-$NREAL_CALIB_DIR/xr_runtime_transform.json}"
  python3 "$ROOT_PROJECT/calibration/nreal_light/factory_to_basalt.py" \
    --factory "$FACTORY_PATH" --out "$BASALT_CALIB" --runtime-transform-out "$RUNTIME_TRANSFORM"
fi
if [[ -z "${RUNTIME_TRANSFORM:-}" ]]; then
  RUNTIME_TRANSFORM="$NREAL_CALIB_DIR/xr_runtime_transform.json"
  python3 "$ROOT_PROJECT/calibration/nreal_light/factory_to_basalt.py" \
    --factory "$FACTORY_PATH" --out "$LOG_DIR/basalt_calib_factory_unused.json" \
    --runtime-transform-out "$RUNTIME_TRANSFORM"
fi

basalt_args=(
  --transport shm
  --registry "$CAPTURE_REGISTRY"
  --cam-calib "$BASALT_CALIB"
  --config-path "$BASALT_VIO_CONFIG"
  --out-dir "$LOG_DIR"
  --pose-registry "$TRACKING_REGISTRY"
  --duration "$DURATION_SEC"
  --image-scale 256
  --no-control-file
)
if [[ -n "${BASALT_TRAJECTORY:-}" ]]; then
  basalt_args+=(--trajectory "$BASALT_TRAJECTORY")
fi
"$BASALT_BIN" "${basalt_args[@]}" >"$LOG_DIR/basalt.log" 2>&1 &
pids+=("$!")

adapter_args=(
  --adapter logging
  --input shm
  --registry "$TRACKING_REGISTRY"
  --tracking-transform-config "$RUNTIME_TRANSFORM"
  --origin-mode none
  --duration "$DURATION_SEC"
  --print-every 300
)
if [[ "$RUNTIME_APP_TCP" == "1" ]]; then
  adapter_args+=(--app-tcp --app-tcp-host "${RUNTIME_APP_TCP_HOST:-127.0.0.1}" --app-tcp-port "${RUNTIME_APP_TCP_PORT:-45810}")
fi
"$ADAPTER_BIN" "${adapter_args[@]}" >"$LOG_DIR/adapter.log" 2>&1 &
pids+=("$!")

echo "[nreal_light] logs in $LOG_DIR (capture.log, basalt.log, adapter.log)"
[[ "$RUNTIME_APP_TCP" == "1" ]] && echo "[nreal_light] python3 tools/xr_app_client.py --host 127.0.0.1"
while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[nreal_light] a component exited; stopping"
      exit 0
    fi
  done
  sleep 0.5
done
