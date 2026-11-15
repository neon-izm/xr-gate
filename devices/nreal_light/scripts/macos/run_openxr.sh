#!/usr/bin/env bash
# Run an OpenXR application on the Nreal Light on macOS:
#   start.sh (capture -> Basalt -> xr_runtime_adapter, runtime_hmd_pose SHM)
#   -> in-process Monado (xr_tracking_runtime HMD, macOS compositor window)
#   -> the application, full screen on the glasses in 3840x1080 SBS mode.
#
#   run_openxr.sh [APP [ARGS...]]
#
# Without arguments it runs hello_xr with the Vulkan2 graphics plugin. Vulkan
# applications must let the runtime create the VkInstance (XR_KHR_vulkan_enable2),
# since MoltenVK needs the portability enumeration flag the runtime adds.
#
# Environment:
#   MONADO_BUILD    Monado macOS build directory with openxr_monado-dev.json
#                   (default third_party/monado_driver/build/macos)
#   HELLO_XR        hello_xr binary for the default application
#   NREAL_SCREEN    NSScreen index or name substring for the compositor window
#                   (default Light)
#   DISPLAY_MODE    glasses display mode passed to capture_daemon.sh (default sbs)
#   start.sh variables (BASALT_CALIB, RUNTIME_TRANSFORM, LOG_DIR, ...) apply.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_PROJECT="${ROOT_PROJECT:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"
MONADO_BUILD="${MONADO_BUILD:-$ROOT_PROJECT/third_party/monado_driver/build/macos}"
HELLO_XR="${HELLO_XR:-$HOME/src/OpenXR-SDK-Source/build/src/tests/hello_xr/hello_xr}"
NREAL_SCREEN="${NREAL_SCREEN:-Light}"
DISPLAY_MODE="${DISPLAY_MODE:-sbs}"
LOG_DIR="${LOG_DIR:-/tmp/xr_gate_nreal_light}"
export LOG_DIR

RUNTIME_JSON="$MONADO_BUILD/openxr_monado-dev.json"
if [[ ! -s "$RUNTIME_JSON" ]]; then
  echo "[run_openxr] missing $RUNTIME_JSON; build Monado with: ninja -C $MONADO_BUILD openxr_monado" >&2
  exit 2
fi
if [[ $# -eq 0 ]]; then
  if [[ ! -x "$HELLO_XR" ]]; then
    echo "[run_openxr] missing $HELLO_XR; pass an application or set HELLO_XR" >&2
    exit 2
  fi
  set -- "$HELLO_XR" -G Vulkan2
fi

"$SCRIPT_DIR/capture_daemon.sh" display-mode "$DISPLAY_MODE"

stack_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$stack_pid" ]]; then
    kill "$stack_pid" 2>/dev/null || true
    wait "$stack_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

mkdir -p "$LOG_DIR"
: >"$LOG_DIR/adapter.log"
RUNTIME_POSE_SHM=1 "$SCRIPT_DIR/start.sh" >"$LOG_DIR/start.log" 2>&1 &
stack_pid=$!
for _ in $(seq 1 600); do
  grep -q "publishing runtime pose stream" "$LOG_DIR/adapter.log" 2>/dev/null && break
  if ! kill -0 "$stack_pid" 2>/dev/null; then
    cat "$LOG_DIR/start.log" >&2
    exit 1
  fi
  sleep 0.1
done
if ! grep -q "publishing runtime pose stream" "$LOG_DIR/adapter.log"; then
  echo "[run_openxr] xr_runtime_adapter did not publish runtime_hmd_pose; see $LOG_DIR" >&2
  exit 1
fi

eval "$(python3 "$ROOT_PROJECT/calibration/nreal_light/factory_display_env.py")"
export XR_RUNTIME_JSON="$RUNTIME_JSON"
export XR_TRACKING_RUNTIME_ENABLE=1
export XRT_COMPOSITOR_MACOS_SCREEN="$NREAL_SCREEN"
echo "[run_openxr] stack logs in $LOG_DIR; running $*"
"$@"
