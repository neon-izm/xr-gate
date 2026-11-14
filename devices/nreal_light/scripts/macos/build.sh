#!/usr/bin/env bash
# Build the Nreal Light stack on macOS: capture_service_cpp with the libusb
# driver, capture_basalt_backend on the Monado Basalt fork, and
# xr_runtime_adapter.
#
#   brew install cmake ninja pkgconf libusb hidapi opencv@4 tbb eigen fmt glew lz4 boost nlohmann-json
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_PROJECT="${ROOT_PROJECT:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"
BREW_PREFIX="$(brew --prefix)"
OPENCV_PREFIX="$(brew --prefix opencv@4)"
JOBS="$(sysctl -n hw.ncpu)"

echo "[nreal_light_build] capture_service_cpp"
cmake -S "$ROOT_PROJECT/capture_service_cpp" -B "$ROOT_PROJECT/build/capture_service_cpp/macos" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$OPENCV_PREFIX;$BREW_PREFIX" \
  -DHIDAPI_ROOT="$(brew --prefix hidapi)"
cmake --build "$ROOT_PROJECT/build/capture_service_cpp/macos" -j"$JOBS"
if ! grep -q "XR_CAPTURE_HAS_NREAL_LIGHT" "$ROOT_PROJECT/build/capture_service_cpp/macos/build.ninja"; then
  echo "[nreal_light_build][ERROR] libusb-1.0 was not found; the nreal_light driver is not built" >&2
  exit 2
fi
mkdir -p "$ROOT_PROJECT/bin/capture_service_cpp"
install -m 0755 "$ROOT_PROJECT/build/capture_service_cpp/macos/capture_service_cpp" "$ROOT_PROJECT/bin/capture_service_cpp/"

echo "[nreal_light_build] capture_basalt_backend"
ROOT_PROJECT="$ROOT_PROJECT" "$ROOT_PROJECT/backends/basalt_vio/scripts/macos/install_basalt.sh"

echo "[nreal_light_build] xr_runtime_adapter"
CLI11_INCLUDE="$ROOT_PROJECT/third_party/basalt/thirdparty/CLI11/include"
cmake -S "$ROOT_PROJECT/runtime_adapters/xr_runtime_adapter" -B "$ROOT_PROJECT/build/xr_runtime_adapter/macos" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$BREW_PREFIX" \
  -DCLI11_INCLUDE_DIR="$CLI11_INCLUDE"
cmake --build "$ROOT_PROJECT/build/xr_runtime_adapter/macos" --target xr_runtime_adapter -j"$JOBS"
mkdir -p "$ROOT_PROJECT/bin/runtime_adapters/xr_runtime_adapter"
install -m 0755 "$ROOT_PROJECT/build/xr_runtime_adapter/macos/xr_runtime_adapter" \
  "$ROOT_PROJECT/bin/runtime_adapters/xr_runtime_adapter/"

echo "[nreal_light_build] done"
