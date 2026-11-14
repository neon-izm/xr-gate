#!/usr/bin/env bash
# Build capture_basalt_backend on macOS with Homebrew dependencies.
#
# Upstream Basalt builds its dependencies through vcpkg presets. On macOS the
# Monado Basalt fork builds with Homebrew packages instead, so this script uses
# that fork and adds the same XR_TRACKING_ROOT hook as the Linux installer.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_PROJECT="${ROOT_PROJECT:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"
BASALT_DIR="${BASALT_DIR:-$ROOT_PROJECT/third_party/basalt}"
BASALT_REPO_URL="${BASALT_REPO_URL:-https://gitlab.freedesktop.org/mateosss/basalt.git}"
BASALT_REF="${BASALT_REF:-df6e970}"
BASALT_BUILD_DIR="${BASALT_BUILD_DIR:-$ROOT_PROJECT/build/backends/basalt_vio/macos}"
INSTALL_BIN_DIR="${INSTALL_BIN_DIR:-$ROOT_PROJECT/bin/backends/basalt_vio}"
BREW_PREFIX="${BREW_PREFIX:-$(brew --prefix)}"
OPENCV_PREFIX="${OPENCV_PREFIX:-$(brew --prefix opencv@4)}"

for formula in cmake ninja opencv@4 tbb eigen fmt glew lz4 boost nlohmann-json; do
  if ! brew list --versions "$formula" >/dev/null 2>&1; then
    echo "[install_basalt_macos][ERROR] missing Homebrew formula: $formula" >&2
    echo "  brew install cmake ninja opencv@4 tbb eigen fmt glew lz4 boost nlohmann-json" >&2
    exit 2
  fi
done

if [[ ! -d "$BASALT_DIR/.git" ]]; then
  git clone --recursive "$BASALT_REPO_URL" "$BASALT_DIR"
fi
if [[ "$(git -C "$BASALT_DIR" rev-parse --short=7 HEAD)" != "${BASALT_REF:0:7}" ]]; then
  git -C "$BASALT_DIR" fetch origin
  git -C "$BASALT_DIR" checkout "$BASALT_REF"
fi
git -C "$BASALT_DIR" submodule update --init --recursive

if ! grep -q "XR_TRACKING_ROOT" "$BASALT_DIR/CMakeLists.txt"; then
  cat >>"$BASALT_DIR/CMakeLists.txt" <<'CMAKE'

set(XR_TRACKING_ROOT "$ENV{XR_TRACKING_ROOT}" CACHE PATH "Path to xr_tracking checkout")
if(XR_TRACKING_ROOT AND EXISTS "${XR_TRACKING_ROOT}/backends/basalt_vio/CMakeLists.txt")
  message(STATUS "Enabling XR Basalt VIO backend from ${XR_TRACKING_ROOT}/backends/basalt_vio")
  add_subdirectory(
    "${XR_TRACKING_ROOT}/backends/basalt_vio"
    "${CMAKE_BINARY_DIR}/xr_tracking/backends/basalt_vio"
  )
else()
  message(STATUS "XR Basalt VIO backend disabled; set XR_TRACKING_ROOT to the xr_tracking checkout")
endif()
CMAKE
fi

export XR_TRACKING_ROOT="$ROOT_PROJECT"
cmake -S "$BASALT_DIR" -B "$BASALT_BUILD_DIR" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$OPENCV_PREFIX;$BREW_PREFIX" \
  -DOpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4" \
  -DBASALT_BUILD_SHARED_LIBRARY_ONLY=ON \
  -DBASALT_BUILD_VISUALIZATION=OFF \
  -DBASALT_INSTANTIATIONS_DOUBLE=OFF \
  -DBASALT_ENABLE_ROSBAG2=OFF \
  -DBUILD_TESTS=OFF \
  -DXR_TRACKING_ROOT="$ROOT_PROJECT"

cmake --build "$BASALT_BUILD_DIR" --target capture_basalt_backend -j"$(sysctl -n hw.ncpu)"

mkdir -p "$INSTALL_BIN_DIR"
install -m 0755 "$BASALT_BUILD_DIR/xr_tracking/backends/basalt_vio/capture_basalt_backend" "$INSTALL_BIN_DIR/"
echo "[install_basalt_macos] installed $INSTALL_BIN_DIR/capture_basalt_backend"
