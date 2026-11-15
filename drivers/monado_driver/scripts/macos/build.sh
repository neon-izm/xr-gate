#!/usr/bin/env bash
# Build the in-process Monado OpenXR runtime with the xr_tracking_runtime driver on
# macOS: pinned upstream checkout + patches/monado/*.patch + project driver sources,
# Vulkan through MoltenVK, and a CAMetalLayer compositor window.
#
# Only the openxr_monado target is built. monado-service and the IPC client need
# epoll and are not ported, so XRT_FEATURE_SERVICE is OFF and the runtime runs
# inside the application process.
#
# Requirements: brew install cmake ninja molten-vk vulkan-loader vulkan-headers
#               glslang shaderc eigen libusb hidapi nlohmann-json
#
# Environment:
#   MONADO_DIR              checkout (default third_party/monado_driver)
#   BUILD_DIR               build directory (default $MONADO_DIR/build/macos)
#   RESET_MONADO_CHECKOUT   1 (default) resets the checkout to MONADO_REF and
#                           drops local changes before applying the patches
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_PROJECT="${ROOT_PROJECT:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"
DRIVER_DIR="$ROOT_PROJECT/drivers/monado_driver"
PATCH_DIR="$DRIVER_DIR/patches/monado"
MONADO_DIR="${MONADO_DIR:-$ROOT_PROJECT/third_party/monado_driver}"
BUILD_DIR="${BUILD_DIR:-$MONADO_DIR/build/macos}"
MONADO_REPO="${MONADO_REPO:-https://gitlab.freedesktop.org/monado/monado.git}"
MONADO_REF="${MONADO_REF:-7363fee94b66671efdce79655b8b143d7c9eeecd}"
RESET_MONADO_CHECKOUT="${RESET_MONADO_CHECKOUT:-1}"
PATCHES=(
  0001-add-xr-tracking-runtime-driver.patch
  0002-openxr-spatial-video.patch
  0003-macos-metal-compositor.patch
)

log() { echo "[monado_macos] $*"; }

if [[ ! -e "$MONADO_DIR/.git" ]]; then
  log "cloning $MONADO_REPO"
  git clone "$MONADO_REPO" "$MONADO_DIR"
fi
if ! git -C "$MONADO_DIR" cat-file -e "$MONADO_REF^{commit}" 2>/dev/null; then
  git -C "$MONADO_DIR" fetch origin
fi

if [[ "$RESET_MONADO_CHECKOUT" == "1" ]]; then
  log "resetting $MONADO_DIR to $MONADO_REF"
  git -C "$MONADO_DIR" checkout -q --detach "$MONADO_REF"
  git -C "$MONADO_DIR" reset -q --hard "$MONADO_REF"
  # build/ is ignored and survives the clean.
  git -C "$MONADO_DIR" clean -fdq -- src tests
  for patch in "${PATCHES[@]}"; do
    log "applying $patch"
    git -C "$MONADO_DIR" apply "$PATCH_DIR/$patch"
  done
else
  log "RESET_MONADO_CHECKOUT=0: using the checkout as is"
fi

DST_DIR="$MONADO_DIR/src/xrt/drivers/xr_tracking_runtime"
mkdir -p "$DST_DIR"
rsync -a --delete --exclude patches --exclude scripts "$DRIVER_DIR/" "$DST_DIR/"

BREW_PREFIX="$(brew --prefix)"
# The driver includes nlohmann/json.hpp without a find_package.
cmake -S "$MONADO_DIR" -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DXRT_FEATURE_SERVICE=OFF \
  -DXR_TRACKING_ROOT="$ROOT_PROJECT" \
  -DCMAKE_PREFIX_PATH="$BREW_PREFIX" \
  -DCMAKE_CXX_FLAGS="-I$BREW_PREFIX/include" \
  -DCMAKE_C_FLAGS="-Wno-error=incompatible-pointer-types-discards-qualifiers"
ninja -C "$BUILD_DIR" openxr_monado
log "runtime manifest: $BUILD_DIR/openxr_monado-dev.json"
