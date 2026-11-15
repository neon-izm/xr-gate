#!/usr/bin/env bash
# Keep capture_service_cpp for Nreal Light running as a launchd daemon, so
# measurements need neither the sudo password nor a fresh OV580 open.
#
#   capture_daemon.sh install    copy the binary and config to a root-owned
#                                directory and load the daemon (asks for sudo)
#   capture_daemon.sh uninstall  remove the daemon (asks for sudo)
#   capture_daemon.sh start      start capture, wait for the stream registry
#   capture_daemon.sh stop       stop capture
#   capture_daemon.sh restart    reopen the glasses, for example after a stall
#   capture_daemon.sh status
#
# launchd keeps the job alive while the enable file exists. The process drops
# to the installing user after claiming USB, so start, stop, and restart run
# without sudo. Rerun install after rebuilding capture_service_cpp or editing
# the capture config; the daemon runs the installed copies.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_PROJECT="${ROOT_PROJECT:-$(cd "$SCRIPT_DIR/../../../.." && pwd)}"
DEVICE_DIR="$ROOT_PROJECT/devices/nreal_light"

LABEL=com.xr-gate.nreal-light-capture
PLIST=/Library/LaunchDaemons/$LABEL.plist
INSTALL_DIR=/usr/local/libexec/xr-gate/nreal_light
DAEMON_BIN=$INSTALL_DIR/capture_service_cpp
DAEMON_CONFIG=$INSTALL_DIR/capture.yaml
DAEMON_LOG=/Library/Logs/xr-gate-nreal-light-capture.log
ENABLE_FILE="${NREAL_CAPTURE_ENABLE_FILE:-$HOME/.config/xr_tracking/nreal_light/capture.enabled}"
CAPTURE_BIN="${CAPTURE_BIN:-$ROOT_PROJECT/bin/capture_service_cpp/capture_service_cpp}"
CAPTURE_CONFIG="${CAPTURE_CONFIG:-$DEVICE_DIR/configs/capture_service/nreal_light_macos_device_time_shm.yaml}"
CAPTURE_REGISTRY="${CAPTURE_REGISTRY:-/tmp/capture_service_streams.json}"
START_TIMEOUT_S="${START_TIMEOUT_S:-30}"
PYTHON_BIN="${PYTHON_BIN:-$HOME/.venvs/xreal_capture_service/bin/python}"

run_root() {
  if [[ -t 0 ]]; then
    sudo /bin/sh -c "$1"
  else
    # No terminal for the sudo prompt; ask through the macOS password dialog.
    local script=${1//\\/\\\\}
    script=${script//\"/\\\"}
    osascript -e "do shell script \"$script\" with administrator privileges" >/dev/null
  fi
}

running_pid() {
  pgrep -f "^$DAEMON_BIN --config $DAEMON_CONFIG" | head -1 || true
}

write_plist() {
  cat <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>$LABEL</string>
  <key>ProgramArguments</key>
  <array>
    <string>$DAEMON_BIN</string>
    <string>--config</string><string>$DAEMON_CONFIG</string>
    <string>--duration</string><string>0</string>
  </array>
  <key>EnvironmentVariables</key>
  <dict>
    <key>SUDO_UID</key><string>$(id -u)</string>
    <key>SUDO_GID</key><string>$(id -g)</string>
  </dict>
  <key>KeepAlive</key>
  <dict>
    <key>PathState</key>
    <dict><key>$ENABLE_FILE</key><true/></dict>
  </dict>
  <key>ThrottleInterval</key><integer>5</integer>
  <key>WorkingDirectory</key><string>/</string>
  <key>StandardOutPath</key><string>$DAEMON_LOG</string>
  <key>StandardErrorPath</key><string>$DAEMON_LOG</string>
</dict>
</plist>
EOF
}

cmd_install() {
  [[ -x "$CAPTURE_BIN" ]] || { echo "missing $CAPTURE_BIN; run build.sh first" >&2; exit 2; }
  mkdir -p "$(dirname "$ENABLE_FILE")"
  local plist_tmp
  plist_tmp=$(mktemp)
  write_plist >"$plist_tmp"
  local was_enabled=0
  [[ -e "$ENABLE_FILE" ]] && was_enabled=1
  rm -f "$ENABLE_FILE"
  local pid
  pid=$(running_pid)
  [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  echo "[capture_daemon] installing $LABEL (administrator password)"
  run_root "
    launchctl bootout system/$LABEL 2>/dev/null || true
    install -d -o root -g wheel -m 755 '$INSTALL_DIR'
    install -o root -g wheel -m 755 '$CAPTURE_BIN' '$DAEMON_BIN'
    install -o root -g wheel -m 644 '$CAPTURE_CONFIG' '$DAEMON_CONFIG'
    install -o root -g wheel -m 644 '$plist_tmp' '$PLIST'
    touch '$DAEMON_LOG' && chmod 644 '$DAEMON_LOG'
    launchctl bootstrap system '$PLIST'
  "
  rm -f "$plist_tmp"
  echo "[capture_daemon] installed; daemon runs $DAEMON_BIN"
  if [[ "$was_enabled" == "1" ]]; then cmd_start; fi
}

cmd_uninstall() {
  cmd_stop
  run_root "
    launchctl bootout system/$LABEL 2>/dev/null || true
    rm -f '$PLIST'
    rm -rf '$INSTALL_DIR'
  "
  echo "[capture_daemon] uninstalled"
}

require_installed() {
  [[ -f "$PLIST" ]] || { echo "[capture_daemon] not installed; run: $0 install" >&2; exit 2; }
}

frames_flowing() {
  # A restarted process can publish the registry and then fail to reopen the
  # OV580, so check that camera0 frames advance.
  [[ -x "$PYTHON_BIN" ]] || return 0
  (cd "$ROOT_PROJECT" && "$PYTHON_BIN" - "$CAPTURE_REGISTRY" <<'EOF'
import sys, time
sys.path.insert(0, ".")
from capture_client.client import CaptureClient
try:
    c = CaptureClient.from_shm_registry(sys.argv[1], required_streams=["camera0"])
    a = c.latest_sequence("camera0")
    time.sleep(1.0)
    b = c.latest_sequence("camera0")
    c.close()
except Exception:
    sys.exit(1)
sys.exit(0 if b > a else 1)
EOF
  )
}

cmd_start() {
  require_installed
  if [[ -n "$(running_pid)" && -s "$CAPTURE_REGISTRY" ]]; then
    if frames_flowing; then
      echo "[capture_daemon] running; registry $CAPTURE_REGISTRY"
      return 0
    fi
    echo "[capture_daemon] no frames; reopening the glasses" >&2
    kill "$(running_pid)" 2>/dev/null || true
  fi
  # Only log lines written from here on describe this start.
  local log_mark
  log_mark=$(wc -c <"$DAEMON_LOG" 2>/dev/null || echo 0)
  rm -f "$CAPTURE_REGISTRY"
  mkdir -p "$(dirname "$ENABLE_FILE")"
  touch "$ENABLE_FILE"
  local deadline=$((SECONDS + START_TIMEOUT_S))
  while ((SECONDS < deadline)); do
    if [[ -s "$CAPTURE_REGISTRY" && -n "$(running_pid)" ]] && frames_flowing; then
      echo "[capture_daemon] running; registry $CAPTURE_REGISTRY"
      return 0
    fi
    if tail -c +$((log_mark + 1)) "$DAEMON_LOG" 2>/dev/null | grep -q "05a9:0680 not found"; then
      echo "[capture_daemon] the OV580 is off the bus; unplug and replug the glasses" >&2
      exit 1
    fi
    sleep 0.2
  done
  echo "[capture_daemon] no registry after ${START_TIMEOUT_S}s; see $DAEMON_LOG" >&2
  tail -5 "$DAEMON_LOG" >&2 || true
  exit 1
}

cmd_stop() {
  rm -f "$ENABLE_FILE"
  # A process still opening the OV580 runs as root and ignores this user's
  # signals until it drops privileges, so keep signalling for a while.
  local pid
  for _ in $(seq 1 60); do
    pid=$(running_pid)
    [[ -z "$pid" ]] && break
    kill "$pid" 2>/dev/null || true
    sleep 0.25
  done
  rm -f "$CAPTURE_REGISTRY"
  echo "[capture_daemon] stopped"
}

cmd_restart() {
  require_installed
  local pid
  pid=$(running_pid)
  rm -f "$CAPTURE_REGISTRY"
  [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  cmd_start
}

cmd_status() {
  if [[ ! -f "$PLIST" ]]; then
    echo "not installed"
    return 0
  fi
  local pid
  pid=$(running_pid)
  echo "installed: $PLIST"
  echo "enabled:   $([[ -e "$ENABLE_FILE" ]] && echo yes || echo no)"
  echo "pid:       ${pid:-none}"
  echo "registry:  $([[ -s "$CAPTURE_REGISTRY" ]] && echo "$CAPTURE_REGISTRY" || echo none)"
  if ! cmp -s "$CAPTURE_BIN" "$DAEMON_BIN" || ! cmp -s "$CAPTURE_CONFIG" "$DAEMON_CONFIG"; then
    echo "note:      the build or config differs from the installed copy; rerun install"
  fi
  tail -3 "$DAEMON_LOG" 2>/dev/null || true
}

case "${1:-}" in
  install) cmd_install ;;
  uninstall) cmd_uninstall ;;
  start) cmd_start ;;
  stop) cmd_stop ;;
  restart) cmd_restart ;;
  status) cmd_status ;;
  *) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
