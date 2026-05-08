#!/usr/bin/env bash
set -euo pipefail

WORLD_FILE="${AIROS_GAZEBO_PROBE_WORLD:-/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf}"
PROBE_SECONDS="${AIROS_GAZEBO_PROBE_SECONDS:-8}"
LOG_FILE="${AIROS_GAZEBO_PROBE_LOG:-/tmp/airos_gazebo_gpu_probe.log}"
RENDER_ENGINE="${AIROS_GAZEBO_RENDER_ENGINE:-ogre}"

section() {
  printf '\n[%s]\n' "$1"
}

fail() {
  printf '[FAIL] %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "missing command: $1"
}

capture_ign_pids() {
  pgrep -f '(^|[[:space:]])ign gazebo([[:space:]]|$)' || true
}

kill_new_ign_pids() {
  local before_file="$1"
  local after_file
  after_file="$(mktemp)"
  capture_ign_pids >"$after_file"

  while read -r pid; do
    [ -n "$pid" ] || continue
    if ! grep -qx "$pid" "$before_file" && [ "$pid" != "$$" ]; then
      kill "$pid" 2>/dev/null || true
    fi
  done <"$after_file"

  sleep 1

  capture_ign_pids >"$after_file"
  while read -r pid; do
    [ -n "$pid" ] || continue
    if ! grep -qx "$pid" "$before_file" && [ "$pid" != "$$" ]; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done <"$after_file"

  rm -f "$after_file"
}

section "GPU: nvidia-smi"
require_cmd nvidia-smi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

section "OpenGL: glxinfo -B"
require_cmd glxinfo
GLX_OUTPUT="$(glxinfo -B)"
printf '%s\n' "$GLX_OUTPUT" | sed -n '1,60p'

printf '%s\n' "$GLX_OUTPUT" | grep -q 'direct rendering: Yes' || fail "OpenGL direct rendering is not enabled"
printf '%s\n' "$GLX_OUTPUT" | grep -q 'Accelerated: yes' || fail "OpenGL renderer is not accelerated"
printf '%s\n' "$GLX_OUTPUT" | grep -Eiq 'D3D12 .*NVIDIA|NVIDIA' || fail "OpenGL renderer is not using the NVIDIA/D3D12 path"
if printf '%s\n' "$GLX_OUTPUT" | grep -Eiq 'llvmpipe|softpipe|Software Rasterizer'; then
  fail "OpenGL is using a software renderer"
fi

section "Gazebo: version and render arguments"
require_cmd ign
ign gazebo --versions
ign gazebo --help | grep -q -- '--render-engine-gui' || fail "ign gazebo does not expose GUI render-engine selection"
[ -f "$WORLD_FILE" ] || fail "probe world does not exist: $WORLD_FILE"

section "Gazebo GUI GPU render probe"
printf 'world=%s\n' "$WORLD_FILE"
printf 'render_engine=%s\n' "$RENDER_ENGINE"
printf 'probe_seconds=%s\n' "$PROBE_SECONDS"
printf 'log=%s\n' "$LOG_FILE"

BEFORE_PIDS="$(mktemp)"
capture_ign_pids >"$BEFORE_PIDS"
rm -f "$LOG_FILE"

set +e
env \
  LIBGL_ALWAYS_SOFTWARE=0 \
  MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}" \
  ign gazebo -v 4 \
    --render-engine "$RENDER_ENGINE" \
    --render-engine-gui "$RENDER_ENGINE" \
    "$WORLD_FILE" >"$LOG_FILE" 2>&1 &
GAZEBO_PID=$!

sleep "$PROBE_SECONDS"

if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
  GAZEBO_EXITED_EARLY=1
else
  GAZEBO_EXITED_EARLY=0
fi

kill_new_ign_pids "$BEFORE_PIDS"
wait "$GAZEBO_PID" 2>/dev/null
GAZEBO_STATUS=$?
set -e
rm -f "$BEFORE_PIDS"

sed -n '1,140p' "$LOG_FILE"

if [ "$GAZEBO_EXITED_EARLY" = "1" ]; then
  fail "Gazebo GUI exited before the ${PROBE_SECONDS}s probe window; status=${GAZEBO_STATUS}"
fi

grep -q 'Ignition Gazebo GUI' "$LOG_FILE" || fail "Gazebo GUI did not start"
grep -q 'MinimalScene' "$LOG_FILE" || fail "Gazebo 3D view plugin did not load"
grep -q 'GzSceneManager' "$LOG_FILE" || fail "Gazebo scene manager plugin did not load"

if grep -Eiq 'OGRE EXCEPTION|Aborted|Segmentation fault|llvmpipe|softpipe|Software Rasterizer' "$LOG_FILE"; then
  fail "Gazebo render probe log contains a fatal renderer/software-rendering signal"
fi

printf '\n[PASS] Gazebo GUI render probe stayed alive for %ss using %s with accelerated NVIDIA/D3D12 OpenGL.\n' \
  "$PROBE_SECONDS" "$RENDER_ENGINE"
