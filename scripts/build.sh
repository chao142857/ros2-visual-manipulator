#!/usr/bin/env bash
# Build only this repository into separate release validation directories.
set -Eeuo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: bash scripts/build.sh [colcon build arguments...]
Builds src/ with --symlink-install into build-release/install-release/log-release.
Run from a clean shell; no installed application overlay is sourced.
See scripts/env.sh --help for optional external overlays and Python environment.
Example: bash scripts/build.sh --packages-select arm_driver gripper_driver camera_perception
EOF
  exit 0
fi
arm_script_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$arm_script_root/scripts/env.sh" --underlay-only
if ! command -v colcon >/dev/null; then
  printf 'ERROR: colcon is missing; install python3-colcon-common-extensions.\n' >&2
  exit 1
fi
# A shell with a previous application overlay can conceal missing dependencies.
IFS=: read -r -a arm_inherited_prefixes <<<"${AMENT_PREFIX_PATH:-}"
for arm_inherited_prefix in "${arm_inherited_prefixes[@]}"; do
  if [[ -f "$arm_inherited_prefix/share/arm_driver/package.xml" || \
        -f "$arm_inherited_prefix/share/tabletop_grasp/package.xml" ]]; then
    printf 'ERROR: application overlay already sourced: %s\nUse a clean shell for this build.\n' \
      "$arm_inherited_prefix" >&2
    exit 1
  fi
done
cd -- "$ARM_WS_ROOT"
exec python3 "$(command -v colcon)" --log-base "$ARM_LOG_BASE" build \
  --base-paths "$ARM_WS_ROOT/src" --build-base "$ARM_BUILD_BASE" \
  --install-base "$ARM_INSTALL_BASE" --symlink-install \
  --executor sequential --event-handlers desktop_notification- "$@"
