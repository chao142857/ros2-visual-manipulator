#!/usr/bin/env bash
# Source this file; it never starts a ROS node or opens a device.

_arm_env_main() {
  local arm_env_mode="${1:-}"
  if [[ "$arm_env_mode" == --help || "$arm_env_mode" == -h ]]; then
    cat <<'EOF'
Usage: source scripts/env.sh [--underlay-only]
Loads ROS 2 Humble, explicitly configured vendor overlays, and install-release
when present. --underlay-only omits the application install for builds.

Optional environment variables:
  ARM_ROS_SETUP       ROS setup file (default /opt/ros/humble/setup.bash)
  ARM_ASTRA_SETUP     Astra overlay local_setup.bash (no implicit sibling lookup)
  ARM_REALSENSE_SETUP RealSense overlay local_setup.bash
  ARM_PYTHON_ENV      ROS-compatible Python 3.10 venv directory
  ARM_BUILD_BASE     Build directory (default <repository>/build-release)
  ARM_INSTALL_BASE   Install directory (default <repository>/install-release)
  ARM_LOG_BASE       Log directory (default <repository>/log-release)
  PYTHONNOUSERSITE   Defaults to 1 to exclude machine-specific ~/.local packages.
                     An explicit empty value restores legacy lookup; 0 still disables it.
EOF
    return 0
  fi
  if [[ $# -gt 1 || ( -n "$arm_env_mode" && "$arm_env_mode" != --underlay-only ) ]]; then
    printf 'ERROR: use source scripts/env.sh [--underlay-only]\n' >&2
    return 2
  fi
  local arm_env_root arm_env_file arm_env_status arm_env_nounset=0
  arm_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
    printf 'ERROR: this project targets ROS 2 Humble; use a clean shell.\n' >&2
    return 1
  fi
  export ARM_WS_ROOT="$arm_env_root"
  export ARM_BUILD_BASE="${ARM_BUILD_BASE:-$arm_env_root/build-release}"
  export ARM_INSTALL_BASE="${ARM_INSTALL_BASE:-$arm_env_root/install-release}"
  export ARM_LOG_BASE="${ARM_LOG_BASE:-$arm_env_root/log-release}"
  export PYTHONNOUSERSITE="${PYTHONNOUSERSITE-1}"
  for arm_env_file in "${ARM_ROS_SETUP:-/opt/ros/humble/setup.bash}" \
      "${ARM_ASTRA_SETUP:-}" "${ARM_REALSENSE_SETUP:-}"; do
    [[ -z "$arm_env_file" ]] && continue
    if [[ ! -r "$arm_env_file" ]]; then
      printf 'ERROR: setup file is not readable: %s\n' "$arm_env_file" >&2
      return 1
    fi
    [[ "$-" == *u* ]] && arm_env_nounset=1
    set +u
    if source "$arm_env_file"; then arm_env_status=0; else arm_env_status=$?; fi
    ((arm_env_nounset)) && set -u
    ((arm_env_status == 0)) || return "$arm_env_status"
  done
  if [[ -n "${ARM_PYTHON_ENV:-}" ]]; then
    if [[ ! -r "$ARM_PYTHON_ENV/bin/activate" ]]; then
      printf 'ERROR: missing venv activation file: %s/bin/activate\n' "$ARM_PYTHON_ENV" >&2
      return 1
    fi
    source "$ARM_PYTHON_ENV/bin/activate" || return
  fi
  if [[ "$arm_env_mode" != --underlay-only && -r "$ARM_INSTALL_BASE/local_setup.bash" ]]; then
    [[ "$-" == *u* ]] && arm_env_nounset=1
    set +u
    if source "$ARM_INSTALL_BASE/local_setup.bash"; then arm_env_status=0; else arm_env_status=$?; fi
    ((arm_env_nounset)) && set -u
    ((arm_env_status == 0)) || return "$arm_env_status"
  fi
  printf '[env] ROS=%s workspace=%s Python=%s\n' "${ROS_DISTRO:-unset}" "$ARM_WS_ROOT" "$(command -v python3)"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    _arm_env_main "$@"
    exit $?
  fi
  printf 'ERROR: source scripts/env.sh to keep its environment.\n' >&2
  exit 2
fi
if _arm_env_main "$@"; then
  unset -f _arm_env_main
  return 0
else
  unset -f _arm_env_main
  return 1
fi
