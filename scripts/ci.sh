#!/usr/bin/env bash
# Public CI subset that does not require redistribution of CAD/URDF assets.
set -Eeuo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: bash scripts/ci.sh
Requires an installed ROS 2 Humble development environment and dependencies.
Checks source structure and runs functional tests for arm_driver, gripper_driver,
and camera_perception, plus launch parameter contracts without node execution.
This is partial software CI, not full model/MoveIt,
camera, gripper, or hardware acceptance. No dependencies are installed here.
EOF
  exit 0
fi
if (($#)); then printf 'ERROR: ci.sh accepts no arguments.\n' >&2; exit 2; fi
arm_script_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE-1}"
python3 "$arm_script_root/scripts/check_structure.py" --software-only
bash "$arm_script_root/scripts/build.sh" \
  --packages-select arm_driver gripper_driver camera_perception
bash "$arm_script_root/scripts/test.sh" functional \
  --packages-select arm_driver gripper_driver camera_perception
# Resolve launch defaults/forwarding without executing any launch Node action.
source "$arm_script_root/scripts/env.sh"
mkdir -p "$ARM_LOG_BASE"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$ARM_LOG_BASE/launch-contract-ros"
python3 -m pytest -q -p no:cacheprovider \
  "$arm_script_root/src/arm_console/test/test_launch_command_safety.py" \
  --junitxml="$ARM_LOG_BASE/launch-contract.xml"
