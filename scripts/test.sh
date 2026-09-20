#!/usr/bin/env bash
# Existing unit/contract tests only: no hardware or live demo is launched.
set -Eeuo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: bash scripts/test.sh [functional|all] [colcon test arguments...]
functional (default): run unit/contract tests and XML validation; exclude style tests.
all: include existing flake8, pep257, copyright, and CMake style checks.
Results and full console output are retained below log-release/test-<mode>-<time>.
Both modes use build-release/install-release and require a preceding build.
Tests use ROS domain 194, localhost, and Qt offscreen; they do not accept hardware.
Example: bash scripts/test.sh functional --packages-select arm_driver gripper_driver camera_perception
EOF
  exit 0
fi
arm_test_mode="${1:-functional}"
if [[ "$arm_test_mode" != functional && "$arm_test_mode" != all ]]; then
  printf 'ERROR: first argument must be functional or all.\n' >&2
  exit 2
fi
(($# == 0)) || shift
arm_script_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$arm_script_root/scripts/env.sh" ''
if [[ ! -r "$ARM_INSTALL_BASE/local_setup.bash" ]]; then
  printf 'ERROR: build first with bash scripts/build.sh.\n' >&2
  exit 1
fi
export ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 QT_QPA_PLATFORM=offscreen
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 MPLBACKEND=Agg
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
arm_test_run="$ARM_LOG_BASE/test-$arm_test_mode-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$arm_test_run"
export ROS_LOG_DIR="$arm_test_run/ros"
arm_test_extra=(--pytest-args -p no:cacheprovider)
if [[ "$arm_test_mode" == functional ]]; then
  arm_test_extra+=(-k 'not flake8 and not pep257 and not copyright')
  arm_test_extra+=(--ctest-args -E 'flake8|pep257|lint_cmake|copyright')
fi
cd -- "$ARM_WS_ROOT"
arm_test_status=0
python3 "$(command -v colcon)" --log-base "$arm_test_run/colcon" test \
  --base-paths "$ARM_WS_ROOT/src" --build-base "$ARM_BUILD_BASE" \
  --install-base "$ARM_INSTALL_BASE" --test-result-base "$arm_test_run/results" \
  --return-code-on-test-failure \
  --python-testing pytest --executor sequential \
  --event-handlers desktop_notification- "$@" "${arm_test_extra[@]}" \
  2>&1 | tee "$arm_test_run/console.log" || arm_test_status=$?
arm_result_status=0
python3 "$(command -v colcon)" test-result --test-result-base "$arm_test_run/results" --verbose \
  2>&1 | tee "$arm_test_run/results.log" || arm_result_status=$?
printf '[test] mode=%s colcon=%s results=%s logs=%s\n' \
  "$arm_test_mode" "$arm_test_status" "$arm_result_status" "$arm_test_run"
printf '[test] Software results do not validate physical motion, cameras, or calibration.\n'
((arm_test_status == 0 && arm_result_status == 0))
