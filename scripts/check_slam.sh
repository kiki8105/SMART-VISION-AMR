#!/usr/bin/env bash
# Exercise a fresh SLAM simulation through the existing PyQt controller.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "$SCRIPT_DIR/../../.." && pwd)
if [[ "${1:-}" == --workspace ]]; then
  if [[ $# -lt 2 || -z "$2" ]]; then
    printf 'Usage: check_slam.sh [--workspace WORKSPACE_ROOT] [VERIFIER_ARGUMENTS ...]\n' >&2
    exit 2
  fi
  WORKSPACE_ROOT=$2
  shift 2
fi
# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
export QT_QPA_PLATFORM=offscreen
printf '+ python3 %q' "$SCRIPT_DIR/verify_slam.py"
if [[ $# -gt 0 ]]; then printf ' %q' "$@"; fi
printf '\n'
exec /usr/bin/python3 "$SCRIPT_DIR/verify_slam.py" "$@"
