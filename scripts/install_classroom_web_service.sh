#!/usr/bin/env bash
# Register the website only; robot bringup/navigation stays under operator control.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
UNIT_NAME=smart-vision-classroom-web.service
TEMPLATE=$SCRIPT_DIR/systemd/$UNIT_NAME
WEB_SCRIPT=$SCRIPT_DIR/run_classroom_web_16.sh
UNIT_DIR=${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user
START=false
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage: install_classroom_web_service.sh [--start] [--dry-run]

Install and enable the current user's classroom website service.
The default does not start or restart the website.

  --start     Also start the service after installation.
              Stop an existing manually launched website first (same port 8790).
  --dry-run   Render and verify the unit, without installing or changing services.
  -h, --help  Show this help.

Run without sudo. To run after boot even before login, the account owner can run:
  loginctl enable-linger "$USER"
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start) START=true ;;
    --dry-run) DRY_RUN=true ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if (( EUID == 0 )); then
  printf 'Run this installer as the desktop user, without sudo.\n' >&2
  exit 1
fi
if [[ ! -f "$TEMPLATE" || ! -x "$WEB_SCRIPT" ]]; then
  printf 'Missing service template or executable launcher: %s\n' "$SCRIPT_DIR" >&2
  exit 1
fi
for command in systemctl systemd-analyze /usr/bin/python3; do
  command -v "$command" >/dev/null || {
    printf 'Required command not found: %s\n' "$command" >&2
    exit 1
  }
done

TEMP_DIR=$(mktemp -d)
trap 'rm -rf -- "$TEMP_DIR"' EXIT
RENDERED_UNIT=$TEMP_DIR/$UNIT_NAME

/usr/bin/python3 - "$TEMPLATE" "$RENDERED_UNIT" "$REPO_DIR" "$WEB_SCRIPT" <<'PY'
from pathlib import Path
import sys

template, destination, repo, launcher = sys.argv[1:]

def unit_quote(value):
    # systemd expands percent specifiers; quote spaces and backslashes as well.
    if '\n' in value or '\r' in value:
        raise SystemExit('Service paths cannot contain newline characters.')
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'

content = Path(template).read_text()
# WorkingDirectory accepts an entire path value, unlike ExecStart's word list;
# surrounding quotes would become literal characters and make it non-absolute.
if '\n' in repo or '\r' in repo:
    raise SystemExit('Service paths cannot contain newline characters.')
content = content.replace('@REPO_DIR@', repo.replace('%', '%%'))
# A colon command prefix disables systemd's $VARIABLE expansion for this path.
content = content.replace('@WEB_SCRIPT@', ':' + unit_quote(launcher))
Path(destination).write_text(content)
PY

systemd-analyze --user verify "$RENDERED_UNIT"

if [[ "$DRY_RUN" == true ]]; then
  cat "$RENDERED_UNIT"
  printf '\nVerified. Would install: %s/%s\n' "$UNIT_DIR" "$UNIT_NAME"
  printf 'Would reload and enable the user service. Start requested: %s\n' "$START"
  exit 0
fi

# Check the user manager before writing anything into its unit directory.
systemctl --user show-environment >/dev/null
mkdir -p -- "$UNIT_DIR"
install -m 0644 -- "$RENDERED_UNIT" "$UNIT_DIR/$UNIT_NAME"
systemctl --user daemon-reload
systemctl --user enable "$UNIT_NAME"
if [[ "$START" == true ]]; then
  systemctl --user start "$UNIT_NAME"
fi

printf 'Installed and enabled: %s/%s\n' "$UNIT_DIR" "$UNIT_NAME"
if [[ "$START" != true ]]; then
  printf 'The running website was not restarted. Start when ready:\n'
  printf '  systemctl --user start %s\n' "$UNIT_NAME"
fi
printf 'Status: systemctl --user status %s\n' "$UNIT_NAME"
printf 'Boot without login: loginctl enable-linger "%s"\n' "$USER"
