#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 backups/<backup>.tar.gz" >&2
  exit 2
fi

BACKUP="$1"
if [[ "$BACKUP" != /* ]]; then
  BACKUP="$ROOT/$BACKUP"
fi
if [[ ! -f "$BACKUP" ]]; then
  echo "Backup not found: $BACKUP" >&2
  exit 2
fi

# Preserve the current complete workspace state before a destructive restore.
"$ROOT/scripts/backup_workspace.sh"
rm -rf "$ROOT/src" "$ROOT/build" "$ROOT/install" "$ROOT/log"
rm -f "$ROOT/README.md" "$ROOT/COMMANDS.md" "$ROOT/.gitignore"
tar -xzf "$BACKUP" -C "$ROOT"

echo "Restored workspace from: $BACKUP"
echo "If the restored archive did not contain build artifacts, rebuild with:"
echo "  colcon build --symlink-install --executor sequential"
