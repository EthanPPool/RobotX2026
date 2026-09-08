#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROOT/backups"
OUT="$ROOT/backups/uav_ws_${STAMP}.tar.gz"

# Preserve the complete workspace state, including build/install/log. Only the
# backup directory itself is excluded to avoid recursively archiving backups.
tar -C "$ROOT" -czf "$OUT" \
  --exclude='./backups' \
  .

echo "Backup created: $OUT"
