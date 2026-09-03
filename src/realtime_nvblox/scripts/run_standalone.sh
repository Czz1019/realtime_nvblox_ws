#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 -m realtime_nvblox.cli --config "$ROOT/config/default.yaml" "$@"
