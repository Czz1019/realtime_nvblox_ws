#!/usr/bin/env bash
set -euo pipefail

ROOT="${REALTIME_NVBLOX_ROOT:-$HOME/realtime_nvblox_ws/src/realtime_nvblox}"
CONFIG="${REALTIME_NVBLOX_CONFIG:-$ROOT/config/default.yaml}"

if [[ ! -d "$ROOT/realtime_nvblox" ]]; then
  echo "ERROR: realtime_nvblox project not found: $ROOT" >&2
  exit 2
fi

# Re-exec in an intentionally clean process environment.  This makes the
# command safe even when ~/.bashrc has sourced Isaac ROS or other workspaces.
exec env -i \
  HOME="$HOME" \
  USER="${USER:-$(id -un)}" \
  PATH="$HOME/.local/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  LD_LIBRARY_PATH="/usr/local/cuda/lib64" \
  ROOT="$ROOT" \
  CONFIG="$CONFIG" \
  bash --noprofile --norc -c '
    set -euo pipefail
    cd "$ROOT"

    CUV_DIR=$(python3 - <<"PY"
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution("cuvslam").locate_file("cuvslam")).resolve())
PY
    )

    NVB_DIR=$(python3 - <<"PY"
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution("nvblox-torch").locate_file("nvblox_torch")).resolve())
PY
    )

    TORCH_LIB=$(python3 - <<"PY"
import os, torch
print(os.path.join(os.path.dirname(torch.__file__), "lib"))
PY
    )

    export LD_LIBRARY_PATH="$CUV_DIR:$NVB_DIR/lib/nvblox:$TORCH_LIB:/usr/local/cuda/lib64"

    echo "[realtime_nvblox] clean runtime"
    echo "[realtime_nvblox] config: $CONFIG"
    exec python3 -m realtime_nvblox.cli --config "$CONFIG" "$@"
  ' bash "$@"
