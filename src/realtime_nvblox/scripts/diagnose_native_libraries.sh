#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
from importlib import metadata
from pathlib import Path

for dist_name, pkg_name, py_pattern, core_name in [
    ('cuvslam', 'cuvslam', 'pycuvslam*.so', 'libcuvslam.so'),
    ('nvblox-torch', 'nvblox_torch', 'lib/nvblox_torch/cpp/libpy_nvblox.so', 'lib/nvblox/libnvblox_lib.so'),
]:
    d = metadata.distribution(dist_name)
    pkg = Path(d.locate_file(pkg_name)).resolve()
    print(f'{dist_name}: {pkg}')
    if '*' in py_pattern:
        matches = list(pkg.glob(py_pattern))
    else:
        matches = [pkg / py_pattern]
    for m in matches:
        print(' binding:', m)
    print(' core   :', pkg / core_name)
PY

echo
echo 'LD_LIBRARY_PATH entries:'
echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | nl -ba

echo
CUV_DIR=$(python3 - <<'PY'
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution('cuvslam').locate_file('cuvslam')).resolve())
PY
)
CUV_BIND=$(find "$CUV_DIR" -maxdepth 2 -name 'pycuvslam*.so' -print -quit)
if [[ -n "$CUV_BIND" ]]; then
  echo 'cuVSLAM binding resolution:'
  ldd "$CUV_BIND" | grep -E 'libcuvslam|not found|cublas|cusolver|cusparse|cudart' || true
fi

echo
NVB_DIR=$(python3 - <<'PY'
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution('nvblox-torch').locate_file('nvblox_torch')).resolve())
PY
)
NVB_BIND="$NVB_DIR/lib/nvblox_torch/cpp/libpy_nvblox.so"
if [[ -f "$NVB_BIND" ]]; then
  echo 'nvblox binding resolution:'
  ldd "$NVB_BIND" | grep -E 'libnvblox_lib|libtorch|libc10|not found' || true
fi
