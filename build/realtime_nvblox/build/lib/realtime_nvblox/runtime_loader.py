from __future__ import annotations

"""Helpers that make wheel-bundled native libraries win over ROS/workspace copies.

The Python-first runtime deliberately does not depend on Isaac ROS.  Users may
still have ROS/Isaac workspaces sourced in an interactive shell, however.  Both
PyCuVSLAM and nvblox_torch ship Python bindings linked against specific native
libraries.  If a different library with the same SONAME is found first on
LD_LIBRARY_PATH, imports fail with an ``undefined symbol`` error.

These helpers locate the native libraries from the installed Python wheel and
load them by absolute path with RTLD_GLOBAL before importing the bindings.
"""

import ctypes
from importlib import metadata
from pathlib import Path
import threading


_LOCK = threading.RLock()
_CUVSLAM_LOADED: Path | None = None
_NVBLOX_LOADED: Path | None = None


def _dist_package_dir(distribution_name: str, package_name: str) -> Path:
    try:
        dist = metadata.distribution(distribution_name)
    except metadata.PackageNotFoundError as exc:
        raise ImportError(f'{distribution_name} is not installed.') from exc

    pkg = Path(dist.locate_file(package_name)).resolve()
    if not pkg.exists():
        raise FileNotFoundError(
            f'Installed distribution {distribution_name!r} does not contain {pkg}'
        )
    return pkg


def cuvslam_package_dir() -> Path:
    return _dist_package_dir('cuvslam', 'cuvslam')


def nvblox_torch_package_dir() -> Path:
    return _dist_package_dir('nvblox-torch', 'nvblox_torch')


def preload_cuvslam_runtime() -> Path:
    """Preload the ``libcuvslam.so`` that belongs to the installed cuvslam wheel."""
    global _CUVSLAM_LOADED
    with _LOCK:
        if _CUVSLAM_LOADED is not None:
            return _CUVSLAM_LOADED

        pkg = cuvslam_package_dir()
        candidates = [
            pkg / 'libcuvslam.so',
            pkg / 'lib' / 'libcuvslam.so',
        ]
        candidates.extend(sorted(pkg.glob('**/libcuvslam.so')))
        lib = next((p.resolve() for p in candidates if p.exists()), None)
        if lib is None:
            searched = '\n  '.join(str(p) for p in candidates[:8])
            raise FileNotFoundError(
                'Could not find wheel-bundled libcuvslam.so. Searched:\n  ' + searched
            )

        # Load the wheel library by absolute path before pycuvslam.so.  This is
        # the critical step when another libcuvslam.so is present in a sourced
        # ROS/Isaac workspace.
        ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)
        _CUVSLAM_LOADED = lib
        return lib


def preload_nvblox_runtime() -> Path:
    """Preload the ``libnvblox_lib.so`` that belongs to nvblox_torch wheel."""
    global _NVBLOX_LOADED
    with _LOCK:
        if _NVBLOX_LOADED is not None:
            return _NVBLOX_LOADED

        # Importing torch first loads libtorch/c10 and their CUDA dependencies.
        import torch  # noqa: F401

        pkg = nvblox_torch_package_dir()
        lib = (pkg / 'lib' / 'nvblox' / 'libnvblox_lib.so').resolve()
        if not lib.exists():
            raise FileNotFoundError(f'nvblox_torch bundled library not found: {lib}')

        ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)
        _NVBLOX_LOADED = lib
        return lib
