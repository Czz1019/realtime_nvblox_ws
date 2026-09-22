from __future__ import annotations

import argparse
import json
import time

from realtime_nvblox.config import load_config
from realtime_nvblox.runtime import RealtimeNvbloxRuntime


def main():
    parser = argparse.ArgumentParser(description='RealSense + PyCuVSLAM + nvblox_torch realtime mapping')
    parser.add_argument('--config', default=None)
    parser.add_argument('--duration', type=float, default=0.0, help='Seconds; 0 means until Ctrl+C')
    args = parser.parse_args()
    cfg = load_config(args.config)
    if cfg.get('pose_source', 'cuvslam') == 'robot':
        parser.error('Robot pose input requires ros_mapper; use the ROS node with this config.')
    rt = RealtimeNvbloxRuntime(cfg)
    rt.on('stats', lambda x: print(json.dumps(x, separators=(',', ':'))))
    rt.start()
    start = time.monotonic()
    try:
        while True:
            if rt.error is not None:
                raise RuntimeError(rt.error)
            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        rt.stop()


if __name__ == '__main__':
    main()
