import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/czz/realtime_nvblox_ws/install/realtime_nvblox'
