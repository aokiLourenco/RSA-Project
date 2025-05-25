import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aoki/RSA/goal_remap_ws/src/goal_remap/install/goal_remap'
