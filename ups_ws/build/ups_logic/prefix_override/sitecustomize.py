import sys
if sys.prefix == '/home/sk/miniconda3/envs/drone_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sk/ups_ws/install/ups_logic'
