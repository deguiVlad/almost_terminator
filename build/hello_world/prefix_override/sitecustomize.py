import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/nssr_sim/hackathon/ros2/install/hello_world'
