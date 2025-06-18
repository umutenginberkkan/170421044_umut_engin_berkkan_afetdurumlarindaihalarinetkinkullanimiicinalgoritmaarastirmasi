import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu22/Desktop/Proje/ros2_ws/install/uav_pack'
