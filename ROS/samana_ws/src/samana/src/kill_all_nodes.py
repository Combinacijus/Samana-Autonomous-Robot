"""
    Sometimes nodes get stuck running in background even after roscore is terminated
    In this script are written all nodes that were catched not terminating properly
    And those are killed
"""

import os

os.system("killall move_base")
os.system("killall static_transform_publisher")