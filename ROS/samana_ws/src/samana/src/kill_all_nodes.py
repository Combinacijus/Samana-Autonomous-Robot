#!/usr/bin/env python
"""
    Sometimes nodes get stuck running in background even after roscore is terminated
    In this script are written all nodes that were catched not terminating properly
    And those are killed
"""

import re
import psutil


count = 0
print("KILLING THESE ROS PROCESSES:")

# Samana python nodes
for proc in psutil.process_iter():
    for line in proc.cmdline():
        LINE1 = '/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/'
        if re.search(LINE1, line) and proc.cmdline()[0] == 'python' and not re.search("kill_all_nodes.py", line):
            print(proc.cmdline())
            proc.kill()
            count += 1

# ROS library nodes
for proc in psutil.process_iter():
    for line in proc.cmdline():
        LINE1 = '/opt/ros/melodic/lib'
        # LINE2 = '/opt/ros/melodic/bin/roscore'
        # LINE3 = '/opt/ros/melodic/bin/rosmaster'
        # print(proc.cmdline())
        if re.search(LINE1, line): # or re.search(LINE2, line) or re.search(LINE3, line):
            print(proc.cmdline())
            proc.kill()
            count += 1

print("TOTAL PROCESSES KILLED: {}".format(count))

# ROS library nodes
for proc in psutil.process_iter():
    for line in proc.cmdline():
        LINE1 = '/opt/ros/melodic/bin/roscore'
        LINE2 = '/opt/ros/melodic/bin/rosmaster'
        if re.search(LINE1, line)or re.search(LINE2, line):
            proc.kill()