#!/usr/bin/env python

from helpers import BashRunner
import subprocess

BASE_DIR = subprocess.check_output("rospack find samana", shell=True).rstrip()


def bag_play(filename, rate=4):
    return "rosbag play {}/../../bags/{}.bag --clock -r {}".format(BASE_DIR, filename, rate)


def map_save(map_name):
    return "rosrun map_server map_saver --occ 100 --free 10 -f ~/Desktop/ros-maps/{} map:=/move_base/local_costmap/costmap".format(map_name)


cmd = BashRunner()
cmd.add("pwd")  # samana_ws directory
cmd.add("roslaunch samana sonar_research.launch", delay=2.0)
# cmd.add(bag_play("room_4_lside_sonar", rate=5))
# cmd.add(bag_play("room_map_3_sonar", rate=4.5))
cmd.add(bag_play("room_map_4_sonar", rate=4.5))
# cmd.add(map_save("map4_rside"))
print(cmd)
cmd.run()
