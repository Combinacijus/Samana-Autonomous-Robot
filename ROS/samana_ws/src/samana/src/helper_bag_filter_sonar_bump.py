#!/usr/bin/env python

from helpers import BashRunner
import subprocess

BASE_DIR = subprocess.check_output("rospack find samana", shell=True).rstrip()

class Data:
    def __init__(self, fin, fout, topics):
        self.fin = fin
        self.fout = fout
        self.topics = topics


def bag_filter_sonar(fin, fout):
    # Leave only needed topic in bag file for sonar research
    return """rosbag filter {0}/../../bags/{1}.bag {0}/../../bags/{2}.bag "topic == '/bump' or topic == '/sonar' or topic == '/tf' or topic == '/tf_static'" """.format(BASE_DIR, fin, fout)


def bag_to_csv(filename, topic):
    return "rostopic echo -b {0}/../../bags/{1}_{2}.bag -p {3} > {1}_{2}.csv".format(BASE_DIR, filename, topic[1:], topic)


data = []
# data.append(Data("room_4_rside_sonar", "room_4_lside_sonar", ["/sonar", "/bump"]))
data.append(Data("room_4_rside_sonar", "room_4_rside", ["/sonar",]))
data.append(Data("room_4_lside_sonar", "room_4_lside", ["/sonar",]))

cmd = BashRunner()
cmd.add("pwd")  # samana_ws directory
for d in data:
    # cmd.add(bag_filter_sonar(d.fin, d.fout))  # Filter data
    for topic in d.topics:
        cmd.add(bag_to_csv(d.fout, topic))  # Convert to CSV

print(cmd)
cmd.run()
