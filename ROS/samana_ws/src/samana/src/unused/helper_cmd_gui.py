#!/usr/bin/env python

"""
    This program allows to excecute bash commands via GUI by selecting it
    from the list. It also allows to edit list via GUI or yaml file.
"""

from easygui import *
import yaml
import os
import threading
import subprocess

BASE_DIR = subprocess.check_output("rospack find samana", shell=True).rstrip()
OPTIONS_PATH = BASE_DIR + "/config/helper_cmd_gui.yaml"
print(OPTIONS_PATH)


def save_options(data, filename=OPTIONS_PATH):
    with open(filename, "w") as f:
        yaml.dump(data, f)


def load_options(filename=OPTIONS_PATH):
    try:
        data = None
        with open(filename, "r+") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
    except Exception as e:
        print(e)
        return []

    return data


ADD = "Add new command"
DELETE = "Delete command"
EDIT = "Edit command"
FLAG_THREAD = "!"
choice = None
output = ""

base_commands = [ADD, EDIT, DELETE]
commands = []
commands.extend(load_options())

while True:
    choice = choicebox("Type {}<cmd> to run in separate thread.\n\
    Press Cancle to exit or go back. It's possible to copy commands here. Scroll.\n\
    All commands saved at {}\n\
    Last cmd: {}\n\
    Last output: {}".format(FLAG_THREAD, OPTIONS_PATH, choice, output.replace("\n", "  |  ")),
    "Bash runner", base_commands + commands)

    if not choice:
        break

    if choice not in base_commands:  # Run command
        if choice[0] == FLAG_THREAD:  # Start in separate thread
            threading.Thread(target=os.system, args=(choice[1:],)).start()
        else:
            output = subprocess.check_output(choice, shell=True)  # Run in blocking mode
    elif choice == ADD:  # Add command
        cmd = enterbox("Add new command", "Add new command")
        if cmd:
            commands.append(cmd)
            save_options(commands)
    elif choice == EDIT:  # Edit command
        choice_edit = choicebox("Choose command to EDIT!", "Bash runner", commands)
        if choice_edit:
            cmd = enterbox("Edit command", "Edit command", choice_edit)
            if cmd:
                commands[commands.index(choice_edit)] = cmd
                choice_edit = cmd
                save_options(commands)
    elif choice == DELETE:  # Delete command
        choice_del = choicebox("Choose command to DELETE!", "Bash runner", commands)
        if choice_del:
            commands.remove(choice_del)
            save_options(commands)
