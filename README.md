# Samana-Autonomous-Robot
Autonomous mobile robot Samana.
Main components: used hoverboard, ROS, Arduinos, Sensors

# Crude instruction for future

To build ROS library for Arduino:

- build samana_msgs
- source devel/setup.bash
- rm -r ~/Documents/Arduino/libraries/ros_lib/
- rosrun rosserial_arduino make_libraries.py /home/combinacijus/Documents/Arduino/libraries/
- If can’t include headers first include ros.h
- wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup


To flash STM32:

- cd to project folder
- make clean; make
- st-flash --reset write build/hover.bin 0x8000000


To accept Arduino node to roscore:

- sudo chmod 666 /dev/ttyUSB*
- rosrun rosserial_arduino serial_node.py _baud:=1000000 _port:=/dev/ttyUSB3 _recal:=false

Object recognision:

- YoloV3 training algorithm will automatically re-scale image to sizes between 288 to 488

