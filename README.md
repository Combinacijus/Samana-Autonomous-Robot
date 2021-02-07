
 <h1 align="center">Samana -<br> Autonomous Mobile Robot</h1>
  <p align="center">GPS waypoints following differential drive mobile robot based on Robot Operating System (ROS) Melodic. It was built to compete in <a href="http://www.robotsintellect.com/en/competitions">Robots’ Intellect 2020</a>.</p>
 
## Table of Contents

* [Intro](#intro)
* [About the Competition](#about-the-competition)
* [Getting Started](#getting-started)
  * [Installation](#installation)
  * [Design decisions](#design-decisions)
* [License](#license)
* [Contact](#contact)


## Intro
This project was developed over the period of 16 months single-handedly. During this time I made some bad and questionable design decisions which led to badly failed attempts at the competition.
The main **purpose of this README file** is to share gained knowledge and hopefully give some insights into development of autonomous robots.

![Bag](https://user-images.githubusercontent.com/16375702/106159061-17a67500-618d-11eb-9af0-52d4ec398bea.gif)
![Wheelie](https://user-images.githubusercontent.com/16375702/106160997-1d9d5580-618f-11eb-997c-e0ad831e3954.gif)

[More images and GIFs](https://github.com/Combinacijus/Samana-Autonomous-Robot/discussions/5)


**Code and whole project** is not well documented so it's for those people who are not afraid to dig into code and analyze it line by line. Although some code might contain valuable comments and you might find template scripts so don't be afraid to check it out. 
Some files (e.g *.config) could be used as a reference for your own ROS based robot.

**Hardware:** robot frame is custom made and I don't have any detailed blueprints for whole robot.

## About The Competition
From official [competition rules](http://www.robotsintellect.com/files/Golden_bag_search_EN_2020_newdocx_.pdf):
> An autonomous mobile robot has to, without any help from any person,
> drive from the starting point to the searching area (indicated by
> specific markings), find a “golden” bag and bring it back to the start
> in under 1 hour.
![](https://lh4.googleusercontent.com/9BSednJBsztT-BAT3jis3229QG2WgfV8W7t6h3jD0mqHqL_8I2yx2hXn2j_y8EQgOsk95g_Ku9j5cw1NDXepri1-uMbAVjRwZb2swq-Y0MALB_mGW9DauIGSjuyE3ryAIu9fI-Ij)

- Start area is indicated by green rectangle0
- Bag search area is indicated by red rectangle
- One way course distance ~273m

## Getting Started
This repository contains almost all project files in a way that it would be as easy as possible **for me** to just simply clone repository without unnecessary installation steps and have all project files in one place. Furthermore, repo acted as a project backup.
Note that "Robot Notes.docx" file contains notes **for myself** therefore it can be confusing.

Feel free to fork this repo and use it for your own needs.

### Installation

1. Clone the repo.
```sh
git clone https://github.com/combinacijus/Samana-Autonomous-Robot.git
```
2. For some further installation/setup steps refer to: [Robot Notes](https://github.com/Combinacijus/Samana-Autonomous-Robot/blob/master/Documentation/Robot%20Notes.docx) "**New PC setup**" section. Note that "combinacijus" is my username and you should change it.
3. Explore. It's not intended for any specific use due do custom robot and no detailed instructions to built it.

Some interesting directories to check:
- [ROS catkin workspace](https://github.com/Combinacijus/Samana-Autonomous-Robot/tree/master/ROS/samana_ws)
- [Arduino files](https://github.com/Combinacijus/Samana-Autonomous-Robot/tree/master/Arduino)
- [Object detection files](https://github.com/Combinacijus/Samana-Autonomous-Robot/tree/master/Python/GoldBagDetector)
- [CAD drawings](https://github.com/Combinacijus/Samana-Autonomous-Robot/tree/master/SamanaPartDrawings)

### Design decisions

**Used hoverboard for base of the robot**
- Pros
	- VERY good price for performance
	- Powerful motors and ESC, big battery pack, strong build capable of carrying human
	- Already [hacked firmware](https://github.com/NiklasFauth/hoverboard-firmware-hack) for control via UART. Also my [repo with Arduino examples](https://github.com/Combinacijus/hoverboard-firmware-hack/tree/master/ArduinoExamples).
- Cons
	- With great power comes great WEIGHT (~11kg). That's not something you can just throw in your backpack
	- With great power also comes great crashes when accidentally it full throttles into a wall (just saying)
	- It's ~60cm in width. Big footprint means it's harder to navigate through tight spaces. Also it's harder to navigate in general because ratio of distance sensor range and robot's footprint is smaller combining it with non-symetrical footprint it becomes really hard to drive autonomously without colliding while turning.

**5 Arduino Nano for interfacing with hardware**
- Why?
	- Arduino supported by ROS
	- It was modular every sensor/group of sensors had it's own Arduino
	- I didn't want to try unfamiliar microcontroller because lack of support from ROS
	- Arduino Nano is slow microcontroller therefore it was impossible to process all data on single one nor to fit all code (half of program memory was used just by ROS)
- Problems
	- 5 Arduinos = 5 USB cables = 5 devices communicating with computer
	- Most of the sensors was powered from Arduino and simple USB hub wasn't enough to supply enough current therefore external power via left over USB was needed
	- Mechanical connection was hard to guarantee for all 5 USBs
	- Communication was sometimes unreliable or even crashed computer for unknown reason. I guess it didn't like to have 5 different devices on same USB port
- Solutions
	- If possible try to use single powerful microcontroller for all sensors
	- Power sensors from external power supply (voltage regulator) to avoid power related problems

**Ultrasonic (sonar) sensors for SLAM**
- Brief
	- I used 10 SR04 ultrasonic sensors all around robot to imitate crude lidar. All sensors are triggered at once at ~20Hz (waiting period is enough to not catch any previous echoes). I was worried for cross-talk but it was unnoticeable. So there was ~200readings/sec which would be enough but sonar data is very noisy and without  heavy filtering built map was unusable but after filtering most of the readings were discarded and delayed which degraded map building. Also ROS didn't have support for localization part of SLAM for sonars. Therefore DON'T USE ULTRASONIC SENSOR FOR SLAM better use stereo camera.

- Pros
	- Fairly cheap
	- Works on different surfaces than other sensors
- Cons
	- Very noisy measurements
	- Is wall is >12-15deg from perpendicular it won't be detected
	- Ultrasound can bounce around corners giving bad readings
	- Wide detection angle (not desirable for SLAM)
	- Not very too accurate
	-  Effective range is reduce due to widening ultrasound cone which can bounce back from small objects on the ground like: grass, pebbles, tiles
	- Slow measurement rate limited by speed of sound (around 20Hz if assumed max range of ~5-6m)
- Solution
	- Lidar is way better than ultrasound sensors array but cheap lidars might not work outside in direct sunlight. But lidar has fixed height and in some environments height of and obstacles might be lower than lidar itself.
	- Stereo camera seems to be best solution which was proven good by other robot which actually completed the track. Stereo camera has good range, good details and non-fixed height which is good for outdoor environments.

**Laptop as main computing power**
		-  Because my robot was big and had enough power to carry a laptop it was a best choice. Laptop is more powerful than usual single board computers like Jetson Tx/Nano or Raspberry Pi. Due to big map for navigation and YoloV3 object detection model which was still slow even on a gpu there was no way I could optimize and cram everything into single board computer (although I don't say it's impossible). Also laptop was charging from robot's battery and it used way more power than all the other electronics and driving combined. So it wasn't power efficient but I needed a lot of computing power.

**Grabbing mechanism made from plywood?**
- Worked well enough but tolerances wasn't very good. Definitely recommend using cnc or 3d printer for such parts.

**Localization**
- Data: odometry, IMU, GPS
- Tried cheap GPS module but it was very inaccurate
- Used android phone as GPS module and in my opinion it is as good as simple GPS receivers for reasonable price. But in testing it turned out that in best case accuracy was 3-5m radius circle and near some building 10-20m which is totally unusable for ROS navigation stack. Basically with such accuracy robot thought it was out of course perimeter which broke path finding.
- Solutions:
	- Don't use algorithms which depends on accurate position estimate
	- Use expensive RTK GPS modules with base station which could provide "centimeter-level accuracy"
	- If allowed use global markers/fiducials which are accurately positioned in know locations

**Obeject Detection**
- For object detection I used [ImageAI](https://github.com/OlafenwaMoses/ImageAI) library with YoloV3 and custom object detection model trained on "gold bag" images. Detection worked well but it's GPU heavy on Nvidia GT 740M it run at ~1.4 FPS which is slow but enough also it didn't use much of CPU which is good.
- Custom model was trained in Jupyter notebook on [Google Colab](https://colab.research.google.com/) GPU enabled server **for free**. Training on ~600 images took about 2-3 hours
- Some notes on how to train model and link to my Jupyter notebook is available at [Robot Notes.docx](https://github.com/Combinacijus/Samana-Autonomous-Robot/blob/master/Documentation/Robot%20Notes.docx) -> Object Detection -> Training new model.

**Final notes**
- Those blue and white thingies around the robot are fancy limit switches which can be pressed from any angle with a little force. Also it can bend which allows crashing into a wall without worrying to break limit switches.
- Big robot -> Long wires  +  Many sensors -> Many wires  =  A mess
- It's fun to manually drive because it's way too overpowered. Its acceleration is limited just by grip. Also it can carry a human which is even more fun to ride.

## License

Distributed under the MIT License. See `LICENSE` for more information.


## Contact
<p> Name: &nbsp; &nbsp; &nbsp;Gintaras Grėbliūnas </p>
<p> Email: &nbsp; &nbsp; &nbsp; combinacijus@gmail.com</p>
<p> LinkedIn:  &nbsp;https://www.linkedin.com/in/gintaras-gr%C4%97bli%C5%ABnas-72214a119/</p>
