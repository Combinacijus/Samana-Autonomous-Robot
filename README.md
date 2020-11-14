<p align="center"> README IS WORK IN PROGRESS!!! </p>
  <h1 align="center">Samana -<br> Autonomous Mobile Robot</h1>
  <p align="center">GPS waypoints following differential drive mobile robot based on Robot Operating System (ROS) Melodic. It was built to compete in <a href="http://www.robotsintellect.com/en/competitions">Robots’ Intellect 2020</a>.</p>
 
## Table of Contents

* [Intro](#intro)
* [About the Competition](#about-the-competition)

---
* [About the Robot](#about-the-robot)
  * [Hardware](#hardware)
     * [Base](#base)
     * [Gripper](#gripper)
     * [Electronics](#electronics)
     
  * [Software](#software)
     * [Arduinos](#arduinos)
	 * [ROS](#ros)
	 * [Object detection](#object-detection)
---

* [Getting Started](#getting-started)
  * [Installation](#installation)
* [License](#license)
* [Contact](#contact)


## Intro
This project was developed over the period of 16 months single-handedly by me. During this time I made some bad and questionable design decisions which led to badly failed attempts at the competition.

The main **purpose of this README file** is to share gained knowledge and hopefully give some insights into development of autonomous robots.

**Code and whole project** is not well documented so it's for those people who are not afraid to dig into code and analyze it line by line. Although some code might contain valuable comments and you might find template scripts so don't be afraid to check it out. 
Some files (e.g *.config) could be used as a reference for your own ROS based robot.

**Hardware:** robot frame is custom made and I don't have any detailed blueprints for whole robot but I'll share main components and why some design decisions were made and what to avoid etc.

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


## About the Robot
It's built on a tight budget which resulted in questionable and bad design decisions .........................
### Hardware
     ### Base
     
     ### Gripper
     
     ### Electronics
     
  * [Software](#software)
	  

## License

Distributed under the MIT License. See `LICENSE` for more information.


## Contact
<p> Name: &nbsp; &nbsp; &nbsp;Gintaras Grėbliūnas </p>
<p> Email: &nbsp; &nbsp; &nbsp; combinacijus@gmail.com</p>
<p> LinkedIn:  &nbsp;https://www.linkedin.com/in/gintaras-gr%C4%97bli%C5%ABnas-72214a119/</p>
