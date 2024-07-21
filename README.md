# SEAL
It is a ROS package that implements a SEAL: Simultaneous exploration and localization for multi-robot systems. It uses the Gaussian process model for environment scanning and occupancy girds as a map representation. This package has three different ROS nodes:

  - SEAL graph node.
  - SEAL Gaussian process node.
  - SEAL convex hull node.

# Publication
If you use this work, please cite our paper:
E. Latif and R. Parasuraman, "SEAL: Simultaneous Exploration and Localization for Multi-Robot Systems," 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Detroit, MI, USA, 2023, pp. 5358-5365, doi: 10.1109/IROS55552.2023.10342157.

IEEE Published version: [https://ieeexplore.ieee.org/document/10342157](https://ieeexplore.ieee.org/document/10342157)
Preprint available at [https://arxiv.org/abs/2306.12623](https://arxiv.org/abs/2306.12623)

# Experiment Demonstration Video

[![Experiment Demo](https://img.youtube.com/vi/zVUjiPdgYIg/0.jpg)](https://www.youtube.com/watch?v=zVUjiPdgYIg)



## Overview
By creating a map using GP, robots first perform relative position-weighted connectivity graphs using RSSI as local sensor data, expanding these graphs based on potential positions at a particular location and then further optimizing to obtain relative position estimates for all connected robots; SEAL seeks to efficiently achieve high localization accuracy. Furthermore, robots also provide fused Gaussian process maps, which are then converted into global maps. SEAL applied convex hull optimization for boundary detection and navigated robots to unexplored boundaries. An overview of the SEAL can be found in the Figure below:

![Overview](/images/seal_overview.png)
## Architecture
The overall functionality of SEAL with respect to single robot and information sharing can be seen in the architecture figure below:

![Overview](/images/seal_architecture_v2.png)

## Installation Requirements
* C++ requirements.   
([pybind11](https://github.com/pybind/pybind11) is also required, but it's built into this repository; you don't need to install)
* python 3.6+
* [Turtlebot Simulation Installation](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation#Installation_Types) is required to launch turtlebot 3e for Gazebo simulation
* [g2o installed](https://github.com/uoip/g2opy.git)

### g2o Installation
```
$ git clone https://github.com/uoip/g2opy.git
$ cd g2opy
$ mkdir build
$ cd build
$ cmake ..
$ make -j8
$ cd ..
$ python setup.py install
6-You should have/install the following python modules:

-OpenCV (cv2)
```sh
$ sudo apt-get install python-opencv
```
-Numpy
```sh
$ sudo apt-get install python-numpy
```
-Sklearn
```sh
$ sudo apt-get install python-scikits-learn
```
- add in the amazon world map by executing the following comments:
```
$ cd ~/catkin_explore/src
$ git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
$ git clone https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
$ cd ~/catkin_explore/
$ catkin_make
```

## 2. Installation
Download the package and place it inside the ```/src``` folder in catkin workspace. And then compile using ```catkin_make```.

## 3. Setting Up Your Robots
This package provides an exploration strategy for cooperative robot along with the multi-robot navigation stack configured move_base node.

### 3.1. Robots Network
For the cooperative robotic configuration, the package doesn't require special network configuration, it simply works by having a single ROS master (can be one of the robots). So on the other robots, the ```ROS_MASTER_URI``` parameter should be pointing at the master's address. 
For more information on setting up ROS on multiple machines, follow [this](http://wiki.ros.org/ROS/NetworkSetup) link.

### 3.2. Robot's frame names in ```tf```
All robot's frames should be prefixed by its name. Naming of robots starts from "/tb3_0", "/tb3_1", ... and so on.

### 3.3. Robot's node and topic names
All the nodes and topics running on a robot must also be prefixed by its name. For tb3_0, node names should look like:  ```/tb3_0/gp```.

And topic names should be like: ```/tb3_0/odom```,  ```/tb3_0/gp```,  ```/tb3_0/ch```, ..etc.


### 3.5. A Gaussian Process node
Each robot should have a Gaussian estimation map generated from the [GPMix](https://github.com/yangggzhang/Heterogeneous-Multi-Robot-Adaptive-Sampling.git) package.

### 3.6. A map Generator node
For the multi-robot case, there is a node that convert all the GP maps into one occupancy map. You can use [this](http://wiki.ros.org/multirobot_map_merge) package.

## 4. Launch
Run the SEAL package after installation on a robot and source bash and ~/catkin_explore/devel/setup.sh file:
```
$ mkdir -p catkin_explore/src
$ cd catkin_explore/src
$ git clone https://github.com/herolab-uga/ROS-SEAL.git
$ cd ~/catkin_explore/
$ catkin_make
$ roslaunch seal seal_bookstore.launch
```


## Core contributors

* **Ehsan Latif, Ph.D.** - Lab Alum

* **Dr. Ramviyas Parasuraman** - Lab Director


## Heterogeneous Robotics (HeRoLab)

**Heterogeneous Robotics Lab (HeRoLab), School of Computing, University of Georgia.** 

For further information, contact Ehsan Latif ehsan.latif@uga.edu or Dr. Ramviyas Parasuraman ramviyas@uga.edu

https://hero.uga.edu/

<p align="center">
<img src="https://herolab.org/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>
