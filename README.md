# SEAL
It is a ROS package that implements a SEAL: Simultaneous exploration and localization for multi-robot systems. It uses gausian process model for environemnt scanning and occupancy girds as a map representation.The packgae has 3 different ROS nodes:

  - Seal graph node.
  - Seal guassian process node.
  - Seal convex hull node.
 
## Overview
By creating map using gp, robots first perform relative position-weighted connectivity graphs using RSSI as local sensor data, expanding these graphs based on potential positions at a particular location, and then further optimizing to obtain relative position estimates for all connected robots, DGORL seeks to efficiently achieve high localization accuracy. Furthermore, robots also provide fused gaussian process map which then convert into global map. SEAL applied convex hull optimization for boundary detection and navigate robots to the unexplored boundries. An overview of the SEAL can be found in Figure below:

![Overview](/images/seal_overview.png)
## Architecture
Overall functionality of SEAL with respect to single robot and information sharing can be seen in the architectre figure below:

![Overview](/images/seal_architecture.png)

## Installation Requirements
* C++ requirements.   
([pybind11](https://github.com/pybind/pybind11) is also required, but it's built in this repository, you don't need to install)
* python 3.6+
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
Each robot should have a gaussian estimation map generated from the [GPMix](https://github.com/yangggzhang/Heterogeneous-Multi-Robot-Adaptive-Sampling.gitg) package.

### 3.6. A map Generator node
For the multi-robot case, there is a node that convert all the GP maps into one occu[ancy map. You can use [this](http://wiki.ros.org/multirobot_map_merge) package.

## 4. Launch
Run the SEAL package after installation on a robot and source bash and /devel/setuup.sh file:
```
$ git clone https://github.com/herolab-uga/ROS-SEAL.git
$ cd ~/catkin_explore/
$ catkin_make``
$ roslaunch seal seal_bookstore.launch
```







