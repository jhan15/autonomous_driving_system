# ADS - BFMC
Autonomous driving systems developed on a 1:10 scaling car and tested in ROS.

## Demo
![ads1](https://user-images.githubusercontent.com/62132206/121524593-becad100-c9f7-11eb-96ff-b69331704710.gif)

## System setup
### 1. Hierarchy overview

    ~/Documents
        |__ bfmc_workspace
        |__ startup_workspace
        |__ python3_ws

### 2. Installation

#### Install basic Python3 packages and virtual environment
    
    # requires python3.8 or later installed in default system
    $ sudo apt-get install python-catkin-tools python3-dev python3-numpy
    $ sudo pip install virtualenv
    $ cd ~/Documents
    $ mkdir -p python3_ws/src
    $ cd python3_ws
    $ virtualenv py3venv --python=python3.8
    $ source ~/Documents/python3_ws/py3venv/bin/activate

#### Install yolov5 and its dependencies

    $ cd ~/Documents/python3_ws/src
    # copy the yolov5 folder to src
    $ cd yolov5
    $ pip install -r requirements.txt
    $ pip install -e .
    $ pip install gym

#### Install other dependencies

    $ cd ~/Documents/python3_ws/src
    # copy geometry and vision_opencv folders to src
    $ pip install pyaml
    $ pip install rospkg
    $ pip install empy
    $ cd ~/Documents/python3_ws
    $ catkin_make -DPYTHON_EXECUTABLE:FILEPATH=/home/<user>/Documents/python3_ws/py3venv/bin/python
    $ source devel/setup.bash

#### Compile startup package with Python3 workspace

    $ cd ~/Documents/startup_workspace
    $ rm -rf build devel
    $ catkin_make -DPYTHON_EXECUTABLE:FILEPATH=/home/<user>/Documents/python3_ws/py3venv/bin/python
    $ source devel/setup.bash

### 3. Launch

    # put below line as first line of each .py file
    #!/usr/bin/env python
    
    # Terminal 1
    $ source ~/Documents/python3_ws/py3venv/bin/activate
    $ cd ~/Documents/bfmc_workspace
    $ roslaunch sim_pkg map_with_all_objects.launch
    
    # Terminal 2
    $ source ~/Documents/python3_ws/py3venv/bin/activate
    $ cd ~/Documents/startup_workspace
    $ rosrun startup_package main.py
