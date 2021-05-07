# ADS - BFMC
Autonomous driving systems for BFMC competition.

## Detection example
<img width="603" alt="o1" src="https://user-images.githubusercontent.com/62132206/117426738-8c274780-af24-11eb-8214-354e22ccf3e9.png">
<img width="595" alt="o2" src="https://user-images.githubusercontent.com/62132206/117426748-8fbace80-af24-11eb-92e2-6cf1c9193000.png">
<img width="587" alt="o3" src="https://user-images.githubusercontent.com/62132206/117426751-90ebfb80-af24-11eb-97da-4b6d0e596f27.png">


## 1. Hierarchy overview

    ~/Documents
        |__ bfmc_workspace
        |__ startup_workspace
        |__ python3_ws

## 2. Installation

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

## 3. Launch

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
