# elas_ros with dynamic_reconfigure

## Introduction
This code is modified from the original source code: https://github.com/jeffdelmerico/cyphy-elas-ros.

Main modification from the source code is the addition of the dynamic_reconfigure function (http://wiki.ros.org/dynamic_reconfigure) for dynamically tuning the elas hyperparameters. 

## Build
```
mkdir -p ~/ros_ws/src && cd ~/ros_ws/
catkin_make
source devel/setup.bash
cd src/ && git clone https://github.com/dkkim93/cyphy-elas-ros
cd ..
catkin_make
```

## Instruction
```
cd devel/lib/elas_ros/
./elas_reconfigure
rosrun rqt_reconfigure rqt_reconfigure
```

## Contact
Dong-Ki Kim

Email: dkkim93@mit.edu
