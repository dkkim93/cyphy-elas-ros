# elas_ros with dynamic_reconfigure

## Introduction
This code is modified from [the original elas_ros source code]( https://github.com/jeffdelmerico/cyphy-elas-ros).

Main modification is an addition of [the dynamic_reconfigure function](http://wiki.ros.org/dynamic_reconfigure) for dynamically tuning the elas hyperparameters. 

## Build
Please follow [the ROS tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for creating a workspace (e.g., ~/ros_ws).
In the workspace src folder, build the package by:

```
cd ~/ros_ws/src/
git clone https://github.com/dkkim93/cyphy-elas-ros
cd ~/ros_ws/ && catkin_make
```

## Instruction
To run the package:

```
cd ~/ros_ws/devel/lib/elas_ros/
./elas_reconfigure
```

And to dynamically reconfigure:

```
rosrun rqt_reconfigure rqt_reconfigure
```

## Contact
Dong-Ki Kim

Email: dkkim93@mit.edu
