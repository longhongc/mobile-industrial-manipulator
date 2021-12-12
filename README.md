# Mobile Industrial Manipulators (MIM)
This package implements different kinds of mobile manipulator that can be used in an industrial environment.  
It also demonstrate the pick and place of the mobile manipulators.   
<img src=videos/warehouse_simulation.jpg width="500" height="350" /> 

Robots:  
1. Differential drive mobile manipulator
2. Differential drive mobile manipulator with rocker-boggie structure  
   <img src=videos/rocker-boggie.png width="200" height="150" /> 



Authors: Chang-Hong Chen, Sparsh Jaiswal

## Environment
Ubuntu 20.04  
ROS Noetic  

## Dependencies
### ROS Packages
ros-control  
ros-controller  
```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```
### Python Library
```
pip3 install queue
```

## Run
### Warehouse Simulation
launch the differential drive robot and the roker-boggie robot
```
roslaunch mim_robots warehouse_simulation.launch
```
run pick and place on the differential drive robot  
<img src=videos/pick_and_place_tuned.gif width="500" height="350" />   
```
cd scripts
python3 diffdrive_pick_place.py
```
### Ramp demo
```
roslaunch mim_robots test_ramp.launch
```
run test ramp to show the advantage of rocker-boogie   
(Down: ordinary differential drive, Up: rocker-boggie) 
<p float="left">
<img src=videos/ramp_test.gif width="500" height="350" />  
<img src=videos/base_design.png width="350" height="400" />  
</p>  

```
cd scripts
python3 test_ramp.py
```

## Reference 
The factory simulation scene is from the following package:  
https://github.com/wh200720041/warehouse_simulation_toolkit
