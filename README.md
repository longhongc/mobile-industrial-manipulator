# Mobile Industrial Manipulators (MIM)
This package implements different kinds mobile manipulators that can be used in an industrial environment.  

Robots:
1. Differential drive mobile manipulator
2. Differential drive mobile manipulator with rocker-boggie structure

author: Chang-Hong Chen, Sparsh Jaiswal

## Run
launch the differential drive robot and the roker-boggie robot
```
roslaunch mim_robots warehouse_simulation.launch
```
run pick and place on the differential drive robot
```
cd scripts
python3 diffdrive_pick_place.py
```

## Reference 
The factory simulation scene is from the following package:  
https://github.com/wh200720041/warehouse_simulation_toolkit
