# Mobile Industrial Manipulators (MIM)
This package implements different kinds mobile manipulators that can be used in an industrial environment.  

Robots:
1. Differential drive mobile manipulator
2. Differential drive mobile manipulator with rocker-boggie structure

Authors: Chang-Hong Chen, Sparsh Jaiswal

## Run
### Warehouse Simulation
launch the differential drive robot and the roker-boggie robot
```
roslaunch mim_robots warehouse_simulation.launch
```
run pick and place on the differential drive robot
```
cd scripts
python3 diffdrive_pick_place.py
```
### Ramp demo
```
roslaunch mim_robots test_ramp.launch
```
run test ramp to show the advantage of rocker-boogie
```
cd scripts
python3 test_ramp.py
```

## Reference 
The factory simulation scene is from the following package:  
https://github.com/wh200720041/warehouse_simulation_toolkit
