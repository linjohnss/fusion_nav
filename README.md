# fusion_nav
iAmech rtabmap  sensor fusion navigation

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/bkO1Wum0htg/0.jpg)](https://youtu.be/bkO1Wum0htg)


## RTAB-Map
```shell
sudo apt install ros-noetic-rtabmap-ros
```
## Velodyne
```shell
git clone https://github.com/ros-drivers/velodyne.git
```
## Mapping
```shell
roslaunch fusion_nav vlp16_mapping.launch
```
## Navigation
```shell
roslaunch fusion_nav vlp16_nav.launch
```
## Send Goal
```shell
rosrun fusion_nav navigation_goals.py
```
