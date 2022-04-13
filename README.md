# fusion_nav
iAmech rtabmap  sensor fusion navigation

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
