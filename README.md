# Description
Driver for Silicon Sensing DMU-class IMU sensors. Supports both DMU11 and DMU41. 

Based on the [original archived repository](https://github.com/leo-drive/dmu_ros), with minor changes to make the driver & rviz work with DMU41.

# Installation Instructions
Instructions are based on the [original published ROS driver](http://wiki.ros.org/dmu11driver).

## Clone directory
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src  
git clone https://github.com/lisusdaniil/dmu_ros
```
## Build Package
```
cd ~/catkin_ws
```
then run
```
catkin_make
```
or
```
catkin build dmu_ros
```
## Source Package
Add this command to your `~/.bashrc` file if you want this driver to be sourced automatically.
```
source devel/setup.bash
```

# Running the Driver
Change the respective launch file. For example, for a sensor connected to `/dev/ttyUSB0`, set:
```
<param name="device" value="/dev/ttyUSB0"/>
```
To launch the driver with an RVIZ visualization, run
```
roslaunch dmu_ros dmu11.launch
```
or
```
roslaunch dmu_ros dmu41.launch
```
depending on the connected sensor. To run the driver in the background with no RVIZ, simply run
```
roslaunch dmu_ros dmu_background.launch
```
