# RoboND Robot Localization Project
## using AMCL ROS Package
## (c) Muthanna A. Attyah 
## May 2018


## Repository installation steps:

* Clone from git repository
```bash
$ cd ~
$ git clone https://github.com/mkhuthir/RoboND_Robot_Localization_Project.git catkin_ws
```

* Install the following packages if it is not yet installed:

```bash
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-move-base
$ rospack profile
$ sudo apt-get install ros-kinetic-amcl
```

* Compile code using `catkin_make` and source it.

```bash
$ cd catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

## Launching udacity_bot Packages

```bash
$ roslaunch udacity_bot udacity_empty_world.launch
```
```bash
$ roslaunch udacity_bot udacity_world.launch
```
```bash
$ roslaunch udacity_bot amcl.launch
```
```bash
$ rosrun udacity_bot navigation_goal
```

## udacity_bot creation steps

<p align="center"> <img src="./misc/udacity_bot_1.jpg"> </p>
<p align="center"> <img src="./misc/udacity_bot_2.jpg"> </p>
<p align="center"> <img src="./misc/udacity_bot_3.jpg"> </p>
<p align="center"> <img src="./misc/udacity_bot_4.jpg"> </p>
<p align="center"> <img src="./misc/udacity_bot_5.jpg"> </p>
<p align="center"> <img src="./misc/udacity_bot_6.jpg"> </p>

## udacity_bot Test Video

## Launching muth_bot Packages

```bash
$ roslaunch muth_bot muth_empty_world.launch
```
```bash
$ roslaunch muth_bot muth_world.launch
```
```bash
$ roslaunch muth_bot amcl.launch
```
```bash
$ rosrun muth_bot goto_goal
```

[![test video](http://img.youtube.com/vi/lxxqGsvKArw/0.jpg)](http://www.youtube.com/watch?v=lxxqGsvKArw)

## muth_bot creation steps

<p align="center"> <img src="./misc/muth_bot_1.jpg"> </p>
<p align="center"> <img src="./misc/muth_bot_2.jpg"> </p>
<p align="center"> <img src="./misc/muth_bot_3.jpg"> </p>
<p align="center"> <img src="./misc/muth_bot_4.jpg"> </p>
<p align="center"> <img src="./misc/muth_bot_5.jpg"> </p>
<p align="center"> <img src="./misc/muth_bot_6.jpg"> </p>

## muth_bot Test Video

[![test video](http://img.youtube.com/vi/ODnIm-24Zgw/0.jpg)](http://www.youtube.com/watch?v=ODnIm-24Zgw)
