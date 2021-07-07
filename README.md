# MURSim


## Prerequisities

The following link details the process of installing ROS and Gazebo on your computer:

[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

For this repository, Ubuntu 18.04, ROS Melodic and an internet connection is required.

Update Gazebo to the latest minor version 9.19.* for use with ROS Melodic, otherwise `gpu_ray` (GPU accelerated LiDAR simulation) will most likely fail to run.

1. Follow upgrade instructions [here](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0) remember that we want `gazebo9`.
2. Upgrade math package `sudo apt upgrade libignition-math2`

Install `tf2_sensor_msgs` and `glog` (Google logging module)

```
sudo apt install ros-melodic-tf2-sensor-msgs libgoogle-glog-dev
```
Install package for effort controller and position controllers

```
sudo apt install ros-melodic-effort-controllers
sudo apt install ros-melodic-position-controllers
```



## Downloading and Running 


1. Create a workspace

    ```bash
    mkdir mursim && cd mursim
    mkdir src && cd src
    ```

2. Clone the repository (current working version) (dev/aldrei21)
   Run `mur_init.sh` found here:https://github.com/MURDriverless/mursim_init/tree/master/mur_init within the `/src` of your ros/catkin workspace IMPORTANT, current branch is "master"


3. Build the package with Catkin and source environment

    ```bash
    cd ..
    catkin build && source devel/setup.bash
    ```

4. Start the sim: using the small track
    ```bash
    roslaunch mursim_gazebo slow_lap.launch
    ```
    using the medium track:
    ```bash
    roslaunch mursim_gazebo slow_lap_2.launch
    ```
    

## Notes

This repository is partially based off the [AMZ FSSIM](https://github.com/AMZ-Driverless/fssim).

