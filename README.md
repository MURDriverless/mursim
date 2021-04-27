# MURSim


## Prerequisities

The following link details the process of installing ROS and Gazebo on your computer:

[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

For this repository, Ubuntu 18.04, ROS Melodic and an internet connection is required.

Update Gazebo to the latest minor version 9.13.2 for use with ROS Melodic, otherwise `gpu_ray` (GPU accelerated LiDAR simulation) will most likely fail to run.

1. Follow upgrade instructions [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Alternativeinstallation:step-by-step), but instead of getting the latest `gazebo11`, we want `gazebo9`.
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

2. Clone the repository (current working version)
   Run `mur_init.sh` found here:https://github.com/MURDriverless/mursim_init/tree/master/mur_init within the `/src` of your ros/catkin workspace IMPORTANT, current branch is "master"


3. Build the package with Catkin and source environment

    ```bash
    cd ..
    catkin build && source devel/setup.bash
    ```

4. Start gazebo: Run the 'src/mursim_gazebo/launch/slow_lap.launch' file with roslaunch
    ```bash
    roslaunch mursim_gazebo spawn_world.launch
    ```
5. Start the autonomous pipeline:   
    In a seperate terminal, navigate to your mursim workspace
    source the envionment, and launch the pipeline. 
    ```bash
    source devel/setup.bash
    roslaunch mursim_gazebo auto_pipeline.launch
    ```
    

## Notes

This repository is partially based off the [AMZ FSSIM](https://github.com/AMZ-Driverless/fssim).

