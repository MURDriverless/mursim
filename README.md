# MURSim


## Prerequisities

The following link details the process of installing ROS and Gazebo on your computer:

[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

For this repository, Ubuntu 18.04, ROS Melodic and an internet connection is required.

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

2. Clone the repository

    ```bash
    git clone https://github.com/MURDriverless/mursim.git
    ```

3. Build the package with Catkin and source environment

    ```bash
    cd ../..
    catkin build && source devel/setup.bash
    ```

4. Run the 'src/mursim_gazebo/launch/spawn_world.launch' file with roslaunch
    ```bash
    roslaunch mursim_gazebo spawn_world.launch
    ```

## Changing Tracks
The track for Formula SAE (US) Lincoln 2014 endurance was added for testing, to enable this, remove the comment braces around the following lines
```
<arg name="map"     value="large_track"/>
<arg name="x_spawn" value="16"/>
<arg name="y_spawn" value="32.25"/>
```
in `mursim_gazebo/launch/slow_lap.launch`

## Vehicle Sensors Packages
The following packages are required to simulate the perception and spatial sensors.
Clone these into your `catkin_ws/src` folder.

Please note that in order to **simulate the LiDAR efficiently**, you would need to update your Gazebo to the latest version.
See official Gazebo tutorial [here](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0) for detailed instructions.

```
# for simulating lidar sensor
git clone https://github.com/MURDriverless/ouster_example

# for simulating gps and imu sensors
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
```

Assuming you have `catkin tools`, build the packages.
```
catkin build
source devel/setup.bash
```

Certain key options of the sensor suite can be updated with the following `yaml` file.
This includes:
* Toggle LiDAR simulation (default: false)
* Toggle stereo camera simulation (default: false)
* Publish topic names for sensors
```
mursim/mursim_description/cars/gotthard/config/sensors.yaml
```

Other sensor specific configurations would need to be made directly in the `.xacro` files.
```
# main vehicle definition
src/mursim/mursim_description/cars/gotthard/urdf/vehicle.xacro

# lidar link/joint and stereo camera
src/mursim/mursim_description/cars/gotthard/urdf/vehicle_sensors.xacro

# sensor links and joints
src/mursim/mursim_description/cars/gotthard/urdf/vehicle_sensors_links.xacro
```

### LiDAR Pipeline Simulation

Use [this particular lidar_dev branch](https://github.com/MURDriverless/lidar_dev/tree/task-262-lidar-sim). For best performance, set the cmake argument as follows before calling `catkin build`.
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Notes

This repository is partially based off the [AMZ FSSIM](https://github.com/AMZ-Driverless/fssim).
