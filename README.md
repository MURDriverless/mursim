# MURSim

## Getting Started

## Getting ROS

The following link details the process of installing ROS and Gazebo on your computer:

[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

For this repository, Ubuntu 18.04, ROS Melodic and an internet connection is required.

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
source deve/setup.bash
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

## Notes

This repository is partially based off the [AMZ FSSIM](https://github.com/AMZ-Driverless/fssim).