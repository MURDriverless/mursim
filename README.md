# MURSim
<<<<<<< HEAD
## How to Launch
    
=======

# Getting Started

## Getting ROS

The following link details the process of installing ROS and Gazebo on your computer: 

[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

For this repository, Ubuntu 18.04, ROS Melodic and an internet connection is required.

## Downloading and Running

1. Create a workspace

    ```bash
    mkdir mursim && cd mursim
    ```

2. Clone the repository

    ```bash
    git clone https://github.com/MURDriverless/mursim.git
    ```

3. Rename the 'mursim' folder to 'src'

    ```bash
    mv mursim src
    ```

4. Build the package with Catkin and source environment

    ```bash
    catkin build && source devel/setup.bash
    ```

5. Run the 'src/mursim_gazebo/launch/spawn_world.launch' file with roslaunch

    ```bash
    roslaunch mursim_gazebo spawn_world.launch
    ```
>>>>>>> fbb82e16778dcbb7cb855a98098c2212c2a77882
