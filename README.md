# Warthog Simulator

Simulator (Gazebo and RVIZ) for the Clearpath Warthog Robot with an attached VLP-16 Lidar and camera.

## Table of Contents
  - [Installation](#installation)
  - [Usage](#usage)
  - [Worlds](#worlds)
  - [Code Structure](#code-structure)

## Installation
1. Follow [this link](http://wiki.ros.org/ROS/Installation) to install ROS
2. Follow [this link](https://www.clearpathrobotics.com/assets/guides/kinetic/warthog/WarthogInstallation.html) to install Warthog packages. Make sure to install both the simulation package and the desktop one:
   * `sudo apt-get install ros-[ROS VERSION]-warthog-desktop`
   * `sudo apt-get install ros-[ROS VERSION]-warthog-simulator`
   
Test your installation of the Warthog packages using the following commands:
   * `roslaunch warthog_gazebo warthog_world.launch`
   * `roslaunch warthog_viz view_robot.launch`
   
Next, `cd` into your `catkin_ws/src` folder and run the following:  

`git clone https://github.com/aaronzberger/CMU_Warthog_Velodyne.git` (adds this  to your catkin workspace)  
`sudo cp -r CMU_Warthog_Velodyne/vine_models/* ~/.gazebo/models` (copies the vine models to your gazebo models folder)  

Test your installation by [running the package](#usage) 

## Usage

`roslaunch warthog_velodyne warthog_vlp16.launch`

## Worlds

This module only contains one world by default: `empty.world`, which is an empty Gazebo world.

However, I also built a module to create new worlds with simulated vineyards.

That module can be found [HERE](https://github.com/aaronzberger/CMU_Vineyard_World_Creator)

## Code Structure

    CMU_Warthog_Velodyne  
    │  
    ├── launch  
    │   ├── description.launch  --> Selects warthog_velodne.urdf.xacro as the custom robot model to use  
    │   ├── spawn_warthog.launch  --> Modify lines 6-9 to change robot starting position (launches description.launch)  
    │   └── warthog_vlp16.launch  --> Modify line 28 to select which world you wish to launch in (launches RVIZ, Gazebo, and spawn_warthog.launch)  
    │  
    ├── urdf  
    │   ├── empty.urdf  --> An empty urdf file is required to run  
    │   ├── VLP-16.urdf  --> Modify lines 5-6 to change Lidar position (launches Lidar)  
    │   └── warthog_velodyne.urdf.xacro  --> Custom Warthog model containing a Lidar and camera
    │  
    ├── vine_models  --> Vine models to copy to your gazebo models folder (~/.gazebo/models)
    │   ├── Vine1  
    │   ├── Vine2  
    │   └── Vine3  
    │  
    ├── worlds  
    │   └── empty.world  --> An empty world, to use until you [create custom ones](https://github.com/aaronzberger/CMU_Vineyard_World_Creator) 
    │
    ├── CMakeLists.txt  --> ROS Required for compilation  
    └── package.xml  --> ROS Required for compilation  
