# HomeBot-ROS2-Navigation
HomeBot-ROS2-Navigation: A ROS2-powered robotic platform for autonomous indoor navigation and mapping.

![HomeBot Image](mobile_robot_gazebo.gif) 

## Project Overview
This repository contains a suite of ROS2 packages for the HomeBot, an autonomous robot designed for indoor navigation and mapping. The provided image showcases the HomeBot's physical design, which the following packages bring to life:

- `robot_description`: Defines the robot's physical parameters, including its URDF files, visual meshes, and essential configurations for simulation purposes.
- `robot_simulation`: Contains the necessary configurations and launch files for simulating the HomeBot in a household environment, using tools like Gazebo and RViz for SLAM and navigation testing.
- `robot_patrol`: Implements a patrolling behavior, directing the robot to navigate autonomously through a series of predefined waypoints based on the generated map data.

These packages represent the core components of the HomeBot's functionality, demonstrating practical applications of ROS2 in robotic indoor navigation.

## Getting Started
### Prerequisites
  *ROS2 (tested with ROS2 Humble)* - [Documentation](https://docs.ros.org/en/humble/index.html)
  
  *Python 3.10*

  *Nav2* - [Github](https://github.com/ros-planning/navigation2) and [Documentation](https://docs.ros.org/en/humble/index.html)
  
  *Slam_toolbox* - [Github](https://github.com/SteveMacenski/slam_toolbox)
    
## Installation

1. Create a ROS 2 workspace (if one does not already exist):
   ```sh
   mkdir ~/your_workspace-name_ws
   cd ~/your_workspace-name_ws
2. Clone this repository into your workspace:
   ```sh
   git clone https://github.com/aimechengineer/HomeBot-ROS2-Navigation.git
3. Install any dependencies using rosdep:
   ```sh
   rosdep install --from-paths src --ignore-src -r -y
5. Build the workspace:
   ```sh
   colcon build --symlink-install
7. Source the setup script:
   ```sh
   source ~/your_workspace-name/install/setup.bash
   
## Usage
### robot_description
Display HomeBot in RViz:

    ros2 launch robot_description display.launch.xml
    


Display HomeBot in Gazebo and Rviz:

    ros2 launch robot_description gazebo.launch.xml


### robot_simulation and robot_patrol
![HomeBot Image](slam.gif)  
![HomeBot Image](navigation.gif) 
