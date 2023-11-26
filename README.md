# HomeBot-ROS2-Navigation
HomeBot-ROS2-Navigation: A ROS2-powered robotic platform for autonomous indoor navigation and mapping.

![HomeBot Image](mobile_robot_gazebo.gif) 

## Project Overview
This repository contains a suite of ROS2 packages for the HomeBot, an autonomous robot designed for indoor navigation and mapping. The provided image showcases the HomeBot's physical design, which the following packages bring to life:

- `robot_description`: Defines the robot's physical parameters, including its URDF files, visual meshes, and essential configurations for simulation purposes.
- `robot_simulation`: Contains the necessary configurations and launch files for simulating the HomeBot in a household environment, using tools like Gazebo and RViz for SLAM and navigation testing.
- `robot_patrol`: Implements a patrolling behavior, directing the robot to navigate autonomously through a series of predefined waypoints based on the generated map data.

These packages represent the core components of the HomeBot's functionality, demonstrating practical applications of ROS2 in robotic indoor navigation.

![HomeBot Image](slam.gif)  // Replace with the actual URL to the image hosted on GitHub or another image hosting service.

![HomeBot Image](navigation.gif) 


## Getting Started
### Prerequisites
  *ROS2 (tested with ROS2 Humble)*
  
  *Python 3.10*

  Nav2
    
## Installation

1. Create a ROS 2 workspace (if one does not already exist):
   ```sh
   mkdir -p ~/my_robot_ws
   cd ~/my_robot_ws
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
   source ~/my_robot_ws/install/setup.bash
   
## Usage
