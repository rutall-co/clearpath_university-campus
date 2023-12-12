# Unicampus Gazebo Simulation with Jackal Robot
## Overview
Welcome to the Unicampus Gazebo Simulation repository! This comprehensive guide is designed to help y
## Custom Gazebo World and Models
In this section, you will find detailed instructions on accessing and utilizing the custom models and ROS (R
### Prerequisites
To ensure a smooth setup, please have the following prerequisites installed:
- **ROS (Kinetic, Melodic, or Noetic)**: ROS is an essential framework for robot software development, pro
- **Gazebo**: A versatile 3D simulation environment that allows you to accurately and efficiently simulate y
- **Jackal Simulation Packages**: These are specialized ROS packages tailored for simulating the Jackal r
### Installation Guide
Follow these steps to set up your simulation environment:
1. **Repository Cloning**:
Clone the repository to your local machine:
```bash
git clone git@github.com:wlaa41/clearpath_university-campus.git
```

2. **Building the Workspace**:
Navigate to your workspace and compile the ROS packages:
```bash
cd ~/Documents/clearpath1
catkin_make
```
This command builds all the necessary components for the simulation.
3. **Environment Setup**:
Source the ROS environment to make the packages available in your current session:
```bash
source devel/setup.bash
```
### Adding Custom Models to Gazebo
Integrate custom models into your Gazebo simulations by following these steps:
1. **Model Transfer**:
Copy the `models` folder from the repository to your Gazebo model directory:
```bash
cp -r models/* ~/.gazebo/models/
```
2. **Verification**:
Ensure the models are correctly copied by checking the Gazebo model directory:
```bash
ls ~/.gazebo/models/
```
The output should list the custom model names.
### Launching the Simulation
Start the simulation by executing these commands:
1. **Gazebo World Launch**:
Initiate the Unicampus Gazebo world along with the Jackal robot:
```bash
roslaunch unicampus unicampus.launch
```
2. **RViz Visualization**:
To visualize the robot and its sensor data in RViz:
```bash
export JACKAL_LASER=1
roslaunch jackal_viz view_robot.launch
```
For an in-depth guide on customizing your simulation environment and additional simulation worlds, refer to
### Repository Structure
- `src`: Contains ROS packages crucial for simulation.
- `models`: Custom Gazebo models designed for the Unicampus world.
### Contributing to the Project
Your contributions to this project are highly appreciated. To contribute:
1. Fork the repository.
2. Make your changes or enhancements.
3. Submit a pull request for review.
### License
This project is licensed under the MIT License. For more details, refer to the LICENSE file in this repository