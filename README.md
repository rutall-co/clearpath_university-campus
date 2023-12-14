
# Unicampus Gazebo Simulation with Jackal Robot
## Overview
Welcome to the Unicampus Gazebo Simulation repository! This comprehensive guide is designed to
help you set up and run a realistic simulation of the Jackal robot in a custom Gazebo world,
specifically tailored for educational and research purposes.
## Custom Gazebo World and Models
In this section, you will find detailed instructions on accessing and utilizing the custom models and
ROS packages developed for the Unicampus Gazebo simulation.
### Prerequisites
To ensure a smooth setup, please have the following prerequisites installed:
- **ROS (Kinetic, Melodic, or Noetic)**: ROS is an essential framework for robot software
development, providing libraries and tools to help software developers create robot applications.
- **Gazebo**: A versatile 3D simulation environment that allows you to accurately and efficiently
simulate your robot in complex environments.
- **Jackal Simulation Packages**: These are specialized ROS packages tailored for simulating the
Jackal robot, including its dynamics and sensor suite.
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
Also, in the repository, there is a file named `launch`. You must run the following command:
```bash
catkin_create_pkg unicampus rospy roscpp
```
Then, place the `launch` file inside the new directory if it is not created.
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
### Repository Structure
- `src`: Contains ROS packages crucial for simulation.
- `models`: Custom Gazebo models designed for the Unicampus world.
### Contributing to the Project
Your contributions to this project are highly appreciated. To contribute:
1. Fork the repository.
2. Make your changes or enhancements.
3. Submit a pull request for review.
### License
This project is licensed under the MIT License. For more details, refer to the LICENSE file in this
repository.
### Optional: Installing teleop_twist_keyboard
For enhanced control of your robot in the simulation, you may opt to install the
`teleop_twist_keyboard` package. This package allows you to control the robot's movement using
your keyboard's arrow keys.
#### Update Your Package List
First, make sure you have the latest information about available packages:
```bash

sudo apt-get update
```
#### Install the teleop_twist_keyboard Package
Depending on your version of ROS, use one of the following commands:
- For ROS Melodic or Noetic:
 ```bash
 sudo apt-get install ros-melodic-teleop-twist-keyboard
 ```
 or
 ```bash
 sudo apt-get install ros-noetic-teleop-twist-keyboard
 ```
- For ROS Kinetic:
 ```bash
 sudo apt-get install ros-kinetic-teleop-twist-keyboard
 ```
#### Using the teleop_twist_keyboard Package
Once the installation is complete, you can launch the keyboard teleoperation interface:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Make sure you have the ROS environment properly sourced in your terminal before running the
`rosrun` command.
### Reactive Navigation Demo
To demonstrate reactive navigation with the Jackal robot:
```bash
roslaunch jackal_navigation odom_navigation_demo.launch
```
Then, visualize the robot and its environment in RViz:
```bash
rviz roslaunch jackal_viz view_robot.launch config:=navigation
```
### Mapping with Gmapping Demo
For mapping using gmapping:
```bash
roslaunch jackal_navigation gmapping_demo.launch
```
Visualize the mapping process in RViz:
```bash
rviz roslaunch jackal_viz view_robot.launch config:=gmapping
```
After mapping, save the map using:
```bash
rosrun map_server map_saver -f mymap

```
### Deliberative Navigation with AMCL Demo
For AMCL-based navigation (requires a pre-existing map):
```bash
roslaunch jackal_navigation amcl_demo.launch [map_file map.yaml]
```
Visualize the AMCL localization in RViz:
```bash
rviz roslaunch jackal_viz view_robot.launch config:=localization
```
Each of these commands launches different aspects of the robot's navigation capabilities, allowing
for experimentation and learning in various scenarios. Ensure that you have the necessary
packages and configurations set up as per the earlier sections of this guide.


# Navigating the Unknown: Mastering AMCL in ROS

## Embarking on the Robotic Odyssey
Embark on a journey into the realm of robotics where understanding your environment is the key to not just surviving, but thriving. Adaptive Monte Carlo Localization (AMCL) is your trusty map and compass in the vast seas of autonomous navigation.

### The Heart of Navigation: AMCL
Adaptive Monte Carlo Localization (AMCL) is a powerful particle filter-based localization system designed for robots to find themselves in a predefined world. It's like giving your robot a sense of 'self' within the digital landscapes it traverses.

### Prerequisites: The Cartographer's Tools
Before setting sail, ensure you have the following:
- A digital map (`map.yaml` and `map.pgm`)—your world from a bird's eye view.
- The `amcl` ROS package—the brain behind the localization.

### Commanding the Helm: Launching AMCL
```bash
roslaunch jackal_navigation amcl_demo.launch map_file:=/path/to/your/map.yaml
```
This incantation conjures up the AMCL wizardry. Your robot awakens to the world around it, using lasers to feel out the space and odometry as its internal compass.

### The Seer's Vision: Visualizing with RViz
```bash
roslaunch jackal_viz view_robot.launch config:=localization
```
With RViz, gaze into the crystal ball where you see your robot's thoughts—the map, its path, and how it perceives its surroundings.

## The Alchemy of Parameters: AMCL's Inner Workings
Tune the potions and spells—AMCL parameters—to refine your robot's senses.
- `~use_map_topic`: Decides if the robot should listen for map updates.
- `~odom_frame_id`: The name tag for your robot's odometry frame.
- `~base_frame_id`: The name tag for your robot's chassis.
- `~global_frame_id`: The grand stage, usually dubbed 'the map'.

## Sharpening Your Robot's Intuition: Tuning AMCL
Your robot's intuition is only as sharp as its calibration. Like a finely tuned instrument, adjust the odometry and sensor models to resonate with the environment.

## Conclusion: The Enlightened Machine
Armed with AMCL and RViz, your robot is no longer a mere wanderer but an enlightened traveler with a purpose and a destination.

Remember, the art of robotics is a path of continuous learning. Each challenge is a lesson, and every failure is a stepping stone to success.

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