# ROS2 Autonomous Patrol with Turtlebot3

This repository houses the implementation of an autonomous patrol behavior for the Turtlebot3, developed using ROS2 and C++. It demonstrates a simple yet effective method for a robot to navigate and avoid obstacles within a designated area, both in a simulated environment and on a real robot.

## Project Overview

The goal of this project is to apply the foundational concepts of ROS2 to control a Turtlebot3 robot, enabling it to autonomously patrol while avoiding obstacles. It was tested in a Gazebo simulation as well as on a real Turtlebot3 robot operated remotely.

## Getting Started

### Prerequisites

- ROS2 Foxy Fitzroy
- Gazebo simulation environment
- A Turtlebot3 simulation package or access to a real Turtlebot3 robot

### Installation

1. **Set up your ROS2 environment** if you haven't already. Detailed instructions can be found on the official ROS2 documentation page.

2. **Clone this repository into your ROS2 workspace src folder:**

```bash
cd ~/ros2_ws/src
git clone https://github.com/MiguelSolisSegura/citylab_project.git
```

3. **Build the project:**

```bash
cd ~/ros2_ws
colcon build --packages-select robot_patrol
```

4. **Source your ROS2 environment:**

```bash
source ~/ros2_ws/install/setup.bash
```

### Running the Patrol Program

#### In Simulation

1. **Launch the Turtlebot3 simulation:**

Make sure you have the Turtlebot3 simulation environment set up. Then, launch the simulation:

```bash
source ~/simulation_ws/install/setup.bash
ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml
```

2. **Execute the patrol behavior:**

Open a new terminal, and ensure your ROS2 workspace is sourced. Then, launch the patrol program:

```bash
ros2 launch robot_patrol start_patrolling.launch.py
```

The robot should now start moving around the simulated environment, avoiding any obstacles in its path.

#### On the Real Robot

To run the program on a real Turtlebot3 robot, you will need access to a robot set up for remote operation. Follow the setup instructions provided by your robot's administrator. Once set up, you can launch the patrol program as described above.

## How to Test

To verify the patrol behavior, observe the robot (either in simulation or real life) as it navigates around the environment. It should autonomously move while avoiding obstacles, demonstrating the patrol behavior.
