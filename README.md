# ROS2 Autonomous Patrol with Turtlebot3

This repository houses the implementation of an autonomous patrol behavior for the Turtlebot3, developed using ROS2 and C++. It demonstrates a simple yet effective method for a robot to navigate and avoid obstacles within a designated area, both in a simulated environment and on a real robot.

## Project Overview

The project extends basic ROS2 services and actions to control a Turtlebot3 robot for autonomous patrolling and precise navigation to designated positions. It includes testing in a Gazebo simulation and operations on an actual Turtlebot3, emphasizing the transition from simulation to real-world application.

## Getting Started

### Prerequisites

- ROS2 Humble
- Gazebo for simulation
- Turtlebot3 simulation package or a real Turtlebot3 robot setup

### Installation

1. **Set up the ROS2 environment**, referring to the ROS2 documentation if necessary.

2. **Clone this repository to your ROS2 workspace's src directory:**

```bash
cd ~/ros2_ws/src
git clone https://github.com/MiguelSolisSegura/citylab_project.git
```

3. **Compile the project:**

```bash
cd ~/ros2_ws
colcon build --packages-select robot_patrol robot_navigation
```

4. **Prepare the ROS2 environment:**

```bash
source ~/ros2_ws/install/setup.bash
```

### Execution Instructions

#### Simulation Environment

1. **Start the Turtlebot3 simulation:**

Ensure the Turtlebot3 simulation environment is ready, then initiate it:

```bash
source ~/simulation_ws/install/setup.bash
ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml
```

2. **Run the advanced navigation and patrol program:**

After sourcing your ROS2 workspace, launch the improved patrol program:

```bash
ros2 launch robot_patrol main.launch.py
```

The robot will begin its patrol, utilizing advanced navigation strategies and obstacle avoidance mechanisms.

#### Real Robot Execution

For deployment on a real Turtlebot3, ensure you have remote access to a properly set up robot. Adhere to the setup instructions provided by the robot's administrator. You can then execute the enhanced patrol program as described above.

## Testing and Validation

### Testing Services and Actions

#### Services

To test the ROS2 services developed for dynamic obstacle avoidance:

1. **Launch the service server:**

```bash
ros2 launch robot_patrol start_direction_service.launch.py
```

2. **Execute the service test client:**

```bash
ros2 launch robot_patrol start_test_service.launch.py
```

This will simulate laser data to the service and print the recommended navigation direction based on obstacle proximity.

#### Actions

To test the ROS2 actions for navigating to specific positions:

1. **Initiate the action server:**

```bash
ros2 launch robot_patrol start_gotopose_action.launch.py
```

2. **Send a goal position to the robot:**

```bash
ros2 action send_goal /go_to_pose robot_patrol/action/GoToPose "{goal_pos: {x: 1.0, y: 0.5, theta: 45.0}}"
```

Monitor the robot as it moves to the specified position, demonstrating the action's effectiveness.

### Observational Testing

For both simulated and real-world setups, observe the robot as it performs its patrol duties. It should autonomously navigate the environment, avoid obstacles efficiently, and reach designated goal positions, showcasing the integrated advanced functionalities.
