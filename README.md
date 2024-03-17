# Autonomous Driving on F1/10th Vehicle with PID Controller

This repository contains the implementation of a PID controller on an F1/10th vehicle for autonomous driving，including ROS, LIDAR data handling, F110 Simulation environments, and the principles and application of follow the gap, AEB control, PID control algorithms and different mode swiching for autonomous vehicles.

## Features
- **Basic Manual Controls**: Implementations for both keyboard and joystick control.
- **Autonomous Driving Algorithm**: An autonomous driving algorithm using follow the gap controller, AEB control algorithms, a PID controller algorithms and different mode swiching for autonomous vehicles
- **Simulation and Real-World Testing**: The project can be run in a simulation environment as well as on a real track for hands-on experience.

## Requirements
- Ubuntu 20.04
- ROS noetic
- Additional installation of geometry2, navigation, navigation_msgs, PlotJuggler, ackermann_msgs and other packages is required.
- Suitable hardware for running the F1/10th vehicle if testing in the real world
  
## Algorithms
- **Follow the Gap Algorithm**: This is an obstacle avoidance strategy that initially attempts to avoid collisions by finding the largest distance between the vehicle and walls. However, since the largest distance often appears behind the vehicle, a strategy of limiting the LIDAR measurement range is introduced. By restricting the measurement angle range and increasing the minimum distance between the vehicle and obstacles upon detection, the system tries to improve obstacle avoidance performance. Yet, it still faces challenges at high speeds and during sharp turns.
- **Automatic Emergency Braking (AEB)**: This function is achieved by calculating the Time to Collision (TTC) with obstacles ahead, enabling automatic emergency braking. When a collision risk is detected, the AEB module intervenes, taking precedence over other control modes, to automatically decelerate or stop the vehicle, preventing a collision. To ensure safety, a "reverse safety" feature is also implemented, where the vehicle automatically reverses to increase the distance from the obstacle ahead after stopping.
- **PID Control**: To maintain a certain distance between the vehicle and the left wall, the project utilizes the PID control algorithm. The vehicle is guided to safely drive along the wall by measuring the minimum distance to the left wall and adjusting the steering angle based on the distance error.
- **Input Multiplexing**: The project accepts keyboard commands through the keyboard_teleop node, allowing users to switch between different modes. The system manages these different input modes by subscribing to and publishing specific ROS topics.

## Getting Started

### Setting up the Environment
Ensure that your system meets all the requirements listed above. The following steps will guide you through setting up the project environment.

1. **Enter Workspace Folder**
    Navigate to the workspace folder where you want to build the project.

    ```bash
    cd your_workspace_path
    ```

2. **Build the Project**
    Use `catkin_make` to build the ROS packages.

    ```bash
    catkin_make
    ```

3. **Source the Setup Script**
    To configure the environment variables for the project, source the setup script.

    ```bash
    source devel/setup.sh
    ```

### Launching the Project
To start the simulation or the real-world application, use the following command:

```bash
roslaunch f110_simulator simulator.launch
