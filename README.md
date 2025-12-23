# ROS Competition — Autorace 2025

This repository contains a ROS2 package for participation in the AutoRace 2025 competition.
The solution implements a robot control algorithm for completing a track with:

- starting at a traffic light,
- detecting and passing an intersection,
- stopping and publishing the result to a ROS2 topic.

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws    # your workspace folder
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

2. Build the project

    ```bash
    colcon build
    ```

3. Source the workspace

    ```bash
    . ~/template_ws/install/setup.bash
    ```

4. Launching the map and robot for competition

    ```bash
    ros2 launch robot_bringup autorace_2025.launch.py
    ```

5. Launching the algorithm for competition

    ```bash
    ros2 launch autorace_core_ros_sos autorace_core.launch.py
    ```

## About the project

Algorithm:

1. Waits for the green light based on camera data.
2. Drives along the center of the road
3. Detects the intersection and the direction arrow using OpenCV.
4. Uses lidar to estimate the distance to the sign and the start of the turn.
5. Drives through the intersection, reversing using lidar.
6. After a successful pass, publishes a message to the topic.

## Sensors Used

|Sensor|Role|
|------|-------|
|RGB Camera|Analysis of road markings, colors, and signs|
|Depth Camera|Refining distances in front of the robot|
|Lidar|Measures distances for state transitions|
|IMU|Orientation and motion stabilization|
|ROS2 Topics|Control and Publish Results|

## Requirements

- ROS2 Humble/Jazzy
- Autorace 2025 Gazebo scene from the official repository
- Standard robot model without modifications